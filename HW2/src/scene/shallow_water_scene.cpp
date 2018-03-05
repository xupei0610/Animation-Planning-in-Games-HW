#include <sstream>
#include <iomanip>
#include <cstring>
#include <cblas.h>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/transform.hpp>

#include "scene/shallow_water_scene.hpp"
#include "item/floor.hpp"
#include "shader/skybox.hpp"
#include "app.hpp"
#include "global.hpp"
#include "util/random.hpp"
#include "util/cuda.hpp"

#include "stb_image.hpp"
#include <cuda_gl_interop.h>

using namespace px;

class scene::ShallowWaterScene::impl
{
public:
    bool need_upload;
    unsigned int vao[6], vbo[7];

    Shader *water_shader, *bound_shader, *mesh_shader;
    SkyBox *skybox;
    item::Floor *floor;
    struct cudaGraphicsResource *res[2];

    float step_size;
    int max_steps;
    float height_min, height_max;
    unsigned int n_triangles;

    bool request_drop, render_cpu, render_mesh;

    float field_height;
    float half_g;
    float damping;
    float *h, *u, *v, *h_x, *u_x, *v_x, *h_y, *u_y, *v_y;
    glm::vec3 *n;
    int grid_x, grid_y;
    float gap_x, gap_y;
    float inv_gap_x, inv_gap_y;

    const char *WATER_VS = "#version 420 core\n"
            "layout (location = 0) in vec2  pos;"
            "layout (location = 1) in float h;"
            "layout (location = 2) in vec3 norm;"
            ""
            "layout (std140, binding = 0) uniform GlobalAttributes"
            "{"
            "   mat4 view;"
            "   mat4 proj;"
            "   vec3 cam_pos;"
            "};"
            ""
            "const vec3 light = normalize(vec3(5,5,5));"
            ""
            "out vec3 gLight;"
            "out vec3 gPos;"
            "out vec3 gNorm;"
            ""
            "uniform mat4 model;"
            ""
            "void main(){"
            "   mat4 MV = view * model;"
            "   vec4 pos4 = MV * vec4(pos.x, h, pos.y, 1.f);"
            ""
            "   gLight = (view * vec4(light, 0.f)).xyz;"
            "   gPos = pos4.xyz/pos4.w;"
            "   gNorm = (transpose(inverse(MV)) * vec4(normalize(norm),0.0)).xyz;"
            "   gl_Position = proj * pos4;"
            "}";
    const char *WATER_FS = "#version 330 core\n"
            "in vec3 gLight;"
            "in vec3 gPos;"
            "in vec3 gNorm;"
            ""
            "out vec4 fragColor;"
            ""
            "void main(){"
            ""
            "   vec3 L = normalize(gPos-gLight);"
            "   vec3 V = normalize(-gPos);"
            "   vec3 H = normalize(L + V);"
            ""
            "   float ambient = .25f;"
            "   float diffuse = .05f * max(0.f, dot(gNorm, L));"
            "   float spec = pow(max(0.f, dot(gNorm, H)), 6);"
            ""
            "   fragColor = vec4(vec3(0.2588f, 0.5255f, 0.9569f) * (ambient + diffuse + vec3(.05f) * spec), 0.75f);"
            "}";

    const char *DEPTH_VS = "#version 420 core\n"
            "layout (location = 0) in vec2  pos1;"
            "layout (location = 1) in vec2  pos2;"
            "layout (location = 2) in float h1;"
            "layout (location = 3) in float h2;"
            ""
            "out VS_OUT"
            "{"
            "   vec2 pos1;"
            "   vec2 pos2;"
            "   float h1;"
            "   float h2;"
            "} primitive;"
            ""
            "void main()"
            "{"
            "   primitive.pos1 = pos1;"
            "   primitive.pos2 = pos2;"
            "   primitive.h1 = h1;"
            "   primitive.h2 = h2;"
            "}";
    const char *DEPTH_GS = "#version 430 core\n"
            "layout (points) in;"
            "layout (triangle_strip, max_vertices = 4) out;"
            ""
            "layout (std140, binding = 0) uniform GlobalAttributes"
            "{"
            "   mat4 view;"
            "   mat4 proj;"
            "   vec3 cam_pos;"
            "};"
            "uniform mat4 model;"
            "uniform float field_height;"
            "uniform vec3 norm;"
            ""
            "in VS_OUT"
            "{"
            "   vec2 pos1;"
            "   vec2 pos2;"
            "   float h1;"
            "   float h2;"
            "} primitive[];"
            ""
            "out vec4 gColor;"
            "out vec3 gPos;"
            "out vec3 gLight;"
            "out vec3 gNorm;"
            ""
            "const vec3 light = normalize(vec3(1, 1, 1));"
            ""
            "void main()"
            "{"
            ""
            "   mat4 MV = view * model;"
            "   gLight = (view * vec4(light, 0.f)).xyz;"
            "   gNorm = (transpose(inverse(MV)) * vec4(normalize(norm), 0.0)).xyz;"
            ""
            "   gColor = vec4(0.1f, 0.1f, 0.44f, 1.f);"
            "   vec4 pos4 = MV * vec4(primitive[0].pos1.x, field_height, primitive[0].pos1.y, 1.f);"
            "   gPos = pos4.xyz/pos4.w;"
            "   gl_Position = proj * pos4;"
            "   EmitVertex();"
            "   gColor = vec4(0.2588f, 0.5255f, 0.9569f, .75f);"
            "   pos4 = MV * vec4(primitive[0].pos1.x, primitive[0].h1, primitive[0].pos1.y, 1.f);"
            "   gPos = pos4.xyz/pos4.w;"
            "   gl_Position = proj * pos4;"
            "   EmitVertex();"
            "   gColor = vec4(0.1f, 0.1f, 0.44f, 1.f);"
            "   pos4 = MV * vec4(primitive[0].pos2.x, field_height, primitive[0].pos2.y, 1.f);"
            "   gPos = pos4.xyz/pos4.w;"
            "   gl_Position = proj * pos4;"
            "   EmitVertex();"
            "   gColor = vec4(0.2588f, 0.5255f, 0.9569f, .75f);"
            "   pos4 = MV * vec4(primitive[0].pos2.x, primitive[0].h2, primitive[0].pos2.y, 1.f);"
            "   gPos = pos4.xyz/pos4.w;"
            "   gl_Position = proj * pos4;"
            "   EmitVertex();"
            ""
            "   EndPrimitive();"
            "}";
    const char *DEPTH_FS = "#version 330 core\n"
            "in vec4 gColor;"
            "in vec3 gPos;"
            "in vec3 gLight;"
            "in vec3 gNorm;"
            ""
            "out vec4 fragColor;"
            ""
            "void main(){"
            ""
            "   vec3 L = normalize(gLight);"
            "   vec3 V = normalize(-gPos);"
            "   vec3 H = normalize(L + V);"
            ""
            "   float ambient = .25f;"
            "   float diffuse = .05f * max(0.f, dot(gNorm, L));"
            "   float spec = pow(max(0.f, dot(gNorm, H)), 6);"
            ""
            "   fragColor = vec4(gColor.rgb * (ambient + diffuse + vec3(1.f) * spec), gColor.a);"
            "}";
    const char *MESH_VS = "#version 420 core\n"
            "layout (location = 0) in vec2  pos;"
            "layout (location = 1) in float h;"
            ""
            "layout (std140, binding = 0) uniform GlobalAttributes"
            "{"
            "   mat4 view;"
            "   mat4 proj;"
            "   vec3 cam_pos;"
            "};"
            ""
            "uniform mat4 model;"
            ""
            "void main(){"
            "   gl_Position = proj * view * model * vec4(pos.x, h, pos.y, 1.f);"
            "}";
    const char *MESH_FS = "#version 330 core\n"
            ""
            "out vec4 fragColor;"
            ""
            "void main(){"
            ""
            "   fragColor = vec4(0.2588f, 0.5255f, 0.9569f, 1.f);"
            "}";

    impl() : vao{0}, vbo{0},
             water_shader(nullptr), bound_shader(nullptr), mesh_shader(nullptr),
             skybox(nullptr), floor(nullptr),
             res{nullptr},
             h(nullptr),   u(nullptr),   v(nullptr),
             h_x(nullptr), u_x(nullptr), v_x(nullptr),
             h_y(nullptr), u_y(nullptr), v_y(nullptr),
             n(nullptr)
    {}
    ~impl()
    {
        clearGLObjs();

        delete water_shader;
        delete bound_shader;
        delete mesh_shader;
        delete skybox;
        delete floor;

        clearStateArray();

        if (res[0] != nullptr)
            PX_CUDA_CHECK(cudaGraphicsUnregisterResource(res[0]));
        if (res[1] != nullptr)
            PX_CUDA_CHECK(cudaGraphicsUnregisterResource(res[1]));
    }

    void clearStateArray()
    {
        delete [] h;   delete [] u;   delete [] v;
        delete [] h_x; delete [] u_x; delete [] v_x;
        delete [] h_y; delete [] u_y; delete [] v_y;
        delete [] n;

        h = nullptr; u = nullptr; v = nullptr;
        h_x = nullptr; u_x = nullptr; v_x = nullptr;
        h_y = nullptr; u_y = nullptr; v_y = nullptr;
        n = nullptr;
    }

    void clearGLObjs()
    {
        glDeleteVertexArrays(6, vao);
        glDeleteBuffers(7, vbo);

        vao[0] = 0; vao[1] = 0; vao[2] = 0; vao[3] = 0; vao[4] = 0; vao[5] = 0;
        vbo[0] = 0; vbo[1] = 0; vbo[2] = 0; vbo[3] = 0;
        vbo[4] = 0; vbo[5] = 0; vbo[6] = 0;
    }

    void genGLObjs()
    {
        glDeleteVertexArrays(6, vao);
        glDeleteBuffers(7, vbo);

        glGenVertexArrays(6, vao);
        glGenBuffers(7, vbo);
    }

    void drop()
    {
        auto boundary = 5;
        auto row = std::ceil(rnd() * (grid_y-(boundary+boundary))) + boundary;
        auto col = std::ceil(rnd() * (grid_x-(boundary+boundary))) + boundary;

        auto height = rnd()*.5f + 0.1f;
        auto r = rnd() * std::min(grid_x, grid_y)*.15f + 0.001;
        auto rand = (.5f + rnd()*.5f);
        auto gap = 1.f / r;
        for (float i = -1; i < 1; i += gap)
        {
            for (float j = -1; j < 1; j += gap)
            {
                auto tar_y = static_cast<int>(row+i*r);
                auto tar_x = static_cast<int>(col+j*r);
                if (tar_y > boundary && tar_y < grid_y - boundary
                    && tar_x > boundary && tar_x < grid_x - boundary)
                {
                    auto tar = tar_y*grid_x+tar_x;
                    h[tar] += rand * height * std::exp(-5.f*(i*i+j*j));
                }
            }
        }

    }

    void solve(float dt)
    {
        if (request_drop)
        {
            request_drop = false;
            drop();
        }

#pragma omp parallel num_threads(6)
        {
#pragma omp for
            for (auto i = 0; i < grid_y - 1; ++i)
            {
                for (auto j = 0; j < grid_x - 2; ++j)
                {
                    auto tar = i * grid_x + j;
                    auto tar01 = tar + 1;
                    auto tar11 = tar01 + grid_x;

                    h_x[tar] = .5f *
                               ((h[tar11] + h[tar01]) -
                                (u[tar11] - u[tar01]) * dt * inv_gap_x);
                    if (h_x[tar] < height_min)
                        h_x[tar] = height_min;
                    else if (h_x[tar] > height_max || std::isnan(h_x[tar]))
                        h_x[tar] = height_max;

                    u_x[tar] = .5f *
                               (damping*(u[tar11] + u[tar01]) - dt * inv_gap_x *
                                                        (u[tar11] * u[tar11] /
                                                         h[tar11] -
                                                         u[tar01] * u[tar01] /
                                                         h[tar01]
                                                         + half_g * (h[tar11] *
                                                                     h[tar11] -
                                                                     h[tar01] *
                                                                     h[tar01])));
                    v_x[tar] = .5f *
                               (damping*(v[tar11] + v[tar01]) - dt * inv_gap_x *
                                                        (u[tar11] * v[tar11] /
                                                         h[tar11] -
                                                         u[tar01] * v[tar01] /
                                                         h[tar01]));
                }
            }
#pragma omp for
            for (auto i = 0; i < grid_y - 2; ++i)
            {
                for (auto j = 0; j < grid_x - 1; ++j)
                {
                    auto tar = i * grid_x + j;
                    auto tar10 = tar + grid_x;
                    auto tar11 = tar10 + 1;

                    h_y[tar] = .5f *
                               ((h[tar11] + h[tar10]) -
                                (v[tar11] - v[tar10]) * dt * inv_gap_y);
                    if (h_y[tar] < height_min)
                        h_y[tar] = height_min;
                    else if (h_y[tar] > height_max || std::isnan(h_y[tar]))
                        h_y[tar] = height_max;

                    u_y[tar] = .5f *
                               (damping*(u[tar11] + u[tar10]) - dt * inv_gap_y *
                                                        (v[tar11] * u[tar11] /
                                                         h[tar11] -
                                                         v[tar10] * u[tar10] /
                                                         h[tar10]));
                    v_y[tar] = .5f *
                               (damping*(v[tar11] + v[tar10]) - dt * inv_gap_y *
                                                        (v[tar11] * v[tar11] /
                                                         h[tar11] -
                                                         v[tar10] * v[tar10] /
                                                         h[tar10]
                                                         + half_g * (h[tar11] *
                                                                     h[tar11] -
                                                                     h[tar10] *
                                                                     h[tar10])));
                }
            }
#pragma omp for
            for (auto i = 1; i < grid_y - 2; ++i)
            {
                for (auto j = 1; j < grid_x - 2; ++j)
                {
                    auto tar = i * grid_x + j;
                    auto tar0_1 = tar - 1;
                    auto tar_1_1 = tar0_1 - grid_x;
                    auto tar_10 = tar_1_1 + 1;

                    h[tar] -= (dt * inv_gap_x) * (u_x[tar0_1] - u_x[tar_1_1])
                              + (dt * inv_gap_y) * (v_y[tar_10] - v_y[tar_1_1]);

                    if (std::isnan(h[tar]))
                    {
                        h[tar] = 1.f;
                        u[tar] = 0.f;
                        v[tar] = 0.f;
                    }
                    else if (h[tar] < height_min)
                    {
                        h[tar] = height_min;
                        u[tar] = 0.f;
                        v[tar] = 0.f;
                    }
                    else
                    {
                        u[tar] -= (dt * inv_gap_x) *
                                  (u_x[tar0_1] * u_x[tar0_1] / h_x[tar0_1] -
                                   u_x[tar_1_1] * u_x[tar_1_1] / h_x[tar_1_1]
                                   + half_g * (h_x[tar0_1] * h_x[tar0_1] -
                                               h_x[tar_1_1] * h_x[tar_1_1]))
                                  + (dt * inv_gap_y) *
                                    (v_y[tar_10] * u_y[tar_10] / h_y[tar_10] -
                                     v_y[tar_1_1] * u_y[tar_1_1] /
                                     h_y[tar_1_1]);
                        v[tar] -= (dt * inv_gap_x) *
                                  (u_x[tar0_1] * v_x[tar0_1] / h_x[tar0_1] -
                                   u_x[tar_1_1] * v_x[tar_1_1] / h_x[tar_1_1])
                                  + (dt * inv_gap_y) *
                                    (v_y[tar_10] * v_y[tar_10] / h_y[tar_10] -
                                     v_y[tar_1_1] * v_y[tar_1_1] / h_y[tar_1_1]
                                     + half_g * (h_y[tar_10] * h_y[tar_10] -
                                                 h_y[tar_1_1] * h_y[tar_1_1]));


                        if (h[tar] > height_max)
                        {
                            auto scale = height_max / h[tar];
                            h[tar] = height_max;
                            u[tar] *= scale;
                            v[tar] *= scale;
                        }
                    }

                }

            }
        }

        cblas_scopy(grid_y, u+1, grid_x, u, grid_x);
        cblas_scopy(grid_y, v+1, grid_x, v, grid_x);
        cblas_sscal(grid_y, -1.f, v, grid_x);

        cblas_scopy(grid_y, u+grid_x-2, grid_x, u+grid_x-1, grid_x);
        cblas_scopy(grid_y, v+grid_x-2, grid_x, v+grid_x-1, grid_x);
        cblas_sscal(grid_y, -1.f, v+grid_x-1, grid_x);

        cblas_scopy(grid_x, u+grid_x, 1, u, 1);
        cblas_scopy(grid_x, v+grid_x, 1, v, 1);
        cblas_sscal(grid_x, -1.f, u, 1);

        cblas_scopy(grid_x, u+grid_x*(grid_y-2), 1, u+grid_x*(grid_y-1), 1);
        cblas_scopy(grid_x, v+grid_x*(grid_y-2), 1, v+grid_x*(grid_y-1), 1);
        cblas_sscal(grid_x, -1.f, u+grid_x*(grid_y-1), 1);

        cblas_scopy(grid_y, h+1, grid_x, h, grid_x);
        cblas_scopy(grid_y, h+grid_x-2, grid_x, h+grid_x-1, grid_x);
        cblas_scopy(grid_x, h+grid_x, 1, h, 1);
        cblas_scopy(grid_x, h+grid_x*(grid_y-2), 1, h+grid_x*(grid_y-1), 1);
    }

    void updateNorm()
    {
#pragma omp parallel for
        for (auto i = 1; i < grid_y-1; ++i)
        {
            for (auto j = 1; j < grid_x-1; ++j)
            {
                auto tar = i * grid_x + j;
                n[tar] =  glm::cross(glm::vec3(gap_x, h[tar+1]-h[tar], 0.f),
                                    glm::vec3(0.f, h[tar+grid_x]-h[tar], gap_y));
            }
        }
    }
};

scene::ShallowWaterScene::ShallowWaterScene()
        : BaseScene(),
          system_name("Shallow Water Simulation")
{
    pimpl = std::unique_ptr<impl>(new impl);
}

scene::ShallowWaterScene::~ShallowWaterScene()
{
    cudaBufferFree();
}

void scene::ShallowWaterScene::init(Scene &scene)
{
    if (pimpl->water_shader == nullptr)
        pimpl->water_shader = new Shader(pimpl->WATER_VS, pimpl->WATER_FS);
    if (pimpl->bound_shader == nullptr)
        pimpl->bound_shader = new Shader(pimpl->DEPTH_VS, pimpl->DEPTH_FS, pimpl->DEPTH_GS);
    if (pimpl->mesh_shader == nullptr)
        pimpl->mesh_shader = new Shader(pimpl->MESH_VS, pimpl->MESH_FS);

    pimpl->genGLObjs();

    pimpl->water_shader->activate();
    pimpl->water_shader->bind("GlobalAttributes", 0);
    glBindVertexArray(pimpl->vao[0]);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, pimpl->vbo[0]);
    glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[1]);
    glEnableVertexAttribArray(0);   // horizontal position
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2*sizeof(float), (void *)(0));
    glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[2]);
    glEnableVertexAttribArray(1);   // height
    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 1*sizeof(float), (void *)(0));
    glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[3]);
    glEnableVertexAttribArray(2);   // norm
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 3*sizeof(float), (void *)(0));

    pimpl->bound_shader->activate();
    pimpl->bound_shader->bind("GlobalAttributes", 0);
    pimpl->mesh_shader->activate();
    pimpl->mesh_shader->bind("GlobalAttributes", 0);
    glBindVertexArray(pimpl->vao[1]);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, pimpl->vbo[0]);
    glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[1]);
    glEnableVertexAttribArray(0);   // horizontal position
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2*sizeof(float), (void *)(0));
    glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[2]);
    glEnableVertexAttribArray(1);   // height
    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 1*sizeof(float), (void *)(0));

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    pimpl->mesh_shader->activate(false);

    if (pimpl->skybox == nullptr)
        pimpl->skybox = new SkyBox(ASSET_PATH "/texture/skybox/right.jpg",
                                   ASSET_PATH "/texture/skybox/left.jpg",
                                   ASSET_PATH "/texture/skybox/top.jpg",
                                   ASSET_PATH "/texture/skybox/bottom.jpg",
                                   ASSET_PATH "/texture/skybox/back.jpg",
                                   ASSET_PATH "/texture/skybox/front.jpg");
    if (pimpl->floor == nullptr)
        pimpl->floor = new item::Floor;
}

void scene::ShallowWaterScene::restart(Scene &scene)
{
    static auto first_run = true;
    if (first_run)
    {
        resetCamera();
        pimpl->render_cpu = true;
        pimpl->render_mesh = false;
        first_run = false;
    }
    pause = false;
    pimpl->need_upload = true;

    pimpl->step_size = 0.001f;
    pimpl->max_steps = 10;
    pimpl->height_min = .01f;
    pimpl->height_max = 2.5f;

    if (pimpl->render_cpu)
    {
        pimpl->grid_x = 300; pimpl->grid_y = 300;
    }
    else
    {
        pimpl->grid_x = 1000; pimpl->grid_y = 1000;
    }

    pimpl->field_height = 0.f;
    pimpl->half_g = 9.8f;
    pimpl->damping = .5f;
    pimpl->gap_x = .025; pimpl->gap_y = .025;
    pimpl->inv_gap_x = 1.f/pimpl->gap_x; pimpl->inv_gap_y = 1.f/pimpl->gap_y;

    pimpl->request_drop = false;
    pimpl->clearStateArray();

    pimpl->floor->scale(glm::vec3(10.f, 1.f, 10.f));
    pimpl->floor->place(glm::vec3(-5.f, pimpl->field_height, -5.f));
    pimpl->floor->setGrid(5.f, 5.f);
    scene.add(pimpl->floor);

    glm::mat4 model = glm::scale(glm::translate(glm::mat4(), glm::vec3(0.f, -.4f, 0.f)),
                                 glm::vec3(.2f, .5f, .2f));
    pimpl->water_shader->activate();
    pimpl->water_shader->set("model", model);
    pimpl->bound_shader->activate();
    pimpl->bound_shader->set("model", model);
    pimpl->bound_shader->set("field_height", pimpl->field_height);
    pimpl->mesh_shader->activate();
    pimpl->mesh_shader->set("model", model);
    pimpl->mesh_shader->activate(false);
}

void scene::ShallowWaterScene::upload(Shader &scene_shader)
{
    auto dim = pimpl->grid_x * pimpl->grid_y;
    if (pimpl->need_upload)
    {
        pimpl->h = new float[dim];
        auto choice = static_cast<int>(rnd()*4);
        if (choice > 3)
        {
            std::fill_n(pimpl->h, dim, 1.f);
        }
        else if (choice > 2)
        {
            std::fill_n(pimpl->h, dim, 1.f);
            pimpl->request_drop = true;
        }
        else if (choice > 1)
        {
            auto tar = 0; auto len = std::sqrt(pimpl->grid_x*pimpl->grid_x+pimpl->grid_y*pimpl->grid_y);
            for (auto i = 0; i < pimpl->grid_y; ++i)
            {
                for (auto j = 0; j < pimpl->grid_x; ++j)
                {
                    pimpl->h[tar] = .5f + std::sqrt((i == 0 ? 1 : (i == pimpl->grid_y-1 ? (pimpl->grid_y-2)*(pimpl->grid_y-2) : i*i))
                                                   +(j == 0 ? 1 : (j == pimpl->grid_x-1 ? (pimpl->grid_x-2)*(pimpl->grid_x-2) : j*j)))/len;
                    tar += 1;
                }
            }
        }
        else
        {

            auto tar = 0; auto len = std::sqrt(pimpl->grid_x*pimpl->grid_x+pimpl->grid_y*pimpl->grid_y)/2;
            for (auto i = 0; i < pimpl->grid_y; ++i)
            {
                for (auto j = 0; j < pimpl->grid_x; ++j)
                {
                    auto d_y = (i==0?1:(i == pimpl->grid_y-1?pimpl->grid_y-2:i))-pimpl->grid_y*.5f;
                    auto d_x = (j==0?1:(j == pimpl->grid_x-1?pimpl->grid_x-2:j))-pimpl->grid_x*.5f;
                    pimpl->h[tar] = .5f + std::sqrt(d_x*d_x + d_y*d_y)/len;
                    tar += 1;
                }
            }
        }

        std::vector<float> position;
        std::vector<float> left_side; left_side.reserve(pimpl->grid_y*2);
        std::vector<float> right_side; right_side.reserve(pimpl->grid_y*2);
        position.reserve(2*dim);
        auto y = -.5f * pimpl->grid_y * pimpl->gap_y;
        for (auto i = 0; i < pimpl->grid_y; ++i)
        {
            auto x = -.5f * pimpl->grid_x * pimpl->gap_x;
            for (auto j = 0; j < pimpl->grid_x; ++j)
            {
                if (j == 0)
                {
                    left_side.push_back(x);
                    left_side.push_back(y);
                }
                else if (j == pimpl->grid_x-1)
                {
                    right_side.push_back(x);
                    right_side.push_back(y);
                }
                position.push_back(x);
                position.push_back(y);
                x += pimpl->gap_x;
            }
            y += pimpl->gap_y;
        }
        std::vector<unsigned int> triangle;
        for (decltype(pimpl->grid_y) row = 0; row < pimpl->grid_y - 1; ++row)
        {
            if (row % 2 == 0)
            {
                for (decltype(pimpl->grid_x) col = 0; col < pimpl->grid_x; ++col)
                {
                    triangle.push_back(col + row*pimpl->grid_x);
                    triangle.push_back(col + (row+1)*pimpl->grid_x);
                }
            }
            else
            {
                for (decltype(pimpl->grid_x) col = pimpl->grid_x - 1; col > 0; --col)
                {
                    triangle.push_back(col + (row+1)*pimpl->grid_x);
                    triangle.push_back(col - 1 + row*pimpl->grid_x);
                }
            }
        }
        if ((pimpl->grid_y&1) && pimpl->grid_y > 2)
            triangle.push_back((pimpl->grid_y-1)*pimpl->grid_x);
        pimpl->n_triangles = static_cast<decltype(pimpl->n_triangles)>(triangle.size());

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, pimpl->vbo[0]);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(float)*pimpl->n_triangles, triangle.data(), GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[1]);
        glBufferData(GL_ARRAY_BUFFER, sizeof(float)*2*dim, position.data(), GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[2]);
        glBufferData(GL_ARRAY_BUFFER, sizeof(float) * dim, pimpl->h, GL_DYNAMIC_DRAW);

        if (pimpl->render_cpu)
        {
            pimpl->n = new glm::vec3[1+dim];
            pimpl->u = new float[dim];
            pimpl->v = new float[dim];
            pimpl->h_x = new float[dim];
            pimpl->u_x = new float[dim];
            pimpl->v_x = new float[dim];
            pimpl->h_y = new float[dim];
            pimpl->u_y = new float[dim];
            pimpl->v_y = new float[dim];

            auto s = sizeof(float) * dim;
            std::memset(pimpl->u, 0, s);
            std::memset(pimpl->v, 0, s);
            std::memset(pimpl->h_x, 0, s);
            std::memset(pimpl->u_x, 0, s);
            std::memset(pimpl->v_x, 0, s);
            std::memset(pimpl->h_y, 0, s);
            std::memset(pimpl->u_y, 0, s);
            std::memset(pimpl->v_y, 0, s);

            auto idx = 0; auto last = pimpl->grid_x*(pimpl->grid_y-1);
            for (auto j = 0; j < pimpl->grid_x; ++j)
            {
                pimpl->n[idx++] = glm::vec3(0.f, 1.f, 0.f);
                pimpl->n[last + idx] = glm::vec3(0.f, 1.f, 0.f);
            }
            idx = pimpl->grid_x;
            for (auto i = 1; i < pimpl->grid_y; ++i)
            {
                pimpl->n[idx] = glm::vec3(0.f, 1.f, 0.f);
                idx += pimpl->grid_x;
                pimpl->n[idx-1] = glm::vec3(0.f, 1.f, 0.f);
            }
            glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[3]);
            glBufferData(GL_ARRAY_BUFFER, 3*sizeof(float)*dim, pimpl->n, GL_DYNAMIC_DRAW);
        }
        else
        {
            glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[3]);
            glBufferData(GL_ARRAY_BUFFER, 3*sizeof(float)*dim, nullptr, GL_DYNAMIC_DRAW);

            if (pimpl->res[0] != nullptr)
                PX_CUDA_CHECK(cudaGraphicsUnregisterResource(pimpl->res[0]));
            if (pimpl->res[1] != nullptr)
                PX_CUDA_CHECK(cudaGraphicsUnregisterResource(pimpl->res[1]));
            PX_CUDA_CHECK(cudaGraphicsGLRegisterBuffer(&pimpl->res[0], pimpl->vbo[2], cudaGraphicsRegisterFlagsNone));
            PX_CUDA_CHECK(cudaGraphicsGLRegisterBuffer(&pimpl->res[1], pimpl->vbo[3], cudaGraphicsRegisterFlagsNone));
            cuda_param.grid_x = pimpl->grid_x;
            cuda_param.grid_y = pimpl->grid_y;
            cuda_param.gap_x = pimpl->gap_x;
            cuda_param.gap_y = pimpl->gap_y;
            cuda_param.inv_gap_x = pimpl->inv_gap_x;
            cuda_param.inv_gap_y = pimpl->inv_gap_y;
            cuda_param.half_g = pimpl->half_g;
            cuda_param.damping = pimpl->damping;
            cuda_param.height_min = pimpl->height_min;
            cuda_param.height_max = pimpl->height_max;

            cudaInit();
        }
        glBindVertexArray(pimpl->vao[2]);
//        glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[1]);
//        glEnableVertexAttribArray(0);   // horizontal position
//        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(float)*2*pimpl->grid_x, (void *)(0));
//        glEnableVertexAttribArray(1);   // horizontal position
//        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(float)*2*pimpl->grid_x, (void *)(sizeof(float)*2*pimpl->grid_x));
        glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[4]);
        glBufferData(GL_ARRAY_BUFFER, sizeof(float)*2*pimpl->grid_y, left_side.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);   // horizontal position
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, (void *)(0));
        glEnableVertexAttribArray(1);   // horizontal position
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, (void *)(sizeof(float)*2));
        glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[2]);
        glEnableVertexAttribArray(2);   // height
        glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, sizeof(float)*pimpl->grid_x, (void *)(0));
        glEnableVertexAttribArray(3);   // height
        glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, sizeof(float)*pimpl->grid_x, (void *)(sizeof(float)*pimpl->grid_x));

        glBindVertexArray(pimpl->vao[3]);
//        glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[1]);
//        glEnableVertexAttribArray(0);   // horizontal position
//        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2*sizeof(float)*pimpl->grid_x, (void *)(2*sizeof(float)*(pimpl->grid_x-1)));
//        glEnableVertexAttribArray(1);   // horizontal position
//        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 2*sizeof(float)*pimpl->grid_x, (void *)(2*sizeof(float)*(pimpl->grid_x*2-1)));
        glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[5]);
        glBufferData(GL_ARRAY_BUFFER, sizeof(float)*2*pimpl->grid_y, right_side.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);   // horizontal position
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, (void *)(0));
        glEnableVertexAttribArray(1);   // horizontal position
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, (void *)(sizeof(float)*2));
        glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[2]);
        glEnableVertexAttribArray(2);   // height
        glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, sizeof(float)*pimpl->grid_x, (void *)(sizeof(float)*(pimpl->grid_x-1)));
        glEnableVertexAttribArray(3);   // height
        glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, sizeof(float)*pimpl->grid_x, (void *)(sizeof(float)*(pimpl->grid_x*2-1)));

        glBindVertexArray(pimpl->vao[4]);
        glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[1]);
        glEnableVertexAttribArray(0);   // horizontal position
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2*sizeof(float), (void *)(0));
        glEnableVertexAttribArray(1);   // horizontal position
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 2*sizeof(float), (void *)(2*sizeof(float)));
        glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[2]);
        glEnableVertexAttribArray(2);   // height
        glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, sizeof(float), (void *)(0));
        glEnableVertexAttribArray(3);   // height
        glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, sizeof(float), (void *)(sizeof(float)));

        glBindVertexArray(pimpl->vao[5]);
        glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[1]);
        glEnableVertexAttribArray(0);   // horizontal position
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2*sizeof(float), (void *)(2*sizeof(float)*(pimpl->grid_x*(pimpl->grid_y-1))));
        glEnableVertexAttribArray(1);   // horizontal position
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 2*sizeof(float), (void *)(2*sizeof(float)*(pimpl->grid_x*(pimpl->grid_y-1)+1)));
        glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[2]);
        glEnableVertexAttribArray(2);   // height
        glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, sizeof(float), (void *)(sizeof(float)*(pimpl->grid_x*(pimpl->grid_y-1))));
        glEnableVertexAttribArray(3);   // height
        glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, sizeof(float), (void *)(sizeof(float)*(pimpl->grid_x*(pimpl->grid_y-1)+1)));

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);

        pimpl->need_upload = false;
    }
    else if (pimpl->render_cpu)
    {
        glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[2]);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(float)*dim, pimpl->h);
        glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[3]);
        glBufferSubData(GL_ARRAY_BUFFER, 0, 3*sizeof(float)*dim, pimpl->n);
    }
}

void scene::ShallowWaterScene::update(float dt)
{
    if (!pause)
    {
        auto n = std::min(pimpl->max_steps, static_cast<decltype(pimpl->max_steps)>(std::ceil(dt / pimpl->step_size)));

        if (pimpl->render_cpu)
        {
            for (decltype(n) i = 0; i < n; ++i)
            {
                pimpl->solve(pimpl->step_size);
            }
            pimpl->updateNorm();
        }
        else
        {
            void *h_buffer, *n_buffer; size_t buffer_size;
            PX_CUDA_CHECK(cudaGraphicsMapResources(2, pimpl->res, 0));
            PX_CUDA_CHECK(cudaGraphicsResourceGetMappedPointer(&h_buffer, &buffer_size, pimpl->res[0]));
            PX_CUDA_CHECK(cudaGraphicsResourceGetMappedPointer(&n_buffer, &buffer_size, pimpl->res[1]));

            cudaUpdate(h_buffer, n_buffer, pimpl->step_size, n, pimpl->request_drop ? rnd() : 0);
            pimpl->request_drop = false;
            PX_CUDA_CHECK(cudaDeviceSynchronize());
            PX_CUDA_CHECK(cudaGraphicsUnmapResources(2, pimpl->res, 0));
        }
    }

    processInput(dt);
}

void scene::ShallowWaterScene::render()
{
    glm::vec3 norm(1.f, 0.f, 0.f);
    pimpl->bound_shader->activate();
    glEnable(GL_BLEND);
    pimpl->bound_shader->set("norm", norm);
    glBindVertexArray(pimpl->vao[2]);
    glDrawArrays(GL_POINTS, 0, pimpl->grid_y-1);
    norm.x = 0.f; norm.z = 1.f;
    pimpl->bound_shader->set("norm", norm);
    glBindVertexArray(pimpl->vao[4]);
    glDrawArrays(GL_POINTS, 0, pimpl->grid_x-1);
    norm.x = -1.f; norm.z = 0.f;
    pimpl->bound_shader->set("norm", norm);
    glBindVertexArray(pimpl->vao[3]);
    glDrawArrays(GL_POINTS, 0, pimpl->grid_y-1);
    norm.x = 0.f; norm.z = -1.f;
    pimpl->bound_shader->set("norm", norm);
    glBindVertexArray(pimpl->vao[5]);
    glDrawArrays(GL_POINTS, 0, pimpl->grid_x-1);

    glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
    pimpl->water_shader->activate();
    glBindVertexArray(pimpl->vao[0]);
    glDrawElements(GL_TRIANGLE_STRIP, pimpl->n_triangles, GL_UNSIGNED_INT, 0);

    pimpl->mesh_shader->activate();
    if (pimpl->render_mesh)
    {
        glLineWidth(.05f);
        glBindVertexArray(pimpl->vao[1]);
        glDrawElements(GL_LINE_STRIP, pimpl->n_triangles, GL_UNSIGNED_INT, 0);
    }
    pimpl->mesh_shader->activate(false);

    glDisable(GL_BLEND);

    pimpl->skybox->render();
    renderInfo();
}

void scene::ShallowWaterScene::resetCamera()
{
    App::instance()->scene.character.reset(-3.f, 1.5f, 0.f, 90.f, 35.f);
    App::instance()->scene.character.setShootable(false);
    App::instance()->scene.character.setFloating(true);
}

void scene::ShallowWaterScene::renderInfo()
{
    App::instance()->text(system_name,
                          10, 10, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::LeftTop);
    App::instance()->text("Grids: " + std::to_string(pimpl->grid_x) + " x " + std::to_string(pimpl->grid_y),
                          10, 30, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::LeftTop);
    App::instance()->text("Step Size: " + std::to_string(pimpl->step_size) + " x " + std::to_string(pimpl->max_steps),
                          10, 50, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::LeftTop);
    App::instance()->text(std::string("Mode: ") + (pimpl->render_cpu ? "CPU" : "GPU"),
                          10, 70, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::LeftTop);

    auto h = App::instance()->frameHeight() - 25;
    auto w = App::instance()->frameWidth() - 10;
    App::instance()->text("Press K to toggle GPU",
                          w, h, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::RightTop);
    h -= 20;
    App::instance()->text("Press G to drop a drop",
                          w, h, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::RightTop);
    h -= 20;
    App::instance()->text("Press V to toggle mesh",
                          w, h, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::RightTop);
}

void scene::ShallowWaterScene::processInput(float dt)
{
    static auto last_key = GLFW_KEY_UNKNOWN;
    auto window = App::instance()->window();
//    static auto sum_dt = 0.f;
//    static auto key_count = 0;

#define HOLD_KEY(Key)                                           \
    (last_key == Key && sum_dt > 0.01f && key_count == 10)

#define STICKY_KEY_CHECK(Key, Cmd)                              \
    if (glfwGetKey(window, Key) == GLFW_PRESS)                  \
    {                                                           \
        if (last_key != Key || sum_dt > 0.1f || HOLD_KEY(Key))  \
        {                                                       \
            { Cmd }                                             \
            sum_dt = 0; if (key_count < 10) ++key_count;        \
        }                                                       \
        else sum_dt += dt;                                      \
        if (last_key != Key)                                    \
        { last_key = Key; key_count = 0; }                      \
    }

    if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS)
        last_key = GLFW_KEY_P;
    else if (glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS)
        last_key = GLFW_KEY_C;
    else if (glfwGetKey(window, GLFW_KEY_G) == GLFW_PRESS)
        last_key = GLFW_KEY_G;
    else if (glfwGetKey(window, GLFW_KEY_K) == GLFW_PRESS)
        last_key = GLFW_KEY_K;
    else if (glfwGetKey(window, GLFW_KEY_V) == GLFW_PRESS)
        last_key = GLFW_KEY_V;
    else
    {
        if (last_key == GLFW_KEY_K)
        {
            pimpl->render_cpu = !pimpl->render_cpu;
            restart(App::instance()->scene);
        }
        else if (last_key == GLFW_KEY_V)
            pimpl->render_mesh = !pimpl->render_mesh;
        else if (last_key == GLFW_KEY_G)
            pimpl->request_drop = true;
        else if (last_key == GLFW_KEY_C)
            resetCamera();
        else if (last_key == GLFW_KEY_P)
            pause = !pause;
        last_key = GLFW_KEY_UNKNOWN;
    }
}
