#include <sstream>
#include <iomanip>
#include <glm/gtx/norm.hpp>
#include <cblas.h>
#include <cstring>

#include "scene/string_scene.hpp"
#include "item/floor.hpp"
#include "shader/skybox.hpp"
#include "app.hpp"
#include "global.hpp"
#include "util/random.hpp"
#include "util/cuda.hpp"

#include "stb_image.hpp"
#include <cuda_gl_interop.h>

using namespace px;

class scene::StringScene::impl
{
public:
    bool render_grid, render_texture, render_cpu, move_sphere;
    bool need_upload;
    unsigned int vao[3], vbo[5], texture[1];
    Shader *node_shader, *string_shader, *cloth_shader;
    SkyBox *skybox;
    item::Floor *floor; item::Sphere *sphere;
    struct cudaGraphicsResource *res;

    unsigned int n_vertices;
    unsigned int n_particles;
    unsigned int n_springs;
    float step_size, last_step_size;
    int max_steps;
    std::vector<glm::vec3> init_x, init_v, a_dx, a_dv, b_dx, b_dv, c_dx, c_dv, d_dx, d_dv;

    std::vector<float> uv_index;
    std::vector<unsigned int> triangle;

    std::vector<std::pair<unsigned int, unsigned int> > link;
    std::vector<glm::vec3> position;
    std::vector<glm::vec3> velocity;
    std::vector<glm::vec3> acceleration;
    std::vector<float> mass;

    float ks, ks_shear, ks_bend; // stretching coefficient
    float kd, kd_shear, kd_bend; // damping coefficient
    float rest_len, rest_len_shear, rest_len_bend;
    float air_friction, field_height, ground_friction;
    glm::vec3 field_norm;
    unsigned int grid_x, grid_y;
    glm::vec3 gravity;
    glm::vec3 wind;
    float cloth_thickness;

    const char *VS = "#version 420 core\n"
            "layout(location = 5) in vec2 vertex;"
            "layout(location = 0) in vec3 position;"
            ""
            "layout (std140, binding = 0) uniform GlobalAttributes"
            "{"
            "   mat4 view;"
            "   mat4 proj;"
            "   vec3 cam_pos;"
            "};"
            ""
            "void main()"
            "{"
            "   vec3 right_vec = vec3(view[0][0], view[1][0], view[2][0]);"
            "   vec3 up_vec = vec3(view[0][1], view[1][1], view[2][1]);"
            "   gl_Position = proj * view "
            "           * vec4(position + (right_vec*vertex.x - up_vec*vertex.y)*.01f,"
            "                  1.f);"
            "}";
    const char *FS = "#version 330 core\n"
            ""
            "out vec4 gColor;"
            ""
            "void main(){"
            "   gColor = vec4(1.f, 1.f, 1.f, 1.f);"
            "}";

    const char *STRING_VS = "#version 420 core\n"
            "layout(location = 0) in vec3 position;"
            ""
            "layout (std140, binding = 0) uniform GlobalAttributes"
            "{"
            "   mat4 view;"
            "   mat4 proj;"
            "   vec3 cam_pos;"
            "};"
            ""
            "void main()"
            "{"
            "   gl_Position = proj * view * vec4(position, 1.f);"
            "}";
    const char *STRING_FS = "#version 330 core\n"
            ""
            "out vec4 gColor;"
            ""
            "void main(){"
            "   gColor = vec4(1.f, 1.f, 1.f, 1.f);"
            "}";
    const char *CLOTH_VS = "#version 420 core\n"
            "layout (location = 0) in vec3 pos;"
            "layout (location = 1) in vec2 tex;"
            ""
            "layout (std140, binding = 0) uniform GlobalAttributes"
            "{"
            "   mat4 view;"
            "   mat4 proj;"
            "   vec3 cam_pos;"
            "};"
            "out vec2 gTexCoord;"
            ""
            "void main(){"
            "   gl_Position = proj * view * vec4(pos, 1.f);"
            "   gTexCoord = tex;"
            "}";
    const char *CLOTH_FS = "#version 330 core\n"
            "in vec2 gTexCoord;"
            ""
            "uniform sampler2D sprite;"
            ""
            "out vec4 gColor;"
            ""
            "void main(){"
            "   gColor = vec4(texture(sprite, gTexCoord).rgb, 1.f);"
            "}";

    impl() : vao{0}, vbo{0}, texture{0},
             node_shader(nullptr), string_shader(nullptr), cloth_shader(nullptr), skybox(nullptr),
             floor(nullptr), sphere(nullptr), res(nullptr)
    {}
    ~impl()
    {
        clearGLObjs();

        delete node_shader;
        delete string_shader;
        delete skybox;
        delete floor;
        delete sphere;
    }

    void clearGLObjs()
    {
        glDeleteVertexArrays(3, vao);
        glDeleteBuffers(5, vbo);
        glDeleteTextures(1, texture);

        vao[0] = 0; vao[1] = 0; vao[2] = 0;
        vbo[0] = 0; vbo[1] = 0; vbo[2] = 0; vbo[3] = 0; vbo[4] = 0;
        texture[0] = 0;
    }

    void genGLObjs()
    {
        glDeleteVertexArrays(2, vao);
        glDeleteBuffers(5, vbo);
        glDeleteBuffers(1, texture);

        glGenVertexArrays(3, vao);
        glGenBuffers(5, vbo);
        glGenTextures(1, texture);
    }

    glm::vec3 force(unsigned int id1, unsigned int id2, float kd, float ks, float rest_length)
    {
//        if (mass[id1] == std::numeric_limits<decltype(particles[id1].mass)>::max())
//            return glm::vec3(0.f);
        auto dx = position[id1] - position[id2];
        auto x = glm::length(dx);
        if (x > std::numeric_limits<float>::min())
        {
            auto dv = velocity[id1] - velocity[id2];
            auto F = dx/x;
            F *= - ks * (x - rest_length)
                 - kd * glm::dot(dv, F);
            return F;
        }
        return glm::vec3(0.f);
    }

    glm::vec3 windForce(unsigned int id1, unsigned int id2, unsigned int id3)
    {
        auto v = (velocity[id1] + velocity[id2] + velocity[id3]) / 3.f;
        v -= wind;
        auto v_len = glm::length(v);

        auto norm = glm::cross(position[id2] - position[id1],
                               position[id3] - position[id1]);

        return -0.001f * v_len * glm::dot(v, norm) / (2 * glm::length(norm)) * norm;
    }

    void applyForce(unsigned int id)
    {
        if (mass[id] == std::numeric_limits<float>::max())
            return;

        auto row = id / grid_x;
        auto col = id % grid_x;
        glm::vec3 f(0.f);
        if (row < grid_y-1)
        {
            f += force(id, id + grid_x, kd, ks, rest_len); // down
            if (row < grid_y - 2)
                f += force(id, id + grid_x + grid_x, kd, ks,
                           rest_len_bend); // down 2
            if (col > 0)
            {
                f += force(id, id + grid_x - 1, kd, ks, rest_len_shear); // bottom left
                f += windForce(id, id - 1, id + grid_x );
            }
            if (col < grid_x - 1)
            {
                f += force(id, id + grid_x + 1, kd, ks, rest_len_shear); // bottom right
                f += windForce(id, id + grid_x, id + 1);
            }
        }
        if (row > 0)
        {
            f += force(id, id - grid_x, kd, ks, rest_len); // top
            if (row > 1)
                f += force(id, id - grid_x - grid_x, kd, ks, rest_len_bend); // top 2
            if (col > 0)
            {
                f += force(id, id - grid_x - 1, kd, ks, rest_len_shear); // top left
                f += windForce(id, id - grid_x, id - 1);
            }
            if (col < grid_x - 1)
            {
                f += force(id, id - grid_x + 1, kd, ks, rest_len_shear); // top right
                f += windForce(id, id + 1, id - grid_x);
            }
        }
        if (col > 0)
        {
            f += force(id, id - 1, kd, ks, rest_len); // left
            if (col > 1)
                f += force(id, id - 2, kd, ks, rest_len_bend); // left 2
        }
        if (col < grid_x - 1)
        {
            f += force(id, id + 1, kd, ks, rest_len); // right
            if (col < grid_x - 2)
                f += force(id, id + 2, kd, ks, rest_len_bend); // right 2
        }
        acceleration[id] = gravity + f / mass[id]
            - velocity[id] * (position[id].y == field_height ? ground_friction : air_friction);
    }

    void collision()
    {
        glm::vec3 sphere_force(0.f);
        for (decltype(n_particles) idx = 0; idx < n_particles; ++idx)
        {
            if (mass[idx] == std::numeric_limits<float>::max()) continue;

            auto x = position[idx] - sphere->pos();
            auto dx = sphere->scal().x + cloth_thickness - glm::length(x);
            if (dx > 0)
            {
                auto norm = glm::normalize(x);
                auto acc = velocity[idx];
                velocity[idx] = 0.9f * glm::reflect(velocity[idx], norm);
                acc = velocity[idx] - acc;
                position[idx] = sphere->pos() + (sphere->scal().x + cloth_thickness + .9f*dx) * norm;
                sphere_force += mass[idx] * acc / step_size;
            }
            if (position[idx].y < field_height + cloth_thickness)
            {
                velocity[idx].y = 0.f;
                position[idx].y = field_height + cloth_thickness;
            }
        }
        sphere->acceleration(sphere_force / sphere->mass());
    }

    void sphereConstraint(float dt)
    {
        auto pos = sphere->pos() + sphere->velocity() * dt;
        if (pos.y <= field_height + sphere->scal().y)
        {
            pos.y = field_height + sphere->scal().y;
            sphere->velocity().y = 0.f;
            sphere->velocity() *= 1 - ground_friction;
        }
        if (pos.z > 2.5f)
        {
            pos.z = 2.5f;
            sphere->velocity() = glm::reflect(sphere->velocity(),
                                              glm::vec3(0.f, 0.f, -1.f));
        }
        else if (pos.z < -2.5f)
        {
            pos.z = -2.5f;
            sphere->velocity() = glm::reflect(sphere->velocity(),
                                              glm::vec3(0.f, 0.f, 1.f));
        }
        sphere->place(pos);
    }

    void euler()
    {
        if (move_sphere)
        {
            sphereConstraint(last_step_size);
            sphere->velocity() -= sphere->acceleration() * last_step_size;
            sphere->velocity() += gravity * last_step_size;
        }
        for (decltype(n_particles) idx = 0; idx < n_particles; ++idx)
            applyForce(idx);

        cblas_saxpy(n_particles*3,
                    step_size, reinterpret_cast<float *>(velocity.data()), 1,
                    reinterpret_cast<float *>(position.data()), 1);
        cblas_saxpy(n_particles*3,
                    step_size, reinterpret_cast<float *>(acceleration.data()), 1,
                    reinterpret_cast<float *>(velocity.data()), 1);
        collision();
    }

    void semiImplicitEuler()
    {
        if (move_sphere)
        {
            sphere->velocity() -= sphere->acceleration() * last_step_size;
            sphere->velocity() += gravity * last_step_size;
            sphereConstraint(last_step_size);
        }
        for (decltype(n_particles) idx = 0; idx < n_particles; ++idx)
            applyForce(idx);


        cblas_saxpy(n_particles*3,
                    step_size, reinterpret_cast<float *>(acceleration.data()), 1,
                    reinterpret_cast<float *>(velocity.data()), 1);
        cblas_saxpy(n_particles*3,
                    step_size, reinterpret_cast<float *>(velocity.data()), 1,
                    reinterpret_cast<float *>(position.data()), 1);
        collision();

    }

    void verlet()
    {
        constexpr auto damping = 1.f;
        glm::vec3 sphere_acc(0.f);

        if (move_sphere)
        {
            auto last_pos = sphere->pos();
            sphereConstraint(last_step_size);
            sphere->velocity((sphere->pos() - last_pos) / last_step_size);
        }
        for (decltype(n_particles) idx = 0; idx < n_particles; ++idx)
            applyForce(idx);

        for (decltype(n_particles) idx = 0; idx < n_particles; ++idx)
        {
            if (mass[idx] == std::numeric_limits<float>::max()) continue;

//            auto temp = position[idx];
//            auto last_pos = position[idx] - velocity[idx] * last_step_size;
//            position[idx] += (position[idx] - last_pos) * damping + acceleration[idx] * step_size * step_size;
//            last_pos = temp;
            auto last_pos = position[idx];
            position[idx] += velocity[idx] * last_step_size * damping + acceleration[idx] * step_size * step_size;

            // collision
            auto x = position[idx] - sphere->pos();
            auto dx = sphere->scal().x + cloth_thickness - glm::length(x);
            if (dx > 0)
            {
                auto norm = glm::normalize(x);
                position[idx] = sphere->pos() + (sphere->scal().x + cloth_thickness + .9f*dx) * norm;
                sphere_acc += mass[idx] * .9f*dx / step_size / step_size;
            }
            if (position[idx].y < field_height + cloth_thickness)
            {
                position[idx].y = field_height + cloth_thickness;
            }
            velocity[idx] = (position[idx] - last_pos) / step_size;
        }
        if (move_sphere)
        {
            sphere_acc /= sphere->mass();
            sphere_acc += gravity;
            sphere->velocity() = sphere->velocity() * last_step_size/step_size * damping +
                                 sphere_acc * step_size;
        }
    }

    void velocityVerlet()
    {
        glm::vec3 sphere_force(0.f);
        if (move_sphere)
        {
            sphere->velocity() -= sphere->acceleration() * (0.5f * last_step_size);
            sphere->velocity() += gravity * (0.5f * last_step_size);
            sphereConstraint(last_step_size);
        }
        for (decltype(n_particles) idx = 0; idx < n_particles; ++idx)
            applyForce(idx);

        cblas_saxpy(n_particles*3,
                    .5f*last_step_size, reinterpret_cast<float *>(acceleration.data()), 1,
                    reinterpret_cast<float *>(velocity.data()), 1);
        if (move_sphere)
        {
            sphere->acceleration() += gravity;
            sphere->velocity() += sphere->acceleration() * (.5f * last_step_size);
        }

        cblas_saxpy(n_particles*3,
                    .5f*step_size, reinterpret_cast<float *>(acceleration.data()), 1,
                    reinterpret_cast<float *>(velocity.data()), 1);
        cblas_saxpy(n_particles*3,
                    step_size, reinterpret_cast<float *>(velocity.data()), 1,
                    reinterpret_cast<float *>(position.data()), 1);
        collision();
    }

    void midpoint()
    {
        std::memcpy(init_x.data(), position.data(), sizeof(glm::vec3)*n_particles);
        std::memcpy(init_v.data(), velocity.data(), sizeof(glm::vec3)*n_particles);
        auto sphere_init_x = sphere->pos();
        auto sphere_init_v = sphere->velocity();
        glm::vec3 sphere_target_x, sphere_target_v;

        if (move_sphere)
        {
            sphere->velocity() -= sphere->acceleration() * last_step_size;
            sphere->velocity() += gravity * last_step_size;
            sphereConstraint(last_step_size);
            sphere_target_x = sphere->pos();
            sphere_target_v = sphere->velocity();
        }
        for (decltype(n_particles) idx = 0; idx < n_particles; ++idx)
            applyForce(idx);

        cblas_saxpy(n_particles*3,
                    0.5f*step_size, reinterpret_cast<const float *>(acceleration.data()), 1,
                    reinterpret_cast<float *>(velocity.data()), 1);
        cblas_saxpy(n_particles*3,
                    0.5f*step_size, reinterpret_cast<const float *>(velocity.data()), 1,
                    reinterpret_cast<float *>(position.data()), 1);
        collision();
        if (move_sphere)
        {
            sphere->velocity() -= sphere->acceleration() * (0.5f * step_size);
            sphere->velocity() += gravity * (0.5f * step_size);
            sphereConstraint((0.5f * step_size));
        }
        for (decltype(n_particles) idx = 0; idx < n_particles; ++idx)
            applyForce(idx);

        std::memcpy(position.data(), init_x.data(), sizeof(glm::vec3)*n_particles);
        std::memcpy(velocity.data(), init_v.data(), sizeof(glm::vec3)*n_particles);
        sphere->place(sphere_init_x);
        sphere->velocity(sphere_init_v);

        cblas_saxpy(n_particles*3,
                    step_size, reinterpret_cast<const float *>(acceleration.data()), 1,
                    reinterpret_cast<float *>(velocity.data()), 1);
        cblas_saxpy(n_particles*3,
                    step_size, reinterpret_cast<const float *>(velocity.data()), 1,
                    reinterpret_cast<float *>(position.data()), 1);
        collision();
        if (move_sphere)
        {
            sphere->place(sphere_target_x);
            sphere->velocity(sphere_target_v);
        }

    }

    void rk4()
    {
        constexpr auto one_over_six = 1.f/6.f;

        static auto init_evaluate = [&](std::vector<glm::vec3> &init_x,
                                        std::vector<glm::vec3> &init_v,
                                        std::vector<glm::vec3> &out_dx,
                                        std::vector<glm::vec3> &out_dv)
        {
            std::memcpy(init_x.data(), position.data(), sizeof(glm::vec3)*n_particles);
            std::memcpy(init_v.data(), velocity.data(), sizeof(glm::vec3)*n_particles);

            for (decltype(n_particles) idx = 0; idx < n_particles; ++idx)
                applyForce(idx);

            std::memcpy(out_dx.data(), velocity.data(), sizeof(glm::vec3)*n_particles);
            std::memcpy(out_dv.data(), acceleration.data(), sizeof(glm::vec3)*n_particles);
        };

        static auto evaluate = [&](float dt,
                                   std::vector<glm::vec3> const &init_x,
                                   std::vector<glm::vec3> const &init_v,
                                   std::vector<glm::vec3> &out_dx,
                                   std::vector<glm::vec3> &out_dv,
                                   std::vector<glm::vec3> const &ref_dx,
                                   std::vector<glm::vec3> const &ref_dv)
        {
            std::memcpy(position.data(), init_x.data(), sizeof(glm::vec3)*n_particles);
            std::memcpy(velocity.data(), init_v.data(), sizeof(glm::vec3)*n_particles);

            cblas_saxpy(n_particles*3,
                        dt, reinterpret_cast<const float *>(ref_dx.data()), 1,
                        reinterpret_cast<float *>(position.data()), 1);
            cblas_saxpy(n_particles*3,
                        dt, reinterpret_cast<const float *>(ref_dv.data()), 1,
                        reinterpret_cast<float *>(velocity.data()), 1);

            for (decltype(n_particles) idx = 0; idx < n_particles; ++idx)
                applyForce(idx);

            std::memcpy(out_dx.data(), velocity.data(), sizeof(glm::vec3)*n_particles);
            std::memcpy(out_dv.data(), acceleration.data(), sizeof(glm::vec3)*n_particles);
        };

        if (move_sphere)
        {
            sphere->velocity() -= sphere->acceleration() * last_step_size;
            sphere->velocity() += gravity * last_step_size;
            sphereConstraint(last_step_size);
        }

        init_evaluate(           init_x, init_v, a_dx, a_dv);
        evaluate(step_size*0.5f, init_x, init_v, b_dx, b_dv, a_dx, a_dv);
        evaluate(step_size*0.5f, init_x, init_v, c_dx, c_dv, b_dx, b_dv);
        evaluate(step_size,      init_x, init_v, d_dx, d_dv, c_dx, c_dv);

        for (decltype(n_particles) idx = 0; idx < n_particles; ++idx)
        {
            if (mass[idx] == std::numeric_limits<float>::max()) continue;

            auto dxdt = one_over_six *
                        (a_dx[idx] + 2.f * (b_dx[idx] + c_dx[idx]) + d_dx[idx]);
            auto dvdt = one_over_six *
                        (a_dv[idx] + 2.f * (b_dv[idx] + c_dv[idx]) + d_dv[idx]);

            position[idx] = init_x[idx] + dxdt * step_size;
            velocity[idx] = init_v[idx] + dvdt * step_size;
        }
        collision();
    }
};

scene::StringScene::StringScene()
        : BaseScene(),
          system_name("Cloth Simulation"),
          integrator(Integrator::RK4), scenery(Scenery::Dangling)
{
    pimpl = std::unique_ptr<impl>(new impl);
    pimpl->render_cpu = true;
}

scene::StringScene::~StringScene()
{
    cudaBufferFree();
}

void scene::StringScene::init(Scene &scene)
{
    if (pimpl->node_shader == nullptr)
        pimpl->node_shader = new Shader(pimpl->VS, pimpl->FS);
    if (pimpl->string_shader == nullptr)
        pimpl->string_shader = new Shader(pimpl->STRING_VS, pimpl->STRING_FS);
    if (pimpl->cloth_shader == nullptr)
        pimpl->cloth_shader = new Shader(pimpl->CLOTH_VS, pimpl->CLOTH_FS);

    pimpl->genGLObjs();

    std::vector<float> vertex;
    constexpr auto grid = 8;
    constexpr auto gap = static_cast<float>(M_PI / grid);
    const auto c = std::cos(gap);
    const auto s = std::sin(gap);
    vertex.reserve(grid * 4);
    vertex.push_back(1.f);
    vertex.push_back(0.f);
    auto x = 1.f;
    auto y = 0.f;
    for (auto i = 1; i < grid; ++i) {
        float tmp_x = c * x - s * y;
        y = s * x + c * y;
        x = tmp_x;

        vertex.push_back(x);
        vertex.push_back(y);

        vertex.push_back(x);
        vertex.push_back(-y);
    }

    vertex.push_back(-1.f);
    vertex.push_back(0.f);

    pimpl->node_shader->activate();
    pimpl->node_shader->bind("GlobalAttributes", 0);
    glBindVertexArray(pimpl->vao[0]);
    glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[0]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float)*vertex.size(), vertex.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(5, 2, GL_FLOAT, GL_FALSE, 0, (void *)0);
    glEnableVertexAttribArray(5);
    glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[1]);
    glEnableVertexAttribArray(0);   // position
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3*sizeof(float), (void *)(0));

    pimpl->string_shader->activate();
    pimpl->string_shader->bind("GlobalAttributes", 0);
    glBindVertexArray(pimpl->vao[1]);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, pimpl->vbo[2]);
    glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[1]);
    glEnableVertexAttribArray(0);   // position
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3*sizeof(float), (void *)(0));

    pimpl->cloth_shader->activate();
    pimpl->cloth_shader->bind("GlobalAttributes", 0);
    glBindVertexArray(pimpl->vao[2]);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, pimpl->vbo[4]);
    glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[1]);
    glEnableVertexAttribArray(0);   // position
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3*sizeof(float), (void *)(0));
    glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[3]);
    glEnableVertexAttribArray(1);   // uv
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 2*sizeof(float), (void *)(0));
    int w, h, ch;
    auto ptr = stbi_load(ASSET_PATH "/texture/umn_logo.png", &w, &h, &ch, 3);
    TEXTURE_LOAD_HELPER(pimpl->texture[0], GL_RGB, GL_REPEAT, GL_LINEAR, w, h, ptr);
    stbi_image_free(ptr);

    pimpl->cloth_shader->activate(false);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    pimpl->n_vertices = vertex.size()/2;

    if (pimpl->skybox == nullptr)
        pimpl->skybox = new SkyBox(ASSET_PATH "/texture/skybox/right.jpg",
                                   ASSET_PATH "/texture/skybox/left.jpg",
                                   ASSET_PATH "/texture/skybox/top.jpg",
                                   ASSET_PATH "/texture/skybox/bottom.jpg",
                                   ASSET_PATH "/texture/skybox/back.jpg",
                                   ASSET_PATH "/texture/skybox/front.jpg");
    if (pimpl->floor == nullptr)
        pimpl->floor = new item::Floor;
    if (pimpl->sphere == nullptr)
    {
        pimpl->sphere = new item::Sphere;
        pimpl->sphere->init();
    }
}

void scene::StringScene::restart(Scene &scene)
{
    static auto first_run = true;
    if (first_run)
    {
        resetCamera();
        first_run = false;
        pimpl->render_grid = true;
        pimpl->render_texture = true;
    }
    pause = false;
    pimpl->need_upload = true;

    pimpl->gravity = glm::vec3(0.f, -9.81f, 0.f);
    pimpl->air_friction = .0025f;
    float mass;
    if (pimpl->render_cpu)
    {
        pimpl->kd = .01f;
        pimpl->kd_shear = .005f;
        pimpl->kd_bend = .005f;  // inner friction
        pimpl->ks = 1.f;
        pimpl->ks_shear = 1.f;
        pimpl->ks_bend = 1.f;  // tot_mass * gravity / desired_stretch;
        pimpl->rest_len = .1f;
        pimpl->grid_x = 50; pimpl->grid_y = 50;
        mass = .1f / (pimpl->grid_x * pimpl->grid_y);
    }
    else
    {
        pimpl->kd = .01f;
        pimpl->kd_shear = .01f;
        pimpl->kd_bend = .01f;  // inner friction
        pimpl->ks = 2.75f;
        pimpl->ks_shear = 2.75f;
        pimpl->ks_bend = 2.75f;  // tot_mass * gravity / desired_stretch;
        pimpl->rest_len = .025f;
        pimpl->grid_x = 200; pimpl->grid_y = 200;
        mass = .55f / (pimpl->grid_x * pimpl->grid_y);
    }
    pimpl->cloth_thickness = 0.01f;
    pimpl->ground_friction = 0.0001f;
    pimpl->field_height = 0.f;
    pimpl->field_norm = glm::vec3(0.f, 1.f, 0.f);
    pimpl->rest_len_shear = pimpl->rest_len*std::sqrt(2.f);
    pimpl->rest_len_bend  = pimpl->rest_len*2;

    pimpl->position.clear();
    pimpl->mass.clear();
    pimpl->link.clear();
    pimpl->uv_index.clear();
    auto gap_u = 1.f/pimpl->grid_x;
    auto gap_v = 1.f/pimpl->grid_y;

    if (scenery == Scenery::Floating || scenery == Scenery::Flag)
        pimpl->move_sphere = false;
    else
        pimpl->move_sphere = true;
    auto z = scenery == Scenery::Falling || scenery == Scenery::Floating ? 0.5f*pimpl->rest_len*pimpl->grid_x : 0.f;
    auto y = scenery == Scenery::Falling || scenery == Scenery::Floating ? 2.f : (pimpl->grid_y-3)*pimpl->rest_len;
    for (decltype(pimpl->grid_y) i = 0; i < pimpl->grid_y; ++i)
    {
        auto x = -0.5f*pimpl->rest_len*pimpl->grid_x;
        for (decltype(pimpl->grid_x) j = 0; j < pimpl->grid_x; ++j)
        {
            pimpl->position.push_back(glm::vec3(x, y, z));

            if (scenery == Scenery::Falling || scenery == Scenery::Floating)
            {
                pimpl->mass.push_back(mass);
            }
            else if (scenery == Scenery::Flag)
            {
                pimpl->mass.push_back(j == 0  ?
                                      std::numeric_limits<float>::max() : mass);
            }
            else
            {
                pimpl->mass.push_back(i == 0 && (j < 2 || j > pimpl->grid_x-3 || j == pimpl->grid_x/2 || j == pimpl->grid_x/2+1) ?
                                      std::numeric_limits<float>::max() : mass);
            }

            pimpl->uv_index.push_back(j*gap_u);
            pimpl->uv_index.push_back(i*gap_v);

            x += pimpl->rest_len;
        }
        if (scenery != Scenery::Flag)
        z -= pimpl->rest_len;
        if (scenery == Scenery::Dangling || scenery == Scenery::Flag)
            y -= pimpl->rest_len*.75f;
    }
    for (decltype(pimpl->grid_y) i = 0; i < pimpl->grid_y; ++i)
    {
        for (decltype(pimpl->grid_x) j = 0; j < pimpl->grid_x; ++j)
        {
            auto idx = i*pimpl->grid_x+j;
            if (i < pimpl->grid_y - 1) // link to bottom
            {
                pimpl->link.emplace_back(idx, idx+pimpl->grid_x);
            }
            if (j < pimpl->grid_x - 1) // link to right
            {
                pimpl->link.emplace_back(idx, idx+1);
            }
            if (i < pimpl->grid_y - 1 && j < pimpl->grid_x - 1) // link to bottom right
            {
                pimpl->link.emplace_back(idx, idx+pimpl->grid_x+1);
            }
            if (i < pimpl->grid_y - 1 && j > 0) // link to bottom left
            {
                pimpl->link.emplace_back(idx, idx+pimpl->grid_x-1);
            }

            if (i < pimpl->grid_y - 2) // link to bottom 2
            {
                pimpl->link.emplace_back(idx, idx+pimpl->grid_x+pimpl->grid_x);
            }
            if (j < pimpl->grid_x - 2) // link to right 2
            {
                pimpl->link.emplace_back(idx, idx+2);
            }
        }
    }
    pimpl->step_size = std::sqrt(mass/pimpl->ks)*M_PI*0.02f;//0.0001f;
    pimpl->max_steps = 10;

    pimpl->n_particles = static_cast<unsigned int>(pimpl->position.size());
    pimpl->n_springs = static_cast<unsigned int>(pimpl->link.size());
    if (pimpl->render_cpu)
    {
        pimpl->velocity.clear();
        pimpl->velocity.resize(pimpl->n_particles, glm::vec3(0.f));
        pimpl->acceleration.resize(pimpl->n_particles);
        pimpl->init_x.resize(pimpl->n_particles); pimpl->init_v.resize(pimpl->n_particles);
        pimpl->a_dx.resize(pimpl->n_particles); pimpl->a_dv.resize(pimpl->n_particles);
        pimpl->b_dx.resize(pimpl->n_particles); pimpl->b_dv.resize(pimpl->n_particles);
        pimpl->c_dx.resize(pimpl->n_particles); pimpl->c_dv.resize(pimpl->n_particles);
        pimpl->d_dx.resize(pimpl->n_particles); pimpl->d_dv.resize(pimpl->n_particles);
        for (decltype(pimpl->n_particles) idx = 0; idx < pimpl->n_particles; ++idx)
        {
            if (pimpl->mass[idx] == std::numeric_limits<float>::max())
            {
                pimpl->acceleration[idx] = glm::vec3(0.f);
            }
            else
                pimpl->applyForce(idx);
        }
    }

    pimpl->floor->scale(glm::vec3(10.f, 1.f, 10.f));
    pimpl->floor->place(glm::vec3(-5.f, pimpl->field_height, -5.f));
    pimpl->floor->setGrid(5.f, 5.f);
    scene.add(pimpl->floor);

    resetBall();

    pimpl->triangle.clear();
    for (decltype(pimpl->grid_y) row = 0; row < pimpl->grid_y - 1; ++row)
    {
        if (row % 2 == 0)
        {
            for (decltype(pimpl->grid_x) col = 0; col < pimpl->grid_x; ++col)
            {
                pimpl->triangle.push_back(col + row*pimpl->grid_x);
                pimpl->triangle.push_back(col + (row+1)*pimpl->grid_x);
            }
        }
        else
        {
            for (decltype(pimpl->grid_x) col = pimpl->grid_x - 1; col > 0; --col)
            {
                pimpl->triangle.push_back(col + (row+1)*pimpl->grid_x);
                pimpl->triangle.push_back(col - 1 + row*pimpl->grid_x);
            }
        }
    }
    if ((pimpl->grid_y&1) && pimpl->grid_y > 2)
        pimpl->triangle.push_back((pimpl->grid_y-1)*pimpl->grid_x);
}

void scene::StringScene::upload(Shader &scene_shader)
{
    if (pimpl->need_upload)
    {
        glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[1]);
        glBufferData(GL_ARRAY_BUFFER, sizeof(float)*3*pimpl->n_particles, pimpl->position.data(), GL_DYNAMIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, pimpl->vbo[2]);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(float)*2*pimpl->n_springs, pimpl->link.data(), GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[3]);
        glBufferData(GL_ARRAY_BUFFER, sizeof(float)*2*pimpl->n_particles, pimpl->uv_index.data(), GL_STATIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, pimpl->vbo[4]);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(float)*pimpl->triangle.size(), pimpl->triangle.data(), GL_STATIC_DRAW);

        if (!pimpl->render_cpu)
        {
            if (pimpl->res != nullptr)
                PX_CUDA_CHECK(cudaGraphicsUnregisterResource(pimpl->res));
            PX_CUDA_CHECK(cudaGraphicsGLRegisterBuffer(&pimpl->res, pimpl->vbo[1], cudaGraphicsRegisterFlagsNone));
            cuda_param.ks = pimpl->ks;
            cuda_param.ks_shear = pimpl->ks_shear;
            cuda_param.ks_bend = pimpl->ks_bend;
            cuda_param.kd = pimpl->kd;
            cuda_param.kd_shear = pimpl->kd_shear;
            cuda_param.kd_bend = pimpl->kd_bend;
            cuda_param.rest_len = pimpl->rest_len;
            cuda_param.rest_len_shear = pimpl->rest_len_shear;
            cuda_param.rest_len_bend = pimpl->rest_len_bend;
            cuda_param.air_friction = pimpl->air_friction;
            cuda_param.ground_friction = pimpl->ground_friction;
            cuda_param.field_height = pimpl->field_height;
            cuda_param.cloth_thickness = pimpl->cloth_thickness;
            cuda_param.gravity.x = pimpl->gravity.x;
            cuda_param.gravity.y = pimpl->gravity.y;
            cuda_param.gravity.z = pimpl->gravity.z;
            cuda_param.wind.x = pimpl->wind.x;
            cuda_param.wind.y = pimpl->wind.y;
            cuda_param.wind.z = pimpl->wind.z;
            cuda_param.field_norm.x = pimpl->field_norm.x;
            cuda_param.field_norm.y = pimpl->field_norm.y;
            cuda_param.field_norm.z = pimpl->field_norm.z;
            cuda_param.grid_x = pimpl->grid_x;
            cuda_param.grid_y = pimpl->grid_y;
            cudaInit(pimpl->n_particles, pimpl->mass.data());
        }

        pimpl->need_upload = false;
    }
    else if (pimpl->render_cpu)
    {
        glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[1]);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(float)*3*pimpl->n_particles, pimpl->position.data());
    }
}

void scene::StringScene::update(float dt)
{
    if (!pause)
    {
        auto n = std::min(pimpl->max_steps, static_cast<int>(std::ceil(dt / pimpl->step_size)));

        if (pimpl->render_cpu)
        {
            if (integrator == Integrator::Euler)
                for (auto i = 0; i < n; ++i)
                {
                    pimpl->euler();
                }
            else if (integrator == Integrator::SemiImplicitEuler)
                for (auto i = 0; i < n; ++i)
                {
                    pimpl->semiImplicitEuler();
                }
            else if (integrator == Integrator::VelocityVerlet)
                for (auto i = 0; i < n; ++i)
                {
                    pimpl->velocityVerlet();;
                }
            else if (integrator == Integrator::Verlet)
                for (auto i = 0; i < n; ++i)
                {
                    pimpl->verlet();
                }
            else if (integrator == Integrator::MidPoint)
                for (auto i = 0; i < n; ++i)
                {
                    pimpl->midpoint();
                }
            else
                for (auto i = 0; i < n; ++i)
                {
                    pimpl->rk4();
                }

            if (n > 0)
                pimpl->last_step_size = pimpl->step_size;
        }
        else
        {
            if (pimpl->move_sphere)
            {
                pimpl->sphere->velocity() += pimpl->gravity * (pimpl->step_size*n);
                auto pos = pimpl->sphere->pos() + pimpl->sphere->velocity() * (pimpl->step_size*n);
                if (pos.y <= pimpl->field_height + pimpl->sphere->scal().y)
                {
                    pos.y = pimpl->field_height + pimpl->sphere->scal().y;
                    pimpl->sphere->velocity().y = 0.f;
                }
                if (pos.z > 2.5f)
                {
                    pos.z = 2.5f;
                    pimpl->sphere->velocity() = glm::reflect(pimpl->sphere->velocity(), glm::vec3(0.f, 0.f, -1.f));
                }
                else if (pos.z < -2.5f)
                {
                    pos.z = -2.5f;
                    pimpl->sphere->velocity() = glm::reflect(pimpl->sphere->velocity(), glm::vec3(0.f, 0.f, 1.f));
                }
                pimpl->sphere->place(pos);

            }
            void *buffer; size_t buffer_size;
            PX_CUDA_CHECK(cudaGraphicsMapResources(1, &pimpl->res, 0));
            PX_CUDA_CHECK(cudaGraphicsResourceGetMappedPointer(&buffer, &buffer_size, pimpl->res));
            cudaUpdate(buffer, pimpl->step_size, n, pimpl->sphere);
            PX_CUDA_CHECK(cudaDeviceSynchronize());
            PX_CUDA_CHECK(cudaGraphicsUnmapResources(1, &pimpl->res, 0));
        }
    }

    processInput(dt);
}

void scene::StringScene::render(Shader &scene_shader)
{
}

void scene::StringScene::render()
{
    if (!pimpl->render_texture || pimpl->render_grid)
    {
        pimpl->node_shader->activate();
        glBindVertexArray(pimpl->vao[0]);
        glVertexAttribDivisor(5, 0); // vertex             a group for each instance
        glVertexAttribDivisor(0, 1); // position           one for each instance
        glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, pimpl->n_vertices, pimpl->n_particles);

        pimpl->string_shader->activate();
        glBindVertexArray(pimpl->vao[1]);
        glDrawElements(GL_LINES, pimpl->n_springs+pimpl->n_springs, GL_UNSIGNED_INT, 0);
    }
    if (pimpl->render_texture)
    {
        pimpl->cloth_shader->activate();
        glBindVertexArray(pimpl->vao[2]);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, pimpl->texture[0]);
        glDrawElements(GL_TRIANGLE_STRIP, pimpl->triangle.size(), GL_UNSIGNED_INT, 0);
        pimpl->cloth_shader->activate(false);
    }
    if (scenery != Scenery::Flag)
        pimpl->sphere->render();
    pimpl->skybox->render();
    renderInfo();

}

void scene::StringScene::resetBall()
{
    pimpl->sphere->scale(glm::vec3(.5f));
    if (scenery == Scenery::Falling)
    {
        pimpl->sphere->place(glm::vec3(0.f, pimpl->field_height + .5f, 0.f));
        pimpl->sphere->velocity(glm::vec3(0.f));
    }
    else if (scenery == Scenery::Floating)
    {
        pimpl->sphere->place(glm::vec3(0.f, pimpl->field_height + 1.5f, 0.f));
        pimpl->sphere->velocity(glm::vec3(0.f));
    }
    else if (scenery == Scenery::Flag)
    {
        pimpl->sphere->place(glm::vec3(0.f, -1000.f, 0.f));
        pimpl->sphere->velocity(glm::vec3(0.f));
    }
    else
    {
        pimpl->sphere->place(glm::vec3(0.f, pimpl->field_height + .5f, 2.5f));
        pimpl->sphere->velocity(glm::vec3(0.f, 0.f, -5.f));
    }

    pimpl->sphere->mass(.5f);
    pimpl->sphere->color().r = rnd();
    pimpl->sphere->color().g = rnd();
    pimpl->sphere->color().b = rnd();
    Light light;
    light.diffuse.r = pimpl->sphere->color().r;
    light.diffuse.g = pimpl->sphere->color().g;
    light.diffuse.b = pimpl->sphere->color().b;
    light.ambient = light.diffuse * .05f;
    light.specular = light.diffuse * .5f;
    pimpl->sphere->enlight(&light);
}

void scene::StringScene::pushBall()
{
    if (scenery == Scenery::Flag) return;

    auto vel = pimpl->sphere->velocity();
    if (vel.z > 0.f)
        vel.z += 5.f;
    else
        vel.z -= 5.f;
    pimpl->sphere->velocity(vel);
}

void scene::StringScene::resetCamera()
{
    App::instance()->scene.character.reset(-5.5f, 4.2, 5.5f, 45.f, 18.f);
    App::instance()->scene.character.setShootable(false);
    App::instance()->scene.character.setFloating(true);
}

void scene::StringScene::renderInfo()
{
    App::instance()->text(system_name,
                          10, 10, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::LeftTop);
    App::instance()->text("Nodes: " + std::to_string(pimpl->n_particles),
                          10, 30, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::LeftTop);
    App::instance()->text("Springs: " + std::to_string(pimpl->n_springs),
                          10, 50, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::LeftTop);
    std::string integral = !pimpl->render_cpu ? "Runge-Kutta 4" :
                           (integrator == Integrator::MidPoint ? "MidPoint" :
                           (integrator == Integrator::Euler ? "Euler" :
                           (integrator == Integrator::SemiImplicitEuler ? "Semi Implicit Euler" :
                           (integrator == Integrator::VelocityVerlet ? "Velocity Verlet" :
                           (integrator == Integrator::Verlet ? "Verlet" :
                                                               "Runge-Kutta 4")))));
    App::instance()->text("Integrator: " + integral,
                          10, 70, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::LeftTop);
    App::instance()->text("Step Size: " + std::to_string(pimpl->step_size) + " x " + std::to_string(pimpl->max_steps),
                          10, 90, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::LeftTop);
    App::instance()->text(std::string("Mode: ") + (pimpl->render_cpu ? "CPU" : "GPU"),
                          10, 110, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::LeftTop);

    std::stringstream ss;
    ss << std::fixed << std::setprecision(4) << pimpl->wind.x << ", " << pimpl->wind.z << std::endl;
    App::instance()->text("Wind: " + ss.str(),
                          10, 130, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::LeftTop);


    auto h = App::instance()->frameHeight() - 25;
    auto w = App::instance()->frameWidth() - 10;
    App::instance()->text("Press Arrow Keys to adjust wind",
                          w, h, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::RightTop);
    h -= 20;
    App::instance()->text("Press N to reset wind",
                          w, h, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::RightTop);
    h -= 20;
    App::instance()->text("Press M to switch integrator",
                          w, h, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::RightTop);
    h -= 20;
    App::instance()->text("Press K to toggle GPU mode",
                          w, h, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::RightTop);
    h -= 20;
    App::instance()->text("Press V to toggle grid",
                          w, h, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::RightTop);
    if (pimpl->move_sphere)
    {
        h -= 20;
        App::instance()->text("Press B to push ball",
                              w, h, .4f,
                              glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                              Anchor::RightTop);
    }
}

void scene::StringScene::processInput(float dt)
{
    static auto last_key = GLFW_KEY_UNKNOWN;
    auto window = App::instance()->window();
    static auto sum_dt = 0.f;
    static auto key_count = 0;

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

#define INCREASE_WIND_X pimpl->wind.x += .2f;
#define DECREASE_WIND_X pimpl->wind.x -= .2f;
#define INCREASE_WIND_Z pimpl->wind.z -= .2f;
#define DECREASE_WIND_Z pimpl->wind.z += .2f;

    static auto start = false;
    static auto request = false;
    static auto sst = 0.f;
    if (start)
    {
        if (sst > 30)
        {
            request = true;
            sst = 0;
        }
        else
            sst += dt;
    }

    STICKY_KEY_CHECK(GLFW_KEY_RIGHT, INCREASE_WIND_X)
    else
    STICKY_KEY_CHECK(GLFW_KEY_LEFT, DECREASE_WIND_X)
    else
    STICKY_KEY_CHECK(GLFW_KEY_UP, INCREASE_WIND_Z)
    else
    STICKY_KEY_CHECK(GLFW_KEY_DOWN, DECREASE_WIND_Z)
    else if (glfwGetKey(window, GLFW_KEY_M) == GLFW_PRESS)
        last_key = GLFW_KEY_M;
    else if (glfwGetKey(window, GLFW_KEY_N) == GLFW_PRESS)
        last_key = GLFW_KEY_N;
    else if (glfwGetKey(window, GLFW_KEY_V) == GLFW_PRESS)
        last_key = GLFW_KEY_V;
    else if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS)
        last_key = GLFW_KEY_P;
    else if (glfwGetKey(window, GLFW_KEY_B) == GLFW_PRESS)
        last_key = GLFW_KEY_B;
    else if (glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS)
        last_key = GLFW_KEY_C;
    else if (glfwGetKey(window, GLFW_KEY_K) == GLFW_PRESS)
        last_key = GLFW_KEY_K;
    else if (glfwGetKey(window, GLFW_KEY_1) == GLFW_PRESS)
        last_key = GLFW_KEY_1;
    else if (glfwGetKey(window, GLFW_KEY_2) == GLFW_PRESS)
        last_key = GLFW_KEY_2;
    else if (glfwGetKey(window, GLFW_KEY_3) == GLFW_PRESS)
        last_key = GLFW_KEY_3;
    else if (glfwGetKey(window, GLFW_KEY_4) == GLFW_PRESS)
        last_key = GLFW_KEY_4;
    else if (glfwGetKey(window, GLFW_KEY_X) == GLFW_PRESS)
        last_key = GLFW_KEY_X;
    else
    {
        if (last_key == GLFW_KEY_X)
        {
            start = true;
            restart(App::instance()->scene);
        }
        if (last_key == GLFW_KEY_1)
        {
            scenery = Scenery::Dangling;
            restart(App::instance()->scene);
        }
        else if (last_key == GLFW_KEY_2)
        {
            scenery = Scenery::Falling;
            restart(App::instance()->scene);
        }
        else if (last_key == GLFW_KEY_3)
        {
            scenery = Scenery::Floating;
            restart(App::instance()->scene);
        }
        else if (last_key == GLFW_KEY_4)
        {
            scenery = Scenery::Flag;
            restart(App::instance()->scene);
        }
        else if (last_key == GLFW_KEY_K)
        {
            pimpl->render_cpu = !pimpl->render_cpu;
            restart(App::instance()->scene);
        }
        else if (last_key == GLFW_KEY_C)
            resetCamera();
        else if (last_key == GLFW_KEY_B)
            pushBall();
        else if (last_key == GLFW_KEY_P)
            pause = !pause;
        else if (last_key == GLFW_KEY_M || request)
        {
            switch (integrator)
            {
                case Integrator::RK4:
                    integrator = Integrator::MidPoint;
                    break;
                case Integrator::MidPoint:
                    integrator = Integrator::Verlet;
                    break;
                case Integrator::Verlet:
                    integrator = Integrator::VelocityVerlet;
                    break;
                case Integrator::VelocityVerlet:
                    integrator = Integrator::Euler;
                    break;
                case Integrator::Euler:
                    integrator = Integrator::SemiImplicitEuler;
                    break;
//                case Integrator::SemiImplicitEuler:
                default:
                    integrator = Integrator::RK4;
            }

            if (request)
                restart(App::instance()->scene);
        }
        else if (last_key == GLFW_KEY_V)
        {
            if (pimpl->render_grid && pimpl->render_texture)
            {
                pimpl->render_grid = false;
            }
            else if (pimpl->render_texture)
            {
                pimpl->render_grid = true;
                pimpl->render_texture = false;
            }
            else
            {
                pimpl->render_grid = true;
                pimpl->render_texture = true;
            }


        }
        else if (last_key == GLFW_KEY_N)
            pimpl->wind = glm::vec3(0.f);
        last_key = GLFW_KEY_UNKNOWN;
    }

    request = false;
}
