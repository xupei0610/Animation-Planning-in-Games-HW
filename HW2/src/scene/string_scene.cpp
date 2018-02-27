#include <sstream>
#include <iomanip>
#include <glm/gtx/norm.hpp>

#include "scene/string_scene.hpp"
#include "item/floor.hpp"
#include "item/sphere.hpp"
#include "shader/skybox.hpp"
#include "app.hpp"
#include "global.hpp"
#include "util/random.hpp"

#include "stb_image.hpp"

using namespace px;

class scene::StringScene::impl
{
public:
    bool render_grid, render_texture;
    bool need_upload;
    unsigned int vao[3], vbo[5], texture[1];
    Shader *node_shader, *string_shader, *cloth_shader;
    SkyBox *skybox;
    item::Floor *floor; item::Sphere *sphere;
    glm::vec3 sphere_last_pos; glm::vec3 sphere_acc;

    unsigned int n_vertices;
    unsigned int n_particles;
    unsigned int n_springs;
    float step_size;
    int max_steps;

    std::vector<float> uv_index;
    std::vector<unsigned int> triangle;

    std::vector<glm::vec3> position;
    struct Particle_t
    {
        glm::vec3 last_pos;
        glm::vec3 velocity;
        glm::vec3 acc;
        glm::vec3 force;
        glm::vec3 last_acc;
        float mass;
    };
    std::vector<std::pair<unsigned int, unsigned int> > link;
//    struct Spring_t
//    {
//        float length;
//    };
    std::vector<Particle_t> particles;
//    std::vector<Spring_t> springs;

    float ks, ks_shear, ks_bend; // stretching coefficient
    float kd, kd_shear, kd_bend; // damping coefficient
    float rest_len, rest_len_shear, rest_len_bend;
    float air_friction, field_height, ground_friction;
    glm::vec3 field_norm;
    unsigned int grid_x, grid_y;
    glm::vec3 gravity;
    glm::vec3 wind;

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
             floor(nullptr), sphere(nullptr)
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
//        if (particles[id1].mass == std::numeric_limits<decltype(particles[id1].mass)>::max())
//            return glm::vec3(0.f);
        auto dx = position[id1] - position[id2];
        auto x = glm::length(dx);
        if (x > std::numeric_limits<float>::min())
        {
            auto dv = particles[id1].velocity - particles[id2].velocity;
            auto F = dx/x;
            F *= - ks * (x - rest_length)
                 - kd * glm::dot(dv, F);
            return F;
        }
        return glm::vec3(0.f);
    }

    glm::vec3 windForce(unsigned int id1, unsigned int id2, unsigned int id3)
    {
        auto v = (particles[id1].velocity + particles[id2].velocity + particles[id3].velocity) / 3.f;
        v -= wind;
        auto v_len = glm::length(v);

        auto norm = glm::cross(position[id2] - position[id1],
                               position[id3] - position[id1]);

        return -0.001f * v_len * glm::dot(v, norm) / (2 * glm::length(norm)) * norm;
    }

    void applyForce(unsigned int id)
    {
        if (particles[id].mass == std::numeric_limits<float>::max())
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
        particles[id].acc = gravity + f / particles[id].mass
            - particles[id].velocity * (position[id].y == field_height ? ground_friction : air_friction);
    }

    void euler()
    {
//        glm::vec3 sphere_force(0.f);
        for (decltype(n_particles) idx = 0; idx < n_particles; ++idx)
        {
            applyForce(idx);
        }
        for (decltype(n_particles) idx = 0; idx < n_particles; ++idx)
        {
            auto &p = particles[idx];
            if (p.mass == std::numeric_limits<float>::max()) continue;
            position[idx] += p.velocity * step_size;
            p.velocity += p.acc * step_size;

            auto x = position[idx] - sphere->pos();
            auto dx = sphere->scal().x - glm::length(x);
            if (dx > 0)
            {
                auto norm = glm::normalize(x);

//                auto acc = p.velocity;
                p.velocity = 0.9f * glm::reflect(p.velocity, norm);
//                acc = p.velocity - acc;

                position[idx] = sphere->pos() + (sphere->scal().x + dx) * norm;

//                sphere_force += p.mass * acc / step_size;
            }
            if (position[idx].y < field_height)
            {
                p.velocity = 0.9f * glm::reflect(p.velocity, field_norm);
                position[idx].y = field_height + 0.9f * (field_height - position[idx].y);
            }
        }

//        auto pos = sphere->pos() + sphere->velocity() * step_size;
//        sphere->velocity() -= sphere_force * (step_size / sphere->mass());
//        sphere->velocity() += (gravity + this->sphere_acc) * step_size;
//        if (pos.y < field_height + sphere->scal().y)
//        {
//            pos.y = field_height + sphere->scal().y;
//            sphere->velocity().y = 0.f;
//        }
//        sphere->place(pos);
    }

    void semiImplicitEuler()
    {
//        glm::vec3 sphere_force(0.f);
        for (decltype(n_particles) idx = 0; idx < n_particles; ++idx)
        {
            auto &p = particles[idx];
            if (p.mass == std::numeric_limits<float>::max()) continue;
            applyForce(idx);
        }
        for (decltype(n_particles) idx = 0; idx < n_particles; ++idx)
        {
            auto &p = particles[idx];
            if (p.mass == std::numeric_limits<float>::max()) continue;
            p.velocity += p.acc * step_size;
            position[idx] += p.velocity * step_size;

            auto x = position[idx] - sphere->pos();
            auto dx = sphere->scal().x - glm::length(x);
            if (dx > 0)
            {
                auto norm = glm::normalize(x);
//                auto acc = p.velocity;
                p.velocity = 0.9f * glm::reflect(p.velocity, norm);
//                acc = p.velocity - acc;

                position[idx] = sphere->pos() + (sphere->scal().x + dx) * norm;
//                sphere_force += p.mass * acc / step_size;
            }
            if (position[idx].y < field_height)
            {
                p.velocity = 0.9f * glm::reflect(p.velocity, field_norm);
                position[idx].y = field_height + 0.9f * (field_height - position[idx].y);
            }
        }
//        sphere->velocity() -= sphere_force * (step_size / sphere->mass());
//        sphere->velocity() += (gravity + this->sphere_acc) * step_size;
//        auto pos = sphere->pos() + sphere->velocity() * step_size;
//        if (pos.y < field_height + sphere->scal().y)
//        {
//            pos.y = field_height + sphere->scal().y;
//            sphere->velocity().y = 0.f;
//        }
//        sphere->place(pos);
    }

    void verlet()
    {
//        glm::vec3 sphere_acc(0.f);
        for (decltype(n_particles) idx = 0; idx < n_particles; ++idx)
        {
            auto &p = particles[idx];
            if (p.mass == std::numeric_limits<float>::max()) continue;
            applyForce(idx);
        }
        for (decltype(n_particles) idx = 0; idx < n_particles; ++idx)
        {
            auto &p = particles[idx];
            if (p.mass == std::numeric_limits<float>::max()) continue;

            auto temp = position[idx];
            position[idx] += position[idx] - p.last_pos + p.acc * step_size * step_size;
            p.last_pos = temp;

            auto x = position[idx] - sphere->pos();
            auto dx = sphere->scal().x - glm::length(x);
            if (dx > 0)
            {
                auto norm = glm::normalize(x);
                position[idx] = sphere->pos() + (sphere->scal().x + dx) * norm;
//                sphere_acc += p.mass * dx / step_size;
            }
            if (position[idx].y < field_height)
            {
                position[idx].y = field_height + 0.9f * (field_height - position[idx].y);
            }
            p.velocity = (position[idx] - p.last_pos) / step_size;
        }
//        sphere_acc /= sphere->mass();
//        sphere_acc += gravity + this->sphere_acc;
//        auto temp = sphere->pos();
//        auto pos = sphere->pos() + sphere->pos() - sphere_last_pos + sphere_acc * step_size * step_size;
//        if (pos.y < field_height + sphere->scal().y)
//            pos.y = field_height + sphere->scal().y;
//        sphere_last_pos = temp;
//        sphere->place(pos);
    }

    void velocityVerlet()
    {
//        glm::vec3 sphere_force(0.f);
        for (decltype(n_particles) idx = 0; idx < n_particles; ++idx)
        {
            auto &p = particles[idx];
            if (p.mass == std::numeric_limits<float>::max()) continue;
            p.velocity += p.acc * .5f * step_size;
            position[idx] += p.velocity * step_size;
        }
        sphere->velocity() += sphere->acceleration() * .5f * step_size;
        auto pos = sphere->pos() + sphere->velocity() * step_size;
        if (pos.y < field_height + sphere->scal().y)
            pos.y = field_height + sphere->scal().y;
        sphere->place(pos);
        for (decltype(n_particles) idx = 0; idx < n_particles; ++idx)
        {
            auto &p = particles[idx];
            if (p.mass == std::numeric_limits<float>::max()) continue;
            auto x = position[idx] - sphere->pos();
            auto dx = sphere->scal().x - glm::length(x);
            if (dx > 0)
            {
                auto norm = glm::normalize(x);
//                auto acc = p.velocity;
                p.velocity = 0.9f * glm::reflect(p.velocity, norm);
//                acc = p.velocity - acc;
                position[idx] = sphere->pos() + (sphere->scal().x + dx) * norm;
//                sphere_force += p.mass * acc / step_size;
            }
            if (position[idx].y < field_height)
            {
                p.velocity = 0.9f * glm::reflect(p.velocity, field_norm);
                position[idx].y = field_height + 0.9f * (field_height - position[idx].y);
            }
            applyForce(idx);
        }
        for (decltype(n_particles) idx = 0; idx < n_particles; ++idx)
        {
            auto &p = particles[idx];
            if (p.mass == std::numeric_limits<float>::max()) continue;
            p.velocity += p.acc * .5f * step_size;
        }
//        sphere->acceleration() = -sphere_force / sphere->mass() + gravity + this->sphere_acc;
//        sphere->velocity() += sphere->acceleration() * .5f * step_size;
    }
    void updateSphere()
    {
        sphere->place(sphere->pos() + sphere->velocity() * step_size);
    }
};

scene::StringScene::StringScene()
        : BaseScene(),
          system_name("Cloth Simulation"),
          _integrator(Integrator::Verlet)
{
    pimpl = std::unique_ptr<impl>(new impl);
}

scene::StringScene::~StringScene()
{}

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
    pimpl->kd = .01f; pimpl->kd_shear = .005f; pimpl->kd_bend = .005f;  // inner friction
    pimpl->ks = 2.f; pimpl->ks_shear = .2f; pimpl->ks_bend = .2f;  // tot_mass * gravity / desired_stretch;
    pimpl->ground_friction = 0.01f;
    pimpl->field_height = 0.f;
    pimpl->field_norm = glm::vec3(0.f, 1.f, 0.f);
    pimpl->grid_x = 50; pimpl->grid_y = 50;
    pimpl->rest_len = .05f;
    pimpl->rest_len_shear = pimpl->rest_len*std::sqrt(2.f);
    pimpl->rest_len_bend  = pimpl->rest_len*2;

    pimpl->particles.clear();
//    pimpl->springs.clear();
    pimpl->position.clear();
    pimpl->link.clear();
    pimpl->uv_index.clear();
    auto gap_u = 1.f/pimpl->grid_x;
    auto gap_v = 1.f/pimpl->grid_y;

    auto z = 0.f; auto y = (pimpl->grid_y-3)*pimpl->rest_len;
    auto mass = .1f / (pimpl->grid_x * pimpl->grid_y);
    for (decltype(pimpl->grid_y) i = 0; i < pimpl->grid_y; ++i)
    {
        auto x = -0.5f*pimpl->rest_len*pimpl->grid_x;
        for (decltype(pimpl->grid_x) j = 0; j < pimpl->grid_x; ++j)
        {
            pimpl->position.push_back(glm::vec3(x, y, z));
            pimpl->particles.push_back({glm::vec3(x, y, z), glm::vec3(0.f), glm::vec3(0.f), glm::vec3(0.f), glm::vec3(0.f),
                                        i == 0 && (j < 2 || j > pimpl->grid_x-3 || j == pimpl->grid_x/2 || j == pimpl->grid_x/2+1) ?
                                            std::numeric_limits<float>::max() : mass});
            pimpl->uv_index.push_back(j*gap_u);
            pimpl->uv_index.push_back(i*gap_v);

            x += pimpl->rest_len;
        }
        z -= pimpl->rest_len;
        y -= pimpl->rest_len*.5f;
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

    pimpl->n_particles = static_cast<unsigned int>(pimpl->particles.size());
    pimpl->n_springs = static_cast<unsigned int>(pimpl->link.size());

    pimpl->floor->scale(glm::vec3(10.f, 1.f, 10.f));
    pimpl->floor->place(glm::vec3(-5.f, pimpl->field_height, -5.f));
    pimpl->floor->setGrid(5.f, 5.f);
    scene.add(pimpl->floor);

    pimpl->sphere->scale(glm::vec3(.25f));
    pimpl->sphere->place(glm::vec3(0.f, pimpl->field_height + .25f, 0.f));
    pimpl->sphere->mass(.5f);
    pimpl->sphere_last_pos = pimpl->sphere->pos();
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

        pimpl->need_upload = false;
    }
    else
    {
        glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[1]);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(float)*3*pimpl->n_particles, pimpl->position.data());
    }
}

void scene::StringScene::update(float dt)
{
    static bool turn = true;

    if (!pause)
    {
        if ((turn && pimpl->sphere->pos().z > 2.f) || (!turn && pimpl->sphere->pos().z < -2.f))
        {
            turn = !turn;
        }
        if (turn)
            pimpl->sphere->velocity(glm::vec3(0.f, 0.f, 2.5f));
        else
            pimpl->sphere->velocity(glm::vec3(0.f, 0.f, -2.5f));

        auto n = std::min(pimpl->max_steps, static_cast<int>(std::ceil(dt / pimpl->step_size)));
        if (integrator() == Integrator::Euler)
            for (auto i = 0; i < n; ++i)
            {
                pimpl->updateSphere();
                pimpl->euler();
            }
        else if (integrator() == Integrator::SemiEuler)
            for (auto i = 0; i < n; ++i)
            {
                pimpl->updateSphere();
                pimpl->semiImplicitEuler();
            }
        else if (integrator() == Integrator::VelocityVerlet)
            for (auto i = 0; i < n; ++i)
            {
                pimpl->updateSphere();
                pimpl->velocityVerlet();
            }
        else
            for (auto i = 0; i < n; ++i)
            {
                pimpl->updateSphere();
                pimpl->verlet();
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
    pimpl->sphere->render();
    pimpl->skybox->render();
    renderInfo();
}

void scene::StringScene::resetCamera()
{
    App::instance()->scene.character.reset(0.f, .75f, 3.f, 0.f, 0.f);
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
    std::string integral =  integrator() == Integrator::Euler ? "Euler" :
                        (integrator() == Integrator::SemiEuler ? "Semi Implicit Euler" :
                        (integrator() == Integrator::VelocityVerlet ? "Velocity Verlet" :
                         "Verlet"));
    App::instance()->text("Integrator: " + integral,
                          10, 70, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::LeftTop);
    std::stringstream ss;
    ss << std::fixed << std::setprecision(4) << pimpl->wind.x << ", " << pimpl->wind.z << std::endl;
    App::instance()->text("Wind: " + ss.str(),
                          10, 90, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::LeftTop);


    auto h = App::instance()->frameHeight() - 25;
    auto w = App::instance()->frameWidth() - 10;
    App::instance()->text("Press Left and Right Arrow to adjust wind along X-axis",
                          w, h, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::RightTop);
    h -= 20;
    App::instance()->text("Press Up and Down Arrow to adjust wind along Z-axis",
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
    App::instance()->text("Press V to toggle grid",
                          w, h, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::RightTop);
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
    else
    {
        if (last_key == GLFW_KEY_B)
            resetCamera();
        else if (last_key == GLFW_KEY_P)
            pause = !pause;
        else if (last_key == GLFW_KEY_M)
        {
            if (integrator() == Integrator::Verlet)
                setIntegrator(Integrator::VelocityVerlet);
            else if (integrator() == Integrator::VelocityVerlet)
                setIntegrator(Integrator::Euler);
            else if (integrator() == Integrator::Euler)
                setIntegrator(Integrator::SemiEuler);
            else
                setIntegrator(Integrator::Verlet);
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
}

void scene::StringScene::setIntegrator(Integrator integrator)
{
    _integrator = integrator;
}