
#include <sstream>
#include <iomanip>

#include "scene/string_scene.hpp"
#include "scene.hpp"
#include "app.hpp"
#include "global.hpp"
#include "util/cuda.hpp"

using namespace px;

class scene::StringScene::impl
{
public:
    bool need_upload;
    unsigned int vao[2], vertex_vbo, vbo, link_vbo;
    Shader *node_shader, *string_shader;

    unsigned int n_vertices;
    unsigned int n_particles;
    unsigned int n_springs;
    float step_size;

    std::vector<glm::vec3> position;
    struct Particle_t
    {
        glm::vec3 velocity;
        glm::vec3 force;
        float mass;
    };
    std::vector<std::pair<unsigned int, unsigned int> > link;
    struct Spring_t
    {
        float length;
    };
    std::vector<Particle_t> particles;
    std::vector<Spring_t> springs;

    float k;
    float kv;
    float air_friction;
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
            "   mat4 VP = proj * view;"
            "   gl_Position = VP * vec4(position, 1.f);"
            "}";
    const char *STRING_FS = "#version 330 core\n"
            ""
            "out vec4 gColor;"
            ""
            "void main(){"
            "   gColor = vec4(1.f, 1.f, 1.f, 1.f);"
            "}";

    impl() : vao{2}, vertex_vbo(0), vbo(0),
             node_shader(nullptr), string_shader(nullptr)
    {}
    ~impl()
    {
        clearGLObjs();

        delete node_shader;
        delete string_shader;
    }

    void clearGLObjs()
    {
        glDeleteVertexArrays(2, vao);
        glDeleteBuffers(1, &vbo);
        glDeleteBuffers(1, &vertex_vbo);
        glDeleteBuffers(1, &link_vbo);

        vao[0] = 0; vao[1] = 0;
        vbo = 0; vertex_vbo = 0; link_vbo = 0;
    }

    void genGLObjs()
    {
        glDeleteVertexArrays(2, vao);
        glDeleteBuffers(1, &vbo);
        glDeleteBuffers(1, &vertex_vbo);
        glDeleteBuffers(1, &link_vbo);

        glGenVertexArrays(2, vao);
        glGenBuffers(1, &vbo);
        glGenBuffers(1, &vertex_vbo);
        glGenBuffers(1, &link_vbo);
    }

};

scene::StringScene::StringScene()
        : BaseScene(),
          system_name("Fireworks Demo")
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
    glBindBuffer(GL_ARRAY_BUFFER, pimpl->vertex_vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float)*vertex.size(), vertex.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(5, 2, GL_FLOAT, GL_FALSE, 0, (void *)0);
    glEnableVertexAttribArray(5);
    glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo);
    glEnableVertexAttribArray(0);   // position
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3*sizeof(float), (void *)(0));

    pimpl->string_shader->activate();
    pimpl->string_shader->bind("GlobalAttributes", 0);
    glBindVertexArray(pimpl->vao[1]);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, pimpl->link_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo);
    glEnableVertexAttribArray(0);   // position
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3*sizeof(float), (void *)(0));
    pimpl->string_shader->activate(false);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    pimpl->n_vertices = vertex.size()/2;
}

void scene::StringScene::restart(Scene &scene)
{
    static auto first_run = true;
    if (first_run)
    {
        resetCamera();
        first_run = false;
    }
    pause = false;
    pimpl->need_upload = true;

    pimpl->particles.clear();
    pimpl->springs.clear();
    pimpl->position.clear();
    pimpl->link.clear();

    auto max_particles = 80;

    auto y = 0.f;
    pimpl->position.push_back(glm::vec3(0.f, 0.f, 0.f));
    pimpl->particles.push_back({glm::vec3(0.f), glm::vec3(0.f), std::numeric_limits<float>::max()});
    for (auto i = 1; i < max_particles; ++i)
    {
        y -= .05f;
        pimpl->position.push_back(glm::vec3(0.f, 0.f, y));
        pimpl->particles.push_back({glm::vec3(0.f), glm::vec3(0.f), .0001f});
    }
    for (auto i = 0; i < max_particles-1; ++i)
    {
        pimpl->link.emplace_back(i, i+1);
        pimpl->springs.push_back({.05f});
    }

    pimpl->gravity = glm::vec3(0.f, -9.81f, 0.f);
    pimpl->air_friction = .01f;
    pimpl->kv = .2f;    // inner friction
    pimpl->k = 5000.f; // tot_mass * gravity / desired_stretch;
    pimpl->step_size = 0.0001f;

    pimpl->n_particles = static_cast<unsigned int>(pimpl->particles.size());
    pimpl->n_springs = static_cast<unsigned int>(pimpl->springs.size());
}

void scene::StringScene::upload(Shader &scene_shader)
{
    if (pimpl->need_upload)
    {
        glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(float)*3*pimpl->n_particles, pimpl->position.data(), GL_STATIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, pimpl->link_vbo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(float)*2*pimpl->n_springs, pimpl->link.data(), GL_STATIC_DRAW);
        pimpl->need_upload = false;
    }
    else
    {
        glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(float)*3*pimpl->n_particles, pimpl->position.data());
    }
}

void scene::StringScene::update(float dt)
{
//    dt = std::min(0.001f, dt);
    if (!pause)
    {
        auto n = static_cast<int>(std::ceil(dt / pimpl->step_size));
        for (auto i = 0; i < n; ++i)
        {
            for (decltype(pimpl->n_springs) idx = 0; idx < pimpl->n_springs; ++idx)
            {
                auto F = pimpl->position[pimpl->link[idx].first] - pimpl->position[pimpl->link[idx].second];
                auto x = glm::length(F);

                if (x > std::numeric_limits<float>::min())
                {
                    F /= x;
                    F *= - pimpl->k * (x - pimpl->springs[idx].length); // k is prop to the natural length of spring
                    F += - pimpl->kv * (pimpl->particles[pimpl->link[idx].first].velocity - pimpl->particles[pimpl->link[idx].second].velocity);

                    pimpl->particles[pimpl->link[idx].first].force += F;
                    pimpl->particles[pimpl->link[idx].second].force -= F;
                }
            }

            for (decltype(pimpl->n_particles) idx = 0; idx < pimpl->n_particles; ++idx)
            {
                auto & p = pimpl->particles[idx];

                if (p.mass != std::numeric_limits<float>::max()) // fixed point with infinite mass
                {
                    auto acc = pimpl->gravity + pimpl->wind + p.force / p.mass - p.velocity * pimpl->air_friction;
                    p.velocity += acc * pimpl->step_size;
                    pimpl->position[idx] += p.velocity * pimpl->step_size;
                }

                p.force = glm::vec3(0.f);
            }
        }
    }

    processInput(dt);
}

void scene::StringScene::render()
{
    pimpl->node_shader->activate();
    glBindVertexArray(pimpl->vao[0]);
    glVertexAttribDivisor(5, 0); // vertex             a group for each instance
    glVertexAttribDivisor(0, 1); // position           one for each instance
    glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, pimpl->n_vertices, pimpl->n_particles);

    pimpl->string_shader->activate();
    glBindVertexArray(pimpl->vao[1]);
    glDrawElements(GL_LINES, pimpl->n_springs+pimpl->n_springs, GL_UNSIGNED_INT, 0);
    pimpl->string_shader->activate(false);
    glBindVertexArray(0);

    renderInfo();
}

void scene::StringScene::resetCamera()
{
    App::instance()->scene.character.reset(-3.f, 2.f, -3.f, 135.f, 35.f);
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
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << pimpl->wind.x << ", " << pimpl->wind.z << std::endl;
    App::instance()->text("Wind: " + ss.str(),
                          10, 50, .4f,
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
    App::instance()->text("Press M to reset wind",
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

#define INCREASE_WIND_X pimpl->wind.x -= .1f;
#define DECREASE_WIND_X pimpl->wind.x += .1f;
#define INCREASE_WIND_Z pimpl->wind.z += .1f;
#define DECREASE_WIND_Z pimpl->wind.z -= .1f;

    STICKY_KEY_CHECK(GLFW_KEY_RIGHT, INCREASE_WIND_X)
    else
    STICKY_KEY_CHECK(GLFW_KEY_LEFT, DECREASE_WIND_X)
    else
    STICKY_KEY_CHECK(GLFW_KEY_UP, INCREASE_WIND_Z)
    else
    STICKY_KEY_CHECK(GLFW_KEY_DOWN, DECREASE_WIND_Z)
    else if (glfwGetKey(window, GLFW_KEY_M) == GLFW_PRESS)
        last_key = GLFW_KEY_M;
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
            pimpl->wind = glm::vec3(0.f);
        last_key = GLFW_KEY_UNKNOWN;
    }
}
