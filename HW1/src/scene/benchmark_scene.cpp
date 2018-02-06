#include "scene/benchmark_scene.hpp"
#include "scene.hpp"
#include "util/random.hpp"
#include "app.hpp"
#include "config.hpp"

#include <cstring>
#include <glm/gtx/norm.hpp>

using namespace px;

#include "scene/impl/benchmark_transform_feedback_particle_system.hpp"
#include "scene/impl/benchmark_compute_shader_particle_system.hpp"

class scene::BenchmarkScene::impl
{
public:
    std::vector<glm::vec3> position;
    std::vector<glm::vec3> velocity;
    std::vector<glm::vec3> acceleration;
    std::vector<float> rotation;
    std::vector<glm::vec3> life;
};

scene::BenchmarkScene::BenchmarkScene()
        : BaseScene(),
          particle_system(nullptr)
{
    systems.emplace_back("compute shader", new ComputeShaderParticleSystem);
    systems.back().second->max_particles = BENCHMARK_MAX_PARTICLES_COMPUTE_SHADER;
    systems.back().second->birth_rate    =  .12f * systems.back().second->max_particles;
    systems.emplace_back("transform feedback with instancing", new TransformFeedbackParticleSystem);
    systems.back().second->max_particles = BENCHMARK_MAX_PARTICLES_TRANSFORM_FEEDBACK;
    systems.back().second->birth_rate    =  .12f * systems.back().second->max_particles;
    systems.emplace_back("instancing", new Simple2DParticleSystem);
    systems.back().second->max_particles = BENCHMARK_MAX_PARTICLES_SIMPLE_INSTANCING;
    systems.back().second->birth_rate =     .12f * systems.back().second->max_particles;
    systems.emplace_back("geometry shader", new GeometryParticleSystem);
    systems.back().second->max_particles = BENCHMARK_MAX_PARTICLES_SIMPLE_GEOMETRY_SHADER;
    systems.back().second->birth_rate =     .12f * systems.back().second->max_particles;

    constexpr auto PI = static_cast<float>(M_PI);
    constexpr auto double_PI = static_cast<float>(M_2_PI);
    constexpr auto drot = 0.06f * PI;
#include "scene/impl/benchmark_particle_update_fn.hpp"

    for (auto s: systems)
    {
        s.second->spawn_fn  = spawn_fn;
        s.second->update_fn = update_fn;
    }

    pimpl = std::unique_ptr<impl>(new impl);
    system_name = systems.front().first;
    particle_system = systems.front().second;
}

scene::BenchmarkScene::~BenchmarkScene()
{
    for (auto p : systems)
        delete p.second;
}

void scene::BenchmarkScene::init(Scene &scene)
{
    std::vector<float> vertex;
    constexpr auto grid = 32;
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
    for (auto p : systems)
        p.second->init(vertex.data(), vertex.size()/2);
}

void scene::BenchmarkScene::restart(Scene &scene)
{
    pause = false;

    scene.character.reset(0.f, 0.f, -30.f, 180.f, 0.f);
    scene.character.setShootable(false);
    scene.character.setFloating(true);

    particle_system->restart();
    pimpl->position.resize(particle_system->max_particles);
    pimpl->rotation.resize(particle_system->max_particles);
    pimpl->velocity.resize(particle_system->max_particles);
    pimpl->acceleration.resize(particle_system->max_particles);
    pimpl->life.resize(particle_system->max_particles);
}

void scene::BenchmarkScene::upload(Shader &scene_shader)
{
    glClearColor(0.f, 0.f, 0.f, 1.f);
    particle_system->upload();
}

void scene::BenchmarkScene::render(Shader &scene_shader)
{

}

void scene::BenchmarkScene::update(float dt)
{
    if (!pause)
        particle_system->update(dt);
    processInput(dt);
}

void scene::BenchmarkScene::render()
{
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);
    particle_system->render(GL_TRIANGLE_STRIP);
    glDisable(GL_BLEND);

    renderInfo();
}

void scene::BenchmarkScene::renderInfo()
{
    App::instance()->text("Particles: " + std::to_string(particle_system->count()) + "/" + std::to_string(particle_system->total()),
                 10, 10, .4f,
                 glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                 Anchor::LeftTop);
    App::instance()->text("Rendering Mode: " + system_name,
                 10, 30, .4f,
                 glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                 Anchor::LeftTop);

    auto h = App::instance()->frameHeight() - 25;
    if (particle_system->max_particles != particle_system->total())
    {
        App::instance()->text("Max Particles set to " + std::to_string(particle_system->max_particles),
                     10, h - 20, .4f,
                     glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                     Anchor::LeftTop);
        App::instance()->text("Press R to reset the scene",
                     10, h, .4f,
                     glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                     Anchor::LeftTop);
    }
    App::instance()->text("Press M to switch rendering mode",
                 App::instance()->frameWidth() - 10, h, .4f,
                 glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                 Anchor::RightTop);
    App::instance()->text("Press Z to increase particles; Press X to decrease particles",
                          App::instance()->frameWidth() - 10, h-25, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::RightTop);
}

void scene::BenchmarkScene::processInput(float dt)
{
    auto window = App::instance()->window();
    static auto sum_dt = 0.f;
    static auto last_key = GLFW_KEY_UNKNOWN;
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

#define INCREASE_PARTICLES                                              \
    particle_system->max_particles += 1000;                             \
    particle_system->birth_rate = particle_system->max_particles * .1f;
#define DECREASE_PARTICLES                                                  \
    if (particle_system->max_particles > 1000)                              \
    {                                                                       \
        particle_system->max_particles -= 1000;                             \
        particle_system->birth_rate = particle_system->max_particles * .1f; \
    }

    STICKY_KEY_CHECK(GLFW_KEY_Z, INCREASE_PARTICLES)
    else
    STICKY_KEY_CHECK(GLFW_KEY_X, DECREASE_PARTICLES)
    else if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS)
        last_key = GLFW_KEY_P;
    else if (glfwGetKey(window, GLFW_KEY_M) == GLFW_PRESS)
        last_key = GLFW_KEY_M;
    else
    {
        if (last_key == GLFW_KEY_M)
        {
            for (auto i = 0, tot = static_cast<int>(systems.size()); i < tot; ++i)
            {
                if (particle_system == systems[i].second)
                {
                    auto idx = i == tot - 1 ? 0 : i + 1;
                    particle_system = systems[idx].second;
                    system_name = systems[idx].first;
                    restart(App::instance()->scene);
                    break;
                }
            }
        }
        else if (last_key == GLFW_KEY_P)
            pause = !pause;
        last_key = GLFW_KEY_UNKNOWN;
    }

#undef HOLD_KEY
}
