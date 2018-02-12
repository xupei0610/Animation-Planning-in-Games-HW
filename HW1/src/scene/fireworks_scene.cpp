
#include "scene/fireworks_scene.hpp"
#include "scene.hpp"
#include "app.hpp"
#include "global.hpp"
#include "config.h"
#include "util/random.hpp"
#include "util/cuda.hpp"

using namespace px;

class scene::FireworksScene::ComputeShaderParticleSystem::impl
{
public:
    unsigned int vao, vertex_vbo, vbo;
    unsigned int n_vertices;
    float debt;
    unsigned int tot;
    bool upload;
    int trail, trail_rate;
    float debt_particles;
    float end_time;

    Shader *compute_shader, *draw_shader;
    float time;

    const char *CS = "#version 430 core\n"
            "layout (local_size_x = " STR(COMPUTE_SHADER_WORK_GROUP_SIZE) ", local_size_y = 1, local_size_z = 1) in;"
            ""
            "layout(std140, binding = 0) buffer Particle_t"
            "{"
            "   struct"
            "   {"
            "       vec4 position;" // position,  size
            "       vec4 velocity;" // velocity,  alpha
            "       vec4 flag;" //  explode flag, life, trail
            "   } particles[];"
            "};"
            "uniform float dt;"
            "uniform int trail;"
            "uniform int n_particles;"
            ""
            "uniform vec3 gravity;"      // initial position
            "uniform vec3 acceleration;" // initial velocity
            ""
            "uniform float horizontal_resistance = .5f;"
            "const float PI = 3.1415926535897932384626433832795;"
            ""
            "uint gid = gl_GlobalInvocationID.x;"
            ""
            "float rnd(vec2 co)"
            "{"
            "   return fract(sin(mod(dot(co.xy,vec2(12.9898,78.233)), 3.14)) * 43758.5453);"
            "}"
            ""
            "void spawn()"
            "{"
            "   vec2 sd = vec2(particles[gid].position.x, gid);"
            "   float theta = PI * rnd(sd) * 2.f; ++sd.x;"
            "   float phi = acos(rnd(sd) * 2.f - 1.f);    ++sd.x;"
            "   float r0 = pow(rnd(sd)*.000001, 1.f/3.f); ++sd.x;"
            "   particles[gid].position.x = gravity.x + r0 * sin(phi) * cos(theta);"
            "   particles[gid].position.y = gravity.y + r0 * cos(phi);"
            "   particles[gid].position.z = gravity.z + r0 * sin(phi) * sin(theta);"
            "   particles[gid].position.w = 0.01 * (1 + rnd(sd)); ++sd.x;" // size
            "   particles[gid].velocity.xyz = acceleration;"
            "   particles[gid].flag.y = 2.f + rnd(sd);" // life
            "   particles[gid].flag.x = 0.f;" // not exploded
            "   particles[gid].velocity.w = 1.f;" // alpha
            "}"
            ""
            "void explode()"
            "{"
            "   vec2 sd = vec2(particles[gid].position.x, gid);"
            "   float theta = PI * rnd(sd) * 2.f; ++sd.x;"
            "   float phi = acos(rnd(sd) * 2.f - 1.f);    ++sd.x;"
            "   float r0 = pow(6.f*rnd(sd), 1.f/3.f); ++sd.x;"
            "   particles[gid].velocity.x = 4.f * rnd(sd) * r0 * sin(phi) * cos(theta);"
            "   particles[gid].velocity.y = r0 * cos(phi);"
            "   particles[gid].velocity.z = 4.f * rnd(sd) * r0 * sin(phi) * sin(theta);"
            "   particles[gid].flag.x = 1.f;" // exploded
            "}"
            "void update()"
            "{"
            "   particles[gid].velocity.w -= dt / particles[gid].flag.y;"   // alpha
            "   if (gid < trail)" // trail
            "   {"
            "       particles[gid].flag.x = 1.f;"
            "   }"
            "   else"
            "   {"
            "       particles[gid].velocity.xyz += dt * (acceleration + gravity);"
            "       if (particles[gid].velocity.x > 0.f)"
            "           particles[gid].velocity.x -= horizontal_resistance * dt;"
            "       else if (particles[gid].velocity.x < 0.f)"
            "           particles[gid].velocity.x += horizontal_resistance * dt;"
            "       if (particles[gid].velocity.z > 0.f)"
            "           particles[gid].velocity.z -= horizontal_resistance * dt;"
            "       else if (particles[gid].velocity.z < 0.f)"
            "           particles[gid].velocity.z += horizontal_resistance * dt;"
            "       particles[gid].position.xyz += particles[gid].velocity.xyz * dt;"
            "   }"
            "}"
            ""
            "void dead()"
            "{"
            "   particles[gid].flag.y = 0.f;"
            "}"
            ""
            "void main()"
            "{"
            "   if (gid >= n_particles) {return;}"
            "   if (dt < 0.f)"
            "       spawn();"
            "   else if (particles[gid].flag.x == 0.f && particles[gid].velocity.y < 0.f)"
            "       explode();"
            "   else"
            "       update();"
            "}"
            "";

    const char *VS = "#version 420 core\n"
            "layout(location = 5) in vec2 vertex;"
            "layout(location = 0) in vec3 position;"
            "layout(location = 1) in float size;"
            "layout(location = 2) in float alpha;"
            ""
            "layout (std140, binding = 0) uniform GlobalAttributes"
            "{"
            "   mat4 view;"
            "   mat4 proj;"
            "   vec3 cam_pos;"
            "};"
            ""
            "out float gAlpha;"
            ""
            "void main()"
            "{"
            "   mat4 VP = proj * view;"
            ""
            "   vec3 right_vec = vec3(view[0][0], view[1][0], view[2][0]) * size;"
            "   vec3 up_vec = vec3(view[0][1], view[1][1], view[2][1]) * size;"
            "   gl_Position = VP * vec4(position"
            "	        + (right_vec * vertex.x - up_vec * vertex.y), 1.f);"
            "   gAlpha = alpha;"
            "}";

    const char *FS = "#version 330 core\n"
            "in float gAlpha;"
            "uniform vec3 color;"
            ""
            "out vec4 gColor;"
            ""
            "void main(){"
            "   gColor = vec4(color, gAlpha);"
            "}";

    impl() : vao(0), vertex_vbo(0), vbo(0),
             compute_shader(nullptr), draw_shader(nullptr)
    {}
    ~impl()
    {
        clearGLObjs();

        delete compute_shader;
        delete draw_shader;
    }

    void clearGLObjs()
    {
        glDeleteVertexArrays(1, &vao);
        glDeleteBuffers(1, &vbo);
        glDeleteBuffers(1, &vertex_vbo);

        vao = 0;
        vbo = 0;
        vertex_vbo = 0;
    }

    void genGLObjs()
    {
        glDeleteVertexArrays(1, &vao);
        glDeleteBuffers(1, &vbo);
        glDeleteBuffers(1, &vertex_vbo);

        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);
        glGenBuffers(1, &vertex_vbo);
    }

};

scene::FireworksScene::ComputeShaderParticleSystem::ComputeShaderParticleSystem()
        : ParticleSystem()
{
    pimpl = std::unique_ptr<impl>(new impl);
}

scene::FireworksScene::ComputeShaderParticleSystem::~ComputeShaderParticleSystem()
{}

void scene::FireworksScene::ComputeShaderParticleSystem::init(float *, unsigned int v_count,
                                                           unsigned int tex, float *uv, bool atlas)
{
    if (pimpl->compute_shader == nullptr)
        pimpl->compute_shader = new Shader(pimpl->CS);
    if (pimpl->draw_shader == nullptr)
        pimpl->draw_shader = new Shader(pimpl->VS, pimpl->FS);

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

    pimpl->draw_shader->activate();
    pimpl->draw_shader->bind("GlobalAttributes", 0);
    glBindVertexArray(pimpl->vao);
    glBindBuffer(GL_ARRAY_BUFFER, pimpl->vertex_vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float)*vertex.size(), vertex.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(5, 2, GL_FLOAT, GL_FALSE, 0, (void *)0);
    glEnableVertexAttribArray(5);
    glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo);
    glEnableVertexAttribArray(0);   // position
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 12*sizeof(float), (void *)(0));
    glEnableVertexAttribArray(1);   // size
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 12*sizeof(float), (void *)(3*sizeof(float)));
    glEnableVertexAttribArray(2);   // alpha
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, 12*sizeof(float), (void *)(7*sizeof(float)));
    pimpl->draw_shader->activate(false);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    pimpl->n_vertices = vertex.size()/2;

}

void scene::FireworksScene::ComputeShaderParticleSystem::restart()
{
    pimpl->tot = max_particles;
    pimpl->debt = 0.f;
    pimpl->time = 0.f;
    pimpl->upload = true;
}

void scene::FireworksScene::ComputeShaderParticleSystem::upload()
{
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, pimpl->vbo);
    if (!pimpl->upload) return;

    glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float)*12*max_particles, nullptr, GL_STATIC_DRAW);

    n_particles = max_particles;
    pimpl->compute_shader->activate();
    pimpl->compute_shader->set("dt", -1.f);
    pimpl->compute_shader->set("gravity", glm::vec3(rnd_np()*2.f, 0.f, rnd_np()*2.f)); // initial position
    pimpl->compute_shader->set("acceleration", glm::vec3(rnd_np()*2, 7.5f + rnd(), rnd_np()*2)); // initial velocity
    pimpl->compute_shader->set("n_particles", static_cast<int>(total()));
    glDispatchCompute(cuda::blocks(total(), COMPUTE_SHADER_WORK_GROUP_SIZE), 1, 1);
    pimpl->compute_shader->set("gravity", glm::vec3(0.f, -5.f, 0.f)); // restore gravity
    pimpl->compute_shader->set("acceleration", glm::vec3(0.f));
    pimpl->compute_shader->activate(false);

    pimpl->draw_shader->activate();
    pimpl->draw_shader->set("color", glm::vec3(rnd(), rnd(), rnd()));

    pimpl->trail = 0;
    pimpl->debt_particles = 0.f;
    pimpl->trail_rate = FIREWORKS_TRAIL_RATE;
    pimpl->end_time = 3.f + rnd2();
    pimpl->upload = false;
}


void scene::FireworksScene::ComputeShaderParticleSystem::update(float dt, glm::vec3 *cam_pos)
{
    pimpl->compute_shader->activate();
    pimpl->compute_shader->set("dt", dt);

    pimpl->debt_particles += std::min(0.05f, dt) * pimpl->trail_rate;
    auto new_particles = static_cast<int>(pimpl->debt_particles);
    pimpl->debt_particles -= new_particles;
    pimpl->trail += new_particles;

    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, pimpl->vbo);
    pimpl->compute_shader->set("trail", pimpl->trail);
    pimpl->compute_shader->set("n_particles", static_cast<int>(count()));
    glDispatchCompute(cuda::blocks(count(), COMPUTE_SHADER_WORK_GROUP_SIZE), 1, 1);
    pimpl->compute_shader->activate(false);

    pimpl->time += dt;
    if (pimpl->time > pimpl->end_time)
        restart();
}

void scene::FireworksScene::ComputeShaderParticleSystem::render(GLenum gl_draw_mode)
{
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);
    pimpl->draw_shader->activate();
    glBindVertexArray(pimpl->vao);
    glVertexAttribDivisor(5, 0); // vertex             a group for each instance
    glVertexAttribDivisor(0, 1); // position           one for each instance
    glVertexAttribDivisor(1, 1); // size
    glVertexAttribDivisor(2, 1); // alpha
    glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, pimpl->n_vertices, count());

    glBindTexture(GL_TEXTURE_2D, 0);
    glBindVertexArray(0);
    pimpl->draw_shader->activate(false);
    glDisable(GL_BLEND);


//    std::cout << "("
//              << App::instance()->scene.cam.pos().x << ", "
//              << App::instance()->scene.cam.pos().y << ", "
//              << App::instance()->scene.cam.pos().z << ") "
//              << App::instance()->scene.cam.yaw() << ", "
//              << App::instance()->scene.cam.pitch() << std::endl;
}


unsigned int const &scene::FireworksScene::ComputeShaderParticleSystem::count() const noexcept
{
    return n_particles;
}
unsigned int scene::FireworksScene::ComputeShaderParticleSystem::total() const noexcept
{
    return pimpl->tot;
}

scene::FireworksScene::FireworksScene()
        : BaseScene(),
          system_name("Fireworks Demo"),
          rendering_mode("Multiple Particle Systems with compute shader")
{
    systems.reserve(FIREWORKS_AMOUNT*5);
    for (auto i = 0; i < FIREWORKS_AMOUNT; ++i)
    {
        systems.emplace_back(new ComputeShaderParticleSystem);
        systems.back()->max_particles = FIREWORKS_MAX_PARTICLES;
        systems.back()->birth_rate    = systems.back()->max_particles * 1000;
    }
}

scene::FireworksScene::~FireworksScene()
{
    for (auto s : systems)
        delete s;
}

void scene::FireworksScene::init(Scene &scene)
{
    for (auto s : systems)
        s->init(nullptr, 0);
}

void scene::FireworksScene::restart(Scene &scene)
{
    resetCamera();

    for (auto s : systems)
        s->restart();

    pause = false;
}

void scene::FireworksScene::upload(Shader &scene_shader)
{
    glClearColor(0.f, 0.f, 0.f, 1.f);
    for (auto s : systems)
        s->upload();
}

void scene::FireworksScene::update(float dt)
{
    if (!pause)
    {
        for (auto s : systems)
            s->update(dt);
    }

    processInput(dt);
}

void scene::FireworksScene::render()
{
    for (auto s : systems)
        s->render();
    renderInfo();
}

void scene::FireworksScene::resetCamera()
{
    App::instance()->scene.character.reset(-5.f, 10.f, -5.f, 135.f, 35.f);
    App::instance()->scene.character.setShootable(false);
    App::instance()->scene.character.setFloating(true);
}

void scene::FireworksScene::renderInfo()
{
    App::instance()->text(system_name,
                          10, 10, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::LeftTop);
    App::instance()->text("Particles: " + std::to_string(systems[0]->total()) + " x " + std::to_string(systems.size()),
                          10, 30, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::LeftTop);
    App::instance()->text("Rendering Mode: " + rendering_mode,
                          10, 50, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::LeftTop);

    auto h = App::instance()->frameHeight() - 25;
    App::instance()->text("Press Up and Down Arrow to change the number of fireworks",
                          App::instance()->frameWidth() - 10, h - 20, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::RightTop);
    App::instance()->text("Press Z to increase particles; Press X to decrease particles",
                          App::instance()->frameWidth() - 10, h, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::RightTop);
}

void scene::FireworksScene::processInput(float dt)
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
    for (auto s : systems)                                               \
    {                                                                   \
        s->max_particles += 1000;                                       \
        s->birth_rate = s->max_particles * .1f;                         \
    }
#define DECREASE_PARTICLES                                                  \
    if (systems[0]->max_particles > 1000)                              \
    {                                                                   \
        for (auto s : systems)                                               \
        {                                                                  \
            s->max_particles -= 1000;                             \
            s->birth_rate = s->max_particles * .1f; \
        }                                                           \
    }

    STICKY_KEY_CHECK(GLFW_KEY_Z, INCREASE_PARTICLES)
    else
    STICKY_KEY_CHECK(GLFW_KEY_X, DECREASE_PARTICLES)
    else if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS)
        last_key = GLFW_KEY_P;
    else if (glfwGetKey(window, GLFW_KEY_B) == GLFW_PRESS)
        last_key = GLFW_KEY_B;
    else if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
        last_key = GLFW_KEY_UP;
    else if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)
        last_key = GLFW_KEY_DOWN;
    else
    {
        if (last_key == GLFW_KEY_B)
            resetCamera();
        else if (last_key == GLFW_KEY_P)
            pause = !pause;
        else if (last_key == GLFW_KEY_UP)
        {
            systems.emplace_back(new ComputeShaderParticleSystem);
            systems.back()->max_particles = systems[0]->max_particles;
            systems.back()->birth_rate = systems[0]->birth_rate;
            systems.back()->init(nullptr, 0);
            systems.back()->restart();
        }
        else if (last_key == GLFW_KEY_DOWN)
        {
            if (systems.size() > 1)
            {
                delete systems.back();
                systems.pop_back();
            }
        }
        last_key = GLFW_KEY_UNKNOWN;
    }

#undef HOLD_KEY
}
