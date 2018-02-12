#include "scene/snow_scene.hpp"
#include "config.h"
#include "app.hpp"
#include "util/cuda.hpp"
#include "global.hpp"

#include "stb_image.hpp"

#include <sstream>
#include <iomanip>

using namespace px;


class scene::SnowScene::BrownianMotionParticleSystem::impl
{
public:
    unsigned int vao, vbo, texture;
    unsigned int n_vertices;

    float debt;
    unsigned int tot;
    bool upload;

    Shader *compute_shader, *draw_shader;

    const char *CS = "#version 430 core\n"
            "layout (local_size_x = " STR(COMPUTE_SHADER_WORK_GROUP_SIZE) ", local_size_y = 1, local_size_z = 1) in;"
            ""
            "layout(std140, binding = 0) buffer Particle_t"
            "{"
            "   struct"
            "   {"
            "       vec4 position;" // position,  size
            "       vec4 delta_position;" // delta_position, alpha
            "       vec4 brownian;" // brownian, life
            "   } particles[];"
            "};"
            "uniform float dt;"
            "uniform int n_particles;"
            ""
            "uniform float top_height = 7.5f;"
            "uniform float spawn_radius = 8.f;"
            "uniform vec4 field_bound = vec4(-8.f, 8.f, -8.f, 8.f);"
            "uniform float field_height = 0.f;"
            "uniform vec3 gravity = vec3(0.f, -2.f, 0.f);"
            "uniform float brownian_motion = .05f;"
            "uniform vec3 acceleration;"
            ""
            "uint gid = gl_GlobalInvocationID.x;"
            ""
            "float rnd(vec2 co)"
            "{"
            "   return fract(sin(mod(dot(co.xy,vec2(12.9898,78.233)), 3.14)) * 43758.5453);"
            "}"
            ""
            "void spawn(vec2 sd)"
            "{"
            "   particles[gid].position.x = spawn_radius * (1 - 2 * rnd(sd)); ++sd.x;"
            "   particles[gid].position.y = top_height + rnd(sd); ++sd.x;"
            "   particles[gid].position.z = spawn_radius * (1 - 2 * rnd(sd)); ++sd.x;"
            "   particles[gid].position.w = 0.075 + rnd(sd) * 0.025; ++sd.x;"
            "   particles[gid].delta_position.x = 0.f;"
            "   particles[gid].delta_position.y = 0.f;"
            "   particles[gid].delta_position.z = 0.f;"
            "   particles[gid].delta_position.w = 1.f;"
            "   particles[gid].brownian = vec4(0.f, 0.f, 0.f, 1 + rnd(sd));"
            "}"
            ""
            "void update(vec2 sd)"
            "{"
            "   bool above_field = false;"
            "   if (particles[gid].position.y >= field_height)"
            "       above_field = true;"
            ""
            "   particles[gid].delta_position.xyz = gravity * dt;"
            "   particles[gid].position.xyz += particles[gid].delta_position.xyz;"
            ""
            "   if (particles[gid].position.y < field_height)"
            "   {"
            "       particles[gid].delta_position.w -= dt / particles[gid].brownian.w;"
            "       particles[gid].position.w += .1 * dt;"
            "       if (particles[gid].position.x > field_bound[0] && particles[gid].position.x < field_bound[1] &&"
            "          particles[gid].position.z > field_bound[2] && particles[gid].position.z < field_bound[3])"
            "       {"
            "           particles[gid].delta_position.y += field_height - particles[gid].position.y;"
            "           particles[gid].position.xyz += particles[gid].delta_position.xyz;"
            "           return;"
            "       }"
            "   }"
            "   particles[gid].delta_position.xyz = acceleration * dt;"
            "   particles[gid].position.xyz += particles[gid].delta_position.xyz;"
            ""
//            "   particles[gid].brownian.x += (rnd(sd) - .5f) * .0004f; ++sd.x;"
//            "   particles[gid].brownian.y += (rnd(sd) - .5f) * .0004f; ++sd.x;"
//            "   particles[gid].brownian.z += (rnd(sd) - .5f) * .0004f; ++sd.x;"
//            "   particles[gid].brownian.xyz = normalize(particles[gid].brownian.xyz);"
//            "   particles[gid].brownian.xyz *= brownian_motion;"

            "   particles[gid].brownian.x += (2*rnd(sd) - 1) * dt * brownian_motion;  ++sd.x;"  // should use normal random number
            "   particles[gid].brownian.y += (2*rnd(sd) - 1) * dt * brownian_motion;  ++sd.x;"
            "   particles[gid].brownian.z += (2*rnd(sd) - 1) * dt * brownian_motion;  ++sd.x;"
            "   particles[gid].delta_position.xyz += particles[gid].brownian.xyz;"
            "   particles[gid].position.xyz += particles[gid].brownian.xyz;"
            "}"
            ""
            "void main()"
            "{"
            "   if (gid >= n_particles) {return;}"
            "   if (particles[gid].delta_position.w < 0.f || dt < 0.f)"
            "   {"
            "       spawn(vec2(gid + particles[gid].position.x, gid + particles[gid].position.y));"
            "   }"
            "   else"
            "   {"
            "       update(vec2(gid + particles[gid].position.x, gid + particles[gid].position.y));"
            "   }"
            ""
            "}";

    const char *VS = "#version 420 core\n"
            "layout(location = 0) in vec4 position;"
            "layout(location = 1) in vec3 delta_position;"
            "layout(location = 2) in float alpha;"
            ""
            "out VS_OUT"
            "{"
            "   vec3 delta_position;"
            "   float alpha;"
            "} primitive;"
            ""
            "void main()"
            "{"
            "   gl_Position = position;"
            "   primitive.delta_position = delta_position;"
            "   primitive.alpha = alpha;"
            "}";
    const char *GS = "#version 420 core\n"
            "layout (points) in;"
            "layout (triangle_strip, max_vertices = 4) out;"
            ""
            "layout (std140, binding = 0) uniform GlobalAttributes"
            "{"
            "   mat4 view;"
            "   mat4 proj;"
            "   vec3 cam_pos;"
            "};"
            ""
            "in VS_OUT"
            "{"
            "   vec3 delta_position;"
            "   float alpha;"
            "} primitive[];"
            ""
            "out vec2 gTextureCoord;"
            "out float gAlpha;"
            ""
            "void main()"
            "{"
            "   mat4 VP = proj * view;"
            ""
            "   vec3 u = mat3(view) * primitive[0].delta_position;" // movement in view
            "   float w = gl_in[0].gl_Position.w;" // half width
            "   float h = w;"                // half height
            "   float t = 0;"
            "   float nz = abs(normalize(u).z);"
            "   if (nz > 1.f - 1e-7f)"                        // the more the delta position aligns with Z axis
            "       t = (nz - (1.f - 1e-7f)) / 1e-7f;"        // the more t will close to 1 such that h will close to w
            "   else if (dot(u, u) < 1e-7f)"
            "       t = (1e-7f - dot(u,u)) / 1e-7f;"
            "   u.z = 0.f;"
            "   u = normalize(mix(normalize(u), vec3(1.f,0.f,0.f), t));"
            "   h = mix(h, w, t);"
            ""
            "   vec3 v = vec3(-u.y, u.x, 0.f);"
            "   vec3 a = u * mat3(view);"
            "   vec3 b = v * mat3(view);"
            "   vec3 c = cross(a, b);"
            "   mat3 basis = mat3(a, b, c);"
            ""
            "   vec3 right_vec = basis * vec3(0.f, w, 0.f);"
            "   vec3 up_vec = basis * vec3(h, 0.f, 0.f);"
            ""
//            "   vec3 up_vec = vec3(view[0][0], view[1][0], view[2][0]) * gl_in[0].gl_Position.w;"
//            "   vec3 right_vec = vec3(view[0][1], view[1][1], view[2][1]) * gl_in[0].gl_Position.w;"
            ""
            "   gAlpha = primitive[0].alpha;"
            ""
            "   gTextureCoord = vec2(0, 0);"
            "   gl_Position = VP * vec4(gl_in[0].gl_Position.xyz"
            "	        + (-right_vec - up_vec), 1.f);"
            "   EmitVertex();"
            "   gTextureCoord = vec2(1, 0);"
            "   gl_Position = VP * vec4(gl_in[0].gl_Position.xyz"
            "	        + (right_vec - up_vec), 1.f);"
            "   EmitVertex();"
            "   gTextureCoord = vec2(0, 1);"
            "   gl_Position = VP * vec4(gl_in[0].gl_Position.xyz"
            "	        + (-right_vec + up_vec), 1.f);"
            "   EmitVertex();"
            "   gTextureCoord = vec2(1, 1);"
            "   gl_Position = VP * vec4(gl_in[0].gl_Position.xyz"
            "	        + (right_vec + up_vec), 1.f);"
            "   EmitVertex();"
            ""
            "   EndPrimitive();"
            "}";
    const char *FS = "#version 330 core\n"
            "in vec2 gTextureCoord;"
            "in float gAlpha;"
            ""
            "uniform sampler2D sprite;"
            ""
            "out vec4 color;"
            ""
            "void main(){"
            "   color = vec4(1.f, 1.f, 1.f, gAlpha * texture(sprite, gTextureCoord).r);"
            "}";

    impl() : vao(0), vbo(0), texture(0),
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
        glDeleteTextures(1, &texture);

        vao = 0;
        vbo = 0;
        texture = 0;
    }

    void genGLObjs()
    {
        glDeleteVertexArrays(1, &vao);
        glDeleteBuffers(1, &vbo);
        glDeleteTextures(1, &texture);

        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);
        glGenTextures(1, &texture);
    }

};

scene::SnowScene::BrownianMotionParticleSystem::BrownianMotionParticleSystem()
        : ParticleSystem()
{
    pimpl = std::unique_ptr<impl>(new impl);
}

scene::SnowScene::BrownianMotionParticleSystem::~BrownianMotionParticleSystem()
{}

void scene::SnowScene::BrownianMotionParticleSystem::init(float *, unsigned int v_count,
                                                     unsigned int tex, float *uv, bool atlas)
{
    if (pimpl->compute_shader == nullptr)
        pimpl->compute_shader = new Shader(pimpl->CS);
    if (pimpl->draw_shader == nullptr)
        pimpl->draw_shader = new Shader(pimpl->VS, pimpl->FS, pimpl->GS);

    pimpl->genGLObjs();

    pimpl->draw_shader->activate();
    pimpl->draw_shader->bind("GlobalAttributes", 0);
    pimpl->draw_shader->set("sprite", 0);
    glBindVertexArray(pimpl->vao);
    glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo);
    glEnableVertexAttribArray(0);   // position + size
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 12*sizeof(float), (void *)(0));
    glEnableVertexAttribArray(1);   // delta_position
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 12*sizeof(float), (void *)(4*sizeof(float)));
    glEnableVertexAttribArray(2);   // alpha
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, 12*sizeof(float), (void *)(7*sizeof(float)));
    pimpl->draw_shader->activate(false);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    int w, h, ch;
    auto ptr = stbi_load(ASSET_PATH "/texture/particle3.png", &w, &h, &ch, 3);
    TEXTURE_LOAD_HELPER(pimpl->texture, GL_RGB, GL_REPEAT, GL_LINEAR, w, h, ptr);
    stbi_image_free(ptr);

}

void scene::SnowScene::BrownianMotionParticleSystem::restart()
{
    pimpl->tot = max_particles;
    pimpl->debt = 0.f;
    n_particles = 0.f;
    pimpl->upload = true;
}

void scene::SnowScene::BrownianMotionParticleSystem::upload()
{
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, pimpl->vbo);
    if (!pimpl->upload) return;

    glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float)*12*max_particles, nullptr, GL_STATIC_DRAW);

    pimpl->compute_shader->activate();
    pimpl->compute_shader->set("dt", -1.f);
    pimpl->compute_shader->set("n_particles", static_cast<int>(total()));
    glDispatchCompute(cuda::blocks(total(), COMPUTE_SHADER_WORK_GROUP_SIZE), 1, 1);
    pimpl->compute_shader->activate(false);

    pimpl->upload = false;
}


void scene::SnowScene::BrownianMotionParticleSystem::update(float dt, glm::vec3 *cam_pos)
{
    if (n_particles != total())
    {
        pimpl->debt += std::min(0.05f, dt) * birth_rate;
        auto new_particles = static_cast<int>(pimpl->debt);
        pimpl->debt -= new_particles;
        n_particles += new_particles;
        if (n_particles > total()) n_particles = total();
    }

    pimpl->compute_shader->activate();
    pimpl->compute_shader->set("dt", dt);
    pimpl->compute_shader->set("acceleration", acceleration);
    pimpl->compute_shader->set("n_particles", static_cast<int>(count()));
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, pimpl->vbo);
    glDispatchCompute(cuda::blocks(count(), COMPUTE_SHADER_WORK_GROUP_SIZE), 1, 1);
    pimpl->compute_shader->activate(false);
}

void scene::SnowScene::BrownianMotionParticleSystem::render(GLenum gl_draw_mode)
{
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);
    pimpl->draw_shader->activate();
    glBindVertexArray(pimpl->vao);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, pimpl->texture);
    glDrawArrays(GL_POINTS, 0, count());

    glBindTexture(GL_TEXTURE_2D, 0);
    glBindVertexArray(0);
    pimpl->draw_shader->activate(false);
    glDisable(GL_BLEND);
}

unsigned int scene::SnowScene::BrownianMotionParticleSystem::total() const noexcept
{
    return pimpl->tot;
}

scene::SnowScene::SnowScene()
        : BaseScene(),
          system_name("Snow Demo"),
          rendering_mode("Compute and Geometry Shader"),
          particle_system(nullptr)
{
    particle_system = new BrownianMotionParticleSystem;
    particle_system->max_particles = SNOW_MAX_PARTICLES;
    particle_system->birth_rate    = SNOW_MAX_PARTICLES*.15;
}

scene::SnowScene::~SnowScene()
{
    delete particle_system;
}

void scene::SnowScene::init(Scene &scene)
{
    particle_system->init(nullptr, 0);
}

void scene::SnowScene::restart(Scene &scene)
{
    resetCamera();
    pause = false;
    wind = glm::vec3(0.f);
    particle_system->restart();
}

void scene::SnowScene::upload(Shader &scene_shader)
{
    glClearColor(0.f, 0.f, 0.f, 1.f);
    particle_system->upload();
}

void scene::SnowScene::update(float dt)
{
    if (!pause)
    {
        particle_system->acceleration = wind;
        particle_system->update(dt);
    }
    processInput(dt);
}

void scene::SnowScene::render()
{
    particle_system->render();

    renderInfo();
}

void scene::SnowScene::resetCamera()
{
    App::instance()->scene.character.reset(0.f, 3.f, -12.f, 180.f, 10.f);
    App::instance()->scene.character.setShootable(false);
    App::instance()->scene.character.setFloating(true);
}

void scene::SnowScene::renderInfo()
{
    App::instance()->text(system_name,
                          10, 10, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::LeftTop);
    App::instance()->text("Particles: " + std::to_string(particle_system->count()) + "/" + std::to_string(particle_system->total()),
                          10, 30, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::LeftTop);
    App::instance()->text("Rendering Mode: " + rendering_mode,
                          10, 50, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::LeftTop);
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << wind.x << ", " << wind.z << std::endl;
    App::instance()->text("Wind: " + ss.str(),
                          10, 70, .4f,
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
    App::instance()->text("Press Z to increase particles; Press X to decrease particles",
                          App::instance()->frameWidth() - 10, h, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::RightTop);
    App::instance()->text("Press Left and Right Arrow to adjust wind along X-axis",
                          App::instance()->frameWidth() - 10, h - 20, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::RightTop);
    App::instance()->text("Press Up and Down Arrow to adjust wind along Z-axis",
                          App::instance()->frameWidth() - 10, h - 40, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::RightTop);
    App::instance()->text("Press M to reset wind",
                          App::instance()->frameWidth() - 10, h - 60, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::RightTop);
}

void scene::SnowScene::processInput(float dt)
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
    particle_system->birth_rate = particle_system->max_particles * .15f;
#define DECREASE_PARTICLES                                                  \
    if (particle_system->max_particles > 1000)                              \
    {                                                                       \
        particle_system->max_particles -= 1000;                             \
        particle_system->birth_rate = particle_system->max_particles * .15f; \
    }
#define INCREASE_WIND_X wind.x -= .1f;
#define DECREASE_WIND_X wind.x += .1f;
#define INCREASE_WIND_Z wind.z += .1f;
#define DECREASE_WIND_Z wind.z -= .1f;

    STICKY_KEY_CHECK(GLFW_KEY_Z, INCREASE_PARTICLES)
    else
    STICKY_KEY_CHECK(GLFW_KEY_X, DECREASE_PARTICLES)
    else
    STICKY_KEY_CHECK(GLFW_KEY_RIGHT, INCREASE_WIND_X)
    else
    STICKY_KEY_CHECK(GLFW_KEY_LEFT, DECREASE_WIND_X)
    else
    STICKY_KEY_CHECK(GLFW_KEY_UP, INCREASE_WIND_Z)
    else
    STICKY_KEY_CHECK(GLFW_KEY_DOWN, DECREASE_WIND_Z)
    else if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS)
        last_key = GLFW_KEY_P;
    else if (glfwGetKey(window, GLFW_KEY_B) == GLFW_PRESS)
        last_key = GLFW_KEY_B;
    else if (glfwGetKey(window, GLFW_KEY_M) == GLFW_PRESS)
        last_key = GLFW_KEY_M;
    else
    {
        if (last_key == GLFW_KEY_B)
            resetCamera();
        else if (last_key == GLFW_KEY_P)
            pause = !pause;
        else if (last_key == GLFW_KEY_M)
            wind = glm::vec3(0.f);
        last_key = GLFW_KEY_UNKNOWN;
    }
}
