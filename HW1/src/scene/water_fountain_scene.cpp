#include "scene/water_fountain_scene.hpp"
#include "scene.hpp"
#include "util/random.hpp"
#include "app.hpp"
#include "global.hpp"
#include "config.hpp"

#include "stb_image.hpp"

#include <cstring>
#include <glm/gtx/norm.hpp>
#include <iomanip>
#include <sstream>

using namespace px;

class scene::WaterFountainScene::ComputeShaderParticleSystem::impl
{
public:
    unsigned int vao, vbo, texture;

    unsigned int debt, tot;
    bool upload;

    Shader *compute_shader;
    Shader *draw_shader;

    const char *CS = "#version 430 core\n"
            "layout (local_size_x = " STR(COMPUTE_SHADER_WORK_GROUP_SIZE) ", local_size_y = 1, local_size_z = 1) in;"
            ""
            "layout(std140, binding = 0) buffer Particle_t"
            "{"
            "   struct"
            "   {"
            "       vec4 position;"
            "       vec4 velocity;"
            "   } particles [];"
            "};"
            ""
            "uniform float dt;"
            ""
            "const float PI = 3.1415926535897932384626433832795;"
            "const float PI_8 = PI / 8;"
            ""
            "uniform float field_height = -2.f;"
            "uniform vec4 field_bound = vec4(-8.f, 8.f, -8.f, 8.f);"
            "uniform vec3 field_norm = vec3(0.f, 1.f, 0.f);"
            "uniform float vertical_gravity = 10.f;"
            "uniform float horizontal_resistance = .05f;"
            "uniform vec3 acceleration = vec3(0.f, 0.f, 0.f);"
            ""
            "uint gid = gl_GlobalInvocationID.x + gl_GlobalInvocationID.y*gl_NumWorkGroups.x * gl_WorkGroupSize.x;"
            ""
            "float rnd(vec2 co)"
            "{"
            "   return fract(sin(mod(dot(co.xy,vec2(12.9898,78.233)), 3.14)) * 43758.5453);"
            "}"
            ""
            "void spawn(vec2 sd)"
            "{"
            "   float theta = PI * rnd(sd) * 2.f; ++sd.x;"
            "   float phi = PI_8 * (rnd(sd) * 2.f - 1.f);    ++sd.x;"
            "   float r0 = pow(.02f + rnd(sd)*.1f, 1.f/3.f); ++sd.x;"
            "   particles[gid].position.x = r0 * sin(phi) * cos(theta);"
            "   particles[gid].position.y = r0 * cos(phi);"
            "   particles[gid].position.z = r0 * sin(phi) * sin(theta);"
            "   particles[gid].velocity.xyz = 5.f * normalize(vec3(particles[gid].position.x, particles[gid].position.y*.2f, particles[gid].position.z));"
            "   particles[gid].velocity.w = 3.f + (2*rnd(sd) - 1.f);" // life
            "   particles[gid].position.w = 1.f;" // alpha, life fraction
            "}"
            ""
            "void update(vec2 sd)"
            "{"
            "   particles[gid].velocity.xyz += acceleration * dt;"
            "   particles[gid].velocity.y -= vertical_gravity * dt;"
            ""
            "   particles[gid].position.xyz += particles[gid].velocity.xyz * dt;"
            ""
            "   if (particles[gid].position.y < field_height && "
            "       particles[gid].position.x > field_bound[0] && particles[gid].position.x < field_bound[1] &&"
            "       particles[gid].position.z > field_bound[2] && particles[gid].position.z < field_bound[3])"
            "   {"
            "       particles[gid].position.y = field_height;"
            "       particles[gid].velocity.xyz = reflect(particles[gid].velocity.xyz, field_norm);"
            "       particles[gid].velocity.y *= .25f * (rnd(sd) + 1);"
            ""
            "       "
            "       if (particles[gid].position.x * particles[gid].position.x + "
            "           particles[gid].position.z * particles[gid].position.z > 4f)"
            "       {"
            "           if (particles[gid].velocity.x > 0.f)"
            "               particles[gid].velocity.x -= horizontal_resistance * dt;"
            "           else if (particles[gid].velocity.x < 0.f)"
            "               particles[gid].velocity.x += horizontal_resistance * dt;"
            "           if (particles[gid].velocity.z > 0.f)"
            "               particles[gid].velocity.z -= horizontal_resistance * dt;"
            "           else if (particles[gid].velocity.z < 0.f)"
            "               particles[gid].velocity.z += horizontal_resistance * dt;"
            "       }"
            "   }"
            "}"
            ""
            "void main()"
            "{"
            "       if (dt < 0.f)"
            "       {"
            "           spawn(vec2(gid, gid));"
            "       }"
            "       else"
            "       {"
            "           vec2 sd = particles[gid].position.xy + dt;"
            "           particles[gid].velocity.w -= dt;"
            "           if (particles[gid].velocity.w > 0.f)"
            "               update(sd);"
            "           else"
            "               spawn(sd);"
            "       }"
            "}";
    const char *VS = "#version 420 core\n"
            ""
            "layout(location = 0) in vec3 pos;"
            "layout(location = 1) in float life;"
            ""
            "out VS_OUT"
            "{"
            "   float alpha;"
            "} primitive;"
            ""
            "void main()"
            "{"
            "	gl_Position = vec4(pos, 1.0f);"
            ""
            "	primitive.alpha = life;"
            "}";
    const char *GS = "#version 420 core\n"
            "layout (points) in;"
            "layout (triangle_strip, max_vertices = 4) out;"
            ""
            "in VS_OUT"
            "{"
            "   float alpha;"
            "} primitive[];"
            ""
            "layout (std140, binding = 0) uniform GlobalAttributes"
            "{"
            "   mat4 view;"
            "   mat4 proj;"
            "   vec3 cam_pos;"
            "};"
            ""
            "out vec2 gTextureCoord;"
            "out float gAlpha;"
            ""
            "void main()"
            "{"
            "   mat4 VP = proj * view;"
            ""
            "   vec3 right_vec = vec3(view[0][0], view[1][0], view[2][0]) * .025f;"
            "   vec3 up_vec = vec3(view[0][1], view[1][1], view[2][1]) * .025f;"
            ""
            "   gAlpha = primitive[0].alpha;"
            ""
            "   gTextureCoord = vec2(0, 0);"
            "   gl_Position = VP * vec4(gl_in[0].gl_Position.xyz"
            "	        + (-right_vec - up_vec), 1.f);"
            "   EmitVertex();"
            "   gTextureCoord = vec2(1, 0);"
            "   gl_Position = VP * vec4(gl_in[0].gl_Position.xyz"
            "	        + (right_vec -  up_vec), 1.f);"
            "   EmitVertex();"
            "   gTextureCoord = vec2(0, 1);"
            "   gl_Position = VP * vec4(gl_in[0].gl_Position.xyz"
            "	        + (-right_vec +  up_vec), 1.f);"
            "   EmitVertex();"
            "   gTextureCoord = vec2(1, 1);"
            "   gl_Position = VP * vec4(gl_in[0].gl_Position.xyz"
            "	        + (right_vec +  up_vec), 1.f);"
            "   EmitVertex();"
            ""
            "   EndPrimitive();"
            "}";

    const char *FS = "#version 330 core\n"
            "in float gAlpha;"
            "in vec2 gTextureCoord;"
            ""
            "uniform sampler2D sprite;"
            ""
            "out vec4 color;"
            ""
            "void main(){"
            "   color = vec4(0.2588f, 0.5255f, 0.9569f, gAlpha * texture(sprite, gTextureCoord).r);"
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

scene::WaterFountainScene::ComputeShaderParticleSystem::ComputeShaderParticleSystem()
        : ParticleSystem()
{
    pimpl = std::unique_ptr<impl>(new impl);
}

scene::WaterFountainScene::ComputeShaderParticleSystem::~ComputeShaderParticleSystem()
{}

void scene::WaterFountainScene::ComputeShaderParticleSystem::init(float *vertex, unsigned int v_count,
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
    glEnableVertexAttribArray(0);   // position
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8*sizeof(float), (void *)(0));
    glEnableVertexAttribArray(1);   // color alpha
    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 8*sizeof(float), (void *)(3*sizeof(float)));
    pimpl->draw_shader->activate(false);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    {
        int w, h, ch;
        auto ptr = stbi_load(ASSET_PATH "/texture/particle3.png", &w, &h, &ch, 3);
        TEXTURE_LOAD_HELPER(pimpl->texture, GL_RGB, GL_REPEAT, GL_LINEAR, w, h, ptr);
        stbi_image_free(ptr);
    }
}

void scene::WaterFountainScene::ComputeShaderParticleSystem::restart()
{
    pimpl->tot = max_particles;
    pimpl->debt = 0.f;

    pimpl->upload = true;
}

void scene::WaterFountainScene::ComputeShaderParticleSystem::upload()
{
    if (!pimpl->upload) return;

    glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float)*8*max_particles, nullptr, GL_STATIC_DRAW);

    n_particles = max_particles;
    update(-1.f);
    n_particles = 0;

    pimpl->upload = false;
}


void scene::WaterFountainScene::ComputeShaderParticleSystem::update(float dt, glm::vec3 *cam_pos)
{
    if (n_particles != total())
    {
        pimpl->debt += std::min(0.05f, dt) * birth_rate;
        auto new_particles = static_cast<int>(pimpl->debt);
        pimpl->debt -= new_particles;
        n_particles += new_particles;
        if (n_particles > total()) n_particles = total();
    }

    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, pimpl->vbo);
    pimpl->compute_shader->activate();
    pimpl->compute_shader->set("dt", dt);
    pimpl->compute_shader->set("acceleration", acceleration);
    glDispatchCompute(count()/COMPUTE_SHADER_WORK_GROUP_SIZE, 1, 1);
    pimpl->compute_shader->activate(false);
}

void scene::WaterFountainScene::ComputeShaderParticleSystem::render(GLenum gl_draw_mode)
{
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
//    glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ZERO, GL_ONE_MINUS_SRC_ALPHA);
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


unsigned int const &scene::WaterFountainScene::ComputeShaderParticleSystem::count() const noexcept
{
    return n_particles;
}
unsigned int scene::WaterFountainScene::ComputeShaderParticleSystem::total() const noexcept
{
    return pimpl->tot;
}

scene::WaterFountainScene::WaterFountainScene()
        : BaseScene(),
          system_name("Water Fountain Demo"),
          rendering_mode("Compute and Geometry Shader"),
          particle_system(nullptr)
{
    particle_system = new ComputeShaderParticleSystem;
    particle_system->max_particles = WATER_FOUNTAIN_MAX_PARTICLES;
    particle_system->birth_rate    = particle_system->max_particles*1000;
}

scene::WaterFountainScene::~WaterFountainScene()
{
    delete particle_system;
}

void scene::WaterFountainScene::init(Scene &scene)
{
    particle_system->init(nullptr, 0);
}

void scene::WaterFountainScene::restart(Scene &scene)
{
    scene.character.reset(0.f, 0.f, -20.f, 180.f, 0.f);
    scene.character.setShootable(false);
    scene.character.setFloating(true);
    pause = false;
    wind = glm::vec3(0.f);
    particle_system->restart();
}

void scene::WaterFountainScene::upload(Shader &scene_shader)
{
    glClearColor(0.f, 0.f, 0.f, 1.f);
    particle_system->upload();
}

void scene::WaterFountainScene::render(Shader &scene_shader)
{}

void scene::WaterFountainScene::update(float dt)
{
    if (!pause)
    {
        particle_system->acceleration = wind;
        particle_system->update(dt);
    }
    processInput(dt);
}

void scene::WaterFountainScene::render()
{
    particle_system->render();

    renderInfo();
}

void scene::WaterFountainScene::renderInfo()
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
    ss << std::fixed << std::setprecision(2) << wind.x << ", " << wind.y << std::endl;
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
}

void scene::WaterFountainScene::processInput(float dt)
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
    else
    {
        if (last_key == GLFW_KEY_P)
            pause = !pause;
        last_key = GLFW_KEY_UNKNOWN;
    }
}

