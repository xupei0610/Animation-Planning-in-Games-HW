#include "scene/galaxy_scene.hpp"
#include "scene.hpp"
#include "app.hpp"
#include "global.hpp"
#include "config.hpp"

#ifdef USE_CUDA
#include "util/cuda.hpp"
#include <cuda_gl_interop.h>
#endif

#include "stb_image.hpp"

using namespace px;

class scene::GalaxyScene::ComputeShaderParticleSystem::impl
{
public:
    unsigned int vao, vbo, texture;

    unsigned int debt, tot;
    bool upload;

    Shader *update_compute_shader, *spawn_compute_shader;
    Shader *draw_shader;
#ifdef USE_CUDA
    struct cudaGraphicsResource *res;
#endif

    const char *CS_SPAWN = "#version 430 core\n"
            "layout (local_size_x = " STR(COMPUTE_SHADER_WORK_GROUP_SIZE) ", local_size_y = 1, local_size_z = 1) in;"
            ""
            "layout(std140, binding = 0) buffer Particle_t"
            "{"
            "   struct"
            "   {"
            "       vec4 position;"
            "       vec4 velocity;"
            "       vec4 d_pos;"
            "   } particles [];"
            "};"
            ""
            "const float PI = 3.1415926535897932384626433832795;"
            "const float PI_12 = PI / 12.f;"
            ""
            "uint gid = gl_GlobalInvocationID.x + gl_GlobalInvocationID.y*gl_NumWorkGroups.x * gl_WorkGroupSize.x;"
            ""
            "float rnd(vec2 co)"
            "{"
            "   return fract(sin(mod(dot(co.xy,vec2(12.9898,78.233)), 3.14)) * 43758.5453);"
            "}"
            ""
            "void main()"
            "{"
            "   vec2 sd = vec2(gid, gid);"
//            "   float theta = PI * rnd(sd) * 2.f;                ++sd.x;"
//            "   float phi = acos(1.f - 2.f * rnd(sd));           ++sd.x;"
//            "   float r0  = pow(.5f + rnd(sd)* 64f, 1.f/3.f);   ++sd.x;"
//            "   particles[gid].position.x = r0 * sin(phi) * cos(theta);"
//            "   particles[gid].position.y = r0 * cos(phi);"
//            "   particles[gid].position.z = r0 * sin(phi) * sin(theta);"
//            "   if (gid % 3 == 1) particles[gid].position.x += 3.f;"
//            "   else if (gid % 3 == 2) particles[gid].position.z += 3.f;"
            "   float r0 = 4.f;"
            "   particles[gid].position.x = r0 * rnd(sd) + (gid % 2 == 0 ? 1.5f : -1.5f); ++sd.x;"
            "   particles[gid].position.y = r0 * rnd(sd); ++sd.x;"
            "   particles[gid].position.z = r0 * rnd(sd); ++sd.x;"
            "   particles[gid].velocity.x = 0.f;"//-.1f * particles[gid].position.x;"
            "   particles[gid].velocity.y = 0.f;"//-.1f * particles[gid].position.y;"
            "   particles[gid].velocity.z = 0.f;"//-.1f * particles[gid].position.z;"
            "   particles[gid].position.w = .05f + rnd(sd) * 10.f;" // mass
            "   particles[gid].velocity.w = 160706.f;" // flag to be spawned
            "   particles[gid].d_pos.x = 0.f;"
            "   particles[gid].d_pos.y = 0.f;"
            "   particles[gid].d_pos.z = 0.f;"
            "}";
    const char *CS_UPDATE = "#version 430 core\n"
            "layout (local_size_x = " STR(COMPUTE_SHADER_WORK_GROUP_SIZE) ", local_size_y = 1, local_size_z = 1) in;"
            ""
            "layout(std140, binding = 0) buffer Particle_t"
            "{"
            "   struct"
            "   {"
            "       vec4 position;"
            "       vec4 velocity;"
            "       vec4 d_pos;"
            "   } particles [];"
            "};"
            ""
            "uniform float dt;"
            "uniform int n_particles;"
            ""
            "uniform float damping = .999f;"
            "const float EPS = 1e-4f;"
            ""
            "uniform float factor = 0.002f;"
            ""
            "uint gid = gl_GlobalInvocationID.x + gl_GlobalInvocationID.y*gl_NumWorkGroups.x * gl_WorkGroupSize.x;"
            ""
            "void main()"
            "{"
            "   particles[gid].position.xyz += particles[gid].d_pos.xyz;"
            "   particles[gid].d_pos.x = 0.f;"
            "   particles[gid].d_pos.y = 0.f;"
            "   particles[gid].d_pos.z = 0.f;"
            "   vec3 pos = particles[gid].position.xyz;"
            "   vec3 dist; float dist_sqr; float dist_six;"
            "   for (int i = 0; i < n_particles; ++i)"
            "   {"
            "       dist = particles[i].position.xyz - pos;"
            //"       if (dist < threshhold)" // collision
            "       dist_sqr = dist.x * dist.x + dist.y * dist.y + dist.z * dist.z + EPS;"
            "       dist_six = dist_sqr * dist_sqr * dist_sqr;"
            "       dist_sqr = 1.f / sqrt(dist_six);"
            "       particles[gid].d_pos.xyz += dist * (particles[i].position.w * dist_sqr);"
            "   }"
            "   dist_sqr = dt * factor;"
            "   pos = particles[gid].velocity.xyz * dist_sqr;"
            "   particles[gid].velocity.xyz += particles[gid].d_pos.xyz * dist_sqr;"
            "   particles[gid].velocity.xyz *= damping;"
            "   particles[gid].d_pos.xyz = pos;"
            "}";
    const char *VS = "#version 420 core\n"
            "layout(location = 0) in vec3 pos;"
            "layout(location = 1) in vec3 d_pos;"
            "layout(location = 2) in float mass;"
            ""
            "void main()"
            "{"
            "	gl_Position = vec4(pos + d_pos, clamp(.01f * mass, 0.01, 0.05));"
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
            "out vec2 gTextureCoord;"
            ""
            "void main()"
            "{"
            "   mat4 VP = proj * view;"
            ""
            "   vec3 right_vec = vec3(view[0][0], view[1][0], view[2][0]) * gl_in[0].gl_Position.w;"
            "   vec3 up_vec = vec3(view[0][1], view[1][1], view[2][1]) * gl_in[0].gl_Position.w;"
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
            "in vec2 gTextureCoord;"
            ""
            "uniform sampler2D sprite;"
            ""
            "out vec4 color;"
            ""
            "void main(){"
            "   color = vec4(1.f, 0.45f, 0.2f, texture(sprite, gTextureCoord).r);"
            "}";

    impl() : vao(0), vbo(0), texture(0),
             update_compute_shader(nullptr), spawn_compute_shader(nullptr),
             draw_shader(nullptr)
#ifdef USE_CUDA
    , res(nullptr)
#endif
    {}
    ~impl()
    {
        clearGLObjs();

        delete update_compute_shader;
        delete spawn_compute_shader;
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

scene::GalaxyScene::ComputeShaderParticleSystem::ComputeShaderParticleSystem()
        : ParticleSystem()
{
    pimpl = std::unique_ptr<impl>(new impl);
}

scene::GalaxyScene::ComputeShaderParticleSystem::~ComputeShaderParticleSystem()
{}

void scene::GalaxyScene::ComputeShaderParticleSystem::init(float *vertex, unsigned int v_count,
                                                              unsigned int tex, float *uv, bool atlas)
{
#ifndef USE_CUDA
    if (pimpl->spawn_compute_shader == nullptr)
        pimpl->spawn_compute_shader = new Shader(pimpl->CS_SPAWN);
    if (pimpl->update_compute_shader == nullptr)
        pimpl->update_compute_shader = new Shader(pimpl->CS_UPDATE);
#endif
    if (pimpl->draw_shader == nullptr)
        pimpl->draw_shader = new Shader(pimpl->VS, pimpl->FS, pimpl->GS);

    pimpl->genGLObjs();

    pimpl->draw_shader->activate();
    pimpl->draw_shader->bind("GlobalAttributes", 0);
    pimpl->draw_shader->set("sprite", 0);
    glBindVertexArray(pimpl->vao);
    glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo);
    glEnableVertexAttribArray(0);   // position
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 12*sizeof(float), (void *)(0));
    glEnableVertexAttribArray(1);   // d_position
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 12*sizeof(float), (void *)(8*sizeof(float)));
    glEnableVertexAttribArray(2);   // mass for size
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, 12*sizeof(float), (void *)(3*sizeof(float)));
    pimpl->draw_shader->activate(false);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    {
        int w, h, ch;
        auto ptr = stbi_load(ASSET_PATH "/texture/particle3.png", &w, &h, &ch, 3);
        TEXTURE_LOAD_HELPER(pimpl->texture, GL_RGB, GL_REPEAT, GL_LINEAR, w, h, ptr);
        stbi_image_free(ptr);
    }

#ifdef USE_CUDA
    PX_CUDA_CHECK(cudaGLSetGLDevice(0));
#endif
}

void scene::GalaxyScene::ComputeShaderParticleSystem::restart()
{
    pimpl->tot = max_particles;
    pimpl->debt = 0.f;

    pimpl->upload = true;
}

void scene::GalaxyScene::ComputeShaderParticleSystem::upload()
{
    if (!pimpl->upload) return;

    glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float)*12*max_particles, nullptr, GL_STATIC_DRAW);
#ifndef USE_CUDA
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, pimpl->vbo);
    pimpl->spawn_compute_shader->activate();
    glDispatchCompute(total()/COMPUTE_SHADER_WORK_GROUP_SIZE, 1, 1);
    pimpl->spawn_compute_shader->activate(false);
#else
    if (pimpl->res != nullptr)
    {
        PX_CUDA_CHECK(cudaGraphicsUnregisterResource(pimpl->res));
    }
    PX_CUDA_CHECK(cudaGraphicsGLRegisterBuffer(&pimpl->res, pimpl->vbo, cudaGraphicsRegisterFlagsNone));
    void *buffer;
    size_t buffer_size;
    PX_CUDA_CHECK(cudaGraphicsMapResources(1, &pimpl->res, 0));
    PX_CUDA_CHECK(cudaGraphicsResourceGetMappedPointer(&buffer, &buffer_size, pimpl->res));
    cudaSpawn(buffer, total());
    cudaDeviceSynchronize();
    PX_CUDA_CHECK(cudaGraphicsUnmapResources(1, &pimpl->res, 0));
#endif

    n_particles = 0;
    pimpl->upload = false;
}


void scene::GalaxyScene::ComputeShaderParticleSystem::update(float dt, glm::vec3 *cam_pos)
{
    if (n_particles != total())
    {
        pimpl->debt += std::min(0.05f, dt) * birth_rate;
        auto new_particles = static_cast<int>(pimpl->debt);
        pimpl->debt -= new_particles;
        n_particles += new_particles;
        if (n_particles > total()) n_particles = total();
    }

#ifndef USE_CUDA
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, pimpl->vbo);
    pimpl->update_compute_shader->activate();
    pimpl->update_compute_shader->set("dt", std::min(dt, 0.02f));
    pimpl->update_compute_shader->set("n_particles", int(count()));
    glDispatchCompute(count()/COMPUTE_SHADER_WORK_GROUP_SIZE, 1, 1);
    pimpl->update_compute_shader->activate(false);
#else
    void *buffer;
    size_t buffer_size;
    PX_CUDA_CHECK(cudaGraphicsMapResources(1, &pimpl->res, 0));
    PX_CUDA_CHECK(cudaGraphicsResourceGetMappedPointer(&buffer, &buffer_size, pimpl->res));
    cudaUpdate(buffer, total(), dt);
    cudaDeviceSynchronize();
    PX_CUDA_CHECK(cudaGraphicsUnmapResources(1, &pimpl->res, 0));
#endif
}

void scene::GalaxyScene::ComputeShaderParticleSystem::render(GLenum gl_draw_mode)
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


//    std::cout << "("
//              << App::instance()->scene.cam.pos().x << ", "
//              << App::instance()->scene.cam.pos().y << ", "
//              << App::instance()->scene.cam.pos().z << ") "
//              << App::instance()->scene.cam.yaw() << ", "
//              << App::instance()->scene.cam.pitch() << std::endl;
}


unsigned int const &scene::GalaxyScene::ComputeShaderParticleSystem::count() const noexcept
{
    return n_particles;
}
unsigned int scene::GalaxyScene::ComputeShaderParticleSystem::total() const noexcept
{
    return pimpl->tot;
}

scene::GalaxyScene::GalaxyScene()
        : BaseScene(),
          system_name("N-Body Galaxy Simulation"),
          rendering_mode(
#ifdef USE_CUDA
          "CUDA"
#else
          "Compute Shader"
#endif
                        ),
          particle_system(nullptr)
{
    particle_system = new ComputeShaderParticleSystem;

    particle_system->max_particles =
#ifdef USE_CUDA
    GALAXY_MAX_PARTICLES_CUDA
#else
    GALAXY_MAX_PARTICLES_COMPUTE_SHAER
#endif
    ;
    particle_system->birth_rate    = 6000000;
}

scene::GalaxyScene::~GalaxyScene()
{
    delete particle_system;
}

void scene::GalaxyScene::init(Scene &scene)
{
    particle_system->init(nullptr, 0);
}

void scene::GalaxyScene::restart(Scene &scene)
{
    scene.character.reset(-3.f, 9.f, -3.f, 135.f, 45.f);
    scene.character.setShootable(false);
    scene.character.setFloating(true);

    particle_system->restart();
    pause = false;
}

void scene::GalaxyScene::upload(Shader &scene_shader)
{
    glClearColor(0.f, 0.f, 0.f, 1.f);
    particle_system->upload();
}

void scene::GalaxyScene::render(Shader &scene_shader)
{}

void scene::GalaxyScene::update(float dt)
{
    if (!pause)
        particle_system->update(dt);
    processInput(dt);
}

void scene::GalaxyScene::render()
{
    particle_system->render();

    renderInfo();
}

void scene::GalaxyScene::renderInfo()
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
}

void scene::GalaxyScene::processInput(float dt)
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
    else
    {
        if (last_key == GLFW_KEY_P)
            pause = !pause;
        last_key = GLFW_KEY_UNKNOWN;
    }
}
