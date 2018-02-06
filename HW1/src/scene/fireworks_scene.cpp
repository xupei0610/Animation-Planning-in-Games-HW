#include "scene/fireworks_scene.hpp"
#include "scene.hpp"
#include "app.hpp"
#include "global.hpp"

#include "stb_image.hpp"

using namespace px;

#define WORK_GROUP_SIZE 256

class scene::FireworksScene::ComputeShaderParticleSystem::impl
{
public:
    unsigned int vao, vbo, texture;

    unsigned int debt, tot;
    bool upload;

    Shader *draw_shader, *compute_shader;

    const char *CS = "#version 430 core\n"
            "layout (local_size_x = " STR(WORK_GROUP_SIZE) ", local_size_y = 1, local_size_z = 1) in;"
            ""
            "layout(std140, binding = 0) buffer Particle_t"
            "{"
            "   struct"
            "   {"
            "       vec4 position;" // position,  size
            "       vec4 velocity;" // velocity,  life
            "       vec2 flag;" //  explode flag, alpha
            "   } particles[];"
            "};"
            "uniform float dt;"
            ""
            "uniform vec3 gravity;"      // initial position
            "uniform vec3 acceleration;" // initial velocity
            ""
            "uint gid = gl_GlobalInvocationID.x + gl_GlobalInvocationID.y*gl_NumWorkGroups.x * gl_WorkGroupSize.x;"
            ""
            "float rnd(vec2 co)"
            "{"
            "   return fract(sin(mod(dot(co.xy,vec2(12.9898,78.233)), 3.14)) * 43758.5453);"
            "}"
            ""
            "void spawn()"
            "{"
            "   vec2 sd = vec2(gid, gid);"
            "   particles[gid].position.xyz = gravity;"
            "   particles[gid].position.w = 0.0125 * (1 + rnd(sd)); ++sd.x;" // size
            "   particles[gid].velocity.xyz = acceleration"
            "   particles[gid].velocity.w = 4.f + rnd(sd);" // life
            "   particles[gid].flag.x = 0.f;" // not exploded
            "   particles[gid].flag.y = 1.f;" // alpha
            "}"
            ""
            "void explode()"
            "{"
            "   particles[gid].velocity.x = -4.f + 8.f * rnd(sd); ++sd.x;"
            "   particles[gid].velocity.y = -4.f + 8.f * rnd(sd); ++sd.x;"
            "   particles[gid].velocity.z = -4.f + 8.f * rnd(sd);"
            "   particles[gid].flag.x = 1.f;" // exploded
            "}"
            "void update()"
            "{"
            "   particles[gid].flag.y -= dt / particles[gid].velocity.w;"
            "   particles[gid].velocity.xyz += dt * (acceleration + gravity);"
            "   particles[gid].position.xyz += particles[gid].velocity * dt;"
            "}"
            ""
            "void main()"
            "{"
            "   if (dt < 0.f)"
            "       spawn();"
            "   else if (particles[gid].flag.x == 0.f && particles[gid].velocity.y < 0.f)"
            "       explode();"
            "   else if (particles[gid].flag.y > 0.f)"
            "       update();"
            "}"
            "";

    const char *VS = "#version 420 core\n"
            "layout(location = 0) in vec3 position;"
            "layout(location = 1) in float alpha;"
            "layout(location = 2) in float size;"
            ""
            "out V_OUT"
            "{"
            "   float alpha;"
            "   float size;"
            "} primitive;"
            ""
            "void main()"
            "{"
            "	gl_Position = vec4(position, 1.f);"
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
            "in V_OUT"
            "{"
            "   float alpha;"
            "   float size;"
            "} primitive[];"
            ""
            "out vec2 gtEXTUREcOORD;"
            ""
            "void main()"
            "{"
            "   mat4 VP = proj * view;"
            ""
            "   vec3 right_vec = vec3(view[0][0], view[1][0], view[2][0]) * size;"
            "   vec3 up_vec = vec3(view[0][1], view[1][1], view[2][1]) * .size;"
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
            "uniform vec3 color;"
            ""
            "out vec4 gColor;"
            ""
            "void main(){"
            "   gColor = vec4(color, texture(sprite, gTextureCoord).r);"
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

scene::FireworksScene::ComputeShaderParticleSystem::ComputeShaderParticleSystem()
        : ParticleSystem()
{
    pimpl = std::unique_ptr<impl>(new impl);
}

scene::FireworksScene::ComputeShaderParticleSystem::~ComputeShaderParticleSystem()
{}

void scene::FireworksScene::ComputeShaderParticleSystem::init(float *vertex, unsigned int v_count,
                                                           unsigned int tex, float *uv, bool atlas)
{
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
}

void scene::FireworksScene::ComputeShaderParticleSystem::restart()
{
    pimpl->tot = max_particles;
    pimpl->debt = 0.f;

    pimpl->upload = true;
}

void scene::FireworksScene::ComputeShaderParticleSystem::upload()
{
    if (!pimpl->upload) return;

    glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float)*12*max_particles, nullptr, GL_STATIC_DRAW);

    n_particles = 0;
    pimpl->upload = false;
}


void scene::FireworksScene::ComputeShaderParticleSystem::update(float dt, glm::vec3 *cam_pos)
{
    if (n_particles != total())
    {
        pimpl->debt += std::min(0.05f, dt) * birth_rate;
        auto new_particles = static_cast<int>(pimpl->debt);
        pimpl->debt -= new_particles;
        n_particles += new_particles;
        if (n_particles > total()) n_particles = total();
    }
}

void scene::FireworksScene::ComputeShaderParticleSystem::render(GLenum gl_draw_mode)
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
          system_name("N-Body Galaxy Simulation"),
          rendering_mode("transform feedback"),
          particle_system(nullptr)
{
    particle_system = new ComputeShaderParticleSystem;
    particle_system->max_particles =   1000;
    particle_system->birth_rate    = 600000;
}

scene::FireworksScene::~FireworksScene()
{
    delete particle_system;
}

void scene::FireworksScene::init(Scene &scene)
{
    particle_system->init(nullptr, 0);
}

void scene::FireworksScene::restart(Scene &scene)
{
    scene.character.reset(-3.f, 9.f, -3.f, 135.f, 45.f);
    scene.character.setShootable(false);
    scene.character.setFloating(true);

    particle_system->restart();
    pause = false;
}

void scene::FireworksScene::upload(Shader &scene_shader)
{
    glClearColor(0.f, 0.f, 0.f, 1.f);
    particle_system->upload();
}

void scene::FireworksScene::render(Shader &scene_shader)
{}

void scene::FireworksScene::update(float dt)
{
    if (!pause)
        particle_system->update(dt);
    processInput(dt);
}

void scene::FireworksScene::render()
{
    particle_system->render();

    renderInfo();
}

void scene::FireworksScene::renderInfo()
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

void scene::FireworksScene::processInput(float dt)
{
    static auto sum_dt = 0.f;
    static auto last_key = GLFW_KEY_UNKNOWN;
    static auto key_count = 0;

#define HOLD_KEY(Key)                                       \
    (last_key == Key && sum_dt > 0.01f && key_count == 10)

    auto & window = App::instance()->window();
    if (glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS)
    {
        if (last_key != GLFW_KEY_Z || sum_dt > 0.1f || HOLD_KEY(GLFW_KEY_Z))
        {
            particle_system->max_particles += 1000;
            particle_system->birth_rate = particle_system->max_particles * 60.f;
            sum_dt = 0;

            if (key_count < 10) ++key_count;
        }
        else
            sum_dt += dt;

        if (last_key != GLFW_KEY_Z)
        {
            last_key = GLFW_KEY_Z;
            key_count = 0;
        }

    }
    else if (glfwGetKey(window, GLFW_KEY_X) == GLFW_PRESS)
    {
        if (particle_system->max_particles > 1000)
        {
            if (last_key != GLFW_KEY_X || sum_dt > 0.1f || HOLD_KEY(GLFW_KEY_X))
            {
                particle_system->max_particles -= 1000;
                particle_system->birth_rate = particle_system->max_particles * 60.f;
                sum_dt = 0;

                if (key_count < 10) ++key_count;
            }
            else
                sum_dt += dt;
        }

        if (last_key != GLFW_KEY_X)
        {
            last_key = GLFW_KEY_X;
            key_count = 0;
        }
    }
    else if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS)
        last_key = GLFW_KEY_P;
    else
    {
        if (last_key == GLFW_KEY_P)
            pause = !pause;
        last_key = GLFW_KEY_UNKNOWN;
    }

#undef HOLD_KEY
}
