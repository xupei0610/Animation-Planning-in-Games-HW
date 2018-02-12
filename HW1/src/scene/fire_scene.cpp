#include "scene/fire_scene.hpp"
#include "scene.hpp"
#include "util/random.hpp"
#include "global.hpp"
#include "app.hpp"
#include "config.h"
#include "util/cuda.hpp"

#include "stb_image.hpp"

using namespace px;

class scene::FireScene::CurlNoiseParticleSystem::impl
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
            "       vec4 color;"   // alpha
            "       vec4 life;"   //  total life, remaining life
            "       vec4 delta_position;" // velocity
            "   } particles[];"
            "};"
            "uniform float dt;"
            "uniform float current_time;"
            "uniform int n_particles;"
            ""
            "uint gid = gl_GlobalInvocationID.x;"
            ""
            "const float EPS = 1e-4f;"
            "const float INV_2_EPS = 1.f/ (2.f * EPS);"
            ""
            /**
             * @see https://gist.github.com/patriciogonzalezvivo/670c22f3966e662d2f83
             */
            "vec4 permute(vec4 x){return mod(((x*34.0)+1.0)*x, 289.0);}"
            "vec4 taylorInvSqrt(vec4 r){return 1.79284291400159 - 0.85373472095314 * r;}"
            ""
            "float snoise(vec3 v){"
            "   const vec2  C = vec2(1.0/6.0, 1.0/3.0) ;"
            "   const vec4  D = vec4(0.0, 0.5, 1.0, 2.0);"
            // First corner
            "   vec3 i  = floor(v + dot(v, C.yyy) );"
            "   vec3 x0 =   v - i + dot(i, C.xxx) ;"
            ""
            // Other corners
            "   vec3 g = step(x0.yzx, x0.xyz);"
            "   vec3 l = 1.0 - g;"
            "   vec3 i1 = min( g.xyz, l.zxy );"
            "   vec3 i2 = max( g.xyz, l.zxy );"
            //  x0 = x0 - 0. + 0.0 * C
            "   vec3 x1 = x0 - i1 + 1.0 * C.xxx;"
            "   vec3 x2 = x0 - i2 + 2.0 * C.xxx;"
            "   vec3 x3 = x0 - 1. + 3.0 * C.xxx;"
            // Permutations
            "   i = mod(i, 289.0 );"
            "   vec4 p = permute( permute( permute("
            "       i.z + vec4(0.0, i1.z, i2.z, 1.0 ))"
            "         + i.y + vec4(0.0, i1.y, i2.y, 1.0 ))"
            "         + i.x + vec4(0.0, i1.x, i2.x, 1.0 ));"
            // Gradients
            // ( N*N points uniformly over a square, mapped onto an octahedron.)
            "   float n_ = 1.0/7.0;" // N=7
            "   vec3  ns = n_ * D.wyz - D.xzx;"
            "   vec4 j = p - 49.0 * floor(p * ns.z *ns.z);"  //  mod(p,N*N)
            "   vec4 x_ = floor(j * ns.z);"
            "   vec4 y_ = floor(j - 7.0 * x_ );"    // mod(j,N)
            "   vec4 x = x_ *ns.x + ns.yyyy;"
            "   vec4 y = y_ *ns.x + ns.yyyy;"
            "   vec4 h = 1.0 - abs(x) - abs(y);"
            "   vec4 b0 = vec4( x.xy, y.xy );"
            "   vec4 b1 = vec4( x.zw, y.zw );"
            "   vec4 s0 = floor(b0)*2.0 + 1.0;"
            "   vec4 s1 = floor(b1)*2.0 + 1.0;"
            "   vec4 sh = -step(h, vec4(0.0));"
            "   vec4 a0 = b0.xzyw + s0.xzyw*sh.xxyy ;"
            "   vec4 a1 = b1.xzyw + s1.xzyw*sh.zzww ;"
            "   vec3 p0 = vec3(a0.xy,h.x);"
            "   vec3 p1 = vec3(a0.zw,h.y);"
            "   vec3 p2 = vec3(a1.xy,h.z);"
            "   vec3 p3 = vec3(a1.zw,h.w);"
//Normalise gradients
            "   vec4 norm = taylorInvSqrt(vec4(dot(p0,p0), dot(p1,p1), dot(p2, p2), dot(p3,p3)));"
            "   p0 *= norm.x;"
            "   p1 *= norm.y;"
            "   p2 *= norm.z;"
            "   p3 *= norm.w;"
// Mix final noise value
            "   vec4 m = max(0.6 - vec4(dot(x0,x0), dot(x1,x1), dot(x2,x2), dot(x3,x3)), 0.0);"
            "   m = m * m;"
            "   return 42.0 * dot( m*m, vec4( dot(p0,x0), dot(p1,x1),"
            "                             dot(p2,x2), dot(p3,x3) ) );"
            "}"
            ""
            "const float PI = 3.1415926535897932384626433832795;"
            "float rnd(vec2 co)"
            "{"
            "   return fract(sin(mod(dot(co.xy,vec2(12.9898,78.233)), 3.14)) * 43758.5453);"
            "}"
            "vec3 snoise3(vec3 p)"
            "{"
            ""
            "   return vec3(snoise(p),"
            "               snoise(vec3(p.y + 31.416f, p.z - 47.853f, p.x + 12.793f)),"
            "               snoise(vec3(p.z - 233.145f, p.x - 113.408f, p.y - 185.31f)));"
            "}"
            "vec3 curl(float s)"
            "{"
            "    vec3 dx = vec3(EPS, 0.f, 0.f);"
            "    vec3 dy = vec3(0.f, EPS, 0.f);"
            "    vec3 dz = vec3(0.f, 0.f, EPS);"
            ""
            "    vec3 dx0 = snoise3((particles[gid].position.xyz - dx)* s);"
            "    vec3 dx1 = snoise3((particles[gid].position.xyz + dx)* s);"
            "    vec3 dy0 = snoise3((particles[gid].position.xyz - dy)* s);"
            "    vec3 dy1 = snoise3((particles[gid].position.xyz + dy)* s);"
            "    vec3 dz0 = snoise3((particles[gid].position.xyz - dz)* s);"
            "    vec3 dz1 = snoise3((particles[gid].position.xyz + dz)* s);"
            ""
            "    float x = dy1.z - dy0.z - dz1.y + dz0.y;"
            "    float y = dz1.x - dz0.x - dx1.z + dx0.z;"
            "    float z = dx1.y - dx0.y - dy1.x + dy0.x;"
            "    return normalize(vec3(x, y, z) * INV_2_EPS);"
            "}"
            ""
            "uniform int n_octaves = 2;"
            "uniform float turbulence = .5f;"
            "uniform float noise_position_scale = 2.f;"
            "uniform float initial_size = .015f;"
            "uniform float final_size = .001f;"
            "uniform float base_life = 7.5f;"
            "uniform vec4 initial_color = vec4(1.f, 0.45f, 0.2f, 1.f);"
            "uniform vec4 final_color = vec4(1.f, 0.f, 0.f, 0.f);"
            "void main()"
            "{"
            "   if (gid >= n_particles) {return;}"
            "   if (particles[gid].life.y < 0.f || dt < 0.f)"
            "   {"
            "       vec2 sd = vec2(current_time, (gid + dt)*1000);"
            "       float theta = PI * rnd(sd) * 2.f;         sd.x *= sd.x;"
            "       float phi = acos(rnd(sd) * 2.f - 1.f);    sd.x += sd.x;"
            "       float r0 = pow(rnd(sd) * .055f, 1.f/3.f); sd.x += sd.x;"
            "       particles[gid].position = vec4(r0 * sin(phi) * cos(theta),"
            "                                      r0 * cos(phi),"
            "                                      r0 * sin(phi) * sin(theta),"
            "                                      initial_size);"
            "       particles[gid].color =  initial_color;"
            "       particles[gid].life.x = base_life + rnd(sd);"
            "       particles[gid].life.y = particles[gid].life.x;"
            "       particles[gid].delta_position.xyz = vec3(0.f);"
            "   }"
            "   else"
            "   {"
            "       vec3 dx = vec3(EPS, 0.f, 0.f);"
            "       vec3 dy = vec3(0.f, EPS, 0.f);"
            "       vec3 dz = vec3(0.f, 0.f, EPS);"
            ""
            "       float alpha = turbulence;"
            "       float beta  = noise_position_scale;"
            "       vec3 velocity = vec3(0.045 * (rnd(vec2(gid, current_time)) * 2.f - 1), .35f, 0.045 * (rnd(vec2(gid*1000, current_time)) * 2.f - 1));"
            "       for (int i = 0; i < n_octaves; ++i)"
            "       {"
            "           velocity += curl(beta) * alpha;"
            "           alpha *= alpha;"
            "           beta *= beta;"
            "       }"
            "       particles[gid].delta_position.xyz = velocity * dt;"
            "       particles[gid].position.xyz += particles[gid].delta_position.xyz;"
            ""
            "       float scale = dt / particles[gid].life.x;"
            "       particles[gid].position.w += (final_size - initial_size) * scale;"
            "       particles[gid].color += (final_color - initial_color) * scale;"
            "       particles[gid].life.y -= dt;"
            "   }"
            ""
            "}";

    const char *VS = "#version 420 core\n"
            "layout(location = 0) in vec4 position;"
            "layout(location = 1) in vec4 color;"
            "layout(location = 2) in vec3 delta_position;"
            ""
            "out VS_OUT"
            "{"
            "   vec4 color;"
            "   vec3 delta_position;"
            "} primitive;"
            ""
            "void main()"
            "{"
            "   gl_Position = position;"
            "   primitive.color = color;"
            "   primitive.delta_position = delta_position;"
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
            "   vec4 color;"
            "   vec3 delta_position;"
            "} primitive[];"
            ""
            "out vec2 gTextureCoord;"
            "out vec4 gColor;"
            ""
            "void main()"
            "{"
            "   mat4 VP = proj * view;"
            ""
            "   vec3 u = mat3(view) * primitive[0].delta_position;" // movement in view
            "   float w = gl_in[0].gl_Position.w;" // half width
            "   float h = w * 2.f;"                // half height
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
            "   gColor = primitive[0].color;"
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
            "in vec4 gColor;"
            "uniform sampler2D sprite;"
            "out vec4 color;"
            ""
            "void main(){"
            "   color = vec4(gColor.xyz, gColor.a * texture(sprite, gTextureCoord).r);"
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

scene::FireScene::CurlNoiseParticleSystem::CurlNoiseParticleSystem()
        : ParticleSystem()
{
    pimpl = std::unique_ptr<impl>(new impl);
}

scene::FireScene::CurlNoiseParticleSystem::~CurlNoiseParticleSystem()
{}

void scene::FireScene::CurlNoiseParticleSystem::init(float *, unsigned int v_count,
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
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 16*sizeof(float), (void *)(0));
    glEnableVertexAttribArray(1);   // color
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 16*sizeof(float), (void *)(4*sizeof(float)));
    glEnableVertexAttribArray(2);   // delta_position
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 16*sizeof(float), (void *)(12*sizeof(float)));
    pimpl->draw_shader->activate(false);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    int w, h, ch;
    auto ptr = stbi_load(ASSET_PATH "/texture/particle3.png", &w, &h, &ch, 3);
    TEXTURE_LOAD_HELPER(pimpl->texture, GL_RGB, GL_REPEAT, GL_LINEAR, w, h, ptr);
    stbi_image_free(ptr);

}

void scene::FireScene::CurlNoiseParticleSystem::restart()
{
    pimpl->tot = max_particles;
    pimpl->debt = 0.f;
    n_particles = 0.f;
    pimpl->upload = true;
}

void scene::FireScene::CurlNoiseParticleSystem::upload()
{
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, pimpl->vbo);
    if (!pimpl->upload) return;

    glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float)*16*max_particles, nullptr, GL_STATIC_DRAW);

    pimpl->compute_shader->activate();
    pimpl->compute_shader->set("dt", -1.f);
    pimpl->compute_shader->set("current_time", static_cast<float>(glfwGetTime()));
    pimpl->compute_shader->set("n_octaves", 2);
    pimpl->compute_shader->set("turbulence", .5f);
    pimpl->compute_shader->set("noise_position_scale", 2.f);
    pimpl->compute_shader->set("initial_color", glm::vec4(1.f, 0.45f, 0.2f, 1.f));
    pimpl->compute_shader->set("final_color", glm::vec4(1.f, 0.f, 0.f, 0.f));
    pimpl->compute_shader->set("initial_size", .015f);
    pimpl->compute_shader->set("final_size", .001f);
    pimpl->compute_shader->set("base_life", 7.5f);
    pimpl->compute_shader->set("n_particles", static_cast<int>(total()));
    glDispatchCompute(cuda::blocks(total(), COMPUTE_SHADER_WORK_GROUP_SIZE), 1, 1);
    pimpl->compute_shader->activate(false);

    pimpl->upload = false;
}


void scene::FireScene::CurlNoiseParticleSystem::update(float dt, glm::vec3 *cam_pos)
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
    pimpl->compute_shader->set("current_time", static_cast<float>(glfwGetTime()));
    pimpl->compute_shader->set("n_particles", static_cast<int>(count()));
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, pimpl->vbo);
    glDispatchCompute(cuda::blocks(count(), COMPUTE_SHADER_WORK_GROUP_SIZE), 1, 1);
    pimpl->compute_shader->activate(false);
}

void scene::FireScene::CurlNoiseParticleSystem::render(GLenum gl_draw_mode)
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

unsigned int scene::FireScene::CurlNoiseParticleSystem::total() const noexcept
{
    return pimpl->tot;
}

scene::FireScene::FireScene()
        : BaseScene(),
          particle_system(nullptr),
          system_name("Fire Simulation using Curl Noise"),
          rendering_mode("compute shader")
{
    particle_system = new CurlNoiseParticleSystem;

    particle_system->max_particles = FIRE_MAX_PARTICLES;
    particle_system->birth_rate    = FIRE_MAX_PARTICLES * .06f;
}

scene::FireScene::~FireScene()
{
    delete particle_system;
}

void scene::FireScene::init(Scene &scene)
{
    particle_system->init(nullptr, 0);
}

void scene::FireScene::restart(Scene &scene)
{
    resetCamera();

    particle_system->restart();
    pause = false;
}

void scene::FireScene::upload(Shader &scene_shader)
{
    glClearColor(0.f, 0.f, 0.f, 1.f);
    particle_system->upload();
}

void scene::FireScene::update(float dt)
{
    if (!pause)
        particle_system->update(dt);

    processInput(dt);
}

void scene::FireScene::render()
{
    particle_system->render();

    renderInfo();
}

void scene::FireScene::resetCamera()
{
    App::instance()->scene.character.reset(-3.f, 3.f, -3.f, 135.f, 25.f);
    App::instance()->scene.character.setShootable(false);
    App::instance()->scene.character.setFloating(true);
}

void scene::FireScene::renderInfo()
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

void scene::FireScene::processInput(float dt)
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
    particle_system->birth_rate = particle_system->max_particles * .06f;
#define DECREASE_PARTICLES                                                  \
    if (particle_system->max_particles > 1000)                              \
    {                                                                       \
        particle_system->max_particles -= 1000;                             \
        particle_system->birth_rate = particle_system->max_particles * .06f; \
    }

    STICKY_KEY_CHECK(GLFW_KEY_Z, INCREASE_PARTICLES)
    else
    STICKY_KEY_CHECK(GLFW_KEY_X, DECREASE_PARTICLES)
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
        last_key = GLFW_KEY_UNKNOWN;
    }
}
