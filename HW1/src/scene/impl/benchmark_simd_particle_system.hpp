#ifndef PX_CG_SCENE_IMPL_BENCHMARK_SIMD_PARTICLE_SYSTEM
#define PX_CG_SCENE_IMPL_BENCHMARK_SIMD_PARTICLE_SYSTEM

scene::BenchmarkScene::SIMDParticleSystem::SIMDParticleSystem()
        : ParticleSystem(),
          vao(0), vbo{0}, ssbo(0),
          draw_shader(nullptr)
{}
scene::BenchmarkScene::ComputeShaderParticleSystem::ComputeShaderParticleSystem()
    : SIMDParticleSystem(), compute_shader(nullptr)
{}
scene::BenchmarkScene::CUDAParticleSystem::CUDAParticleSystem()
        : SIMDParticleSystem(), res(nullptr)
{}

scene::BenchmarkScene::SIMDParticleSystem::~SIMDParticleSystem()
{
    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(1, &ssbo);
    glDeleteBuffers(2, vbo);

    delete draw_shader;
}
scene::BenchmarkScene::ComputeShaderParticleSystem::~ComputeShaderParticleSystem()
{
    delete compute_shader;
}
scene::BenchmarkScene::CUDAParticleSystem::~CUDAParticleSystem()
{
    if (res != nullptr)
    {
        cudaGraphicsUnregisterResource(res);
    }
}

void scene::BenchmarkScene::SIMDParticleSystem::init(float *vertex, unsigned int v_count,
              unsigned int tex, float *uv, bool atlas)
{
    if (draw_shader == nullptr)
    {
        draw_shader = new Shader(
#include "shader/glsl/simple_2d_particle.vs"
                ,
#include "shader/glsl/simple_particle.fs"
                                );
        draw_shader->bind("GlobalAttributes", 0);
    }

    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(1, &ssbo);
    glDeleteBuffers(2, vbo);

    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &ssbo);
    glGenBuffers(2, vbo);

    n_vertices = v_count;

    draw_shader->activate();
    draw_shader->set("use_texture", 0);
    draw_shader->set("use_atlas", 0);
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);  // vertices
    glBufferData(GL_ARRAY_BUFFER, sizeof(float)*2*n_vertices, vertex, GL_STATIC_DRAW);
    glEnableVertexAttribArray(5);
    glVertexAttribPointer(5, 2, GL_FLOAT, GL_FALSE, 0, (void *)0);
    glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);
    glEnableVertexAttribArray(1);   // scale + rotation
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, (void *)0);
    glBindBuffer(GL_ARRAY_BUFFER, ssbo);
    glEnableVertexAttribArray(0);   // position
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 20*sizeof(float), (void *)(16*sizeof(float)));
    glEnableVertexAttribArray(2);   // color
    glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, 20*sizeof(float), (void *)(12*sizeof(float)));
    draw_shader->activate(false);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}
void scene::BenchmarkScene::ComputeShaderParticleSystem::init(float *vertex, unsigned int v_count,
                                                          unsigned int tex, float *uv, bool atlas)
{
    static const char CS[] = "#version 430 core\n"
            "layout (local_size_x = " STR(COMPUTE_SHADER_WORK_GROUP_SIZE) ", local_size_y = 1, local_size_z = 1) in;"
            ""
            "layout(std140, binding = 0) buffer Particle_t"
            "{"
            "   struct"
            "   {"
            "       vec4 position;"
            "       vec4 velocity;"
            "       vec4 life;"
            "       vec4 color;"
            "       vec4 gPosition;"
            "   } particles [];"
            "};"
            ""
            "uniform float dt;"
            "uniform int n_particles;"
            ""
            "const float PI = 3.1415926535897932384626433832795;"
            "const float double_PI = 2 * PI;"
            "const float drot = 0.06f * PI;"
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
            "   float theta = PI * rnd(sd) * 2.f; ++sd.x;"
            "   float phi = acos(rnd(sd) * 2.f - 1.f);    ++sd.x;"
            "   float r0 = pow(rnd(sd) * 125.f, 1.f/3.f); ++sd.x;"
            "   particles[gid].position.x = r0 * sin(phi) * cos(theta);"
            "   particles[gid].position.y = r0 * cos(phi);"
            "   particles[gid].position.z = r0 * sin(phi) * sin(theta);"
            "   particles[gid].velocity.x = 0.0001f * particles[gid].position.x;"
            "   particles[gid].velocity.y = 0.0001f * particles[gid].position.y;"
            "   particles[gid].velocity.z = 0.0001f * particles[gid].position.z;"
            "   particles[gid].life.x = 4.f + (2*rnd(sd) - 1.f) * (4.f / 4);  ++sd.x;"
            "   particles[gid].life.y = 1.f + (2*rnd(sd) - 1.f) * (1.f / 4);  ++sd.x;"
            "   particles[gid].life.z = 5.f + (2*rnd(sd) - 1.f) * (5.f / 4);"
            "   particles[gid].life.w = 0.f;"
            "   particles[gid].life.w = particles[gid].life.x + particles[gid].life.y + particles[gid].life.z;"
            "   particles[gid].color.x = 1.f;"
            "   particles[gid].color.y = .45f;"
            "   particles[gid].color.z = .2f;"
            "   particles[gid].color.a = 0.f;"
            "   particles[gid].gPosition.x = particles[gid].position.x;"
            "   particles[gid].gPosition.y = particles[gid].position.y;"
            "   particles[gid].gPosition.z = particles[gid].position.z;"
            "}"
            ""
            "void update(vec2 sd)"
            "{"
            "   if (particles[gid].life.w > particles[gid].life.y + particles[gid].life.z)"
            "       particles[gid].color.a += dt / particles[gid].life.x;"
            "   else if (particles[gid].life.w > particles[gid].life.z)"
            "       particles[gid].color.a = 1.f;"
            "   else"
            "   {"
            "       particles[gid].color.a -= dt / particles[gid].life.z;"
            "       particles[gid].velocity.x += .01 * (2*rnd(sd) - 1.f); ++sd.x;"
            "       particles[gid].velocity.y += .01 * (2*rnd(sd) - 1.f); ++sd.x;"
            "       particles[gid].velocity.z += .01 * (2*rnd(sd) - 1.f); ++sd.x;"
            "       particles[gid].position.x += particles[gid].velocity.x;"
            "       particles[gid].position.y += particles[gid].velocity.y;"
            "       particles[gid].position.z += particles[gid].velocity.z;"
            "   }"
            ""
            "   float rotation = drot * (particles[gid].life.x+particles[gid].life.y+particles[gid].life.z - particles[gid].life.w);"
            "   float rot_c = cos(rotation);"
            "   float rot_s = sin(rotation);"
            "   particles[gid].gPosition.x = rot_c * particles[gid].position.x + rot_s * particles[gid].position.z;"
            "   particles[gid].gPosition.y = particles[gid].position.y;"
            "   particles[gid].gPosition.z = rot_c * particles[gid].position.z - rot_s * particles[gid].position.x;"
            "}"
            ""
            "void main()"
            "{"
            "   if (gid >= n_particles){return;}"
            "       if (dt < 0.f)"
            "       {"
            "           spawn(vec2(gid, gid));"
            "       }"
            "       else"
            "       {"
            "           vec2 sd = particles[gid].position.xy;"
            "           particles[gid].life.w -= dt;"
            "           if (particles[gid].life.w > 0.f)"
            "               update(sd);"
            "           else"
            "               spawn(sd);"
            "       }"
            "}";

    if (compute_shader == nullptr)
        compute_shader = new Shader(CS);
    SIMDParticleSystem::init(vertex, v_count, tex, uv, atlas);
}

void scene::BenchmarkScene::SIMDParticleSystem::restart()
{
    _tot_particles = max_particles;
    debt_particles = 0.f;

    need_upload = true;
}

void scene::BenchmarkScene::SIMDParticleSystem::upload()
{
    if (!need_upload) return;

    glBindBuffer(GL_ARRAY_BUFFER, ssbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float)*20*max_particles, nullptr, GL_STATIC_DRAW);

    auto scal_rot = new float[2*max_particles];
#pragma omp parallel for num_threads(8)
    for (decltype(max_particles) i = 0; i < max_particles; ++i)
        scal_rot[i] = i % 2 == 0 ? 0.025f : 0.f;
    glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float)*2*max_particles, scal_rot, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    delete [] scal_rot;

    n_particles = max_particles;
    update(-1.f);
    n_particles = 0;

    need_upload = false;
}
void scene::BenchmarkScene::CUDAParticleSystem::upload()
{
    if (SIMDParticleSystem::need_upload && res != nullptr)
    {
        PX_CUDA_CHECK(cudaGraphicsUnregisterResource(res));
        res = nullptr;
    }
    SIMDParticleSystem::upload();
}

void scene::BenchmarkScene::SIMDParticleSystem::update(float dt, glm::vec3 *cam_pos)
{
    if (n_particles != total())
    {
        debt_particles += std::min(0.05f, dt) * birth_rate;
        auto new_particles = static_cast<int>(debt_particles);
        debt_particles -= new_particles;
        n_particles += new_particles;
        if (n_particles > total()) n_particles = total();
    }
}
void scene::BenchmarkScene::ComputeShaderParticleSystem::update(float dt, glm::vec3 *cam_pos)
{
    SIMDParticleSystem::update(dt, cam_pos);

    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, ssbo);
    compute_shader->activate();
    compute_shader->set("dt", dt);
    compute_shader->set("n_particles", static_cast<int>(count()));
    glDispatchCompute(std::ceil(count()/float(COMPUTE_SHADER_WORK_GROUP_SIZE)), 1, 1);
    compute_shader->activate(false);
}
void scene::BenchmarkScene::CUDAParticleSystem::update(float dt,
                                                       glm::vec3 *cam_pos)
{
    if (res == nullptr)
    {
        PX_CUDA_CHECK(cudaGraphicsGLRegisterBuffer(&res, SIMDParticleSystem::ssbo, cudaGraphicsRegisterFlagsNone));
    }

    SIMDParticleSystem::update(dt, cam_pos);

    PX_CUDA_CHECK(cudaGraphicsMapResources(1, &res, 0));
    PX_CUDA_CHECK(cudaGraphicsResourceGetMappedPointer(&buffer, &buffer_size, res));
    cudaUpdate(buffer, count(), dt);
    cudaDeviceSynchronize();
    PX_CUDA_CHECK(cudaGraphicsUnmapResources(1, &res, 0));
}

void scene::BenchmarkScene::SIMDParticleSystem::render(GLenum gl_draw_mode)
{
    draw_shader->activate();
    glBindVertexArray(vao);
    glVertexAttribDivisor(5, 0);    // vertices
    glVertexAttribDivisor(0, 1);    // position
    glVertexAttribDivisor(2, 1);    // color
    glDrawArraysInstanced(gl_draw_mode, 0, n_vertices, count());

    glBindVertexArray(0);
    draw_shader->activate(false);
}

#endif // PX_CG_SCENE_IMPL_BENCHMARK_SIMD_PARTICLE_SYSTEM
