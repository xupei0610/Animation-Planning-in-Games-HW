#include "util/cuda.hpp"
#include "scene/benchmark_scene.hpp"

#include <curand.h>
#include <curand_kernel.h>
#include <math_functions.hpp>

using namespace px;

__device__
float rnd(float2 *sd)
{
    float m;
    return modff(sinf(fabsf(fmodf(sd->x * 12.9898f + sd->y * 78.233f, 3.14f))) * 43758.5453f, &m);
}

__global__
void benchmarkUpadte(void *buffer,
                     unsigned int n,
                     float dt)
{
    struct Buffer_t
    {
        struct
        {
            float x;
            float y;
            float z;
            float w;
        } position;
        struct
        {
            float x;
            float y;
            float z;
            float w;
        } velocity;
        struct
        {
            float x;
            float y;
            float z;
            float w;
        } life;
        struct
        {
            float r;
            float g;
            float b;
            float a;
        } color;
        struct
        {
            float x;
            float y;
            float z;
            float w;
        } gPosition;
    };

    auto particles = reinterpret_cast<Buffer_t *>(buffer);

    const float PI = 3.1415926535897932384626433832795;
    const float drot = 0.06f * PI;
    float2 sd;

    PX_CUDA_LOOP(i, n)
    {
        if (dt < 0.f || particles[i].life.w < dt)
        {
            sd.x = i; sd.y = i;

            float theta = PI * rnd(&sd) * 2.f;       ++sd.x;
            float phi = acosf(rnd(&sd) * 2.f - 1.f);    ++sd.x;
            float r0 = powf(rnd(&sd) * 125.f, 1.f/3.f); ++sd.x;

            particles[i].position.x = r0 * sin(phi) * cos(theta);
            particles[i].position.y = r0 * cos(phi);
            particles[i].position.z = r0 * sin(phi) * sin(theta);

            particles[i].velocity.x = 0.0001f * particles[i].position.x;
            particles[i].velocity.y = 0.0001f * particles[i].position.y;
            particles[i].velocity.z = 0.0001f * particles[i].position.z;

            particles[i].life.x = 4.f + (2*rnd(&sd) - 1.f) * (4.f / 4);  ++sd.x;
            particles[i].life.y = 1.f + (2*rnd(&sd) - 1.f) * (1.f / 4);  ++sd.x;
            particles[i].life.z = 5.f + (2*rnd(&sd) - 1.f) * (5.f / 4);  ++sd.x;
            particles[i].life.w = particles[i].life.x + particles[i].life.y + particles[i].life.z;

            particles[i].color.r = 1.f;
            particles[i].color.g = .45f;
            particles[i].color.b = .2f;
            particles[i].color.a = 0.f;

            particles[i].gPosition.x = particles[i].position.x;
            particles[i].gPosition.y = particles[i].position.y;
            particles[i].gPosition.z = particles[i].position.z;
        }
        else
        {
            sd.x = particles[i].position.x; sd.y = particles[i].position.y;

            particles[i].life.w -= dt;
            if (particles[i].life.w > particles[i].life.y + particles[i].life.z)
                particles[i].color.a += dt / particles[i].life.x;
            else if (particles[i].life.w > particles[i].life.z)
                particles[i].color.a = 1.f;
            else
            {
                particles[i].color.a -= dt / particles[i].life.z;
                particles[i].velocity.x += .01f * (2.f*rnd(&sd) - 1.f); ++sd.x;
                particles[i].velocity.y += .01f * (2.f*rnd(&sd) - 1.f); ++sd.x;
                particles[i].velocity.z += .01f * (2.f*rnd(&sd) - 1.f);
                particles[i].position.x += particles[i].velocity.x;
                particles[i].position.y += particles[i].velocity.y;
                particles[i].position.z += particles[i].velocity.z;
            }

            float rotation = drot * (particles[i].life.x+particles[i].life.y+particles[i].life.z - particles[i].life.w);
            float rot_c = cos(rotation);
            float rot_s = sin(rotation);
            particles[i].gPosition.x = rot_c * particles[i].position.x + rot_s * particles[i].position.z;
            particles[i].gPosition.y = particles[i].position.y;
            particles[i].gPosition.z = rot_c * particles[i].position.z - rot_s * particles[i].position.x;
        }
    }
}

void scene::BenchmarkScene::CUDAParticleSystem::cudaUpdate(void *buffer,
                                                           unsigned int n,
                                                           float dt)
{
    benchmarkUpadte<<<cuda::blocks(n), PX_CUDA_THREADS_PER_BLOCK>>>(buffer, n, dt);
}