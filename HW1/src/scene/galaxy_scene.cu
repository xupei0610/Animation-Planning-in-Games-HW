#include "util/cuda.hpp"
#include "scene/galaxy_scene.hpp"
#include "config.hpp"

#include <curand.h>
#include <curand_kernel.h>
#include <math_functions.h>

using namespace px;

struct Buffer_t
{
    struct
    {
        float x;
        float y;
        float z;
        float w; // mass
    } position;
    struct
    {
        float x;
        float y;
        float z;
        float w; // flag
    } velocity;
    struct
    {
        float x;
        float y;
        float z;
        float _;
    } d_pos;
};
__global__
void spawnGalaxy(void *buffer, unsigned int n)
{
    auto particles = reinterpret_cast<Buffer_t *>(buffer);
    const float r0 = 4.f;

    curandState_t sd;
    curand_init(uintptr_t(buffer) + threadIdx.x, blockIdx.x, 0, &sd);

    PX_CUDA_LOOP(i, n)
    {
        particles[i].position.x = r0 * curand_uniform(&sd) + (i % 2 == 0 ? 1.5f : -1.5f);
        particles[i].position.y = r0 * curand_uniform(&sd);
        particles[i].position.z = r0 * curand_uniform(&sd);
        particles[i].velocity.x = 0.f;
        particles[i].velocity.y = 0.f;
        particles[i].velocity.z = 0.f;
        particles[i].position.w = .05f + curand_uniform(&sd) * 10.f; // mass
        particles[i].velocity.w = 160706.f; // flag to be spawned
        particles[i].d_pos.x = 0.f;
        particles[i].d_pos.y = 0.f;
        particles[i].d_pos.z = 0.f;
    }
}

__global__
void updateGalaxy(void *buffer, unsigned int n, float dt)
{
    extern __shared__ float4 sh_pos[];

    const auto EPS = 0.0001f;
    const auto factor = 0.002f;
    const auto damping = .999f;
    auto f = dt * factor;

    auto particles = reinterpret_cast<Buffer_t *>(buffer);

    float x, y, z;
    float r_x, r_y, r_z;
    float dist_sqr, dist_six;
    float acc_x, acc_y, acc_z;
    unsigned int p;
    PX_CUDA_LOOP(gid, n)
    {
        particles[gid].position.x += particles[gid].d_pos.x;
        particles[gid].position.y += particles[gid].d_pos.y;
        particles[gid].position.z += particles[gid].d_pos.z;

        x = particles[gid].position.x;
        y = particles[gid].position.y;
        z = particles[gid].position.z;

        acc_x = 0;
        acc_y = 0;
        acc_z = 0;
        for (unsigned int i = 0, idx = threadIdx.x; idx < n; i += blockDim.x, idx += blockDim.x)
        {
            sh_pos[threadIdx.x].x = particles[idx].position.x;
            sh_pos[threadIdx.x].y = particles[idx].position.y;
            sh_pos[threadIdx.x].z = particles[idx].position.z;
            sh_pos[threadIdx.x].w = particles[idx].position.w;
            __syncthreads();

            p = n - i > blockDim.x ? blockDim.x : n - i;
#pragma unroll 256
            for (unsigned int j = 0; j < blockDim.x ; ++j)
            {
                r_x = sh_pos[j].x - x;
                r_y = sh_pos[j].y - y;
                r_z = sh_pos[j].z - z;
                dist_sqr = r_x * r_x + r_y * r_y + r_z * r_z + EPS;
                dist_six = dist_sqr * dist_sqr * dist_sqr;
                dist_sqr = rsqrtf(dist_six);
                dist_six = sh_pos[threadIdx.x].w * dist_sqr;
                acc_x += r_x * dist_six;
                acc_y += r_y * dist_six;
                acc_z += r_z * dist_six;
            }
            __syncthreads();
        }
        particles[gid].velocity.x += acc_x * f;
        particles[gid].velocity.x *= damping;
        particles[gid].velocity.y += acc_y * f;
        particles[gid].velocity.y *= damping;
        particles[gid].velocity.z += acc_z * f;
        particles[gid].velocity.z *= damping;
        particles[gid].d_pos.x = particles[gid].velocity.x * f;
        particles[gid].d_pos.y = particles[gid].velocity.y * f;
        particles[gid].d_pos.z = particles[gid].velocity.z * f;
    }
}

void scene::GalaxyScene::ComputeShaderParticleSystem::cudaSpawn(void * buffer, unsigned int n)
{
    spawnGalaxy<<<cuda::blocks(n), PX_CUDA_THREADS_PER_BLOCK>>>(buffer, n);
}

void scene::GalaxyScene::ComputeShaderParticleSystem::cudaUpdate(void * buffer, unsigned int n, float dt)
{
    updateGalaxy<<<cuda::blocks(n), PX_CUDA_THREADS_PER_BLOCK, sizeof(float4)*PX_CUDA_THREADS_PER_BLOCK>>>(buffer, n, dt);
}
