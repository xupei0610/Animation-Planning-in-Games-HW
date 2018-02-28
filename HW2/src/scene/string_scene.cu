#include "util/cuda.hpp"
#include "scene/string_scene.hpp"

#include <cfloat>
#include <cublas.h>

using namespace px;

__constant__
scene::StringScene::CudaParam_t cloth_param[1];

struct CudaParticle_t
{
    float3 *vel;
    float *mass;

    float3 *temp_x, *a_dv, *b_dx, *b_dv, *c_dx, *c_dv, *d_dx, *d_dv;

    unsigned int n_particles = 0;
} properties;

__device__
float3 force(float3 const &p1, float3 const &p2,
             float3 const &v1, float3 const &v2,
             float kd, float ks, float rest_len)
{
    auto dx = p1 - p2;
    auto x = sqrtf(dx.x * dx.x + dx.y * dx.y + dx.z * dx.z);
    if (x > FLT_MIN)
    {
        auto dv = v1 - v2;
        auto F = dx/x;
        return F * (- ks * (x - rest_len)
                    - kd * (dv.x*F.x + dv.y*F.y + dv.z*F.z));
    }
    return make_float3(0.f, 0.f, 0.f);
}

__global__
void applyForce(const float3 *pos, const float3 *vel, const float *mass,
                float3 *acc,
                unsigned int n)
{
    PX_CUDA_LOOP(idx, n)
    {
        if (mass[idx] != FLT_MAX)
        {
            auto row = idx / cloth_param->grid_x;
            auto col = idx % cloth_param->grid_x;
            float3 f{0.f, 0.f, 0.f};
            auto p = pos[idx]; auto v = vel[idx];
            if (row < cloth_param->grid_y-1)
            {
                f = f + force(p, pos[idx + cloth_param->grid_x],
                              v, vel[idx + cloth_param->grid_x],
                              cloth_param->kd, cloth_param->ks, cloth_param->rest_len); // down
                if (row < cloth_param->grid_y - 2)
                    f = f + force(p, pos[idx + cloth_param->grid_x + cloth_param->grid_x],
                                  v, vel[idx + cloth_param->grid_x + cloth_param->grid_x],
                                  cloth_param->kd, cloth_param->ks, cloth_param->rest_len_bend); // down 2
                if (col > 0)
                {
                    f = f + force(p, pos[idx + cloth_param->grid_x - 1],
                                  v, vel[idx + cloth_param->grid_x - 1],
                                  cloth_param->kd, cloth_param->ks, cloth_param->rest_len_shear); // bottom left
//                    f = f + windForce(idx, idx - 1, idx + cloth_param->grid_x);
                }
                if (col < cloth_param->grid_x - 1)
                {
                    f = f + force(p, pos[idx + cloth_param->grid_x + 1],
                                  v, vel[idx + cloth_param->grid_x + 1],
                                  cloth_param->kd, cloth_param->ks, cloth_param->rest_len_shear); // bottom right
//                    f = f + windForce(idx, idx + cloth_param->grid_x, idx + 1);
                }
            }
            if (row > 0)
            {
                f = f + force(p, pos[idx - cloth_param->grid_x],
                              v, vel[idx - cloth_param->grid_x],
                              cloth_param->kd, cloth_param->ks, cloth_param->rest_len); // top
                if (row > 1)
                    f = f + force(p, pos[idx - cloth_param->grid_x - cloth_param->grid_x],
                                  v, vel[idx - cloth_param->grid_x - cloth_param->grid_x],
                                  cloth_param->kd, cloth_param->ks, cloth_param->rest_len_bend); // top 2
                if (col > 0)
                {
                    f = f + force(p, pos[idx - cloth_param->grid_x - 1],
                                  v, vel[idx - cloth_param->grid_x - 1],
                                  cloth_param->kd, cloth_param->ks, cloth_param->rest_len_shear); // top left
//                    f = f + windForce(idx, idx - cloth_param->grid_x, idx - 1);
                }
                if (col < cloth_param->grid_x - 1)
                {
                    f = f + force(p, pos[idx - cloth_param->grid_x + 1],
                                  v, vel[idx - cloth_param->grid_x + 1],
                                  cloth_param->kd, cloth_param->ks, cloth_param->rest_len_shear); // top right
//                    f = f + windForce(idx, idx + 1, idx - cloth_param->grid_x);
                }
            }
            if (col > 0)
            {
                f = f + force(p, pos[idx - 1],
                              v, vel[idx - 1],
                              cloth_param->kd, cloth_param->ks, cloth_param->rest_len); // left
                if (col > 1)
                    f = f + force(p, pos[idx - 2],
                                  v, vel[idx - 2],
                                  cloth_param->kd, cloth_param->ks, cloth_param->rest_len_bend); // left 2
            }
            if (col < cloth_param->grid_x - 1)
            {
                f = f + force(p, pos[idx + 1],
                              v, vel[idx + 1],
                              cloth_param->kd, cloth_param->ks, cloth_param->rest_len); // right
                if (col < cloth_param->grid_x - 2)
                    f = f + force(p, pos[idx + 2],
                                  v, vel[idx + 2],
                                  cloth_param->kd, cloth_param->ks, cloth_param->rest_len_bend); // right 2
            }
            acc[idx] = cloth_param->gravity + f / mass[idx]
                     - vel[idx] * (pos[idx].y == cloth_param->field_height ? cloth_param->ground_friction : cloth_param->air_friction);
        }
    }
}

__global__
void applyAcc(float dt, float3 sphere_pos, float sphere_radius,
              float3 *init_pos, float3 * init_vel, float *mass,
              const float3 *a_dv,
              const float3 *b_dx, const float3 *b_dv,
              const float3 *c_dx, const float3 *c_dv,
              const float3 *d_dx, const float3 *d_dv,
              unsigned int n)
{
    PX_CUDA_LOOP(idx, n)
    {
        auto m = mass[idx];
        if (m != FLT_MAX)
        {
            auto dxdt = 1.f / 6.f * (init_vel[idx] + 2.f*(b_dx[idx]+c_dx[idx]) + d_dx[idx]);
            auto dvdt = 1.f / 6.f * (a_dv[idx] + 2.f*(b_dv[idx]+c_dv[idx]) + d_dv[idx]);

            auto p = init_pos[idx] + dxdt * dt;
            auto v = init_vel[idx] + dvdt * dt;

            auto x = p - sphere_pos;
            auto x_len = sqrtf(x.x*x.x + x.y*x.y + x.z*x.z);
            auto dx = sphere_radius - x_len;
            if (dx > 0)
            {
                auto norm = x / x_len;
                v = 0.9f * (v - norm * (2.f * (v.x*norm.x+v.y*norm.y+v.z*norm.z)));
                p = sphere_pos + (sphere_radius + cloth_param->cloth_thickness + .9f*dx) * norm;
            }
            if (p.y < cloth_param->field_height)
            {
                v.y = 0.f;
                p.y = cloth_param->field_height + cloth_param->cloth_thickness;
            }

            init_pos[idx] = p;
            init_vel[idx] = v;
        }
    }
}


void cudaRK4(float dt, float3 *init_x, item::Sphere *sphere)
{

    auto a_dx = reinterpret_cast<float *>(properties.vel);
    applyForce<<<cuda::blocks(properties.n_particles), PX_CUDA_THREADS_PER_BLOCK>>>
            (init_x, properties.vel, properties.mass, properties.a_dv, properties.n_particles);

    PX_CUDA_CHECK(cudaMemcpy(properties.temp_x, init_x, properties.n_particles*sizeof(float3), cudaMemcpyDeviceToDevice));
    PX_CUDA_CHECK(cudaMemcpy(properties.b_dx,   properties.vel, properties.n_particles*sizeof(float3), cudaMemcpyDeviceToDevice));
    cublasSaxpy(properties.n_particles, dt*0.5f,
                a_dx, 1,
                reinterpret_cast<float *>(properties.temp_x), 1);
    cublasSaxpy(properties.n_particles, dt*0.5f,
                reinterpret_cast<float *>(properties.a_dv), 1,
                reinterpret_cast<float *>(properties.b_dx), 1);
    applyForce<<<cuda::blocks(properties.n_particles), PX_CUDA_THREADS_PER_BLOCK>>>
            (properties.temp_x, properties.b_dx, properties.mass, properties.b_dv, properties.n_particles);

    PX_CUDA_CHECK(cudaMemcpy(properties.temp_x, init_x, properties.n_particles*sizeof(float3), cudaMemcpyDeviceToDevice));
    PX_CUDA_CHECK(cudaMemcpy(properties.c_dx,   properties.vel, properties.n_particles*sizeof(float3), cudaMemcpyDeviceToDevice));
    cublasSaxpy(properties.n_particles, dt*0.5f,
                reinterpret_cast<float *>(properties.b_dx), 1,
                reinterpret_cast<float *>(properties.temp_x), 1);
    cublasSaxpy(properties.n_particles, dt*0.5f,
                reinterpret_cast<float *>(properties.b_dv), 1,
                reinterpret_cast<float *>(properties.c_dx), 1);
    applyForce<<<cuda::blocks(properties.n_particles), PX_CUDA_THREADS_PER_BLOCK>>>
            (properties.temp_x, properties.c_dx, properties.mass, properties.c_dv, properties.n_particles);

    PX_CUDA_CHECK(cudaMemcpy(properties.temp_x, init_x, properties.n_particles*sizeof(float3), cudaMemcpyDeviceToDevice));
    PX_CUDA_CHECK(cudaMemcpy(properties.d_dx,   properties.vel, properties.n_particles*sizeof(float3), cudaMemcpyDeviceToDevice));
    cublasSaxpy(properties.n_particles, dt,
                reinterpret_cast<float *>(properties.c_dx), 1,
                reinterpret_cast<float *>(properties.temp_x), 1);
    cublasSaxpy(properties.n_particles, dt,
                reinterpret_cast<float *>(properties.c_dv), 1,
                reinterpret_cast<float *>(properties.d_dx), 1);
    applyForce<<<cuda::blocks(properties.n_particles), PX_CUDA_THREADS_PER_BLOCK>>>
            (properties.temp_x, properties.d_dx, properties.mass, properties.d_dv, properties.n_particles);

    applyAcc<<<cuda::blocks(properties.n_particles), PX_CUDA_THREADS_PER_BLOCK>>>
            (dt, make_float3(sphere->pos().x, sphere->pos().y, sphere->pos().z), sphere->scal().x,
             init_x, properties.vel, properties.mass,
             properties.a_dv,
             properties.b_dx, properties.b_dv,
             properties.c_dx, properties.c_dv,
             properties.d_dx, properties.d_dv,
             properties.n_particles);
}

void scene::StringScene::cudaInit(unsigned int n_particles, const float *mass)
{
    PX_CUDA_CHECK(cudaMemcpyToSymbol(cloth_param, &cuda_param, sizeof(CudaParam_t), 0,
                                     cudaMemcpyHostToDevice));
    if (properties.n_particles != n_particles)
    {
        cudaBufferFree();

        PX_CUDA_CHECK(cudaMalloc(&properties.vel,  sizeof(float3)*n_particles));
        PX_CUDA_CHECK(cudaMalloc(&properties.mass, sizeof(float)*n_particles));
        PX_CUDA_CHECK(cudaMalloc(&properties.temp_x, sizeof(float3)*n_particles));
        PX_CUDA_CHECK(cudaMalloc(&properties.a_dv, sizeof(float3)*n_particles));
        PX_CUDA_CHECK(cudaMalloc(&properties.b_dx, sizeof(float3)*n_particles));
        PX_CUDA_CHECK(cudaMalloc(&properties.b_dv, sizeof(float3)*n_particles));
        PX_CUDA_CHECK(cudaMalloc(&properties.c_dx, sizeof(float3)*n_particles));
        PX_CUDA_CHECK(cudaMalloc(&properties.c_dv, sizeof(float3)*n_particles));
        PX_CUDA_CHECK(cudaMalloc(&properties.d_dx, sizeof(float3)*n_particles));
        PX_CUDA_CHECK(cudaMalloc(&properties.d_dv, sizeof(float3)*n_particles));

        properties.n_particles = n_particles;
    }

    PX_CUDA_CHECK(cudaMemcpy(properties.mass, mass, sizeof(float)*n_particles, cudaMemcpyHostToDevice));
    PX_CUDA_CHECK(cudaMemset(properties.vel, 0, sizeof(float3)*n_particles));
}
void scene::StringScene::cudaUpdate(void *buffer, float dt, unsigned int n_iter, item::Sphere *sphere)
{
    auto pos = reinterpret_cast<float3 *>(buffer);
    for (decltype(n_iter) i = 0; i < n_iter; ++i)
    {
        cudaRK4(dt, pos, sphere);
    }
}

void scene::StringScene::cudaBufferFree()
{
    if (properties.n_particles == 0)
        return;

    PX_CUDA_CHECK(cudaFree(properties.vel));
    PX_CUDA_CHECK(cudaFree(properties.mass));
    PX_CUDA_CHECK(cudaFree(properties.temp_x));
    PX_CUDA_CHECK(cudaFree(properties.a_dv));
    PX_CUDA_CHECK(cudaFree(properties.b_dx));
    PX_CUDA_CHECK(cudaFree(properties.b_dv));
    PX_CUDA_CHECK(cudaFree(properties.c_dx));
    PX_CUDA_CHECK(cudaFree(properties.c_dv));
    PX_CUDA_CHECK(cudaFree(properties.d_dx));
    PX_CUDA_CHECK(cudaFree(properties.d_dv));
    properties.n_particles = 0;
}

