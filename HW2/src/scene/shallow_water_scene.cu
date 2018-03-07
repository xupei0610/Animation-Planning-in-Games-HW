#include "util/cuda.hpp"
#include "scene/shallow_water_scene.hpp"

#include <cublas.h>
#include <curand.h>
#include <curand_kernel.h>

using namespace px;

__constant__
scene::ShallowWaterScene::CudaParam_t shallow_water_param[1];

struct CudaShallowWaterProperties
{
    float *u, *v, *h_x, *u_x, *v_x, *h_y, *u_y, *v_y;
    glm::vec3 *n;

    int n_grids = 0;
} sw_prop;

__global__
void shallowWaterDrop(float *h, int seed)
{
    curandState_t sd;
    curand_init(clock64(), blockIdx.x, 0, &sd);

    auto boundary = 5;
    auto row = std::ceil(curand_uniform(&sd) * (shallow_water_param->grid_y-(boundary+boundary))) + boundary;
    auto col = std::ceil(curand_uniform(&sd) * (shallow_water_param->grid_x-(boundary+boundary))) + boundary;

    auto height = curand_uniform(&sd)*.75f + 0.1f;
    auto r = curand_uniform(&sd) * min(shallow_water_param->grid_x, shallow_water_param->grid_y) * .1f + 0.001f;
    auto gap = 1.f / r;
    for (float i = -1; i < 1; i += gap)
    {
        for (float j = -1; j < 1; j += gap)
        {
            auto tar_y = static_cast<int>(row+i*r);
            auto tar_x = static_cast<int>(col+j*r);
            if (tar_y > boundary && tar_y < shallow_water_param->grid_y - boundary
                && tar_x > boundary && tar_x < shallow_water_param->grid_x - boundary)
            {
                auto tar = tar_y*shallow_water_param->grid_x+tar_x;
                h[tar] += (.5f + curand_uniform(&sd)*.5f) * height * expf(-5.f*(i*i+j*j));
            }
        }
    }
}

__global__
void shallowWaterX(unsigned int n, float dt,
                   float *h, float *u, float *v,
                   float *h_x, float *u_x, float *v_x)
{
    PX_CUDA_LOOP(tar, n)
    {
        auto row = tar / shallow_water_param->grid_x;
        auto col = tar % shallow_water_param->grid_x;

        if (row < shallow_water_param->grid_y - 1 &&
            col < shallow_water_param->grid_x - 2)
        {
            auto tar01 = tar + 1;
            auto tar11 = tar01 + shallow_water_param->grid_x;

            h_x[tar] = .5f *
                       ((h[tar11] + h[tar01]) - (u[tar11] - u[tar01]) * dt *
                                                shallow_water_param->inv_gap_x);
            if (h_x[tar] < shallow_water_param->height_min)
                h_x[tar] = shallow_water_param->height_min;
            else if (isnan(h_x[tar]))
                h_x[tar] = 1.f;

            u_x[tar] = .5f *
                       (shallow_water_param->damping*(u[tar11] + u[tar01]) -
                        dt * shallow_water_param->inv_gap_x *
                        (u[tar11] * u[tar11] / h[tar11] -
                         u[tar01] * u[tar01] / h[tar01]
                         + shallow_water_param->half_g *
                           (h[tar11] * h[tar11] - h[tar01] * h[tar01])));
            v_x[tar] = .5f *
                       (shallow_water_param->damping*(v[tar11] + v[tar01]) -
                        dt * shallow_water_param->inv_gap_x *
                        (u[tar11] * v[tar11] / h[tar11] -
                         u[tar01] * v[tar01] / h[tar01]));
        }

    }
}

__global__
void shallowWaterY(unsigned int n, float dt,
                   float *h, float *u, float *v,
                   float *h_y, float *u_y, float *v_y)
{
    PX_CUDA_LOOP(tar, n)
    {
        auto row = tar / shallow_water_param->grid_x;
        auto col = tar % shallow_water_param->grid_x;

        if (row < shallow_water_param->grid_y - 2 &&
            col < shallow_water_param->grid_x - 1)
        {
            auto tar10 = tar + shallow_water_param->grid_x;
            auto tar11 = tar10 + 1;

            h_y[tar] = .5f *
                       ((h[tar11]+h[tar10]) - (v[tar11]-v[tar10])*dt*shallow_water_param->inv_gap_y);
            if (h_y[tar] < shallow_water_param->height_min)
                h_y[tar] = shallow_water_param->height_min;
            else if (isnan(h_y[tar]))
                h_y[tar] = 1.f;

            u_y[tar] = .5f *
                       (shallow_water_param->damping*(u[tar11]+u[tar10]) - dt*shallow_water_param->inv_gap_y *
                                              (v[tar11]*u[tar11]/h[tar11] - v[tar10]*u[tar10]/h[tar10]));
            v_y[tar] = .5f *
                       (shallow_water_param->damping*(v[tar11]+v[tar10]) - dt*shallow_water_param->inv_gap_y *
                                              (v[tar11]*v[tar11]/h[tar11] - v[tar10]*v[tar10]/h[tar10]
                                               + shallow_water_param->half_g * (h[tar11]*h[tar11] - h[tar10]*h[tar10])));
        }
    }
}

__global__
void shallowWaterCompose(unsigned int n, float dt,
                         float *h, float *u, float *v,
                         float *h_x, float *u_x, float *v_x,
                         float *h_y, float *u_y, float *v_y)
{
    PX_CUDA_LOOP(tar, n)
    {
        auto row = tar / shallow_water_param->grid_x;
        auto col = tar % shallow_water_param->grid_x;

        if (row > 0 && row < shallow_water_param->grid_y - 1 &&
            col > 0 && col < shallow_water_param->grid_x - 1)
        {
            auto tar0_1 = tar - 1;
            auto tar_1_1 = tar0_1 - shallow_water_param->grid_x;
            auto tar_10 = tar_1_1 + 1;

            auto x = h[tar];
            auto d = (dt*shallow_water_param->inv_gap_x) * (u_x[tar0_1] - u_x[tar_1_1])
                     + (dt*shallow_water_param->inv_gap_y) * (v_y[tar_10] - v_y[tar_1_1]);
            x -= d;

            if (isnan(x))
            {
                h[tar] = 1.f;
                u[tar] = 0.f;
                v[tar] = 0.f;
            }
            else if (x < shallow_water_param->height_min)
            {
                h[tar] = shallow_water_param->height_min;
                u[tar] = 0.f;
                v[tar] = 0.f;
            }
            else
            {
                u[tar] -=   (dt*shallow_water_param->inv_gap_x) * (u_x[tar0_1]*u_x[tar0_1]/h_x[tar0_1] - u_x[tar_1_1]*u_x[tar_1_1]/h_x[tar_1_1]
                                                                   + shallow_water_param->half_g * (h_x[tar0_1]*h_x[tar0_1] - h_x[tar_1_1]*h_x[tar_1_1]))
                            + (dt*shallow_water_param->inv_gap_y) * (v_y[tar_10]*u_y[tar_10]/h_y[tar_10] - v_y[tar_1_1]*u_y[tar_1_1]/h_y[tar_1_1]);
                v[tar] -=   (dt*shallow_water_param->inv_gap_x) * (u_x[tar0_1]*v_x[tar0_1]/h_x[tar0_1] - u_x[tar_1_1]*v_x[tar_1_1]/h_x[tar_1_1])
                            + (dt*shallow_water_param->inv_gap_y) * (v_y[tar_10]*v_y[tar_10]/h_y[tar_10] - v_y[tar_1_1]*v_y[tar_1_1]/h_y[tar_1_1]
                                                                     + shallow_water_param->half_g * (h_y[tar_10]*h_y[tar_10] - h_y[tar_1_1]*h_y[tar_1_1]));

                if (x > shallow_water_param->height_max)
                {
                    auto scale = shallow_water_param->height_max / x;
                    h[tar] = shallow_water_param->height_max;
                    u[tar] *= scale;
                    v[tar] *= scale;
                }
                else
                    h[tar] = x;
            }
        }
    }
}
__global__
void shallowWaterNorm(float3 *norm, const float *h, unsigned int n)
{
    PX_CUDA_LOOP(tar, n)
    {
        auto row = tar / shallow_water_param->grid_x;
        auto col = tar % shallow_water_param->grid_x;
        if (row > 0 && row < shallow_water_param->grid_y-1 && col > 0 && col < shallow_water_param->grid_x-1)
            norm[tar] = cross(make_float3(shallow_water_param->gap_x, h[tar+1]-h[tar], 0.f),
                              make_float3(0.f, h[tar+shallow_water_param->grid_x]-h[tar], shallow_water_param->gap_y));
        else
            norm[tar] = make_float3(0.f, 1.f, 0.f);
    }
}

void scene::ShallowWaterScene::cudaInit()
{
    auto n_grids = cuda_param.grid_x * cuda_param.grid_y;

    PX_CUDA_CHECK(cudaMemcpyToSymbol(shallow_water_param, &cuda_param, sizeof(CudaParam_t), 0,
                                     cudaMemcpyHostToDevice));
    if (sw_prop.n_grids != n_grids)
    {
        cudaBufferFree();
        PX_CUDA_CHECK(cudaMalloc(&sw_prop.u,   sizeof(float)*n_grids));
        PX_CUDA_CHECK(cudaMalloc(&sw_prop.v,   sizeof(float)*n_grids));
        PX_CUDA_CHECK(cudaMalloc(&sw_prop.h_x, sizeof(float)*n_grids));
        PX_CUDA_CHECK(cudaMalloc(&sw_prop.u_x, sizeof(float)*n_grids));
        PX_CUDA_CHECK(cudaMalloc(&sw_prop.v_x, sizeof(float)*n_grids));
        PX_CUDA_CHECK(cudaMalloc(&sw_prop.h_y, sizeof(float)*n_grids));
        PX_CUDA_CHECK(cudaMalloc(&sw_prop.u_y, sizeof(float)*n_grids));
        PX_CUDA_CHECK(cudaMalloc(&sw_prop.v_y, sizeof(float)*n_grids));
        sw_prop.n_grids = n_grids;
    }

    PX_CUDA_CHECK(cudaMemset(sw_prop.u,   0, sizeof(float)*n_grids));
    PX_CUDA_CHECK(cudaMemset(sw_prop.v,   0, sizeof(float)*n_grids));
    PX_CUDA_CHECK(cudaMemset(sw_prop.h_x, 0, sizeof(float)*n_grids));
    PX_CUDA_CHECK(cudaMemset(sw_prop.u_x, 0, sizeof(float)*n_grids));
    PX_CUDA_CHECK(cudaMemset(sw_prop.v_x, 0, sizeof(float)*n_grids));
    PX_CUDA_CHECK(cudaMemset(sw_prop.h_y, 0, sizeof(float)*n_grids));
    PX_CUDA_CHECK(cudaMemset(sw_prop.u_y, 0, sizeof(float)*n_grids));
    PX_CUDA_CHECK(cudaMemset(sw_prop.v_y, 0, sizeof(float)*n_grids));
}

void scene::ShallowWaterScene::cudaUpdate(void *h_buffer, void *n_buffer, float dt,
                                          unsigned int n_iter, float seed)
{
    auto h = reinterpret_cast<float*>(h_buffer);
    auto n = reinterpret_cast<float3*>(n_buffer);

    if (seed != 0)
        shallowWaterDrop<<<1, 1>>>(h, seed);

    for (decltype(n_iter) i = 0; i < n_iter; ++i)
    {
        shallowWaterX<<<cuda::blocks(sw_prop.n_grids), PX_CUDA_THREADS_PER_BLOCK>>>
              (sw_prop.n_grids, dt,
                      h, sw_prop.u, sw_prop.v,
                      sw_prop.h_x,  sw_prop.u_x, sw_prop.v_x);
        shallowWaterY<<<cuda::blocks(sw_prop.n_grids), PX_CUDA_THREADS_PER_BLOCK>>>
              (sw_prop.n_grids, dt,
                      h, sw_prop.u, sw_prop.v,
                      sw_prop.h_y,  sw_prop.u_y, sw_prop.v_y);
        shallowWaterCompose<<<cuda::blocks(sw_prop.n_grids), PX_CUDA_THREADS_PER_BLOCK>>>
              (sw_prop.n_grids, dt,
                      h, sw_prop.u, sw_prop.v,
                      sw_prop.h_x,  sw_prop.u_x, sw_prop.v_x,
                      sw_prop.h_y,  sw_prop.u_y, sw_prop.v_y);

        cublasScopy(cuda_param.grid_y, h+1, cuda_param.grid_x, h, cuda_param.grid_x);
        cublasScopy(cuda_param.grid_y, sw_prop.u+1, cuda_param.grid_x, sw_prop.u, cuda_param.grid_x);
        cublasScopy(cuda_param.grid_y, sw_prop.v+1, cuda_param.grid_x, sw_prop.v, cuda_param.grid_x);
        cublasSscal(cuda_param.grid_y, -1.f, sw_prop.v, cuda_param.grid_x);

        cublasScopy(cuda_param.grid_y, h+cuda_param.grid_x-2, cuda_param.grid_x, h+cuda_param.grid_x-1, cuda_param.grid_x);
        cublasScopy(cuda_param.grid_y, sw_prop.u+cuda_param.grid_x-2, cuda_param.grid_x, sw_prop.u+cuda_param.grid_x-1, cuda_param.grid_x);
        cublasScopy(cuda_param.grid_y, sw_prop.v+cuda_param.grid_x-2, cuda_param.grid_x, sw_prop.v+cuda_param.grid_x-1, cuda_param.grid_x);
        cublasSscal(cuda_param.grid_y, -1.f, sw_prop.v+cuda_param.grid_x-1, cuda_param.grid_x);

        cublasScopy(cuda_param.grid_x, h+cuda_param.grid_x, 1, h, 1);
        cublasScopy(cuda_param.grid_x, sw_prop.u+cuda_param.grid_x, 1, sw_prop.u, 1);
        cublasScopy(cuda_param.grid_x, sw_prop.v+cuda_param.grid_x, 1, sw_prop.v, 1);
        cublasSscal(cuda_param.grid_x, -1.f, sw_prop.u, 1);

        cublasScopy(cuda_param.grid_x, h+cuda_param.grid_x*(cuda_param.grid_y-2), 1, h+cuda_param.grid_x*(cuda_param.grid_y-1), 1);
        cublasScopy(cuda_param.grid_x, sw_prop.u+cuda_param.grid_x*(cuda_param.grid_y-2), 1, sw_prop.u+cuda_param.grid_x*(cuda_param.grid_y-1), 1);
        cublasScopy(cuda_param.grid_x, sw_prop.v+cuda_param.grid_x*(cuda_param.grid_y-2), 1, sw_prop.v+cuda_param.grid_x*(cuda_param.grid_y-1), 1);
        cublasSscal(cuda_param.grid_x, -1.f, sw_prop.u+cuda_param.grid_x*(cuda_param.grid_y-1), 1);
    }
    shallowWaterNorm<<<cuda::blocks(sw_prop.n_grids), PX_CUDA_THREADS_PER_BLOCK>>>
            (n, h, sw_prop.n_grids);
}

void scene::ShallowWaterScene::cudaBufferFree()
{
    if (sw_prop.n_grids == 0)
        return;

    PX_CUDA_CHECK(cudaFree(sw_prop.u));
    PX_CUDA_CHECK(cudaFree(sw_prop.v));
    PX_CUDA_CHECK(cudaFree(sw_prop.h_x));
    PX_CUDA_CHECK(cudaFree(sw_prop.u_x));
    PX_CUDA_CHECK(cudaFree(sw_prop.v_x));
    PX_CUDA_CHECK(cudaFree(sw_prop.h_y));
    PX_CUDA_CHECK(cudaFree(sw_prop.u_y));
    PX_CUDA_CHECK(cudaFree(sw_prop.v_y));
    sw_prop.n_grids = 0;
}