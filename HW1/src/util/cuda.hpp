#ifndef PX_CG_CUDA_HPP
#define PX_CG_CUDA_HPP

#include <exception>
#include <string>
#include <algorithm>

#ifdef USE_CUDA
#include <cuda.h>
#include <cuda_runtime.h>
#endif

namespace px
{
class CUDAError;
}

class px::CUDAError : public std::exception
{
public:
    CUDAError(const std::string &msg, const int &code=0)
            : msg(msg), err_code(code)
    {}
    const char *what() const noexcept override
    {
        return msg.data();
    }
    inline int code() const
    {
        return err_code;
    }
protected:
    std::string msg;
    int err_code;
};

#ifdef USE_CUDA
#   define _PX_CUDA_CHECK_STR_HELPER(X) #X
#   define _PX_CUDA_CHECK_STR(X) _PX_CUDA_CHECK_STR_HELPER(X)
#   define PX_CUDA_CHECK(cmd)                                               \
    do {                                                                    \
        auto res = (cmd);                                                   \
        if (cudaSuccess != res)                                             \
            throw px::CUDAError(std::string(                                \
                                  "CUDA runtime error at " __FILE__         \
                                  " (line " _PX_CUDA_CHECK_STR(__LINE__)    \
                                  ") code " + std::to_string(res) +         \
                                  " with message: \n    ") +               \
                                  cudaGetErrorString(res),                  \
                                  static_cast<int>(res));                   \
    }                                                                       \
    while (0);


#define PX_CUDA_THREADS_PER_BLOCK    256
#define PX_CUDA_MAX_STREAMS    8

#define PX_CUDA_LOOP(index_var, total)                                          \
    for (int index_var = blockIdx.x * blockDim.x + threadIdx.x, __delta = blockDim.x * gridDim.x;                 \
    index_var < (total); index_var += __delta)

namespace px { namespace cuda
{

inline int blocks(const int &num)
{
    return (num + PX_CUDA_THREADS_PER_BLOCK - 1) / PX_CUDA_THREADS_PER_BLOCK;
}

}}

#else

#define __device__
#define __host__
#define __global__
#define __restrict__

#define curandState_t int

#endif

#define PX_CUDA_CALLABLE __device__ __host__

#endif // PX_CG_CUDA_HPP