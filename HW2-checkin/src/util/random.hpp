#ifndef PX_CG_UTIL_RANDOM_HPP
#define PX_CG_UTIL_RANDOM_HPP

#include <random>

namespace px
{
inline float rnd2()
{
    std::uniform_real_distribution<float> static rd(1, 2);
    std::mt19937 static sd(std::random_device{}());

    return rd(sd);
}
inline float rnd_np()
{
    std::uniform_real_distribution<float> static rd(-1, 1);
    std::mt19937 static sd(std::random_device{}());

    return rd(sd);
}
inline float rnd()
{
    std::uniform_real_distribution<float> static rd(0, 1);
    std::mt19937 static sd(std::random_device{}());

    return rd(sd);
}
}
#endif // PX_CG_UTIL_RANDOM_HPP