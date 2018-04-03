#ifndef PX_CG_GLOBAL_HPP
#define PX_CG_GLOBAL_HPP

#ifndef ASSET_PATH
#define ASSET_PATH "../asset"
#endif

#ifndef ITEM_REGISTER_CAPACITY
#define ITEM_REGISTER_CAPACITY 10000
#endif

#ifndef OBJECT_REGISTER_CAPACITY
#define OBJECT_REGISTER_CAPACITY 1000
#endif

#if __cplusplus < 201300

#include <memory>
namespace std
{
template<typename T, typename... Args>
std::unique_ptr<T> make_unique(Args &&... args)
{
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}
}
#endif

#endif
