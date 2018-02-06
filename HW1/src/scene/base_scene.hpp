#ifndef PX_CG_SCENE_BASE_SCENE_HPP
#define PX_CG_SCENE_BASE_SCENE_HPP

#include "shader/base_shader.hpp"
namespace px {
class Scene;

namespace scene
{
class BaseScene;
} }

class px::scene::BaseScene
{
public:
    virtual void init(Scene &scene) {};
    virtual void restart(Scene &scene) {};
    virtual void upload(Shader &scene_shader) {};
    virtual void update(float dt) {}
    // pre-render
    virtual void render(Shader &scene_shader) {};
    // post-render
    virtual void render() {};

    virtual ~BaseScene() = default;
protected:
    BaseScene() = default;

private:
    BaseScene &operator=(BaseScene const &) = delete;
    BaseScene &operator=(BaseScene &&) = delete;
};

#endif // PX_CG_SCENE_BASE_SCENE_HPP
