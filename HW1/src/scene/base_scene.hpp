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
    virtual void init(Scene &scene) = 0;
    virtual void restart(Scene &scene) = 0;
    virtual void upload(Shader &scene_shader) = 0;
    virtual void render(Shader &scene_shader,
                        glm::mat4 const &view, glm::mat4 const &proj) = 0;
    virtual void render(glm::mat4 const &view, glm::mat4 const &proj) = 0;

    virtual ~BaseScene() = default;
protected:
    BaseScene() = default;
};

#endif // PX_CG_SCENE_BASE_SCENE_HPP
