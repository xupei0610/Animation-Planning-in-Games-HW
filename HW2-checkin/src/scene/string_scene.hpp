#ifndef PX_CG_SCENE_STRING_SCENE_HPP
#define PX_CG_SCENE_STRING_SCENE_HPP

#include "scene/base_scene.hpp"
#include "shader/base_shader.hpp"

namespace px { namespace scene
{
class StringScene;
} }

class px::scene::StringScene : public BaseScene
{
public:
    void init(Scene &scene) override;
    void restart(Scene &scene) override;
    void upload(Shader &scene_shader) override;
    void update(float dt) override;
    void render(Shader &scene_shader) override;
    void render() override;

    StringScene();
    ~StringScene() override;

protected:
    unsigned int vao[1], vbo[2];
    Shader *shader;

private:
    class impl;
    std::unique_ptr<impl> pimpl;
};

#endif // PX_CG_SCENE_STRING_SCENE_HPP
