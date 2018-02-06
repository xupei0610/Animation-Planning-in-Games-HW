#ifndef PX_CG_SCENE_EMPTY_SCENE_HPP
#define PX_CG_SCENE_EMPTY_SCENE_HPP

#include "scene/base_scene.hpp"
#include "shader/skybox.hpp"

namespace px { namespace scene
{
class EmptyScene;
} }

class px::scene::EmptyScene : public BaseScene
{
public:
    glm::vec3 gravity;
    glm::vec3 resistance;

    unsigned int width, height;

public:
    void init(Scene &scene) override;
    void restart(Scene &scene) override;
    void upload(Shader &scene_shader) override;
    void render(Shader &scene_shader) override;
    void render() override;

    EmptyScene();
    ~EmptyScene() override;

protected:
    unsigned int vao[1], vbo[1], texture[4];
    float floor_v[66];
    SkyBox *skybox;
    bool need_upload;
};

#endif // PX_CG_SCENE_EMPTY_SCENE_HPP