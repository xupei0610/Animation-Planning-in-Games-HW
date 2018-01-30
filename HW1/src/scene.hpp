#ifndef PX_CG_SCENE_HPP
#define PX_CG_SCENE_HPP

#include <string>

#include "character.hpp"
#include "shader/base_shader.hpp"
#include "item.hpp"
#include "scene/base_scene.hpp"

namespace px {
class Scene;
}

class px::Scene
{
public:
    Option *opt;
    Camera cam;
    Character character;

public:
    Scene(Option *opt);

    void init();
    bool run(float dt);
    void render();

    void restart();
    void resize(int w, int h);
    void upload();
    std::shared_ptr<Item> const &add(Item *obj);

    [[noreturn]]
    void err(std::string const & msg);

    ~Scene();
    Scene &operator=(Scene const &) = default;
    Scene &operator=(Scene &&) = default;

    void load(scene::BaseScene *scene);
    inline const scene::BaseScene *scene() { return _scene; }
protected:
    Shader *geo_shader;
    Shader *light_shader;

    std::vector<std::shared_ptr<Item> > objs;

    unsigned int ubo;
    unsigned int deferred_vao, deferred_vbo;
    unsigned int deferred_fbo[2], deferred_rbo[2], deferred_out[7];

private:
    scene::BaseScene *_scene;
};

#endif
