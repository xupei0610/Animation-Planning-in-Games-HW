#ifndef PX_CG_SCENE_BOUNCE_HPP
#define PX_CG_SCENE_BOUNCE_HPP

#include "scene/empty.hpp"
#include "shader/skybox.hpp"

namespace px { namespace scene
{
class Bounce;
} }

class px::scene::Bounce : public Empty
{
public:
    using Empty::width;
    using Empty::height;

    void restart(Scene &scene) override;

    Bounce();
    ~Bounce() override = default;
};

#endif // PX_CG_SCENE_BOUNCE_HPP
