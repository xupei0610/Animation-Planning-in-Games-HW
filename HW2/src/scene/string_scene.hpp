#ifndef PX_CG_SCENE_STRING_SCENE_HPP
#define PX_CG_SCENE_STRING_SCENE_HPP

#include <memory>

#include "scene/base_scene.hpp"
#include "shader/base_shader.hpp"
#include "particle.hpp"

namespace px { namespace scene
{
class StringScene;
} }

class px::scene::StringScene : public BaseScene
{
public:
    enum class Integrator
    {
        Verlet,
        VelocityVerlet,
        Euler,
        SemiEuler
    };

    const std::string system_name;
    bool pause;

    void init(Scene &scene) override;
    void restart(Scene &scene) override;
    void upload(Shader &scene_shader) override;
    void update(float dt) override;
    void render(Shader &scene_shader) override;
    void render() override;

    StringScene();
    ~StringScene() override;

    void resetCamera();

    void setIntegrator(Integrator integrator);
    Integrator integrator() {return _integrator;}

protected:
    void renderInfo();
    void processInput(float dt);


private:
    class impl;
    std::unique_ptr<impl> pimpl;
    Integrator _integrator;
};

#endif // PX_CG_SCENE_STRING_SCENE_HPP
