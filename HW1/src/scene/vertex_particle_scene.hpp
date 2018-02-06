#ifndef PX_CG_SCENE_VERTEX_PARTICLE_SCENE_HPP
#define PX_CG_SCENE_VERTEX_PARTICLE_SCENE_HPP

#include "scene/base_scene.hpp"
#include "shader/text.hpp"
#include "item/light_ball.hpp"

namespace px { namespace scene
{
class VertexParticleScene;
} }

class px::scene::VertexParticleScene : public BaseScene
{
public:
    unsigned int max_particles;

    void init(Scene &scene) override;
    void restart(Scene &scene) override;
    void update(float dt) override;
    void render() override;

    VertexParticleScene();
    ~VertexParticleScene() override = default;

    inline const unsigned int &count() const noexcept { return _count; }
protected:
    TextShader *text;
    void processInput(float dt);

    std::vector<float> life;
    std::vector<item::LightBall *> particles;
private:
    unsigned int _count;
};

#endif // PX_CG_SCENE_VERTEX_PARTICLE_SCENE_HPP
