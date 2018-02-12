#ifndef PX_CG_SCENE_SNOW_SCENE_HPP
#define PX_CG_SCENE_SNOW_SCENE_HPP

#include <memory>

#include "shader/base_shader.hpp"
#include "scene/base_scene.hpp"
#include "particle.hpp"

namespace px { namespace scene
{
class SnowScene;
}}

class px::scene::SnowScene : public BaseScene
{
public:
    const std::string system_name;
    const std::string rendering_mode;
    bool pause;
    glm::vec3 wind;

    void init(Scene &scene) override;
    void restart(Scene &scene) override;
    void upload(Shader &scene_shader) override;
    void update(float dt) override;
    void render() override;

    SnowScene();
    ~SnowScene() override;

    void resetCamera();
    void processInput(float dt);
    void renderInfo();

    class BrownianMotionParticleSystem : public ParticleSystem
    {
    public:
        glm::vec3 acceleration;

    public:
        void init(float *vertex, unsigned int v_count,
                  unsigned int tex = 0, float *uv = nullptr, bool atlas = false) override;
        void restart() override;
        void upload() override;
        void update(float dt, glm::vec3 *cam_pos = nullptr) override;
        void render(GLenum gl_draw_mode = GL_POINT) override;

        BrownianMotionParticleSystem();
        ~BrownianMotionParticleSystem();

        unsigned int total() const noexcept override;

    private:
        class impl;
        std::unique_ptr<impl> pimpl;
    };

    inline const BrownianMotionParticleSystem *particleSystem() const noexcept { return particle_system; }
protected:
    BrownianMotionParticleSystem *particle_system;
};

#endif // PX_CG_SCENE_SNOW_SCENE_HPP
