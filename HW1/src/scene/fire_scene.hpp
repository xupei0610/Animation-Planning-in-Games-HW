#ifndef PX_CG_SCENE_FIRE_SCENE_HPP
#define PX_CG_SCENE_FIRE_SCENE_HPP

#include <memory>
#include "particle_system/simple_2d.hpp"
#include "scene/base_scene.hpp"
#include "shader/base_shader.hpp"
#include "shader/text.hpp"

namespace px { namespace scene
{
class FireScene;
} }

class px::scene::FireScene : public BaseScene
{
public:
    void init(Scene &scene) override;
    void restart(Scene &scene) override;
    void upload(Shader &scene_shader) override;
    void update(float dt) override;
    void render() override;

    inline ParticleSystem * const & particleSystem() const noexcept { return particle_system; }

    FireScene();
    ~FireScene() override;

    void resetCamera();

    class CurlNoiseParticleSystem : public ParticleSystem
    {
    public:
        void init(float *vertex, unsigned int v_count,
                  unsigned int tex = 0, float *uv = nullptr, bool atlas = false) override;
        void restart() override;
        void upload() override;
        void update(float dt, glm::vec3 *cam_pos = nullptr) override;
        void render(GLenum gl_draw_mode = GL_POINT) override;

        CurlNoiseParticleSystem();
        ~CurlNoiseParticleSystem();

        unsigned int total() const noexcept override;

    private:
        class impl;
        std::unique_ptr<impl> pimpl;
    };

protected:
    ParticleSystem *particle_system;

    const std::string system_name;
    const std::string rendering_mode;

    bool pause;
protected:
    void renderInfo();
    void processInput(float dt);
};

#endif // PPX_CG_SCENE_FIRE_SCENE_HPP
