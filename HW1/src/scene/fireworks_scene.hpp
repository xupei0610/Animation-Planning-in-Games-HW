#ifndef PX_CG_SCENE_FIREWORKS_SCENE_HPP
#define PX_CG_SCENE_FIREWORKS_SCENE_HPP

#include <memory>

#include "particle.hpp"
#include "scene/base_scene.hpp"
#include "shader/base_shader.hpp"

namespace px { namespace scene
{
class FireworksScene;
} }

class px::scene::FireworksScene : public BaseScene
{
public:
    const std::string system_name;
    const std::string rendering_mode;
    bool pause;

    void init(Scene &scene) override;
    void restart(Scene &scene) override;
    void upload(Shader &scene_shader) override;
    void update(float dt) override;
    void render() override;

    FireworksScene();
    ~FireworksScene() override;

    void resetCamera();

    class ComputeShaderParticleSystem : public ParticleSystem
    {
    public:

        void init(float *vertex, unsigned int v_count,
                  unsigned int tex = 0, float *uv = nullptr, bool atlas = false) override;
        void restart() override;
        void upload() override;
        void update(float dt, glm::vec3 *cam_pos = nullptr) override;
        void render(GLenum gl_draw_mode = GL_POINT) override;

        ComputeShaderParticleSystem();
        ~ComputeShaderParticleSystem();

        unsigned int const &count() const noexcept override;
        unsigned int total() const noexcept override;

    private:
        class impl;
        std::unique_ptr<impl> pimpl;
    };

protected:
    std::vector<ParticleSystem *> systems;
protected:
    void renderInfo();
    void processInput(float dt);
};

#endif // PX_CG_SCENE_FIREWORKS_SCENE_HPP
