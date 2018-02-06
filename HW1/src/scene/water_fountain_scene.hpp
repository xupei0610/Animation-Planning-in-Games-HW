#ifndef PX_CG_SCENE_WATER_FOUNTAIN_SCENE_HPP
#define PX_CG_SCENE_WATER_FOUNTAIN_SCENE_HPP

#include <memory>

#include "particle.hpp"
#include "scene/base_scene.hpp"
#include "shader/base_shader.hpp"

namespace px { namespace scene
{
class WaterFountainScene;
} }

class px::scene::WaterFountainScene : public BaseScene
{
public:
    const std::string system_name;
    const std::string rendering_mode;
    glm::vec3 wind;
    bool pause;

public:
    void init(Scene &scene) override;
    void restart(Scene &scene) override;
    void upload(Shader &scene_shader) override;
    void update(float dt) override;
    void render(Shader &scene_shader) override;
    void render() override;

    WaterFountainScene();
    ~WaterFountainScene() override;

    class ComputeShaderParticleSystem : public ParticleSystem
    {
    public:
        glm::vec3 acceleration;

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
    ComputeShaderParticleSystem *particle_system;

protected:
    void renderInfo();
    void processInput(float dt);
};

#endif // PX_CG_SCENE_WATER_FOUNTAIN_SCENE_HPP
