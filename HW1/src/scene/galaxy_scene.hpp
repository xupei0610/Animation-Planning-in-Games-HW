#ifndef PX_CG_SCENE_GALAXY_SCENE_HPP
#define PX_CG_SCENE_GALAXY_SCENE_HPP

#include <memory>

#include "particle.hpp"
#include "scene/base_scene.hpp"
#include "shader/base_shader.hpp"

namespace px { namespace scene
{
class GalaxyScene;
} }

class px::scene::GalaxyScene : public BaseScene
{
public:
    const std::string system_name;
    bool pause;

    void init(Scene &scene) override;
    void restart(Scene &scene) override;
    void upload(Shader &scene_shader) override;
    void update(float dt) override;
    void render(Shader &scene_shader) override;
    void render() override;

    GalaxyScene();
    ~GalaxyScene() override;

    void resetCamera();

    class SIMDParticleSystem : public ParticleSystem
    {
    public:
        void init(float *vertex, unsigned int v_count,
                  unsigned int tex = 0, float *uv = nullptr, bool atlas = false) override;
        void restart() override;
        void upload() override;
        void update(float dt, glm::vec3 *cam_pos = nullptr) override;
        void render(GLenum gl_draw_mode = GL_POINT) override;

        unsigned int const &count() const noexcept override;
        unsigned int total() const noexcept override;

    protected:
        SIMDParticleSystem();
        ~SIMDParticleSystem() override;

        class impl;
        std::unique_ptr<impl> pimpl;
    };

    class ComputeShaderParticleSystem : public SIMDParticleSystem
    {
    public:
        void init(float *vertex, unsigned int v_count,
                  unsigned int tex = 0, float *uv = nullptr, bool atlas = false) override;
        void upload() override;
        void update(float dt, glm::vec3 *cam_pos = nullptr) override;

        ComputeShaderParticleSystem();
        ~ComputeShaderParticleSystem() override;

    private:
        class impl;
        std::unique_ptr<impl> pimpl;
    };

    class CUDAParticleSystem : public SIMDParticleSystem
    {
    public:
        void upload() override;
        void update(float dt, glm::vec3 *cam_pos = nullptr) override;

        CUDAParticleSystem();
        ~CUDAParticleSystem() override;

    private:
        void cudaSpawn(void * buffer, unsigned int n, float radius);
        void cudaUpdate(void * buffer, unsigned int n, float dt);

        class impl;
        std::unique_ptr<impl> pimpl;
    };

protected:
    ParticleSystem *particle_system;

    ParticleSystem *cuda_particle_system, *compute_shader_particle_system;
protected:
    void renderInfo();
    void processInput(float dt);
};

#endif // PX_CG_SCENE_GALAXY_SCENE_HPP
