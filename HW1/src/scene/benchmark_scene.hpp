#ifndef PX_CG_SCENE_BENCHMARK_SCENE_HPP
#define PX_CG_SCENE_BENCHMARK_SCENE_HPP

#include <memory>

#include "particle_system/simple_2d.hpp"
#include "particle_system/geometry.hpp"
#include "scene/base_scene.hpp"
#include "shader/base_shader.hpp"

#include "util/cuda.hpp"
#include <cuda_gl_interop.h>

namespace px { namespace scene
{
class BenchmarkScene;
} }

class px::scene::BenchmarkScene : public BaseScene
{
public:
    bool pause;

public:
    void init(Scene &scene) override;
    void restart(Scene &scene) override;
    void upload(Shader &scene_shader) override;
    void update(float dt) override;
    void render() override;

    BenchmarkScene();
    ~BenchmarkScene() override;

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

        ~SIMDParticleSystem() override;

        inline unsigned int const &count() const noexcept override { return n_particles; }
        inline unsigned int total() const noexcept override { return _tot_particles; }

    protected:
        SIMDParticleSystem();

        unsigned int vao, vbo[2], ssbo;

        Shader *compute_shader;
        Shader *draw_shader;

        unsigned int n_vertices;
        float debt_particles;
        bool need_upload;
    private:
        unsigned int _tot_particles;
    };

    class ComputeShaderParticleSystem : public SIMDParticleSystem
    {
    public:
        void init(float *vertex, unsigned int v_count,
                  unsigned int tex = 0, float *uv = nullptr, bool atlas = false) override;
        void update(float dt, glm::vec3 *cam_pos = nullptr) override;

        ComputeShaderParticleSystem();
        ~ComputeShaderParticleSystem() override;

    protected:
        Shader *compute_shader;
    };
    class CUDAParticleSystem : public SIMDParticleSystem
    {
    public:
        void upload() override;
        void update(float dt, glm::vec3 *cam_pos = nullptr) override;

        CUDAParticleSystem();
        ~CUDAParticleSystem() override;

    protected:
        struct cudaGraphicsResource *res;
        void *buffer;
        size_t buffer_size;
    private:
        void cudaUpdate(void *buffer, unsigned int n, float dt);
    };
    class TransformFeedbackParticleSystem : public ParticleSystem
    {
    public:
        void init(float *vertex, unsigned int v_count,
                  unsigned int tex = 0, float *uv = nullptr, bool atlas = false) override;
        void restart() override;
        void upload() override;
        void update(float dt, glm::vec3 *cam_pos = nullptr) override;
        void render(GLenum gl_draw_mode = GL_POINT) override;

        TransformFeedbackParticleSystem();
        ~TransformFeedbackParticleSystem() override;

        inline unsigned int const &count() const noexcept override { return n_particles; }
        inline unsigned int total() const noexcept override { return _tot_particles; }

    protected:
        unsigned int vao[3], vbo[4], tfbo[2];

        Shader *compute_vertex_shader;
        Shader *draw_shader;

        unsigned int n_vertices;
        float debt_particles;
        bool need_upload;
    private:
        unsigned int _tot_particles;
    };

protected:
    ParticleSystem *particle_system;
    std::string system_name;

    std::vector<std::pair<std::string, ParticleSystem *> > systems;

protected:
    void renderInfo();
    void processInput(float dt);

private:
    class impl;
    std::unique_ptr<impl> pimpl;
};

#endif // PPX_CG_SCENE_INSTANCING_PARTICLE_SCENE_HPP
