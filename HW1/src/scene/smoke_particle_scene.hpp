#ifndef PX_CG_SCENE_SMOKE_PARTICLE_SCENE_HPP
#define PX_CG_SCENE_SMOKE_PARTICLE_SCENE_HPP

#include "particle_system/simple_2d.hpp"
#include "scene/base_scene.hpp"
#include "shader/base_shader.hpp"
#include "shader/text.hpp"

namespace px { namespace scene
{
class SmokeParticleScene;
} }

class px::scene::SmokeParticleScene : public BaseScene
{
public:
    void init(Scene &scene) override;
    void restart(Scene &scene) override;
    void upload(Shader &scene_shader) override;
    void update(float dt) override;
    void render(Shader &scene_shader) override;
    void render() override;

    inline ParticleSystem * const & particleSystem() const noexcept { return particle_system; }

    SmokeParticleScene();
    ~SmokeParticleScene() override;

    /**
     * @see http://prideout.net/blog/?p=67
     */
    class SmokeParticleSystem : public ParticleSystem
    {
    public:
        void init(float *vertex, unsigned int v_count,
                  unsigned int tex = 0, float *uv = nullptr, bool atlas = false) override;
        void restart() override;
        void upload() override;
        void update(float dt, glm::vec3 *cam_pos = nullptr) override;
        void render(GLenum gl_draw_mode = GL_POINT) override;

        SmokeParticleSystem();
        ~SmokeParticleSystem();

        inline unsigned int const &count() const noexcept override { return n_particles; }
        inline unsigned int total() const noexcept override { return _tot_particles; }

    protected:
        unsigned int vao[3], vbo[2], tfbo[2], texture[2];

        Shader *compute_shader;
        Shader *draw_shader;

        bool need_upload;
    private:
        unsigned int _tot_particles;
    };

protected:
    TextShader *text;
    ParticleSystem *particle_system;

protected:
    void renderInfo();
    void processInput(float dt);
};

#endif // PPX_CG_SCENE_SMOKE_PARTICLE_SCENE_HPP
