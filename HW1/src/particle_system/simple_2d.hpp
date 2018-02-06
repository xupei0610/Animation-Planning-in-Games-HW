#ifndef PX_CG_PARTICLE_SYSTEM_SIMPLE_2D_HPP
#define PX_CG_PARTICLE_SYSTEM_SIMPLE_2D_HPP

#include "shader/base_shader.hpp"
#include "particle.hpp"

namespace px
{
class Simple2DParticleSystem;
}

class px::Simple2DParticleSystem : public ParticleSystem
{
public:
    void init(float *vertex, unsigned int v_count,
              unsigned int tex = 0, float *uv = nullptr, bool atlas = false) override;
    void activateTexture(unsigned int tex = 0, float *uv = nullptr, bool atlas = false) override;
    void activateAtlas(bool atlas = false) override;
    void restart() override;
    void upload() override;
    void update(float dt, glm::vec3 *cam_pos = nullptr) override;
    void render(GLenum gl_draw_mode = GL_POINT) override;

    Simple2DParticleSystem();
    ~Simple2DParticleSystem() override;

protected:
    unsigned int vao, vbo[4], vbo_uvs[2], texture;
    Shader *shader;

    std::vector<float> life;
    std::vector<float> position;
    std::vector<float> size_rot;
    std::vector<float> color;
    std::vector<float> uv_offset;

    int debt_particles;
    unsigned int n_vertices;
    bool need_upload, use_atlas;
};

#endif // PX_CG_PARTICLE_SYSTEM_SIMPLE_2D_HPP
