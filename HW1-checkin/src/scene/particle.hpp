#ifndef PX_CG_SCENE_PARTICLE_HPP
#define PX_CG_SCENE_PARTICLE_HPP

#include "scene/base_scene.hpp"
#include "shader/base_shader.hpp"
#include "shader/text.hpp"

namespace px { namespace scene
{
class Particle;
} }

class px::scene::Particle : public BaseScene
{
public:
    struct Particle_t
    {
        glm::vec3 position, velocity;
        struct { float r, g, b, a; } color;
        float size;
    };

    glm::vec3 gravity;

    unsigned int max_particles;
    float birth_rate;

public:
    void init(Scene &scene) override;
    void restart(Scene &scene) override;
    void upload(Shader &scene_shader) override;
    void update(float dt) override;
    void render(Shader &scene_shader) override;
    void render() override;

    Particle();
    ~Particle() override;

    inline std::size_t count() const noexcept { return n_particles; }
    inline std::size_t total() const noexcept { return particles.size(); }

protected:
    unsigned int vao, vbo[3];

    std::vector<Particle_t> particles;

    std::vector<float> position; // for upload
    std::vector<float> color;    // for upload
    std::vector<float> life;

    unsigned int n_particles;
    Shader *shader;
    TextShader *text;

    int n_vertex;
    float debt_particles;
};

#endif // PPX_CG_SCENE_PARTICLE_HPP
