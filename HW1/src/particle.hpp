#ifndef PX_CG_PARTICLE_HPP
#define PX_CG_PARTICLE_HPP

#include <functional>

#include "shader/base_shader.hpp"

namespace px
{
struct Particle_t
{
    glm::vec3 position;
    glm::vec4 color;
    float size, rotation;
    float u_offset, v_offset, texture_scale;

    Particle_t();
};

using ParticleSpawnFn_t = std::function<bool(Particle_t &, float &, int id)>;
using ParticleUpdateFn_t = std::function<bool(Particle_t &, float &, float, int id)>;

class ParticleSystemError;
class ParticleSystem;
}

class px::ParticleSystemError : public std::exception
{
public:
    ParticleSystemError(const std::string &msg, const int code=0)
            : msg(msg), err_code(code)
    {}
    const char *what() const noexcept override
    {
        return msg.data();
    }
    inline int code() const
    {
        return err_code;
    }

protected:
    std::string msg;
    int err_code;
};

class px::ParticleSystem
{
public:
    unsigned int max_particles;
    unsigned int birth_rate;

    ParticleSpawnFn_t spawn_fn;
    ParticleUpdateFn_t update_fn;

public:
    virtual void init(float *vertex, unsigned int v_count,
                      unsigned int tex = 0, float *uv = nullptr, bool atlas = false) = 0;
    virtual void activateTexture(unsigned int tex = 0, float *uv = nullptr, bool atlas = false) {};
    virtual void activateAtlas(bool atlas = false) {};
    virtual void restart() = 0;
    virtual void upload() {};
    virtual void update(float dt, glm::vec3 *cam_pos = nullptr)  = 0;
    virtual void render(GLenum gl_draw_mode = GL_POINT) = 0;

    virtual unsigned int const &count() const noexcept { return n_particles; }
    virtual unsigned int total() const noexcept { return static_cast<unsigned int>(particles.size()); }

    [ [noreturn]]
    void err(std::string const &msg);
    virtual ~ParticleSystem() = default;
protected:
    ParticleSystem();

protected:
    unsigned int n_particles;
    std::vector<Particle_t> particles;
};

#endif // PX_CG_PARTICLE_HPP
