#ifndef PX_CG_SCENE_STRING_SCENE_HPP
#define PX_CG_SCENE_STRING_SCENE_HPP

#include <memory>

#include "scene/base_scene.hpp"
#include "shader/base_shader.hpp"
#include "util/cuda.hpp"
#include "item/sphere.hpp"

namespace px { namespace scene
{
class StringScene;
} }

class px::scene::StringScene : public BaseScene
{
public:
    enum class Scenery
    {
        Flag,
        Falling,
        Floating,
        Dangling
    };
    enum class Integrator
    {
        RK4,
        MidPoint,
        Verlet,
        VelocityVerlet,
        Euler,
        SemiImplicitEuler
    };

    const std::string system_name;
    bool pause;
    Integrator integrator;
    Scenery scenery;

    void init(Scene &scene) override;
    void restart(Scene &scene) override;
    void upload(Shader &scene_shader) override;
    void update(float dt) override;
    void render(Shader &scene_shader) override;
    void render() override;

    StringScene();
    ~StringScene() override;

    void resetCamera();
    void resetBall();
    void pushBall();

    struct CudaParam_t
    {
        float ks, ks_shear, ks_bend;
        float kd, kd_shear, kd_bend;
        float rest_len, rest_len_shear, rest_len_bend;
        float air_friction, ground_friction;
        float field_height, cloth_thickness;

        float3 gravity, wind, field_norm;

        unsigned int grid_x, grid_y;
    } cuda_param;
    void cudaInit(unsigned int n_particles, const float *mass);
    void cudaUpdate(void *buffer, float dt, unsigned int n_iter, item::Sphere *sphere);
    void cudaBufferFree();

protected:
    void renderInfo();
    void processInput(float dt);

private:
    class impl;
    std::unique_ptr<impl> pimpl;
};

#endif // PX_CG_SCENE_STRING_SCENE_HPP
