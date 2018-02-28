#ifndef PX_CG_SCENE_SHALLOW_WATER_SCENE_HPP
#define PX_CG_SCENE_SHALLOW_WATER_SCENE_HPP

#include <memory>

#include "scene/base_scene.hpp"
#include "shader/base_shader.hpp"

namespace px { namespace scene
{
class ShallowWaterScene;
} }

class px::scene::ShallowWaterScene : public BaseScene
{
public:
    const std::string system_name;
    bool pause;

    void init(Scene &scene) override;
    void restart(Scene &scene) override;
    void upload(Shader &scene_shader) override;
    void update(float dt) override;
    void render() override;

    ShallowWaterScene();
    ~ShallowWaterScene() override;

    void resetCamera();

    struct CudaParam_t
    {
        int grid_x, grid_y;
        float inv_gap_x, inv_gap_y;
        float half_g;
        float height_eps;
    } cuda_param;
    void cudaInit(void *buffer);
    void cudaUpdate(void *buffer, float dt, unsigned int n_iter, float drop_seed);
    void cudaBufferFree();

protected:
    void renderInfo();
    void processInput(float dt);

private:
    class impl;
    std::unique_ptr<impl> pimpl;
};

#endif // PX_CG_SCENE_SHALLOW_WATER_SCENE_HPP
