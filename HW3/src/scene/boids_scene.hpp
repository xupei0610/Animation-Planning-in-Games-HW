#ifndef PX_CG_SCENE_BOIDS_SCENE_HPP
#define PX_CG_SCENE_BOIDS_SCENE_HPP

#include <memory>

#include "scene/base_scene.hpp"
#include "shader/base_shader.hpp"
#include "particle.hpp"

namespace px { namespace scene
{
class BoidsScene;
} }

class px::scene::BoidsScene : public BaseScene
{
public:
    bool pause;

    struct SceneParameter
    {
        unsigned int n_agents;
        unsigned int n_predators;
        unsigned int n_obstacles;
        glm::vec3 upper_bound;
        glm::vec3 lower_bound;
    } scene_param;
    struct BoidParameter
    {
        float visual_r;
        float visual_ang;
        float agent_size;
        float v_max;
        float a_max;
        float visual_r_predator;
        float visual_ang_predator;
        float predator_size;
        float v_max_predator;
        float a_max_predator;
        float f_alpha; // separation weight
        float v_alpha;  // alignment weight
        float p_alpha;  // cohesion weight
        float s_alpha; // obstacle avoidance weight
        float e_alpha; // escape force weight
        float c_alpha; // chase force weight
        float g_alpha; // goal navigation
        float satiation; // time gap between two predations

        bool has_goal;
        glm::vec3 goal;
    } boids;

    void init(Scene &scene) override;
    void restart(Scene &scene) override;
    void upload(Shader &scene_shader) override;
    void update(float dt) override;
    void render() override;

    BoidsScene();
    ~BoidsScene() override;

    void resetCamera();

protected:
    void renderInfo();
    void processInput(float dt);

private:
    class impl;
    std::unique_ptr<impl> pimpl;
};

#endif // PX_CG_SCENE_BOIDS_SCENE_HPP
