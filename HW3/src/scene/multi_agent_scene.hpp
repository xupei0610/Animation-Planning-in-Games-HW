#ifndef PX_CG_SCENE_MULTI_AGENT_SCENE_HPP
#define PX_CG_SCENE_MULTI_AGENT_SCENE_HPP

#include <memory>

#include "scene/base_scene.hpp"
#include "shader/base_shader.hpp"
#include "particle.hpp"

namespace px { namespace scene
{
class MultiAgentScene;
} }

class px::scene::MultiAgentScene : public BaseScene
{
public:
    float v_max;
    float v_pref;
    float a_max;
    float rrt_step_min;  // for RRT and RRT*
    float rrt_step_max;  // for RRT and RRT*
    float rewire_radius; // for RRT*
    float neighbor_radius; // for VO, RVO, HRVO, ORCA
    float f_k;  // force factor for TTC
    float t_h;  // for TTC
    float tau;  // for ROCA

    bool pause;
    enum class PlanningMethod
    {
        VO,         // Velocity Obstacles
        RVO,        // Reciprocal Velocity Obstacles
        HRVO,       // Hybrid Reciprocal Velocity Obstacles
        ORCA,       // Optimal Reciprocal Collision Avoidance,
        TTC,        // Time-to-Collision
        PRM,        // Probability Road Map
        LazyPRM,    // Lazy Probability Road Map
        RRT,        // Rapidly-Exploring Random Trees
        RRTStar     // Optimal Rapidly-Exploring Random Trees
    } planning_method;
    enum class PathFinder
    {
        UniformCost,
        AStar
    } pathfinder;

    enum class Scenario
    {
        Circle, Blocks, Passage
    } scenario;

    void init(Scene &scene) override;
    void restart(Scene &scene) override;
    void upload(Shader &scene_shader) override;
    void update(float dt) override;
    void render() override;

    MultiAgentScene();
    ~MultiAgentScene() override;

    void resetCamera();

protected:
    void renderInfo();
    void processInput(float dt);

private:
    class impl;
    std::unique_ptr<impl> pimpl;
};

#endif // PX_CG_SCENE_MULTI_AGENT_SCENE_HPP
