#ifndef PX_CG_SCENE_ROADMAP_SCENE_HPP
#define PX_CG_SCENE_ROADMAP_SCENE_HPP

#include <memory>

#include "scene/base_scene.hpp"
#include "shader/base_shader.hpp"
#include "particle.hpp"

namespace px { namespace scene
{
class RoadmapScene;
} }

class px::scene::RoadmapScene : public BaseScene
{
public:
    float v_max;
    float rrt_step_min; // for RRT and RRT*
    float rrt_step_max; // for RRT and RRT*
    float rewire_radius; // for RRT*
    float neighbor_radius; // for VO, RVO, HRVO, ORCA
    float t_h;  // for TTC
    float tau; // for ROCA

    bool smooth_path;

    int n_obstacles;
    bool keep_obstacles; // keep current obstacles or not when restart
    bool keep_milestones;   // keep current milestone samples or not when restart
    bool pause;
    enum class PlanningMethod
    {
        VO,         // Velocity Obstacles
        // RVO,     // Reciprocal Velocity Obstacles       // this scene only has only static obstacles such that
        // HRVO,    // Hybrid Reciprocal Velocity Obstacles // these two methods perform in the same way with Velocity Obstacles method.
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

    void init(Scene &scene) override;
    void restart(Scene &scene) override;
    void upload(Shader &scene_shader) override;
    void update(float dt) override;
    void render() override;

    RoadmapScene();
    ~RoadmapScene() override;

    void resetCamera();

protected:
    void renderInfo();
    void processInput(float dt);

private:
    class impl;
    std::unique_ptr<impl> pimpl;
};

#endif // PX_CG_SCENE_ROADMAP_SCENE_HPP
