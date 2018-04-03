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
    float t_h;  // for TTC
    float tau; // for ROCA

    float n_obstacles;
    bool keep_obstacles; // keep current obstacles or not when restart
    bool keep_samples;
    bool pause;
    enum class PlanningMethod
    {
        VelocityObstacles,
        OptimalReciprocalCollisionAvoidance,
        TimeToCollision,
        ProbabilityRoadMap,
        LazyProbabilityRoadMap,
        RapidlyExploringRandomTrees,
        OptimalRapidlyExploringRandomTrees
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
