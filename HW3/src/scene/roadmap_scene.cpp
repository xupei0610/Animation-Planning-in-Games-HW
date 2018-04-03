#include <sstream>
#include <iomanip>
#include <cstring>
#include <thread>
#include <list>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/transform.hpp>

#include "scene/roadmap_scene.hpp"
#include "scene.hpp"
#include "app.hpp"
#include "global.hpp"
#include "shader/skybox.hpp"
#include "item/floor.hpp"
#include "item/pillar.hpp"
#include "util/random.hpp"
#include "util/motion_planning.hpp"

using namespace px;

class scene::RoadmapScene::impl
{
public:
    bool need_upload;
    bool is_planning, planning_success, sampling_complete, sampling_upload, road_upload;

    float &v_max, &t_h, &tau;
    int n_sampling; int k; int n_road; int solution_size;
    bool resampling;
    glm::vec2 upper_bound, lower_bound;
    glm::vec3 agent_pos; glm::vec2 agent_vel;
    glm::vec2 goal_pos;
    std::vector<glm::vec2> path;
    std::vector<glm::vec3> obstacle_pos;
    std::vector<std::shared_ptr<Item> > obstacles;
    std::vector<glm::vec2> milestones;
    std::vector<std::vector<float> > roadmap;

    std::thread *worker_thread;

    std::unique_ptr<SkyBox> skybox;
    std::unique_ptr<Shader> mark_shader, edge_shader, road_shader;
    std::shared_ptr<item::Floor> floor;
    std::shared_ptr<item::Pillar> agent;
    unsigned int vao[3], vbo[4];

    const char *MARK_VS = "#version 420 core\n"
            "layout (location = 0) in vec2 vert;"
            "layout (location = 1) in vec2 pos;"
            ""
            "layout (std140, binding = 0) uniform GlobalAttributes"
            "{"
            "   mat4 view;"
            "   mat4 proj;"
            "   vec3 cam_pos;"
            "};"
            ""
            "const vec3 light_dir = normalize(vec3(1));"
            ""
            "out vec3 gLight;"
            "out vec3 gPos;"
            "out vec3 gNorm;"
            ""
            "void main(){"
            "   vec4 pos4 = view * vec4(vert.x+pos.x, 0.02f, vert.y-pos.y, 1.f);"
            ""
            "   gLight = (view * vec4(light_dir, 0.f)).xyz;"
            "   gPos = pos4.xyz/pos4.w;"
            "   gNorm = (transpose(inverse(view)) * vec4(0.f, 1.f, 0.f, 0.f)).xyz;"
            "   gl_Position = proj * pos4;"
            "}";

    const char *EDGE_VS = "#version 420 core\n"
            "layout (location = 0) in vec2 pos;"
            ""
            "layout (std140, binding = 0) uniform GlobalAttributes"
            "{"
            "   mat4 view;"
            "   mat4 proj;"
            "   vec3 cam_pos;"
            "};"
            ""
            "void main(){"
            "   gl_Position = proj * view * vec4(pos.x, 0.02f, -pos.y, 1.f);;"
            "}";
    const char *EDGE_FS = "#version 330 core\n"
            "out vec4 color;"
            "void main() {"
            "   color = vec4(.75f, .75f, .75f, 1.f);"
            "}";
    const char *ROAD_VS = "#version 420 core\n"
            "layout (location = 0) in vec2 pos;"
            ""
            "layout (std140, binding = 0) uniform GlobalAttributes"
            "{"
            "   mat4 view;"
            "   mat4 proj;"
            "   vec3 cam_pos;"
            "};"
            ""
            "void main(){"
            "   gl_Position = proj * view * vec4(pos.x, 0.03f, -pos.y, 1.f);;"
            "}";
    const char *ROAD_FS = "#version 330 core\n"
            "out vec4 color;"
            "void main() {"
            "   color = vec4(1.f, 0.f, 0.f, 1.f);"
            "}";

    impl(scene::RoadmapScene *parent)
            : v_max(parent->v_max), t_h(parent->t_h), tau(parent->tau),
              worker_thread(nullptr),
              skybox(nullptr),
              mark_shader(nullptr), edge_shader(nullptr), road_shader(nullptr),
              floor(nullptr), agent(nullptr),
              vao{0}, vbo{0}
    {}
    ~impl()
    {

        if (worker_thread && worker_thread->joinable())
            worker_thread->join();
        delete worker_thread;

        clearGLObjs();
    }

    void clearGLObjs()
    {
        glDeleteVertexArrays(3, vao);
        glDeleteBuffers(4, vbo);
    }
    void genGLObjs()
    {
        clearGLObjs();
        glGenVertexArrays(3, vao);
        glGenBuffers(4, vbo);
    }

    void plan(scene::RoadmapScene::PlanningMethod planning_method,
              scene::RoadmapScene::PathFinder pathfinder)
    {
        is_planning = true;

        sampling_complete = false;
        std::list<std::size_t> path;
        if (planning_method == scene::RoadmapScene::PlanningMethod::ProbabilityRoadMap)
        {
            if (milestones.empty())
            {
                milestones.emplace_back(agent_pos.x, agent_pos.y);
                milestones.emplace_back(goal_pos.x, goal_pos.y);
                sampleCollisionFree(n_sampling, milestones);
            }
            if (pathfinder == scene::RoadmapScene::PathFinder::UniformCost)
            {
                roadmap = planning::PRM<float>(
                        milestones, 0, 1, path,
                        glm::distance2<float, glm::highp, glm::tvec2>,
                        std::bind(&impl::collide, this, std::placeholders::_1,
                                  std::placeholders::_2),
                        planning::uniformCost<float>,
                        k);
            }
            else
            {
                auto h = [&](std::size_t i)
                {
                    return glm::distance2(milestones[1], milestones[i]);
                };
                roadmap = planning::PRM<float>(
                        milestones, 0, 1, path,
                        glm::distance2<float, glm::highp, glm::tvec2>,
                        std::bind(&impl::collide, this,
                                  std::placeholders::_1, std::placeholders::_2),
                        std::bind(planning::AStar<float, decltype(h)>,
                                  std::placeholders::_1, std::placeholders::_2,
                                  std::placeholders::_3, std::placeholders::_4,
                                  h),
                        k);
            }
        }
        else if (planning_method == scene::RoadmapScene::PlanningMethod::LazyProbabilityRoadMap)
        {
            if (milestones.empty())
            {
                milestones.emplace_back(agent_pos.x, agent_pos.y);
                milestones.emplace_back(goal_pos.x, goal_pos.y);
                sample(n_sampling, milestones);
            }
            if (pathfinder == scene::RoadmapScene::PathFinder::UniformCost)
                roadmap = planning::PRMLazy<float>(
                        milestones, 0, 1, path,
                        glm::distance2<float, glm::highp, glm::tvec2>,
                        std::bind(&impl::collide, this, std::placeholders::_1,
                                  std::placeholders::_2),
                        planning::uniformCost<float>);
            else
            {
                std::vector<float> h_val(n_sampling, -1.f);
                auto h = [&](std::size_t i)
                {
                    if (h_val[i] == -1.f)
                        h_val[i] = glm::distance2(milestones[1], milestones[i]);
                    return h_val[i];
                };
                roadmap = planning::PRMLazy<float>(
                        milestones, 0, 1, path,
                        glm::distance2<float, glm::highp, glm::tvec2>,
                        std::bind(&impl::collide, this, std::placeholders::_1,
                                  std::placeholders::_2),
                        std::bind(planning::AStar<float, decltype(h)>,
                                  std::placeholders::_1, std::placeholders::_2,
                                  std::placeholders::_3, std::placeholders::_4,
                                  h)
                        );
            }
        }
        else if (planning_method == scene::RoadmapScene::PlanningMethod::RapidlyExploringRandomTrees)
        {
            if (milestones.empty())
            {
                milestones.emplace_back(agent_pos.x, agent_pos.y);
                milestones.emplace_back(goal_pos.x, goal_pos.y);
                sample(n_sampling, milestones);
            }
            auto lower_x = lower_bound.x + agent_pos.z;
            auto lower_y = lower_bound.y + agent_pos.z;
            auto s_x = upper_bound.x - (lower_x + agent_pos.z);
            auto s_y = upper_bound.y - (lower_y + agent_pos.z);

            roadmap = planning::RRT<float>(
                    milestones, 0, 1, path,
                    glm::distance<float, glm::highp, glm::tvec2>,
                    std::bind(&impl::collide, this,
                              std::placeholders::_1, std::placeholders::_2),
                    std::bind(&impl::collideFromTo, this,
                              std::placeholders::_1, std::placeholders::_2),
                    .1f, 20.f,
                    std::function<void(glm::vec2 &)>([&](glm::vec2 &s)
                    {
                        for (;;)
                        {
                            auto x = rnd()*s_x + lower_x;
                            auto y = rnd()*s_y + lower_y;
                            if ((x != agent_pos.x || y != agent_pos.y)
                                && (x != goal_pos.x || y != goal_pos.y))
                            {
                                s.x = x; s.y = y;
                                return;
                            }
                        }
                    }));
        }
        else if (planning_method == scene::RoadmapScene::PlanningMethod::OptimalRapidlyExploringRandomTrees)
        {
            if (milestones.empty())
            {
                milestones.emplace_back(agent_pos.x, agent_pos.y);
                milestones.emplace_back(goal_pos.x, goal_pos.y);
                sample(n_sampling, milestones);
            }
            auto lower_x = lower_bound.x + agent_pos.z;
            auto lower_y = lower_bound.y + agent_pos.z;
            auto s_x = upper_bound.x - (lower_x + agent_pos.z);
            auto s_y = upper_bound.y - (lower_y + agent_pos.z);

            roadmap = planning::RRTStar<float>(
                    milestones, 0, 1, path,
                    glm::distance2<float, glm::highp, glm::tvec2>,
                    std::bind(&impl::collide, this,
                              std::placeholders::_1, std::placeholders::_2),
                    std::bind(&impl::collideFromTo, this,
                              std::placeholders::_1, std::placeholders::_2),
                    .1f, 20.f, 5.f,
                    std::function<void(glm::vec2 &)>([&](glm::vec2 &s)
                    {
                        for (;;)
                        {
                            auto x = rnd()*s_x + lower_x;
                            auto y = rnd()*s_y + lower_y;
                            if ((x != agent_pos.x || y != agent_pos.y)
                                && (x != goal_pos.x || y != goal_pos.y))
                            {
                                s.x = x; s.y = y;
                                return;
                            }
                        }
                    }));
        }
        else
        {
            milestones.clear();
            milestones.emplace_back(agent_pos.x, agent_pos.y);
            milestones.emplace_back(goal_pos.x, goal_pos.y);
            roadmap.clear();
            sampling_complete = true;
            std::vector<glm::vec2>{milestones[1], milestones[0]}.swap(this->path);
            planning_success = true;
            is_planning = false;
            return;
        }
        sampling_complete = true;
        planning_success = !path.empty();

        std::cout << "Path Solution:\n  ";
        for (auto p : path)
            std::cout << "("<< milestones[p].x << ", " << milestones[p].y << "), ";
        std::cout << std::endl;

        // smooth path
        planning::smoothe(20, path, [&](const std::size_t p1, const std::size_t p2) {
                    return collide(milestones[p1], milestones[p2]);
                });
        this->path.resize(path.size()); auto n_v = 0;
        for (const auto &p : path) this->path[n_v++] = milestones[p];
        is_planning = false;

        std::cout << "After Smoothing:\n  ";
        for (auto p : this->path)
            std::cout << "("<< p.x << ", " << p.y << "), ";
        std::cout << "\n" << std::endl;
    }

    void sampleCollisionFree(std::size_t n, std::vector<glm::vec2> &milestones)
    {
        auto lower_x = lower_bound.x + agent_pos.z;
        auto lower_y = lower_bound.y + agent_pos.z;
        auto s_x = upper_bound.x - (lower_x + agent_pos.z);
        auto s_y = upper_bound.y - (lower_y + agent_pos.z);
        while (milestones.size() < n)
        {
            glm::vec2 tar(rnd()*s_x + lower_x, rnd()*s_y + lower_y);
            if ((tar.x != agent_pos.x || tar.y != agent_pos.y)
                && (tar.x != goal_pos.x || tar.y != goal_pos.y)
                && isFree(tar))
                milestones.push_back(tar);
        }
    }
    void sample(std::size_t n, std::vector<glm::vec2> &milestones)
    {
        auto lower_x = lower_bound.x + agent_pos.z;
        auto lower_y = lower_bound.y + agent_pos.z;
        auto s_x = upper_bound.x - (lower_x + agent_pos.z);
        auto s_y = upper_bound.y - (lower_y + agent_pos.z);
        while (milestones.size() < n)
        {
            auto x = rnd()*s_x + lower_x;
            auto y = rnd()*s_y + lower_y;
            if ((x != agent_pos.x || y != agent_pos.y)
                && (x != goal_pos.x || y != goal_pos.y))
                milestones.emplace_back(x, y);
        }
    }

    bool isFree(glm::vec2 const &pos)
    {
        for (auto const &o : obstacle_pos)
        {
            auto r2 = this->agent_pos.z + o.z; r2 *= r2;
            auto dx = pos.x - o.x;
            auto dy = pos.y - o.y;
            auto d2 = dx*dx + dy*dy;
            if (d2 < r2) return false;
        }
        return true;
    }
    bool collide(glm::vec2 const &p1, glm::vec2 const &p2)
    {
        if (p1.x == p2.x && p1.y == p2.y)
        {
            for (auto const &o : obstacle_pos)
            {
                auto r = this->agent_pos.z + o.z; r *= r;
                if (glm::distance2(glm::vec2(o.x, o.y), p1) < r)
                    return true;
            }
        }
        else
        {
            // line of p1,p2: ax + by + c = 0
            // squared distance from obstacle to the line:  (a*o.x + b*o.y + c)^2 / (a*a+b*b)
            auto a = p2.y - p1.y;
            auto b = p1.x - p2.x;
            auto c = -b * p1.y - a * p1.x;
            auto den = a * a + b * b;
            for (auto const &o : obstacle_pos)
            {
                auto r = this->agent_pos.z + o.z; r *= r;
                auto d = a*o.x+b*o.y+c; d *= d; d /= den;
                if (d < r)
                {
                    auto d1 = glm::distance2(glm::vec2(o.x, o.y), p1);
                    auto d2 = glm::distance2(glm::vec2(o.x, o.y), p2);
                    if (d1 < r || d2 < r || std::max(d1, d2) - d < glm::distance2(p1, p2))
                        return true;
                }
            }
        }
        return false;
    }
    float collideFromTo(glm::vec2 const &p1, glm::vec2 const &p2)
    {
        auto alpha = 1.f;
        if (p1.x == p2.x && p1.y == p2.y)
        {
            for (auto const &o : obstacle_pos)
            {
                auto r = this->agent_pos.z + o.z; r *= r;
                if (glm::distance2(glm::vec2(o.x, o.y), p1) < r)
                    return 0.f;
            }
        }
        else
        {
            // line of p1,p2: ax + by + c = 0
            // squared distance from obstacle to the line:  (a*o.x + b*o.y + c)^2 / (a*a+b*b)
            auto a = p2.y - p1.y;
            auto b = p1.x - p2.x;
            auto c = -b * p1.y - a * p1.x;
            auto den = a * a + b * b;
            for (auto const &o : obstacle_pos)
            {
                auto r = this->agent_pos.z + o.z; r *= r;
                auto d1 = glm::distance2(glm::vec2(o.x, o.y), p1);
                if (d1 <= r)
                {
                    return 0.f;
                }

                auto d = a*o.x+b*o.y+c; d *= d; d /= den;
                if (d < r)
                {
                    auto d2 = glm::distance2(glm::vec2(o.x, o.y), p2);
                    auto x  = glm::distance2(p1, p2);
                    if (d2 < r || std::max(d1, d2) - d < x)
                    {
                        auto tmp_a = (std::sqrt(d1 - d) - std::sqrt(r - d)) / std::sqrt(x);
                        if (tmp_a < alpha) alpha = tmp_a;
                    }
                }
            }
        }
        return alpha;
    }

    glm::vec2 ttc(const glm::vec2 &v_pref, const glm::vec2 &pos, float dt)
    {
        auto v = planning::TTC(t_h, v_max, dt,
                               0, obstacle_pos,
                               std::vector<glm::vec3>{glm::vec3(pos, agent_pos.z)},
                               std::vector<glm::vec2>{v_pref});
        return v + steerBoundary(v_pref, pos) * dt;
    }

    glm::vec2 vo(const glm::vec2 &v_pref, const glm::vec2 &pos, const float dt)
    {
        auto vos = planning::VO2D< planning::VelocityObstacleMethod::VO>(
                dt,
                0, obstacle_pos,
                std::vector<glm::vec3>{glm::vec3(pos, agent_pos.z)},
                std::vector<glm::vec2>{agent_vel},
                std::vector<glm::vec2>{v_pref});

        auto max_positive_v_x = std::min(v_max,  (upper_bound.x - agent_pos.z - pos.x)/dt);
        auto max_negative_v_x = std::max(-v_max, (lower_bound.x + agent_pos.z - pos.x)/dt);
        auto max_positive_v_y = std::min(v_max,  (upper_bound.y - agent_pos.z - pos.y)/dt);
        auto max_negative_v_y = std::max(-v_max, (lower_bound.y + agent_pos.z - pos.y)/dt);
        return planning::clearPath(v_max, v_pref, vos,
                                   std::function<bool(glm::vec2 &)>(
                                   [&](glm::vec2 &v)
                                   {
                                       auto s = 1.f;
                                       if (v.x > max_positive_v_x)
                                           s = max_positive_v_x/v.x;
                                       else if (v.x < max_negative_v_x)
                                           s = max_negative_v_x/v.x;
                                       if (v.y > max_positive_v_y)
                                           s = std::min(max_positive_v_y/v.y, s);
                                       else if (v.y < max_negative_v_y)
                                           s = std::min(max_negative_v_y/v.y, s);

                                       v.x *= s;
                                       v.y *= s;

                                       return true;
                                   }));
    }

    glm::vec2 roca(const glm::vec2 &v_pref, const glm::vec2 &pos, float dt)
    {
        auto max_positive_v_x = std::min( v_max, (upper_bound.x - agent_pos.z - pos.x)/dt);
        auto max_negative_v_x = std::max(-v_max, (lower_bound.x + agent_pos.z - pos.x)/dt);
        auto max_positive_v_y = std::min( v_max, (upper_bound.y - agent_pos.z - pos.y)/dt);
        auto max_negative_v_y = std::max(-v_max, (lower_bound.y + agent_pos.z - pos.y)/dt);
        return planning::ORCA(tau, v_max, dt,
                              0, obstacle_pos,
                              std::vector<glm::vec3>{glm::vec3(pos, agent_pos.z)},
                              std::vector<glm::vec2>{agent_vel},
                              std::vector<glm::vec2>{v_pref},
                              glm::vec2(max_positive_v_x, max_positive_v_y),
                              glm::vec2(max_negative_v_x, max_negative_v_y));

    }

    glm::vec2 steerBoundary(const glm::vec2 &vel, const glm::vec2 &pos)
    {
        glm::vec2 steer(0.f);
        if (pos.x < upper_bound.x)
        {
            auto d = upper_bound.x - pos.x - agent_pos.z;
            if (d > 1.f)
            {
                auto mag = 1.f / d;
                steer.x -= mag;
                if (vel.y > 0.f)
                    steer.y += mag;
                else if (vel.y < 0.f)
                    steer.y -= mag;
            }
        }
        if (pos.x > lower_bound.x)
        {
            auto d = lower_bound.x - pos.x - agent_pos.z;
            if (d > 1.f)
            {
                auto mag = 1.f / d;
                steer.x += mag;
                if (vel.y > 0.f)
                    steer.y += mag;
                else if (vel.y < 0.f)
                    steer.y -= mag;
            }
        }
        if (pos.y < upper_bound.y)
        {
            auto d = upper_bound.y - pos.y - agent_pos.z;
            if (d > 1.f)
            {
                auto mag = 1.f /d ;
                steer.y -= mag;
                if (vel.x > 0.f)
                    steer.x += mag;
                else if (vel.x == 0.f)
                    steer.x -= mag;
            }
        }
        if (pos.y > lower_bound.y)
        {
            auto d = lower_bound.y - pos.y - agent_pos.z;
            if (d > 1.f)
            {
                auto mag = 1.f/ d;
                steer.y += mag;
                if (vel.x > 0.f)
                    steer.x += mag;
                else if (vel.x == 0.f)
                    steer.x -= mag;
            }
        }
        return steer;
    }
};

scene::RoadmapScene::RoadmapScene()
        : BaseScene(),
          v_max(7.5f), t_h(1.f), tau(2.f),
          n_obstacles(10),
          planning_method(PlanningMethod::ProbabilityRoadMap),
          pathfinder(PathFinder::AStar)
{
    pimpl = std::make_unique<impl>(this);
}

scene::RoadmapScene::~RoadmapScene()
{}

void scene::RoadmapScene::init(Scene &scene)
{
    pimpl->genGLObjs();

    if (pimpl->skybox == nullptr)
        pimpl->skybox = std::make_unique<SkyBox>(ASSET_PATH "/texture/skybox/right.jpg",
                                                ASSET_PATH "/texture/skybox/left.jpg",
                                                ASSET_PATH "/texture/skybox/top.jpg",
                                                ASSET_PATH "/texture/skybox/bottom.jpg",
                                                ASSET_PATH "/texture/skybox/back.jpg",
                                                ASSET_PATH "/texture/skybox/front.jpg");
    if (pimpl->floor == nullptr)
        pimpl->floor = std::make_shared<item::Floor>();
    if (pimpl->agent == nullptr)
        pimpl->agent = std::make_shared<item::Pillar>();
    if (pimpl->mark_shader == nullptr)
    {
        pimpl->mark_shader = std::make_unique<Shader>(pimpl->MARK_VS,
#include "shader/glsl/simple_phong.fs"
                                       );
    }
    if (pimpl->edge_shader == nullptr)
        pimpl->edge_shader = std::make_unique<Shader>(pimpl->EDGE_VS, pimpl->EDGE_FS);
    if (pimpl->road_shader == nullptr)
        pimpl->road_shader = std::make_unique<Shader>(pimpl->ROAD_VS, pimpl->ROAD_FS);

    std::vector<float> vertex;
    constexpr auto grid = 32;
    constexpr auto gap = static_cast<float>(2*M_PI / grid);
    vertex.reserve(2*(grid+2));
    vertex.push_back(0.f);
    vertex.push_back(0.f);
    for (auto i = 0; i < grid+1; ++i)
    {
        vertex.push_back(.1f * std::cos(i*gap));
        vertex.push_back(.1f * std::sin(i*gap));
    }
    glBindVertexArray(pimpl->vao[0]);  // milestone
    glVertexAttribDivisor(1, 1);
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[0]);
    glBufferData(GL_ARRAY_BUFFER, vertex.size()*sizeof(float), vertex.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, (void *)(0));
    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[1]);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, (void *)(0));

    glBindVertexArray(pimpl->vao[1]); // road
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[1]);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, (void *)(0));

    glBindVertexArray(pimpl->vao[2]); // path solution
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[2]);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, (void *)(0));

    pimpl->road_shader->activate();
    pimpl->road_shader->bind("GlobalAttributes", 0);
    pimpl->edge_shader->activate();
    pimpl->edge_shader->bind("GlobalAttributes", 0);
    pimpl->mark_shader->activate();
    pimpl->mark_shader->bind("GlobalAttributes", 0);
    pimpl->mark_shader->activate();
    pimpl->mark_shader->set("material.alpha", 1.f);
    pimpl->mark_shader->set("material.diffuse", glm::vec3(0.1961f, 0.8039f, 0.1961f));
    pimpl->mark_shader->set("material.specular", glm::vec3(0.5f));
    pimpl->mark_shader->set("material.shininess", 4.f);

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    pimpl->mark_shader->activate(false);
}

void scene::RoadmapScene::restart(Scene &scene)
{
    static auto first_run = true;
    if (first_run)
    {
        pause = false;
        keep_obstacles = false;
        keep_samples = false;
        resetCamera();
        first_run = false;
    }
    pimpl->n_sampling = 500;
    pimpl->k = 15;//pimpl->n_sampling; // they said 15 is a magic number for this problem
    pimpl->upper_bound = glm::vec2(10.f, 10.f);
    pimpl->lower_bound = glm::vec2(-10.f, -10.f);
    pimpl->goal_pos = glm::vec2(9.f, 9.f);
    pimpl->agent_pos = glm::vec3(-9.f, -9.f, .5f);
    pimpl->agent_vel = glm::vec2(0.f);

    // generate obstacles
    if (!keep_obstacles)
    {
        pimpl->obstacle_pos.clear();
        pimpl->obstacle_pos.emplace_back(0.f, 0.f, 2.f);
        auto s_x = pimpl->upper_bound.x - pimpl->lower_bound.x - 4.f;
        auto s_y = pimpl->upper_bound.y - pimpl->lower_bound.y - 4.f;
        for (auto i = 0; i < n_obstacles; ++i)
        {
            auto x = rnd()*s_x + pimpl->lower_bound.x + 2.f;
            auto y = rnd()*s_y + pimpl->lower_bound.y + 2.f;
            auto r = .1f + rnd()*1.9f;
            if ((x-pimpl->agent_pos.x)*(x-pimpl->agent_pos.x) + (y-pimpl->agent_pos.y)*(y-pimpl->agent_pos.y)
                <= (r+pimpl->agent_pos.z)*(r+pimpl->agent_pos.z) ||
                (x-pimpl->goal_pos.x)*(x-pimpl->goal_pos.x) + (y-pimpl->goal_pos.y)*(y-pimpl->goal_pos.y)
                <= (r+pimpl->agent_pos.z)*(r+pimpl->agent_pos.z))
                continue;

            auto collide = false;
            for (auto const &o : pimpl->obstacle_pos)
            {
                if ((x-o.x)*(x-o.x)+(y-o.y)*(y-o.y) <= (r + o.z)*(r + o.z))
                {
                    collide = true;
                    break;
                }
            }
            if (!collide) pimpl->obstacle_pos.emplace_back(x, y, r);
        }
        pimpl->obstacles.clear();
        for (auto const &o : pimpl->obstacle_pos)
            pimpl->obstacles.emplace_back(new item::Pillar(glm::vec3(o.x, .52f, -o.y), o.z, 1.f));
    }

    pimpl->floor->scale(glm::vec3(pimpl->upper_bound.x-pimpl->lower_bound.x, 1.f, pimpl->upper_bound.y-pimpl->lower_bound.y));
    pimpl->floor->place(glm::vec3(pimpl->lower_bound.x, 0.f, pimpl->lower_bound.y));
    pimpl->floor->setGrid(pimpl->upper_bound.x-pimpl->lower_bound.x, pimpl->upper_bound.y-pimpl->lower_bound.y);
    scene.add(pimpl->floor);

    pimpl->agent->scale(glm::vec3(pimpl->agent_pos.z, .5f, pimpl->agent_pos.z));
    pimpl->agent->place(glm::vec3(pimpl->agent_pos.x, .52f, -pimpl->agent_pos.y));
    pimpl->agent->color.ambient = glm::vec3(0.0588f, 0.2824f, .5f);
    pimpl->agent->color.diffuse = glm::vec3(0.1176f, 0.5647f, 1.f);
    pimpl->agent->color.specular = glm::vec3(0.1f);
    scene.add(pimpl->agent);
    for (auto o : pimpl->obstacles) scene.add(o);

    if (pimpl->worker_thread && pimpl->worker_thread->joinable())
        pimpl->worker_thread->join();
    if (!keep_samples) pimpl->milestones.clear();
    pimpl->need_upload = true;
    pimpl->sampling_upload = false;
    pimpl->road_upload = false;
}

void scene::RoadmapScene::upload(Shader &scene_shader)
{
    if (pimpl->need_upload)
    {
        scene_shader.activate();
        scene_shader.set("headlight.ambient", glm::vec3(.3f, .25f, .15f));
        scene_shader.set("headlight.diffuse", glm::vec3(.125f, .1f, .0625f));
        scene_shader.set("headlight.specular", glm::vec3(.25f, .25f, .25f));
        scene_shader.set("headlight.coef_a0", 1.f);
        scene_shader.set("headlight.coef_a1", 0.f);
        scene_shader.set("headlight.coef_a2", 0.f);

        pimpl->is_planning = true;
        pimpl->sampling_complete = false;
        if (pimpl->worker_thread && pimpl->worker_thread->joinable())
            pimpl->worker_thread->join();
        delete pimpl->worker_thread;
        pimpl->worker_thread = new std::thread(&impl::plan, pimpl.get(), planning_method, pathfinder);
        pimpl->need_upload = false;
    }

    if (!pimpl->sampling_upload && pimpl->sampling_complete)
    {
        glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[1]);
        glBufferData(GL_ARRAY_BUFFER, pimpl->milestones.size()*sizeof(float)*2, pimpl->milestones.data(), GL_STATIC_DRAW);

        std::vector<int> road; road.reserve(pimpl->roadmap.size()*pimpl->roadmap.size());
        for (auto i = 0, n_v = static_cast<int>(pimpl->roadmap.size()); i < n_v; ++i)
        {
            for (auto j = 0; j < n_v; ++j)
            {
                if (pimpl->roadmap[i][j] != std::numeric_limits<float>::infinity())
                {
                    road.push_back(i);
                    road.push_back(j);
                }
            }
        }

        glBindVertexArray(pimpl->vao[1]);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, pimpl->vbo[3]);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, road.size()*sizeof(int), road.data(), GL_STATIC_DRAW);
        pimpl->n_road =  static_cast<int>(road.size());

        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

        pimpl->sampling_upload = true;
    }

    if (!pimpl->road_upload && pimpl->sampling_upload && !pimpl->is_planning)
    {
        if (pimpl->planning_success)
        {
            glBindVertexArray(pimpl->vao[2]);
            glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[2]);
            glBufferData(GL_ARRAY_BUFFER, pimpl->path.size()*sizeof(float)*2, pimpl->path.data(), GL_STATIC_DRAW);
            pimpl->solution_size = static_cast<int>(pimpl->path.size());

            glBindVertexArray(0);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        }

        pimpl->road_upload = true;
    }

}

void scene::RoadmapScene::update(float dt)
{
    processInput(dt);
    if (pause || pimpl->is_planning || !pimpl->planning_success || pimpl->path.empty()) return;

    constexpr auto pos_eps  = 1e-6f;  // eps to check if the agent reaches the marker

    auto agent_pos = pimpl->agent->pos();
    glm::vec2 pos(agent_pos.x, -agent_pos.z);

    auto tar = pimpl->path.back();
    auto dist = glm::distance(tar, glm::vec2(pos.x, pos.y));
    glm::vec2 v_pref;
    bool goal_reachable = false;
    if ((v_max * dt) >= dist)
    {
        goal_reachable = true;
        v_pref.x = (tar.x - pos.x) / dt;
        v_pref.y = (tar.y - pos.y) / dt;
    }
    else
    {
        v_pref.x = v_max * (tar.x - pos.x) / dist;
        v_pref.y = v_max * (tar.y - pos.y) / dist;
    }

    switch (planning_method)
    {
        case PlanningMethod::VelocityObstacles:
            pimpl->agent_vel = pimpl->vo(v_pref, pos, dt);
            break;
        case PlanningMethod::OptimalReciprocalCollisionAvoidance:
            pimpl->agent_vel = pimpl->roca(v_pref, pos, dt);
            break;
        case PlanningMethod::TimeToCollision:
            pimpl->agent_vel = pimpl->ttc(v_pref, pos, dt);
            break;
        default:
            pimpl->agent_vel = v_pref;
    }

    agent_pos.x += pimpl->agent_vel.x*dt;
    agent_pos.z -= pimpl->agent_vel.y*dt;
    if (goal_reachable && glm::distance2(pimpl->agent_vel, pimpl->agent_vel)*dt < pos_eps)
        pimpl->path.pop_back();

    pimpl->agent->place(agent_pos);
}

void scene::RoadmapScene::render()
{
    if (pimpl->sampling_upload)
    {
        pimpl->mark_shader->activate();
        glBindVertexArray(pimpl->vao[0]);
        glDrawArraysInstanced(GL_TRIANGLE_FAN, 0, 34, pimpl->milestones.size());
        if (!pimpl->is_planning)
        {
            pimpl->edge_shader->activate();
            glBindVertexArray(pimpl->vao[1]);
            glDrawElements(GL_LINES, pimpl->n_road, GL_UNSIGNED_INT, 0);

            if (pimpl->planning_success)
            {
                pimpl->road_shader->activate();
                glBindVertexArray(pimpl->vao[2]);
                glDrawArrays(GL_LINE_STRIP, 0, pimpl->solution_size);
            }
        }
    }

    pimpl->skybox->render();
    renderInfo();
}

void scene::RoadmapScene::resetCamera()
{
    App::instance()->scene.character.reset(0.f, 20.f, 14.f, 0.f, 60.f);
    App::instance()->scene.character.setShootable(false);
    App::instance()->scene.character.setFloating(true);
}

void scene::RoadmapScene::renderInfo()
{
    auto h = 10;
    App::instance()->text("Planning Method: " + std::string(
            planning_method == PlanningMethod::RapidlyExploringRandomTrees ?
                          "Rapidly-Exploring Random Trees" :
            (planning_method == PlanningMethod::OptimalRapidlyExploringRandomTrees ?
                          "Optimal Rapidly-Exploring Random Trees" :
            (planning_method == PlanningMethod::ProbabilityRoadMap ?
                          "Probability Road Map" :
            (planning_method == PlanningMethod::VelocityObstacles ?
                          "Velocity Obstacles" :
            (planning_method == PlanningMethod::OptimalReciprocalCollisionAvoidance ?
                          "Optimal Reciprocal Collision Avoidance" :
            (planning_method == PlanningMethod::LazyProbabilityRoadMap ?
                          "Lazy Probability Road Map" :
                          "Time-to-Collision")))))),
                          10, h, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::LeftTop);
    if (planning_method == PlanningMethod::RapidlyExploringRandomTrees
        || planning_method == PlanningMethod::OptimalRapidlyExploringRandomTrees
        || planning_method == PlanningMethod::ProbabilityRoadMap
        || planning_method == PlanningMethod::LazyProbabilityRoadMap)
    {
        h += 20;
        App::instance()->text("Samples: " + std::to_string(pimpl->n_sampling),
                              10, h, .4f,
                              glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                              Anchor::LeftTop);
    }
    if (planning_method == PlanningMethod::ProbabilityRoadMap)
    {
        h += 20;
        App::instance()->text("Neighbor Check: " + std::to_string(pimpl->k),
                              10, h, .4f,
                              glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                              Anchor::LeftTop);
    }
    if (planning_method == PlanningMethod::ProbabilityRoadMap
            || planning_method == PlanningMethod::LazyProbabilityRoadMap)
    {
        h += 20;
        App::instance()->text("Path Finder Algorithm: " + std::string(
                pathfinder == PathFinder::UniformCost ? "Uniform Cost Search" : "A Star"),
                              10, h, .4f,
                              glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                              Anchor::LeftTop);

    }
    h = App::instance()->frameHeight();
    auto w = App::instance()->frameWidth();
    if (pimpl->is_planning)
        App::instance()->text("Planning...",
                              w/2, h/2, .4f,
                              glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                              Anchor::Center);
    else if (!pimpl->planning_success)
        App::instance()->text("Planning Failed",
                              w/2, h/2, .4f,
                              glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                              Anchor::Center);
    if (planning_method == PlanningMethod::ProbabilityRoadMap
        || planning_method == PlanningMethod::LazyProbabilityRoadMap)
    {
        h -= 25;
        App::instance()->text("Press N to switch path finder algorithm",
                              w, h, .4f,
                              glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                              Anchor::RightTop);
    }

}

void scene::RoadmapScene::processInput(float dt)
{
    static auto last_key = GLFW_KEY_UNKNOWN;
    auto window = App::instance()->window();

    if (glfwGetKey(window, GLFW_KEY_M) == GLFW_PRESS)
        last_key = GLFW_KEY_M;
    else if (glfwGetKey(window, GLFW_KEY_N) == GLFW_PRESS)
        last_key = GLFW_KEY_N;
    else if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS)
        last_key = GLFW_KEY_P;
    else if (glfwGetKey(window, GLFW_KEY_B) == GLFW_PRESS)
        last_key = GLFW_KEY_B;
    else
    {
        if (last_key == GLFW_KEY_B)
            resetCamera();
        else if (last_key == GLFW_KEY_P)
            pause = !pause;
        else if (last_key == GLFW_KEY_M)
        {
            if (planning_method == PlanningMethod::ProbabilityRoadMap)
                planning_method = PlanningMethod::RapidlyExploringRandomTrees;
            else if (planning_method == PlanningMethod::RapidlyExploringRandomTrees)
                planning_method = PlanningMethod::OptimalRapidlyExploringRandomTrees;
            else if (planning_method == PlanningMethod::OptimalRapidlyExploringRandomTrees)
                planning_method = PlanningMethod::LazyProbabilityRoadMap;
            else if (planning_method == PlanningMethod::LazyProbabilityRoadMap)
                planning_method = PlanningMethod::VelocityObstacles;
            else if (planning_method == PlanningMethod::VelocityObstacles)
                planning_method = PlanningMethod::OptimalReciprocalCollisionAvoidance;
            else if (planning_method == PlanningMethod::OptimalReciprocalCollisionAvoidance)
                planning_method = PlanningMethod::TimeToCollision;
            else // if (planning_method = PlanningMethod::TimeToCollision)
                planning_method = PlanningMethod::ProbabilityRoadMap;

            auto tmp = keep_obstacles;
            keep_obstacles = true;
            App::instance()->scene.restart();
            keep_obstacles = tmp;
        }
        else if (last_key == GLFW_KEY_N &&
                (planning_method == PlanningMethod::ProbabilityRoadMap
                 || planning_method == PlanningMethod::LazyProbabilityRoadMap))
        {
            if (pathfinder == PathFinder::UniformCost)
                pathfinder = PathFinder::AStar;
            else // if (pathfinder == PathFinder::AStar)
                pathfinder = PathFinder::UniformCost;
            auto tmp1 = keep_obstacles;
            auto tmp2 = keep_samples;
            keep_obstacles = true;
            keep_samples = true;
            App::instance()->scene.restart();
            keep_obstacles = tmp1;
            keep_samples = tmp2;
        }
        last_key = GLFW_KEY_UNKNOWN;
    }
}
