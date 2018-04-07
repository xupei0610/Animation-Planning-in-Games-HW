
#include <memory>
#include <glm/gtx/norm.hpp>

#include "scene/multi_agent_scene.hpp"
#include "scene.hpp"
#include "app.hpp"
#include "global.hpp"
#include "shader/skybox.hpp"
#include "item/floor.hpp"
#include "item/pillar.hpp"
#include "util/motion_planning.hpp"

using namespace px;

class scene::MultiAgentScene::impl
{
public:
    bool need_upload;

    float &v_max, &rrt_step_min, &rrt_step_max, &rewire_radius, &neighbor_radius, &t_h, &tau;

    glm::vec2 scene_upper_bound, scene_lower_bound;
    std::vector<glm::vec3> agent_pos;
    std::vector<glm::vec2> agent_vel;
    std::vector<glm::vec2> agent_goal;
    std::vector<std::list<glm::vec2> > agent_path;
    std::vector<glm::vec3> agent_color;
    std::vector<glm::vec3> obstacle_pos;
    std::vector<glm::vec2> obstacle_vel;
    std::function<void(float)> update_obstacles;

    class Dist2Helper
    {
    public:
        inline float operator()(const glm::vec3 &vec3_pos, const glm::vec2 &vec2_pos)
        {
            return (vec3_pos.x - vec2_pos.x)*(vec3_pos.x - vec2_pos.x)
                   + (vec3_pos.y - vec2_pos.y)*(vec3_pos.y - vec2_pos.y);
        }
    } dist2;

    std::unique_ptr<SkyBox> skybox;
    std::shared_ptr<item::Floor> floor_obj;
    std::vector<std::shared_ptr<item::Pillar> > obstacle_obj;
    std::vector<std::shared_ptr<item::Pillar> > agent_obj;

    class MarkerShader
    {
    public:
        unsigned int vao[1], vbo[3];
        unsigned int n_instance;
        std::unique_ptr<Shader> shader;

        const char *MARKER_VS = "#version 420 core\n"
                "layout (location = 0) in vec2 vert;"
                "layout (location = 1) in vec2 pos;"
                "layout (location = 2) in vec3 color;"
                ""
                "layout (std140, binding = 0) uniform GlobalAttributes"
                "{"
                "   mat4 view;"
                "   mat4 proj;"
                "   vec3 cam_pos;"
                "};"
                ""
                "flat out vec3 gColor;"
                ""
                "void main(){"
                "   gColor = color;"
                "   gl_Position = proj * view * vec4(vert.x+pos.x, 0.02f, vert.y-pos.y, 1.f);"
                "}";
        const char *MARKER_FS = "#version 330 core\n"
                "flat in vec3 gColor;"
                ""
                "out vec4 color;"
                ""
                "void main()"
                "{"
                "   color = vec4(gColor, 1.f);"
                "}";

        MarkerShader() : vao{0}, vbo{0}, n_instance(0), shader(nullptr)
        {}
        ~MarkerShader()
        {
            clearGLObjs();
        }
        void clearGLObjs()
        {
            glDeleteVertexArrays(1, vao);
            glDeleteBuffers(3, vbo);
        }
        void init()
        {
            clearGLObjs();
            glGenVertexArrays(1, vao);
            glGenBuffers(3, vbo);

            if (shader == nullptr)
            {
                shader = std::make_unique<Shader>(MARKER_VS, MARKER_FS);
            }
            std::vector<float> vertex;
            constexpr auto grid = 32;
            constexpr auto gap = 2*static_cast<float>(M_PI) / grid;
            vertex.reserve(2*(grid+2));
            vertex.push_back(0.f);
            vertex.push_back(0.f);
            for (auto i = 0; i < grid+1; ++i)
            {
                vertex.push_back(.3f * std::cos(i*gap));
                vertex.push_back(.3f * std::sin(i*gap));
            }
            glBindVertexArray(vao[0]);  // goal marker
            glVertexAttribDivisor(1, 1);
            glVertexAttribDivisor(2, 1);
            glEnableVertexAttribArray(0);
            glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);  // vertices
            glBufferData(GL_ARRAY_BUFFER, vertex.size()*sizeof(float), vertex.data(), GL_STATIC_DRAW);
            glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, (void *)(0));
            glEnableVertexAttribArray(1);
            glBindBuffer(GL_ARRAY_BUFFER, vbo[1]); // pos
            glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, (void *)(0));
            glEnableVertexAttribArray(2);
            glBindBuffer(GL_ARRAY_BUFFER, vbo[2]); // color
            glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, (void *)(0));
        }
        void upload(const std::vector<glm::vec2> &marker_pos,
                    const std::vector<glm::vec3> &marker_color)
        {
            assert(marker_pos.size() == marker_color.size());

            n_instance = static_cast<unsigned int>(marker_pos.size());
            glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);
            glBufferData(GL_ARRAY_BUFFER, n_instance*sizeof(glm::vec2), marker_pos.data(), GL_STATIC_DRAW);
            glBindBuffer(GL_ARRAY_BUFFER, vbo[2]);
            glBufferData(GL_ARRAY_BUFFER, n_instance*sizeof(glm::vec3), marker_color.data(), GL_STATIC_DRAW);
        }
        void render()
        {
            shader->activate();
            glBindVertexArray(vao[0]);
            glDrawArraysInstanced(GL_TRIANGLE_FAN, 0, 34, n_instance);
            shader->activate(false);
            glBindVertexArray(0);
        }
    } marker;


    explicit impl(scene::MultiAgentScene *parent)
            : need_upload(false),
              v_max(parent->v_max),
              rrt_step_min(parent->rrt_step_min),
              rrt_step_max(parent->rrt_step_max),
              rewire_radius(parent->rewire_radius),
              neighbor_radius(parent->neighbor_radius),
              t_h(parent->t_h), tau(parent->tau),
              update_obstacles{},
              skybox(nullptr), floor_obj(nullptr)
    {}
    ~impl()
    {
        marker.clearGLObjs();
    }

    void setupCircleScenario()
    {
        constexpr auto agent_r = .2f;
        constexpr auto r = 28.f;
        constexpr std::size_t n_agents = 200;
        constexpr auto gap = 2.f * static_cast<float>(M_PI) / n_agents;
        constexpr auto color_gap = 2*static_cast<float>(M_PI)/3;
        agent_pos.clear();
        agent_vel.clear();
        agent_path.clear();
        agent_goal.clear();
        agent_color.clear();
        obstacle_pos.clear();
        obstacle_vel.clear();
        update_obstacles = {};
        scene_upper_bound = glm::vec2(30.f, 30.f);
        scene_lower_bound = glm::vec2(-30.f, -30.f);
        agent_pos.reserve(n_agents);
        agent_path.reserve(n_agents);
        agent_goal.reserve(n_agents);
        agent_color.reserve(n_agents);
        agent_vel.resize(n_agents, glm::vec3(0.f));
        glm::vec3 color, next_color; float alpha;
        for (std::remove_const<decltype(n_agents)>::type i = 0; i < n_agents; ++i)
        {
            auto theta = gap*i;
            agent_pos.emplace_back(r * std::cos(theta),
                                   r * std::sin(theta),
                                   agent_r);
            agent_goal.emplace_back(-agent_pos.back());
            agent_path.emplace_back(1, agent_goal.back());

            theta = std::modf(theta/color_gap, &alpha);
            if (alpha < 1)
            {
                color = glm::vec3(1.f, 0.f, 0.f);
                next_color = glm::vec3(0.f, 1.f, 0.f);
            }
            else if (alpha < 2)
            {
                color = glm::vec3(0.f, 1.f, 0.f);
                next_color = glm::vec3(0.f, 0.f, 1.f);
            }
            else
            {
                color = glm::vec3(0.f, 0.f, 1.f);
                next_color = glm::vec3(1.f, 0.f, 0.f);
            }
            agent_color.emplace_back((1.f - theta) * color + theta * next_color);
        }
    }

    void setupBlocksScenario()
    {
        constexpr auto agent_r = .2f;
        constexpr std::size_t row = 5;
        constexpr std::size_t col = 5;
        constexpr auto n_agents = 4*row*col;
        agent_pos.clear();
        agent_vel.clear();
        agent_path.clear();
        agent_goal.clear();
        agent_color.clear();
        obstacle_pos.clear();
        obstacle_vel.clear();
        scene_upper_bound = glm::vec2(30.f, 30.f);
        scene_lower_bound = glm::vec2(-30.f, -30.f);
        agent_pos.reserve(n_agents);
        agent_path.reserve(n_agents);
        agent_goal.reserve(n_agents);
        agent_color.reserve(n_agents);
        agent_vel.resize(n_agents, glm::vec3(0.f));
        auto pos = 12.5f; auto gap_pos = 2.25f;
        for (std::remove_const<decltype(row)>::type r = 0; r < row; ++r)
        {
            for (std::remove_const<decltype(col)>::type c = 0; c < col; ++c)
            {
                agent_pos.emplace_back(pos + gap_pos*c,
                                              -pos - gap_pos*r,
                                              agent_r);
                agent_goal.emplace_back(-pos - gap_pos*c,
                                               pos + gap_pos*r);
                agent_path.emplace_back(1, agent_goal.back());
                agent_color.emplace_back(.5f, 0.f, .5f);

                agent_pos.emplace_back(pos + gap_pos*c,
                                              pos + gap_pos*r,
                                              agent_r);
                agent_goal.emplace_back(-pos - gap_pos*c,
                                               -pos - gap_pos*r);
                agent_path.emplace_back(1, agent_goal.back());
                agent_color.emplace_back(0.298f, 0.7333f, 0.09f);

                agent_pos.emplace_back(-pos - gap_pos*c,
                                              pos + gap_pos*r,
                                              agent_r);
                agent_goal.emplace_back(pos + gap_pos*c,
                                               -pos - gap_pos*r);
                agent_path.emplace_back(1, agent_goal.back());
                agent_color.emplace_back(0.1176f, 0.5647f, 1.f);

                agent_pos.emplace_back(-pos - gap_pos*c,
                                              -pos - gap_pos*r,
                                              agent_r);
                agent_goal.emplace_back(pos + gap_pos*c,
                                               pos + gap_pos*r);
                agent_path.emplace_back(1, agent_goal.back());
                agent_color.emplace_back(1.f, .5f, .2f);
            }
        }
        obstacle_pos.reserve(4);
        obstacle_pos.emplace_back( pos*.5f, -pos*.5f, 2.5f);
        obstacle_vel.emplace_back(0.f);
        obstacle_pos.emplace_back( pos*.5f,  pos*.5f, 2.5f);
        obstacle_vel.emplace_back(0.f);
        obstacle_pos.emplace_back(-pos*.5f,  pos*.5f, 2.5f);
        obstacle_vel.emplace_back(0.f);
        obstacle_pos.emplace_back(-pos*.5f, -pos*.5f, 2.5f);
        obstacle_vel.emplace_back(0.f);
    }

    void setupPassageScenario()
    {
        constexpr auto agent_r = .2f;
        constexpr std::size_t row = 5;
        constexpr std::size_t col = 6;
        constexpr auto n_agents = 2*row*col;
        auto gap_pos = 3.f;
        auto pos_y = (row*.5f-.5f)*gap_pos;
        auto pos_x = 10.5f;
        auto ob_r = 1.f;
        auto ob_pos_x = pos_x * .2f;
        auto ob_pos_y = pos_y + ob_r*2.f;
        agent_pos.clear();
        agent_vel.clear();
        agent_path.clear();
        agent_goal.clear();
        agent_color.clear();
        obstacle_pos.clear();
        obstacle_vel.clear();
        scene_upper_bound = glm::vec2(30.f, 30.f);
        scene_lower_bound = glm::vec2(-30.f, -30.f);
        agent_pos.reserve(n_agents);
        agent_path.reserve(n_agents);
        agent_goal.reserve(n_agents);
        agent_color.reserve(n_agents);
        agent_vel.resize(n_agents, glm::vec3(0.f));
        for (std::remove_const<decltype(row)>::type r = 0; r < row; ++r)
        {
            for (std::remove_const<decltype(col)>::type c = 0; c < col; ++c)
            {
                agent_pos.emplace_back(pos_x + gap_pos*c,
                                       pos_y - gap_pos*r,
                                       agent_r);
                agent_goal.emplace_back(-pos_x - gap_pos*c,
                                        pos_y - gap_pos*r);
                agent_path.emplace_back(1, agent_goal.back());
                agent_color.emplace_back(0.298f, 0.7333f, 0.09f);

                agent_pos.emplace_back(-pos_x - gap_pos*c,
                                       pos_y - gap_pos*r,
                                       agent_r);
                agent_goal.emplace_back(pos_x + gap_pos*c,
                                        pos_y - gap_pos*r);
                agent_path.emplace_back(1, agent_goal.back());
                agent_color.emplace_back(0.1176f, 0.5647f, 1.f);
            }
        }
        obstacle_pos.reserve(4);
        obstacle_pos.emplace_back(-ob_pos_x*3.f, ob_pos_y+.01f, ob_r);
        obstacle_vel.emplace_back(0.f, 2.2f);
        obstacle_pos.emplace_back(-ob_pos_x,    -ob_pos_y-.01f, ob_r);
        obstacle_vel.emplace_back(0.f, -2.2f);
        obstacle_pos.emplace_back(ob_pos_x,      ob_pos_y+.01f, ob_r);
        obstacle_vel.emplace_back(0.f, 2.2f);
        obstacle_pos.emplace_back(ob_pos_x*3.f, -ob_pos_y-.01f, ob_r);
        obstacle_vel.emplace_back(0.f, -2.2f);

        static std::vector<bool> move_down{true, false, true, false};
        update_obstacles = [ob_pos_y, this](float dt)
        {
            for (auto i = 0; i < 4; ++i)
            {
                auto pos = obstacle_obj[i]->pos();
                if (-pos.z > ob_pos_y || -pos.z < -ob_pos_y)
                {
                    move_down[i] = !move_down[i];
                    obstacle_vel[i].y = -obstacle_vel[i].y;
                }

                pos.x += obstacle_vel[i].x * dt;
                pos.z -= obstacle_vel[i].y * dt;

                obstacle_pos[i].x = pos.x;
                obstacle_pos[i].y = -pos.z;

                obstacle_obj[i]->place(pos);
            }
        };
    }

};

scene::MultiAgentScene::MultiAgentScene()
        : BaseScene(),
          v_max(5.f), v_pref(3.75f), a_max(std::numeric_limits<float>::infinity()),
          rrt_step_min(.2f),
          rrt_step_max(1.f),
          rewire_radius(5.f),
          neighbor_radius(2.f),
          f_k(.75f), t_h(.5f), tau(2.f),
          planning_method(PlanningMethod::ORCA),
          pathfinder(PathFinder::AStar),
          scenario(Scenario::Circle)
{
    pimpl = std::make_unique<impl>(this);
}

scene::MultiAgentScene::~MultiAgentScene()
{}

void scene::MultiAgentScene::init(Scene &scene)
{
    pimpl->marker.init();

    if (pimpl->skybox == nullptr)
        pimpl->skybox = std::make_unique<SkyBox>(ASSET_PATH"/texture/skybox/right.jpg",
                                                 ASSET_PATH"/texture/skybox/left.jpg",
                                                 ASSET_PATH"/texture/skybox/top.jpg",
                                                 ASSET_PATH"/texture/skybox/bottom.jpg",
                                                 ASSET_PATH"/texture/skybox/back.jpg",
                                                 ASSET_PATH"/texture/skybox/front.jpg");
    if (pimpl->floor_obj == nullptr)
        pimpl->floor_obj = std::make_shared<item::Floor>();
}

void scene::MultiAgentScene::restart(Scene &scene)
{
    static auto first_run = true;
    if (first_run)
    {
        resetCamera();
        first_run = false;
    }
    pause = false;
    App::instance()->scene.character.setForwardSpeed(Character::FORWARD_SP * 2.f);
    App::instance()->scene.character.setBackwardSpeed(Character::FORWARD_SP * 2.f);
    App::instance()->scene.character.setSidestepSpeed(Character::FORWARD_SP * 2.f);

    if (scenario == Scenario::Blocks)
        pimpl->setupBlocksScenario();
    else if (scenario == Scenario::Passage)
        pimpl->setupPassageScenario();
    else
        pimpl->setupCircleScenario();

    pimpl->floor_obj->scale(glm::vec3(
            pimpl->scene_upper_bound.x - pimpl->scene_lower_bound.x,
            1.f,
            pimpl->scene_upper_bound.y - pimpl->scene_lower_bound.y));
    pimpl->floor_obj->place(glm::vec3(
            pimpl->scene_lower_bound.x, 0.f, pimpl->scene_lower_bound.y));
    pimpl->floor_obj->setGrid(
            (pimpl->scene_upper_bound.x - pimpl->scene_lower_bound.x)*.5f,
            (pimpl->scene_upper_bound.y - pimpl->scene_lower_bound.y)*.5f);

    auto n = pimpl->agent_pos.size();
    pimpl->agent_obj.resize(n, nullptr);
    for (decltype(n) i = 0; i < n; ++i)
    {
        if (pimpl->agent_obj[i] == nullptr)
            pimpl->agent_obj[i] = std::make_shared<item::Pillar>();
        pimpl->agent_obj[i]->scale(glm::vec3(
                pimpl->agent_pos[i].z, .5f,  pimpl->agent_pos[i].z));
        pimpl->agent_obj[i]->place(glm::vec3(
                pimpl->agent_pos[i].x, .5f, -pimpl->agent_pos[i].y));
        pimpl->agent_obj[i]->color.ambient = pimpl->agent_color[i]*.5f;
        pimpl->agent_obj[i]->color.diffuse = pimpl->agent_color[i];
        pimpl->agent_obj[i]->color.specular = glm::vec3(0.1f);
    }
    n = pimpl->obstacle_pos.size();
    pimpl->obstacle_obj.resize(n, nullptr);
    for (decltype(n) i = 0; i < n; ++i)
    {
        if (pimpl->obstacle_obj[i] == nullptr)
            pimpl->obstacle_obj[i] = std::make_shared<item::Pillar>();
        pimpl->obstacle_obj[i]->scale(glm::vec3(
                pimpl->obstacle_pos[i].z, .75f,  pimpl->obstacle_pos[i].z));
        pimpl->obstacle_obj[i]->place(glm::vec3(
                pimpl->obstacle_pos[i].x, .75f, -pimpl->obstacle_pos[i].y));
    }

    scene.add(pimpl->floor_obj);
    for (const auto &o : pimpl->obstacle_obj)
        scene.add(o);
    for (const auto &a : pimpl->agent_obj)
        scene.add(a);

    pimpl->need_upload = true;
}

void scene::MultiAgentScene::upload(Shader &scene_shader)
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

        pimpl->marker.upload(pimpl->agent_goal, pimpl->agent_color);
        pimpl->need_upload = false;
    }

}

void scene::MultiAgentScene::update(float dt)
{
    processInput(dt);
    if (pause || dt == 0.f) return;

    constexpr auto goal_eps2 = .2f*.2f;  // eps to check if the agent reaches the goal marker

    auto n_agents = pimpl->agent_pos.size();
    auto a_max2 = a_max * a_max * dt;
    std::vector<glm::vec2> pref_vel(n_agents);
    std::vector<glm::vec2> vel(n_agents);
    std::vector<bool> goal_reach(n_agents, true);

#pragma omp parallel num_threads(6)
    {
#pragma omp for
        for (decltype(n_agents) i = 0; i < n_agents; ++i)  // obtain v_pref
        {
            if (pimpl->agent_path[i].empty())
            {
                pref_vel[i] = glm::vec2(0.f);
                continue;
            }

            auto obj_pos = pimpl->agent_obj[i]->pos();
            const auto &goal = pimpl->agent_path[i].back();

            pimpl->agent_pos[i].x =  obj_pos.x;   // update the cpu stored data
            pimpl->agent_pos[i].y = -obj_pos.z;

            // set preferred velocity
            pref_vel[i].x = (goal.x - pimpl->agent_pos[i].x)/dt;
            pref_vel[i].y = (goal.y - pimpl->agent_pos[i].y)/dt;
            auto d = glm::length(pref_vel[i]);
            if (d > v_pref)
                pref_vel[i] *= v_pref/ d;
        }
#pragma omp for
        for (decltype(n_agents) i = 0; i < n_agents; ++i) // do planning
        {
#define BOUNDARY_MAX_V_HELPER   \
    auto max_pos_v_x = std::min(v_max,  (pimpl->scene_upper_bound.x - pimpl->agent_pos[i].z - pimpl->agent_pos[i].x)/dt);   \
    auto max_neg_v_x = std::max(-v_max, (pimpl->scene_lower_bound.x + pimpl->agent_pos[i].z - pimpl->agent_pos[i].x)/dt);   \
    auto max_pos_v_y = std::min(v_max,  (pimpl->scene_upper_bound.y - pimpl->agent_pos[i].z - pimpl->agent_pos[i].y)/dt);   \
    auto max_neg_v_y = std::max(-v_max, (pimpl->scene_lower_bound.y + pimpl->agent_pos[i].z - pimpl->agent_pos[i].y)/dt);
#define VO_FN_CALLER(VO)    \
    {   \
    BOUNDARY_MAX_V_HELPER  \
    vel[i] = planning::clearPath(v_max, pref_vel[i],                                                        \
                                 planning::VO2D<planning::VelocityObstacleMethod::VO>(                      \
                                         neighbor_radius, dt, i, pimpl->obstacle_pos, pimpl->obstacle_vel,  \
                                         pimpl->agent_pos, pimpl->agent_vel, pref_vel,                      \
                                         scenario == Scenario::Blocks ? .03f : .0f),                        \
                                 std::function<bool(glm::vec2 &)>(                                          \
                                         [&](glm::vec2 &v) {                                                \
                                             auto s = 1.f;                                                  \
                                             if (v.x > max_pos_v_x) s = max_pos_v_x/v.x;                    \
                                             else if (v.x < max_neg_v_x) s = max_neg_v_x/v.x;               \
                                             if (v.y > max_pos_v_y) s = std::min(max_pos_v_y/v.y, s);       \
                                             else if (v.y < max_neg_v_y) s = std::min(max_neg_v_y/v.y, s);  \
                                             v.x *= s; v.y *= s;                                            \
                                             return true;                                                   \
                                         }));                                                               \
    }

            switch (planning_method)
            {
                case PlanningMethod::VO:
                    VO_FN_CALLER(VO);
                    break;
                case PlanningMethod::RVO:
                    VO_FN_CALLER(RVO);
                    break;
                case PlanningMethod::HRVO:
                    VO_FN_CALLER(HRVO);
                    break;
                case PlanningMethod::ORCA:
                {
                    BOUNDARY_MAX_V_HELPER
                    vel[i] = planning::ORCA(tau, v_max, neighbor_radius, dt,
                                          i, pimpl->obstacle_pos, pimpl->obstacle_vel,
                                          pimpl->agent_pos,
                                          pimpl->agent_vel,
                                          pref_vel,
                                          glm::vec2(max_pos_v_x, max_pos_v_y),
                                          glm::vec2(max_neg_v_x, max_neg_v_y));
                }
                    break;
                default:
                    vel[i] =
                            planning::TTC(f_k, t_h, v_max, dt, i,
                                          pimpl->obstacle_pos,
                                          pimpl->obstacle_vel,
                                          pimpl->agent_pos, pimpl->agent_vel,
                                          pref_vel);
            }
        }
#pragma omp for
        for (decltype(n_agents) i = 0; i < n_agents; ++i) // update agent state
        {
            auto dv = vel[i] - pimpl->agent_vel[i];
            if (dv.x*dv.x + dv.y*dv.y > a_max2)
            {
                auto alpha = a_max * dt / std::sqrt(dv.x*dv.x + dv.y*dv.y);
                vel[i] = (1.f - alpha) * pimpl->agent_vel[i] + alpha * vel[i];
            }

            auto dp = vel[i] * dt; // movement
            pimpl->agent_vel[i] = vel[i];
            pimpl->agent_pos[i].x += dp.x;
            pimpl->agent_pos[i].y += dp.y;

            // check if the goal position has been reached
            if (!pimpl->agent_path[i].empty())
            {
                auto dg = pimpl->agent_path[i].back();
                dg.x -= pimpl->agent_pos[i].x;
                dg.y -= pimpl->agent_pos[i].y;
                if (dg.x*dg.x + dg.y*dg.y > goal_eps2)
                    goal_reach[i] = false;
                else if (pimpl->agent_path[i].size() > 1)
                    pimpl->agent_path[i].pop_back();
            }

            // update object place
            pimpl->agent_obj[i]->move(glm::vec3(dp.x, 0.f, -dp.y));
        }
    }

    if (std::find(goal_reach.begin(), goal_reach.end(), false) == goal_reach.end())
    {
        std::cout << "All agents reached their goals." << std::endl;
        pause = true;
    }

    if (pimpl->update_obstacles)
        pimpl->update_obstacles(dt);

// //     stop navigation for agents having no goal
// //     move such agents into the obstacle vector
//    for (auto i = 0, n = static_cast<int>(n_agents); i < n && i > -1; ++i)
//    {
//        if (pimpl->agent_path[i].empty())
//        {
//            pimpl->obstacle_pos.emplace_back(pimpl->agent_pos[i]);
//            std::swap(pimpl->agent_pos[i], pimpl->agent_pos.back());
//            std::swap(pimpl->agent_vel[i], pimpl->agent_vel.back());
//            std::swap(pimpl->agent_goal[i], pimpl->agent_goal.back());
//            std::swap(pimpl->agent_path[i], pimpl->agent_path.back());
//            std::swap(pimpl->agent_obj[i], pimpl->agent_obj[--n]);
//            pimpl->agent_pos.pop_back();
//            pimpl->agent_vel.pop_back();
//            pimpl->agent_goal.pop_back();
//            pimpl->agent_path.pop_back();
//
//            --i;
//        }
//    }
}

void scene::MultiAgentScene::render()
{
    pimpl->marker.render();
    pimpl->skybox->render();
    renderInfo();
}

void scene::MultiAgentScene::resetCamera()
{
    App::instance()->scene.character.reset(0.f, 30.f, 50.f, 0.f, 40.f);
    App::instance()->scene.character.setShootable(false);
    App::instance()->scene.character.setFloating(true);
}

void scene::MultiAgentScene::renderInfo()
{
    auto h = 10;
    App::instance()->text("Planning Method: " + std::string(
            planning_method == PlanningMethod::RRT ?
                          "Rapidly-Exploring Random Trees" :
            (planning_method == PlanningMethod::RRTStar ?
                          "Optimal Rapidly-Exploring Random Trees" :
            (planning_method == PlanningMethod::PRM ?
                          "Probability Road Map" :
            (planning_method == PlanningMethod::VO ?
                          "Velocity Obstacles" :
            (planning_method == PlanningMethod::RVO ?
                          "Reciprocal Velocity Obstacles" :
            (planning_method == PlanningMethod::HRVO ?
                          "Hybrid Reciprocal Velocity Obstacles" :
            (planning_method == PlanningMethod::ORCA ?
                          "Optimal Reciprocal Collision Avoidance" :
            (planning_method == PlanningMethod::LazyPRM ?
                          "Lazy Probability Road Map" :
                          "Time-to-Collision")))))))),
                          10, h, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::LeftTop);

}

void scene::MultiAgentScene::processInput(float dt)
{
    static auto last_key = GLFW_KEY_UNKNOWN;
    auto window = App::instance()->window();

#define PRESS_KEY_CHECK(Key)                                \
    if (glfwGetKey(window, GLFW_KEY_##Key) == GLFW_PRESS) { \
        last_key = GLFW_KEY_##Key;                          \
    }
#define KEY_PRESSED(Key)    (last_key == GLFW_KEY_##Key)
#define RESET_PRESSED_KEY   last_key = GLFW_KEY_UNKNOWN

#define CHANGE_SCENARIO(Scene)                           \
    {if (scenario != Scenario::Scene) {                  \
        scenario = Scenario::Scene;                      \
        App::instance()->scene.restart();                \
    }}

    PRESS_KEY_CHECK(M)
    else PRESS_KEY_CHECK(N)
    else PRESS_KEY_CHECK(P)
    else PRESS_KEY_CHECK(B)
    else PRESS_KEY_CHECK(1)
    else PRESS_KEY_CHECK(2)
    else PRESS_KEY_CHECK(3)
    else
    {
        if (KEY_PRESSED(B))
            resetCamera();
        else if (KEY_PRESSED(P))
            pause = !pause;
        else if (KEY_PRESSED(1))
            CHANGE_SCENARIO(Circle)
        else if (KEY_PRESSED(2))
            CHANGE_SCENARIO(Blocks)
        else if (KEY_PRESSED(3))
            CHANGE_SCENARIO(Passage)
        else if (KEY_PRESSED(M))
        {
//            if (planning_method == PlanningMethod::PRM)
//                planning_method = PlanningMethod::RRT;
//            else if (planning_method == PlanningMethod::RRT)
//                planning_method = PlanningMethod::RRTStar;
//            else if (planning_method == PlanningMethod::RRTStar)
//                planning_method = PlanningMethod::LazyPRM;
//            else if (planning_method == PlanningMethod::LazyPRM)
//                planning_method = PlanningMethod::VO;
//            else
            if (planning_method == PlanningMethod::VO)
                planning_method = PlanningMethod::RVO;
            else if (planning_method == PlanningMethod::RVO)
                planning_method = PlanningMethod::HRVO;
            else if (planning_method == PlanningMethod::HRVO)
                planning_method = PlanningMethod::ORCA;
            else if (planning_method == PlanningMethod::ORCA)
                planning_method = PlanningMethod::TTC;
            else // if (planning_method = PlanningMethod::TTC)
//                planning_method = PlanningMethod::PRM
                planning_method = PlanningMethod::VO;

            App::instance()->scene.restart();
        }
        else if (KEY_PRESSED(N) &&
                 (planning_method == PlanningMethod::PRM
                  || planning_method == PlanningMethod::LazyPRM))
        {
            if (pathfinder == PathFinder::UniformCost)
                pathfinder = PathFinder::AStar;
            else // if (pathfinder == PathFinder::AStar)
                pathfinder = PathFinder::UniformCost;
            App::instance()->scene.restart();
        }

        RESET_PRESSED_KEY;
    }
}
