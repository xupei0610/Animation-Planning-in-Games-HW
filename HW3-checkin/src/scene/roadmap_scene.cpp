#include <sstream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <glm/gtx/norm.hpp>

#include "scene/roadmap_scene.hpp"
#include "scene.hpp"
#include "app.hpp"
#include "global.hpp"
#include "shader/skybox.hpp"
#include "item/floor.hpp"
#include "item/pillar.hpp"
#include "util/random.hpp"

using namespace px;

class scene::RoadmapScene::impl
{
public:
    bool need_upload;
    bool is_planning, planning_success;

    int n_sampling;
    glm::vec2 upper_bound, lower_bound;
    glm::vec3 agent_pos;
    glm::vec2 goal_pos;
    std::vector<glm::vec2> path;
    std::vector<glm::vec3> obstacle_pos;

    std::thread *worker_thread;

    SkyBox *skybox;
    item::Floor *floor;
    item::Pillar *agent;

    impl() : worker_thread(nullptr),
             skybox(nullptr), floor(nullptr),
             agent(nullptr)
    {}
    ~impl()
    {
        if (worker_thread && worker_thread->joinable())
            worker_thread->join();
        delete skybox;
        delete floor;
        delete agent;
    }

    void plan()
    {
        planning_success = false;
        while (planning_success == false)
        {
            // sampling uniformly
            std::vector<glm::vec2> coord;
            for (auto i = 0; i < n_sampling; ++i)
            {
                glm::vec2 tar(rnd()*(upper_bound.x - lower_bound.x) + lower_bound.x,
                              rnd()*(upper_bound.y - lower_bound.y) + lower_bound.y);
                if (!collide(tar)) coord.push_back(tar);
            }

            // Dijkstra
            coord.push_back(glm::vec2(goal_pos.x, goal_pos.y));
            coord.push_back(glm::vec2(agent_pos.x, agent_pos.y));
            auto n_v = static_cast<int>(coord.size());
            std::vector<bool> visited(n_v, false);
            std::vector<float> dist(n_v, std::numeric_limits<float>::infinity());
            dist.back() = 0.f;

            std::vector<float*> len(n_v, nullptr);
            std::vector<int> prev(n_v, -1);
            for (auto i = 0; i < n_v; ++i)
                len[i] = new float[n_v];
            for (auto i = 0; i < n_v; ++i)
            {
                len[i][i] = 0.f;
                for (auto j = i+1; j < n_v; ++j)
                {
                    if (collide(coord[i], coord[j]))
                        len[i][j] = std::numeric_limits<float>::infinity();
                    else
                        len[i][j] = glm::distance(coord[i], coord[j]);
                    len[j][i] = len[i][j];
                }
            }

            int u = n_v-1; float min_dist;
            for (auto i = 0; i < n_v-1; ++i)
            {
                min_dist = std::numeric_limits<float>::max();
                for (auto j = 0; j < n_v; ++j)
                {
                    if (visited[j] == false && dist[j] <= min_dist)
                    {
                        min_dist = dist[j];
                        u = j;
                    }
                }

                visited[u] = true;

                for (auto v = 0; v < n_v; ++v)
                {
                    if (visited[v] == false)
                    {
                        if (len[u][v] != std::numeric_limits<float>::infinity()
                            && dist[u] != std::numeric_limits<float>::infinity()
                            && dist[u] + len[u][v] < dist[v])
                        {
                            dist[v] = dist[u] + len[u][v];
                            prev[v] = u;
                        }
                    }
                }
            }
            for (auto &p : len)
                delete [] p;

            std::cout << "Path Solution:\n" << "  Node\tPrevious\n";
            for (auto i = 0; i < n_v-2; ++i)
                std::cout << std::setw(6) << i << "\t" << std::setw(6) << prev[i] << "\n";
            std::cout << std::setw(6) << n_v-2 << "\t" << std::setw(6) << prev[n_v-2] << "(goal)\n"
                      << std::setw(6) << n_v-1 << "\t" << std::setw(6) << prev[n_v-1] << "(start)\n"
                      << std::endl;

            path.clear();
            u = n_v-2;
            path.push_back(coord[u]);
            planning_success = true;
            while (true)
            {
                if (prev[u] == -1)    // current node has no previous node
                {
                    if (u != n_v-1)  // current node is not the start node, path planning failed
                        planning_success = false;
                    break;
                }
                else
                {
                    u = prev[u];
                    path.push_back(coord[u]);
                }
            }
        }

        is_planning = false;
    }

    bool collide(glm::vec2 const &agent_pos)
    {
        for (auto const &o : obstacle_pos)
        {
            if (glm::distance(glm::vec2(agent_pos.x, agent_pos.y), glm::vec2(o.x, o.y)) < this->agent_pos.z + o.z)
                return true;
        }
        return false;
    }
    bool collide(glm::vec2 const &p1, glm::vec2 const &p2)
    {
        auto slope = (p2.y-p1.y)/(p2.x-p1.x);
        auto intercept = p1.y - slope*p1.x;
        for (auto const &o : obstacle_pos)
        {
            if (std::isnan(std::abs(slope*o.x-o.y+intercept)/std::sqrt(slope*slope+1))
                || this->agent_pos.z + o.z > std::abs(slope*o.x-o.y+intercept)/std::sqrt(slope*slope+1))
                return true;
        }
        return false;
    }
};

scene::RoadmapScene::RoadmapScene()
        : BaseScene(),
          system_name("PRM Demo")
{
    pimpl = std::unique_ptr<impl>(new impl);
}

scene::RoadmapScene::~RoadmapScene()
{}

void scene::RoadmapScene::init(Scene &scene)
{
    if (pimpl->skybox == nullptr)
        pimpl->skybox = new SkyBox(ASSET_PATH "/texture/skybox/right.jpg",
            ASSET_PATH "/texture/skybox/left.jpg",
            ASSET_PATH "/texture/skybox/top.jpg",
            ASSET_PATH "/texture/skybox/bottom.jpg",
            ASSET_PATH "/texture/skybox/back.jpg",
            ASSET_PATH "/texture/skybox/front.jpg");
    if (pimpl->floor == nullptr)
        pimpl->floor = new item::Floor;
    if (pimpl->agent == nullptr)
        pimpl->agent = new item::Pillar;
}

void scene::RoadmapScene::restart(Scene &scene)
{
    static auto first_run = true;
    if (first_run)
    {
        resetCamera();
        first_run = false;
    }

    pimpl->n_sampling = 1000;
    pimpl->upper_bound = glm::vec2(10.f, 10.f);
    pimpl->lower_bound = glm::vec2(-10.f, -10.f);
    pimpl->agent_pos = glm::vec3(-9.f, -9.f, .5f);
    pimpl->goal_pos = glm::vec2(9.f, 9.f);
    pimpl->obstacle_pos.clear();
    pimpl->obstacle_pos.push_back(glm::vec3(0.f, 0.f, 2.f));

    pimpl->floor->scale(glm::vec3(pimpl->upper_bound.x-pimpl->lower_bound.x, 1.f, pimpl->upper_bound.y-pimpl->lower_bound.y));
    pimpl->floor->place(glm::vec3(pimpl->lower_bound.x, 0.f, pimpl->lower_bound.y));
    pimpl->floor->setGrid(pimpl->upper_bound.x-pimpl->lower_bound.x, pimpl->upper_bound.y-pimpl->lower_bound.y);
    scene.add(pimpl->floor);

    pimpl->agent->scale(glm::vec3(pimpl->agent_pos.z, 1.f, pimpl->agent_pos.z));
    pimpl->agent->place(glm::vec3(pimpl->agent_pos.x, 1.02f, pimpl->agent_pos.y));
    scene.add(pimpl->agent);

    for (auto const &o : pimpl->obstacle_pos)
    {
        scene.add(new item::Pillar(glm::vec3(o.x, 2.02f, o.y), o.z, 2.02f));
    }

    if (pimpl->worker_thread && pimpl->worker_thread->joinable())
        pimpl->worker_thread->join();
    pimpl->need_upload = true;
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
        pimpl->worker_thread = new std::thread(&impl::plan, pimpl.get());
        pimpl->need_upload = false;
    }
}

void scene::RoadmapScene::update(float dt)
{
    processInput(dt);
    if (pimpl->is_planning || pause || pimpl->path.empty()) return;

    float movement = .1f;
    auto pos = pimpl->agent->pos();
    auto tar = pimpl->path.back();
    auto dist = glm::distance(tar, glm::vec2(pos.x, pos.z));
    if (dist <= movement)
    {
        pimpl->agent->place(glm::vec3(tar.x, pos.y, tar.y));
        pimpl->path.pop_back();
    }
    else
    {
        dist = movement / dist;
        pimpl->agent->place(glm::vec3(pos.x+(tar.x-pos.x)*dist, pos.y, pos.z+(tar.y-pos.z)*dist));
    }
}

void scene::RoadmapScene::render()
{
    pimpl->skybox->render();

    renderInfo();
}

void scene::RoadmapScene::resetCamera()
{
    App::instance()->scene.character.reset(0.f, 25.f, 0.f, 90.f, 90.f);
    App::instance()->scene.character.setShootable(false);
    App::instance()->scene.character.setFloating(true);
}

void scene::RoadmapScene::renderInfo()
{
    App::instance()->text(system_name,
                          10, 10, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::LeftTop);
    auto h = App::instance()->frameHeight() - 25;
    auto w = App::instance()->frameWidth() - 10;
    if (pimpl->is_planning)
        App::instance()->text("Planning...",
                              w/2-10, h/2, .4f,
                              glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                              Anchor::LeftTop);
}

void scene::RoadmapScene::processInput(float dt)
{
    static auto last_key = GLFW_KEY_UNKNOWN;
    auto window = App::instance()->window();

    if (glfwGetKey(window, GLFW_KEY_M) == GLFW_PRESS)
        last_key = GLFW_KEY_M;
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
        last_key = GLFW_KEY_UNKNOWN;
    }
}
