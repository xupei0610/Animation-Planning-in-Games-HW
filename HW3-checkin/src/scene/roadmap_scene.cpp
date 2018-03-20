#include <sstream>
#include <iomanip>
#include <cstring>
#include <thread>
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

using namespace px;

class scene::RoadmapScene::impl
{
public:
    bool need_upload;
    bool is_planning, planning_success, sampling_complete, sampling_upload, road_upload;

    int n_sampling; int k; int n_road; int solution_size;
    glm::vec2 upper_bound, lower_bound;
    glm::vec3 agent_pos;
    glm::vec2 goal_pos;
    std::vector<glm::vec2> path;
    std::vector<glm::vec3> obstacle_pos;
    std::vector<std::shared_ptr<Item> > obstacles;
    std::vector<glm::vec2> sampling_coord;
    std::vector<std::vector<float> > roadmap;

    std::thread *worker_thread;

    SkyBox *skybox; Shader *mark_shader, *edge_shader, *road_shader;
    item::Floor *floor;
    item::Pillar *agent;
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
            "   vec4 pos4 = view * vec4(vert.x+pos.x, 0.02f, vert.y+pos.y, 1.f);"
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
            "   gl_Position = proj * view * vec4(pos.x, 0.02f, pos.y, 1.f);;"
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
            "   gl_Position = proj * view * vec4(pos.x, 0.03f, pos.y, 1.f);;"
            "}";
    const char *ROAD_FS = "#version 330 core\n"
            "out vec4 color;"
            "void main() {"
            "   color = vec4(1.f, 0.f, 0.f, 1.f);"
            "}";

    impl() : worker_thread(nullptr),
             skybox(nullptr),
             mark_shader(nullptr), edge_shader(nullptr), road_shader(nullptr),
             floor(nullptr), agent(nullptr),
             vao{0}, vbo{0}
    {}
    ~impl()
    {
        if (worker_thread && worker_thread->joinable())
        {
            worker_thread->join();
            delete worker_thread;
        }
        delete skybox;
        delete mark_shader;
        delete floor;
        delete agent;

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

    void plan()
    {
        // sample uniformly
        while (static_cast<int>(sampling_coord.size()) < n_sampling-2)
        {
            glm::vec2 tar(rnd()*(upper_bound.x - lower_bound.x) + lower_bound.x,
                          rnd()*(upper_bound.y - lower_bound.y) + lower_bound.y);
            if ((tar.x != agent_pos.x || tar.y != agent_pos.y)
                && (tar.x != goal_pos.x || tar.y != goal_pos.y)
                && !collide(tar))
                sampling_coord.push_back(tar);
        }
        sampling_coord.emplace_back(goal_pos.x, goal_pos.y);
        sampling_coord.emplace_back(agent_pos.x, agent_pos.y);
        sampling_complete = true;
        auto n_v = static_cast<int>(sampling_coord.size());

        // build roadmap
        auto k = std::min(n_v, std::max(1, this->k));
        std::vector<std::vector<float> >(n_v, std::vector<float>(n_v, std::numeric_limits<float>::infinity())).swap(roadmap);
        std::vector<float> knn_tmp(n_v);
        for (auto i = 0; i < n_v; ++i)
        {
//            edge[i][i] = 0.f;
            for (auto j = i+1; j < n_v; ++j)
            {
                if (!collide(sampling_coord[i], sampling_coord[j]))
                {
                    roadmap[i][j] = glm::distance(sampling_coord[i], sampling_coord[j]);
                    roadmap[j][i] = roadmap[i][j];
                }
            }
            if (k < n_v)
            {
                std::memcpy(knn_tmp.data(), roadmap[i].data(), sizeof(float)*n_v);
                std::nth_element(knn_tmp.begin(), knn_tmp.begin()+k-1, knn_tmp.end());
                auto knn_shortest = knn_tmp[k-1];
                for (auto j = 0; j < n_v; ++j)
                {
                    if (roadmap[i][j] > knn_shortest)
                        roadmap[i][j] = std::numeric_limits<float>::infinity();
                }
            }
        }

        // Dijkstra
        dijkstra(roadmap);
        is_planning = false;

        std::cout << "Roadmap\n";
        for (auto i = 0; i < n_v; ++i)
        {
            for (auto j = 0; j < n_v; ++j)
            {
                std::cout << roadmap[i][j] << " ";
            }
            std::cout << "\n";
        }
        std::cout << "Path Solution:\n  ";
        for (auto p : path)
        {
            std::cout << "("<< p.x << ", " << p.y << "), ";
        }
        std::cout << std::endl;
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
        if (p1.x == p2.x)
        {
            for (auto const &o : obstacle_pos)
            {
                auto r = this->agent_pos.z + o.z;
                auto delta = r*r - (p1.x - o.x) * (p1.x - o.x);
                if (delta > 0)
                {
                    delta = std::sqrt(delta);
                    auto intersect = o.y + delta;
                    if ((intersect > p1.y && intersect < p2.y) || (intersect > p2.y && intersect < p1.y))
                        return true;
                    intersect = o.y - delta;
                    if ((intersect > p1.y && intersect < p2.y) || (intersect > p2.y && intersect < p1.y))
                        return true;
                }
            }
        }
        else if (p1.y == p2.y)
        {
            for (auto const &o : obstacle_pos)
            {
                auto r = this->agent_pos.z + o.z;
                auto delta = r*r - (p1.y - o.y) * (p1.y - o.y);
                if (delta > 0)
                {
                    delta = std::sqrt(delta);
                    auto intersect = o.x + delta;
                    if ((intersect > p1.x && intersect < p2.x) || (intersect > p2.x && intersect < p1.x))
                        return true;
                    intersect = o.x - delta;
                    if ((intersect > p1.x && intersect < p2.x) || (intersect > p2.x && intersect < p1.x))
                        return true;
                }
            }
        }
        else
        {
            auto k = (p2.y-p1.y)/(p2.x-p1.x);
            auto b = p1.y - k*p1.x;
            auto den = std::sqrt(k*k+1);
            for (auto const &o : obstacle_pos)
            {
                auto r = this->agent_pos.z + o.z;
                auto d = std::abs(k*o.x-o.y+b)/den;
                if (d < r)
                {
                    auto d2 = glm::distance(glm::vec2(o.x, o.y), p2);
                    if (glm::distance(glm::vec2(o.x, o.y), p1) < r || d2 < r || d2*d2 - d*d < glm::distance2(p1, p2))
                        return true;
                }
            }
        }
        return false;
    }

    void dijkstra(std::vector<std::vector<float> > const &roadmap)
    {
        auto n_v = static_cast<int>(roadmap.size());
        std::vector<bool> visited(n_v, false);
        std::vector<int>  prev(n_v, -1);
        std::vector<float> dist(n_v, std::numeric_limits<float>::infinity());
        dist.back() = 0.f;

        auto u = n_v-1;
        for (auto i = 0; i < n_v-1; ++i)
        {
            auto min_dist = std::numeric_limits<float>::max();
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
                if (roadmap[u][v] == std::numeric_limits<float>::infinity())
                    continue;
                auto alt = dist[u] + roadmap[u][v];
                if (visited[v] == false && alt < dist[v])
                {
                    dist[v] = alt;
                    prev[v] = u;
                }
            }
        }
        // build path
        path.clear();
        u = n_v-2;
        path.push_back(sampling_coord[u]);
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
                path.push_back(sampling_coord[u]);
            }
        }
        std::cout << "Dijkstra Solution:\n  Node\tPrevious\n";
        for (auto i = 0; i < n_v-2; ++i)
            std::cout << std::setw(6) << i << "\t" << std::setw(6) << prev[i] << "\n";
        std::cout << std::setw(6) << n_v-2 << "\t" << std::setw(6) << prev[n_v-2] << "(goal)\n"
                  << std::setw(6) << n_v-1 << "\t" << std::setw(6) << prev[n_v-1] << "(start)\n"
                  << std::endl;
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

    pimpl->genGLObjs();

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
    if (pimpl->mark_shader == nullptr)
    {
        pimpl->mark_shader = new Shader(pimpl->MARK_VS,
#include "shader/glsl/simple_phong.fs"
                                       );
    }
    if (pimpl->edge_shader == nullptr)
        pimpl->edge_shader = new Shader(pimpl->EDGE_VS, pimpl->EDGE_FS);
    if (pimpl->road_shader == nullptr)
        pimpl->road_shader = new Shader(pimpl->ROAD_VS, pimpl->ROAD_FS);

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
        resetCamera();
        first_run = false;
    }

    pimpl->n_sampling = 10;
    pimpl->k = pimpl->n_sampling;
    pimpl->sampling_coord.clear();
    pimpl->obstacle_pos.clear();
    pimpl->upper_bound = glm::vec2(10.f, 10.f);
    pimpl->lower_bound = glm::vec2(-10.f, -10.f);
    pimpl->goal_pos = glm::vec2(9.f, 9.f);
    pimpl->agent_pos = glm::vec3(-9.f, -9.f, .5f);
    pimpl->obstacle_pos.emplace_back(0.f, 0.f, 2.f);

    pimpl->floor->scale(glm::vec3(pimpl->upper_bound.x-pimpl->lower_bound.x, 1.f, pimpl->upper_bound.y-pimpl->lower_bound.y));
    pimpl->floor->place(glm::vec3(pimpl->lower_bound.x, 0.f, pimpl->lower_bound.y));
    pimpl->floor->setGrid(pimpl->upper_bound.x-pimpl->lower_bound.x, pimpl->upper_bound.y-pimpl->lower_bound.y);
    scene.add(pimpl->floor);

    pimpl->agent->scale(glm::vec3(pimpl->agent_pos.z, .5f, pimpl->agent_pos.z));
    pimpl->agent->place(glm::vec3(pimpl->agent_pos.x, .52f, pimpl->agent_pos.y));
    pimpl->agent->color.ambient = glm::vec3(0.0588f, 0.2824f, .5f);
    pimpl->agent->color.diffuse = glm::vec3(0.1176f, 0.5647f, 1.f);
    pimpl->agent->color.specular = glm::vec3(0.1f);
    scene.add(pimpl->agent);
    for (auto const &o : pimpl->obstacle_pos)
    {
        pimpl->obstacles.emplace_back(new item::Pillar(glm::vec3(o.x, .52f, o.y), o.z, 1.f));
        scene.add(pimpl->obstacles.back().get());
    }

    if (pimpl->worker_thread && pimpl->worker_thread->joinable())
        pimpl->worker_thread->join();
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
        {
            pimpl->worker_thread->join();
            delete pimpl->worker_thread;
        }
        pimpl->worker_thread = new std::thread(&impl::plan, pimpl.get());
        pimpl->need_upload = false;

    }

    if (!pimpl->sampling_upload && pimpl->sampling_complete)
    {
        glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[1]);
        glBufferData(GL_ARRAY_BUFFER, pimpl->sampling_coord.size()*sizeof(float)*2, pimpl->sampling_coord.data(), GL_STATIC_DRAW);
        pimpl->sampling_upload = true;
    }

    if (!pimpl->road_upload && pimpl->sampling_upload && !pimpl->is_planning)
    {
        std::vector<int> road;
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

        if (pimpl->planning_success)
        {
            glBindVertexArray(pimpl->vao[2]);
            glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[2]);
            glBufferData(GL_ARRAY_BUFFER, pimpl->path.size()*sizeof(float)*2, pimpl->path.data(), GL_STATIC_DRAW);
            pimpl->solution_size = static_cast<int>(pimpl->path.size());
        }
        pimpl->n_road =  static_cast<int>(road.size());
        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

        pimpl->road_upload = true;
    }

}

void scene::RoadmapScene::update(float dt)
{
    processInput(dt);
    if (pause || pimpl->is_planning || !pimpl->planning_success || pimpl->path.empty()) return;

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
    if (pimpl->sampling_upload)
    {
        pimpl->mark_shader->activate();
        glBindVertexArray(pimpl->vao[0]);
        glDrawArraysInstanced(GL_TRIANGLE_FAN, 0, 34, pimpl->sampling_coord.size());
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
    App::instance()->scene.character.reset(-14.f, 20.f, 0.f, 90.f, 60.f);
    App::instance()->scene.character.setShootable(false);
    App::instance()->scene.character.setFloating(true);
}

void scene::RoadmapScene::renderInfo()
{
    App::instance()->text(system_name,
                          10, 10, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::LeftTop);
    App::instance()->text("Sampling: " + std::to_string(pimpl->n_sampling),
                          10, 30, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::LeftTop);
    App::instance()->text("Neighbor Check: " + std::to_string(pimpl->k),
                          10, 50, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::LeftTop);
    auto h = App::instance()->frameHeight();
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
