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
    bool is_planning, planning_success, sampling_complete, sampling_upload;

    int n_sampling; int k;
    glm::vec2 upper_bound, lower_bound;
    glm::vec3 agent_pos;
    glm::vec2 goal_pos;
    std::vector<glm::vec2> path;
    std::vector<glm::vec3> obstacle_pos;
    std::vector<glm::vec2> sampling_coord;

    std::thread *worker_thread;

    SkyBox *skybox; Shader *mark_shader;
    item::Floor *floor;
    item::Pillar *agent;
    unsigned int vao[1], vbo[2];

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

    impl() : worker_thread(nullptr),
             skybox(nullptr), mark_shader(nullptr),
             floor(nullptr), agent(nullptr),
             vao{0}, vbo{0}
    {}
    ~impl()
    {
        if (worker_thread && worker_thread->joinable())
            worker_thread->join();
        delete skybox;
        delete mark_shader;
        delete floor;
        delete agent;
    }

    void plan()
    {
        // sampling uniformly
        for (auto i = 0; i < n_sampling; ++i)
        {
            glm::vec2 tar(rnd()*(upper_bound.x - lower_bound.x) + lower_bound.x,
                          rnd()*(upper_bound.y - lower_bound.y) + lower_bound.y);
            if (!collide(tar)) sampling_coord.push_back(tar);
        }

        // Dijkstra
        sampling_coord.push_back(glm::vec2(goal_pos.x, goal_pos.y));
        sampling_coord.push_back(glm::vec2(agent_pos.x, agent_pos.y));
        sampling_complete = true;
        auto n_v = static_cast<int>(sampling_coord.size());
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
                if (collide(sampling_coord[i], sampling_coord[j]))
                    len[i][j] = std::numeric_limits<float>::infinity();
                else
                    len[i][j] = glm::distance(sampling_coord[i], sampling_coord[j]);
                len[j][i] = len[i][j];
            }
        }


        int u = n_v-1; float min_dist;
        auto k = std::min(n_v, std::max(1, this->k));
        std::vector<float> knn_tmp;
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

            auto knn_shortest = std::numeric_limits<float>::infinity();
            knn_tmp.clear();
            for (auto v = 0; v < n_v; ++v)
            {
                if (visited[v] == false && len[u][v] <= knn_shortest
                    && dist[u] != std::numeric_limits<float>::infinity()
                    && dist[u] + len[u][v] < dist[v])
                {
                    dist[v] = dist[u] + len[u][v];
                    prev[v] = u;

                    if (k == 1)
                        knn_shortest = len[u][v];
                    else if (k < n_v)
                    {
                        knn_tmp.push_back(len[u][v]);
                        if (static_cast<int>(knn_tmp.size()) > k)
                        {
                            std::nth_element(knn_tmp.begin(), knn_tmp.begin()+k-1, knn_tmp.end());
                            knn_shortest = knn_tmp[k-1];
                        }
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
            auto dist = std::abs(slope*o.x-o.y+intercept)/std::sqrt(slope*slope+1);
            if (std::isnan(dist) || this->agent_pos.z + o.z > dist)
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
    if (pimpl->mark_shader == nullptr)
    {
        pimpl->mark_shader = new Shader(pimpl->MARK_VS,
#include "shader/glsl/simple_phong.fs"
                                       );

        glGenVertexArrays(1, pimpl->vao);
        glGenBuffers(2, pimpl->vbo);
    }

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
    glBindVertexArray(pimpl->vao[0]);
    glVertexAttribDivisor(1, 1);
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[0]);
    glBufferData(GL_ARRAY_BUFFER, vertex.size()*sizeof(float), vertex.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, (void *)(0));
    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[1]);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, (void *)(0));

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

    pimpl->agent->scale(glm::vec3(pimpl->agent_pos.z, .5f, pimpl->agent_pos.z));
    pimpl->agent->place(glm::vec3(pimpl->agent_pos.x, .52f, pimpl->agent_pos.y));
    pimpl->agent->color.ambient = glm::vec3(0.0588f, 0.2824f, .5f);
    pimpl->agent->color.diffuse = glm::vec3(0.1176f, 0.5647f, 1.f);
    pimpl->agent->color.specular = glm::vec3(0.1f);
    scene.add(pimpl->agent);

    for (auto const &o : pimpl->obstacle_pos)
    {
        scene.add(new item::Pillar(glm::vec3(o.x, 1.02f, o.y), o.z, 1.f));
    }

    if (pimpl->worker_thread && pimpl->worker_thread->joinable())
        pimpl->worker_thread->join();
    pimpl->need_upload = true;
    pimpl->sampling_upload = false;
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
        pimpl->worker_thread = new std::thread(&impl::plan, pimpl.get());
        pimpl->need_upload = false;

    }

    if (!pimpl->sampling_upload && pimpl->sampling_complete)
    {
        glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[1]);
        glBufferData(GL_ARRAY_BUFFER, pimpl->sampling_coord.size()*sizeof(float)*2, pimpl->sampling_coord.data(), GL_STATIC_DRAW);
        pimpl->sampling_upload = true;
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
