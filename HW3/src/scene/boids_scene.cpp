#include <sstream>
#include <iomanip>
#include <cstring>
#include <thread>
#include <list>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/transform.hpp>

#include "scene/boids_scene.hpp"
#include "shader/base_shader.hpp"
#include "item/pillar.hpp"
#include "item/floor.hpp"
#include "item/ball.hpp"
#include "shader/skybox.hpp"
#include "scene.hpp"
#include "app.hpp"
#include "global.hpp"
#include "util/random.hpp"
#include "util/kd_tree.hpp"

using namespace px;

class scene::BoidsScene::impl
{
public:
    bool need_upload;

    const scene::BoidsScene::SceneParameter &scene;
    const scene::BoidsScene::BoidParameter &param;
    struct
    {
        glm::vec3 pos;
        float radius;
        glm::vec3 vel;
    } cam;
    struct Agent
    {
        glm::vec3 pos;
        glm::vec3 vel;
        std::int8_t alive;
        inline float &operator[](std::size_t k)
        {
            return pos[k];
        }
    };
    struct Obstacle
    {
        glm::vec2 pos;
        float radius;
        inline float &operator[](std::size_t k)
        {
            return pos[k];
        }
    };
    struct DistHelper
    {
        inline float operator()(glm::vec2 const &p, glm::vec2 const &o) const
        {
            return glm::distance2(p, o);
        }
        inline float operator()(impl::Agent const &a, impl::Agent const &o) const
        {
            return glm::distance2(a.pos, o.pos);
        }
        inline float operator()(glm::vec2 const &p, impl::Obstacle const &o) const
        {
            return glm::distance2(p, o.pos);
        }
        inline float operator()(glm::vec3 const &p, impl::Agent const &a) const
        {
            return glm::distance2(p, a.pos);
        }
    };
    DistHelper dist_fn;
    KdTree<impl::Obstacle, float, DistHelper> ob_tree;
    KdTree<impl::Agent, float, DistHelper> agent_tree;

    std::vector<Agent> agents;
    std::vector<Agent> predators;
    std::vector<Obstacle> obstacles;
    std::vector<float> hungry;
    glm::vec3 cam_last_pos;
    float max_obstacle_radius;

    unsigned int vao[2], vbo[2];
    std::vector<std::shared_ptr<Item> > obstacle_objs;
    std::unique_ptr<Shader> agent_body_shader, agent_border_shader;
    std::unique_ptr<Shader> predator_body_shader, predator_border_shader;
    std::shared_ptr<item::Ball> goal_marker;
    std::shared_ptr<item::Floor> floor;
    std::unique_ptr<SkyBox> skybox;

    impl(const scene::BoidsScene::SceneParameter &scene,
         const scene::BoidsScene::BoidParameter &param)
            : scene(scene), param(param),
              ob_tree(dist_fn, 2), agent_tree(dist_fn, 3),
              vao{0}, vbo{0},
              agent_body_shader(nullptr), agent_border_shader(nullptr),
              predator_body_shader(nullptr), predator_border_shader(nullptr),
              goal_marker(nullptr), floor(nullptr), skybox(nullptr)
    {}
    ~impl()
    {
        clearGLObjs();
    }

    void clearGLObjs()
    {
        glDeleteVertexArrays(2, vao);
        glDeleteBuffers(2, vbo);
    }
    void genGLObjs()
    {
        clearGLObjs();
        glGenVertexArrays(2, vao);
        glGenBuffers(2, vbo);
    }

    const char *AGENT_VS = "#version 420 core\n"
            "layout (location = 0) in vec3 pos;"
            "layout (location = 1) in vec3 vel;"
            "layout (location = 2) in float alive;"
            ""
            "out VS_OUT"
            "{"
            "   vec3 dir;"
            "   vec3 pos;"
            "   bool alive;"
            "} primitive;"
            ""
            "void main()"
            "{"
            "   float n = length(vel);"
            "   if (vel.x == 0.f && vel.y == 0.f && vel.z == 0.f)"
            "       primitive.dir = vec3(0.f, 0.f, 1.f);"
            "   else"
            "       primitive.dir = normalize(vel);"
            "   primitive.dir.z = -primitive.dir.z;"
            "   primitive.pos.x = pos.x;"
            "   primitive.pos.y = pos.y;"
            "   primitive.pos.z = -pos.z;"
            "   primitive.alive = alive == 1.f;"
            "}";
    const char *AGENT_GS2 = "#version 430 core\n"
            "layout (points) in;"
            "layout (line_strip, max_vertices = 9) out;"
            ""
            "layout (std140, binding = 0) uniform GlobalAttributes"
            "{"
            "   mat4 view;"
            "   mat4 proj;"
            "   vec3 cam_pos;"
            "};"
            "uniform float size;"
            ""
            "in VS_OUT"
            "{"
            "   vec3 dir;"
            "   vec3 pos;"
            "   bool alive;"
            "} primitive[];"
            ""
            "void main()"
            "{"
            "   if (primitive[0].alive)"
            "   {"
            "   float theta = atan(primitive[0].dir.x, primitive[0].dir.z);"
            "   float phi = -atan(primitive[0].dir.y, abs(primitive[0].dir.z));"
            ""
            "   mat4 model = mat4(cos(theta), 0, -sin(theta), 0," // first column
            "                     0,          1,           0, 0,"
            "                     sin(theta), 0,   cos(theta), 0,"
            "                     primitive[0].pos.x, primitive[0].pos.y, primitive[0].pos.z, 1)"
            "              * mat4(1, 0, 0, 0,"
            "                     0,  cos(phi), sin(phi), 0,"
            "                     0, -sin(phi), cos(phi), 0,"
            "                     0, 0, 0, 1)"
            "              * mat4(size,   0,   0, 0,"
            "                       0, size,   0, 0,"
            "                       0,   0, size, 0,"
            "                       0,   0,   0, 1);"
            ""
            "   mat4 MVP = proj * view * model;"
            ""
            "   gl_Position = MVP * vec4(0.0f, 0.f, 1.f, 1.f);" // head
            "   EmitVertex();"
            "   gl_Position = MVP * vec4(-1.f, 0.f, -1.f, 1.f);" // left bottom
            "   EmitVertex();"
            "   gl_Position = MVP * vec4(1.f, 0.f, -1.f, 1.f);" // right bottom
            "   EmitVertex();"
            "   gl_Position = MVP * vec4(0.0f, 0.f, 1.f, 1.f);" // head
            "   EmitVertex();"
            "   gl_Position = MVP * vec4(0.0f, 0.f, -1.f, 1.f);" // bottom
            "   EmitVertex();"
            "   EndPrimitive();"
            ""
            "   gl_Position = MVP * vec4(0.f, -.001f, -1.f, 1.f);" // bottom
            "   EmitVertex();"
            "   gl_Position = MVP * vec4(0.f, -.001f, 1.f, 1.f);" // head
            "   EmitVertex();"
            "   gl_Position = MVP * vec4(0.f, -1.f, -1.f, 1.f);"   // button
            "   EmitVertex();"
            "   gl_Position = MVP * vec4(0.f, -.001f, -1f, 1.f);" // bottom
            "   EmitVertex();"
            "   EndPrimitive();"
            "   }"
            ""
            "}";
    const char *AGENT_GS = "#version 430 core\n"
            "layout (points) in;"
            "layout (triangle_strip, max_vertices = 7) out;"
            ""
            "layout (std140, binding = 0) uniform GlobalAttributes"
            "{"
            "   mat4 view;"
            "   mat4 proj;"
            "   vec3 cam_pos;"
            "};"
            ""
            "in VS_OUT"
            "{"
            "   vec3 dir;"
            "   vec3 pos;"
            "   bool alive;"
            "} primitive[];"
            ""
            "uniform float size;"
            ""
            "void main()"
            "{"
            "   if (primitive[0].alive)"
            "   {"
            "   float theta = atan(primitive[0].dir.x, primitive[0].dir.z);"
            "   float phi = -atan(primitive[0].dir.y, abs(primitive[0].dir.z));"
            ""
            "   mat4 model = mat4(cos(theta), 0, -sin(theta), 0," // first column
            "                     0,          1,           0, 0,"
            "                     sin(theta), 0,  cos(theta), 0,"
            "                     primitive[0].pos.x, primitive[0].pos.y, primitive[0].pos.z, 1)"
            "              * mat4(1, 0, 0, 0,"
            "                     0,  cos(phi), sin(phi), 0,"
            "                     0, -sin(phi), cos(phi), 0,"
            "                     0, 0, 0, 1)"
            "              * mat4(size,   0,   0, 0,"
            "                       0, size,   0, 0,"
            "                       0,   0, size, 0,"
            "                       0,   0,   0, 1);"
            ""
            "   mat4 MVP = proj * view * model;"
            ""
            "   gl_Position = MVP * vec4(-1.f, 0, -1.f, 1.f);" // left bottom
            "   EmitVertex();"
            "   gl_Position = MVP * vec4(0.0f, 0,  1.f, 1.f);" // head
            "   EmitVertex();"
            "   gl_Position = MVP * vec4(1.f, 0,  -1.f, 1.f);" // right bottom
            "   EmitVertex();"
            "   EndPrimitive();"
            ""
            "   gl_Position = MVP * vec4(0.f, -.001f, -1.f, 1.f);" // bottom
            "   EmitVertex();"
            "   gl_Position = MVP * vec4(0.f, -.001f,  1.f, 1.f);" // head
            "   EmitVertex();"
            "   gl_Position = MVP * vec4(0.f, -1.f,   -1.f, 1.f);" // button
            "   EmitVertex();"
            "   EndPrimitive();"
            "   }"
            "}";
    const char *AGENT_FS = "#version 330 core\n"
            "out vec4 color;"
            ""
            "void main()"
            "{"
            "   color = vec4(1.f, 1.f, 1.f, 1.f);"
            "}";
    const char *AGENT_FS2 = "#version 330 core\n"
            "out vec4 color;"
            ""
            "void main()"
            "{"
            "   color = vec4(0.f, 0.f, 0.f, 1.f);"
            "}";
    const char *AGENT_FS3 = "#version 330 core\n"
            "out vec4 color;"
            ""
            "void main()"
            "{"
            "   color = vec4(1.f, 0.f, 0.f, 1.f);"
            "}";

    void drawAgents()
    {
        agent_body_shader->activate();
        glBindVertexArray(vao[0]);
        glDrawArrays(GL_POINTS, 0, agents.size());
        agent_border_shader->activate();
        glBindVertexArray(vao[0]);
        glDrawArrays(GL_POINTS, 0, agents.size());
        predator_body_shader->activate();
        glBindVertexArray(vao[1]);
        glDrawArrays(GL_POINTS, 0, predators.size());
        predator_border_shader->activate();
        glBindVertexArray(vao[1]);
        glDrawArrays(GL_POINTS, 0, predators.size());

        agent_body_shader->activate(false);
        glBindVertexArray(0);
    }

    glm::vec3 collide(Agent &a, const float agent_r, const float visual_r)
    {
        glm::vec3 s(0.f);
        {   // avoid camera
            auto dp = cam.pos - a.pos;
            auto d2 = glm::length2(dp);
            auto r = cam.radius + agent_r;
            auto r2 = r * r;

            if (d2 < r2) // collide already;
            { }
            else if ((std::sqrt(d2) - cam.radius) < visual_r)
            {
                // solve || (pos_b + v_b * t) - (pos_a + v_a *t) || = r_a + r_b
                // to get the time-to-collision
                auto v = a.vel - cam.vel;
                auto a0 = glm::length2(v);
                auto b = glm::dot(dp, v);
                auto c = glm::length2(dp) - r2;
                auto discriminant = b*b - a0*c;
                if (discriminant > 0.f)
                {
                    auto t = (b - std::sqrt(discriminant)) / a0;
                    if (t >= 0.f)
                    {
                        auto dir = v * t - dp;
                        if (dir.x != 0.f || dir.y != 0.f)
                        {
                            dir /= glm::length(dir);

                            if (dir.x * v.y == dir.y * v.x && dir.x * v.z == dir.z * v.x) // if avoidance force could make the agent stop
                                dir += (rnd_np()/a0) * glm::vec3(-v.y, 0.f, v.x); // move along the tangent line randomly

                            auto mag = 1.f / (t*t);

                            s = mag * dir;
                        }
                    }
                }
            }
        }
        {   // avoid boundary
#define BOUNDARY_COLLISION_CHECK(AXIS)                                      \
    if (a.pos.AXIS >= scene.upper_bound.AXIS) {                             \
        a.alive = 0.f; return s;                                            \
    } else if (a.pos.AXIS <= scene.lower_bound.AXIS) {                      \
        a.alive = 0.f; return s;                                            \
    } else if (a.vel.AXIS != 0.f ) {                                        \
        auto gap = (a.vel.AXIS > 0.f ? scene.upper_bound.AXIS : scene.lower_bound.AXIS) \
                - a.pos.AXIS;                                               \
        if (std::abs(gap) < visual_r) {                                     \
            /*auto t = gap / a.vel.AXIS;*/                                  \
            /*s.AXIS += (a.vel.AXIS > 0.f ? -1.f : 1.f)/(t + 0.1f);*/       \
            s.AXIS -= 1.f/gap;                                              \
            if (s.x+s.y+s.z-s.AXIS == 0.f) {                                \
                s += rnd_np()*1.f/visual_r;                                 \
            }                                                               \
        }                                                                   \
    }
            BOUNDARY_COLLISION_CHECK(x)
            BOUNDARY_COLLISION_CHECK(y)
            BOUNDARY_COLLISION_CHECK(z)
        }
        {   // avoid pillars
            glm::vec2 a_pos(a.pos.x, a.pos.z); // pillar is infinite high
            glm::vec2 a_vel(a.vel.x, a.vel.z); // such that only consider horizontal velocity
            auto nn = ob_tree.around<glm::vec2>(
                    a_pos, obstacles, (visual_r+max_obstacle_radius)*(visual_r+max_obstacle_radius));
            for (const auto &i : nn)
            {   const auto &o = obstacles[i.first]; auto &d2 = i.second;
//            for (const auto &o : obstacles)
//            {
                auto dp = o.pos - a_pos;
//                auto d2 = dp.x * dp.x + dp.y * dp.y;
                auto r = o.radius + agent_r;
                auto r2 = r*r;
                if (d2 < r2)// collided already
                { a.alive = 0; return s; }
                else if ((std::sqrt(d2) - o.radius) < visual_r)
                {
                    // solve || (pos_b + v_b * t) - (pos_a + v_a *t) || = r_a + r_b
                    // to get the time-to-collision
                    auto tmp1 = a_vel.x*a_vel.x + a_vel.y*a_vel.y;
                    auto tmp2 = dp.x*a_vel.x + dp.y*a_vel.y;
                    auto discriminant = tmp2*tmp2 - tmp1*(d2 - r2);
                    if (discriminant < 0) continue;

                    auto t = (tmp2 - std::sqrt(discriminant))/tmp1;
//                            t = (std::sqrt(d2) - r)/std::sqrt(tmp1);
                    if (t >= 0.f)
                    {
                        // avoidance direction (pos_a + v_a * t) - (pos_b + v_b * t)
                        auto dir = a_vel * t - dp;
                        if (dir.x == 0.f && dir.y == 0.f) continue;
                        dir /= glm::length(dir);
                        if (dir.x * a_vel.y == dir.y * a_vel.x) // if avoidance force could make the agent stop
                            dir += rnd_np()/tmp1 * glm::vec2(-a_vel.y, a_vel.x); // move along the tangent line randomly

                        // avoidance force
                        auto mag = 1.f / (t*t+0.01f);

                        s.x += mag * dir.x;
                        s.z += mag * dir.y;
                    }
                }
            }
        }
        return s;
    }

    std::tuple<glm::vec3, glm::vec3, glm::vec3>
    boids(const Agent &a, float visual_r2, float visual_ang,
          const std::vector<Agent> &agents)
    {
        glm::vec3 f(0.f), v(0.f), p(0.f);
        auto count  = 0;
        auto a_vel_normalized = a.vel.x == 0 && a.vel.y == 0 && a.vel.z == 0 ?
                                glm::vec3(0.f, 0.f, 1.f) : glm::normalize(a.vel);
        auto ignore_ang = - visual_ang;
        for (const auto &b: agents)
        {
            if (!b.alive || a.pos == b.pos) continue;
            auto dp = a.pos - b.pos;
            auto d2 = glm::length2(dp);
            if (d2 < visual_r2)
            {
                if (glm::dot(a_vel_normalized, dp)/std::sqrt(d2) > ignore_ang )
                    continue;

                ++count;
                // separate
                f += dp / d2;
                // align
                v += b.vel;
                // cohesion
                p -= dp;
            }
        }
        if (count > 0)
        {
            f /= count; v /= count; p /= count;
            // v -= a.vel;
        }
        return std::make_tuple(f, v, p);
    }
    std::tuple<glm::vec3, glm::vec3, glm::vec3>
    boids(const Agent &a, float visual_r2, float visual_ang,
          const std::vector<Agent> &agents, const decltype(agent_tree) &kd_tree)
    {
        glm::vec3 f(0.f), v(0.f), p(0.f);
        auto count  = 0;
        auto a_vel_normalized = a.vel.x == 0 && a.vel.y == 0 && a.vel.z == 0 ?
                                glm::vec3(0.f, 0.f, 1.f) : glm::normalize(a.vel);
        auto ignore_ang = - visual_ang;
        auto nn = kd_tree.around(a, agents, visual_r2);
        for (const auto &i : nn)
        {
            const auto &b = agents[i.first];
//        for (const auto &b: agents)
//        {
            if (!b.alive || a.pos == b.pos) continue;
            auto dp = a.pos - b.pos;
//            auto d2 = glm::length2(dp);
//            if (d2 < visual_r2)
//            {
                if (glm::dot(a_vel_normalized, dp)/std::sqrt(i.second) > ignore_ang )
                    continue;

                ++count;
                // separate
                f += dp / i.second;
                // align
                v += b.vel;
                // cohesion
                p -= dp;
//            }
        }
        if (count > 0)
        {
            f /= count; v /= count; p /= count;
            // v -= a.vel;
        }
        return std::make_tuple(f, v, p);
    }

    glm::vec3 flee(const Agent &a, float visual_r, float visual_ang,
                   const std::vector<Agent> &agents, float agent_r)
    {
        glm::vec3 e(0.f);
        visual_ang *= - glm::length(a.vel);
        visual_r += agent_r;
        for (const auto &b: agents)
        {
            if (!b.alive || a.pos == b.pos) continue;
            auto dp = a.pos - b.pos;
            auto d = glm::length(dp);
            if (d < visual_r)
            {
                dp /= d;
                if (glm::dot(a.vel, dp)> visual_ang)
                    continue;
                e += dp;
            }
        }
        return e;
    }

    glm::vec3 chase(const Agent &a, float agent_r, float visual_r, float visual_ang,
                    std::vector<Agent> &agents)
    {
        glm::vec3 c(0.f);
        visual_ang *= glm::length(a.vel);
        visual_r *= visual_r;
        for (auto &b: agents)
        {
            if (!b.alive) continue;
            auto dp = b.pos - a.pos;
            auto d2 = glm::length2(dp);
            if (d2 < visual_r)
            {
                c = dp - (a.vel - b.vel);
                if (c.x != 0.f || c.y != 0.f || c.z != 0.f)
                    c = glm::normalize(c);
                visual_r = d2;
            }
        }
        return c;
    }
    glm::vec3 chase(const Agent &a, float agent_r, float visual_r, float visual_ang,
                    std::vector<Agent> &agents, const decltype(agent_tree) &kd_tree)
    {
        glm::vec3 c(0.f);
        visual_ang *= glm::length(a.vel);
        visual_r *= visual_r;
        auto nn = kd_tree.around(a, agents, visual_r);
        for (const auto &i: nn)
        {
            const auto &b = agents[i.first];
//        for (auto &b: agents)
//        {
            if (!b.alive) continue;
//            auto d2 = glm::length2(dp);
            if (i.second < visual_r)
            {
                auto dp = b.pos - a.pos;
                c = dp - (a.vel - b.vel);
                if (c.x != 0.f || c.y != 0.f || c.z != 0.f)
                    c = glm::normalize(c);
                visual_r = i.second;
            }
        }
        return c;
    }

    void update(float dt)
    {
//        agent_tree.build(agents);

        auto visual_r2 = param.visual_r * param.visual_r;
        auto visual_r2_predator = param.visual_r_predator * param.visual_r_predator;
        auto predator_size2 = param.predator_size * param.predator_size;

        auto v_max = param.v_max * dt;
        auto a_max = param.a_max * dt;
        auto v_max_predator = param.v_max_predator * dt;
        auto a_max_predator = param.a_max_predator * dt;

        std::vector<glm::vec3> acc(agents.size(), glm::vec3(0.f));
        std::vector<glm::vec3> push(agents.size(), glm::vec3(0.f));
        std::vector<glm::vec3> acc_pred(predators.size(), glm::vec3(0.f));
        std::vector<glm::vec3> push_pred(agents.size(), glm::vec3(0.f));

        auto tot = static_cast<int>(agents.size());
        auto tot_pred = static_cast<int>(predators.size());

        cam.pos = App::instance()->scene.cam.pos();
        cam.radius = App::instance()->scene.character.characterHalfSize()*30.f;
        cam.vel = cam.pos - cam_last_pos;

#pragma omp parallel num_threads(6)
        {
#pragma omp for
            for (auto i = 0; i < tot; ++i)
            {
                auto &a = agents[i];
                if (!a.alive) continue;

                // collision avoidance
                auto s = collide(a, param.agent_size, param.visual_r);
                if (!a.alive) continue;
                // boids rules
                glm::vec3 f, v, p;
                std::tie(f, v, p) = boids(a, visual_r2, param.visual_ang, agents);
//                std::tie(f, v, p) = boids(a, visual_r2, param.visual_ang, agents, agent_tree);
                // flee from predators
                push[i] = param.e_alpha * flee(a, param.visual_r, param.visual_ang, predators, param.predator_size);
                // seek goal
                if (param.has_goal)
                    acc[i] = glm::normalize(param.goal - a.pos) * param.g_alpha;

                acc[i] += param.f_alpha * f + param.v_alpha * v + param.p_alpha * p + param.s_alpha * s;
                auto mag = glm::length(acc[i]);
                if (mag > a_max) acc[i] *= param.a_max/mag;
            }
#pragma omp for
            for (auto i = 0; i < tot_pred; ++i)
            {
                auto &a = predators[i];
                if (!a.alive) continue;
                // collision avoidance
                auto s = collide(a, param.predator_size, param.visual_r_predator);
                if (!a.alive) continue;
                // boids rules
                glm::vec3 f, v/*, p*/;
                std::tie(f, v, std::ignore) =
                        boids(a, visual_r2_predator, param.visual_ang_predator, predators);
                // chase prey
                if (hungry[i] < 0.f)
                push_pred[i] = param.c_alpha *
                               chase(a, param.predator_size, param.visual_r_predator, param.visual_ang_predator, agents);
//                               chase(a, param.predator_size, param.visual_r_predator, param.visual_ang_predator, agents, agent_tree);

                acc_pred[i] = param.f_alpha * f + param.v_alpha * v + /*param.p_alpha * p*/ + param.s_alpha * s;
                auto mag = glm::length(acc_pred[i]);
                if (mag > a_max_predator) acc_pred[i] *= param.a_max_predator/mag;
            }
#pragma omp for
            for (auto i = 0; i < tot; ++i)
            {
                auto &a = agents[i];
                if (!a.alive) continue;

                a.vel += acc[i];
                auto mag = glm::length(a.vel);
                if (mag > v_max) a.vel *= param.v_max/mag;

                a.vel += push[i] * dt;
                a.pos += a.vel;

                mag = glm::length(a.vel);
                if (mag > v_max) a.vel *= param.v_max/mag;
            }
#pragma omp for
            for (auto i = 0; i < tot_pred; ++i)
            {
                auto &a = predators[i];
                if (!a.alive) continue;

                a.vel += acc_pred[i];
                auto mag = glm::length(a.vel);
                if (mag > v_max_predator) a.vel *= param.v_max_predator/mag;

                if (hungry[i] < 0.f)
                {
                    a.vel += push_pred[i] * dt;
                    auto m = glm::length(a.vel);
                    auto a_vel_normalized = a.vel / m;
                    m += param.predator_size;

                    // kill preds
//                    auto nn = agent_tree.around(a, agents, m*m);
//                    for (const auto &j : nn)
//                    {
//                        auto &b = agents[j.first];
                    for (auto &b : agents)
                    {
                        if (!b.alive) continue;
                        auto dp = b.pos-a.pos;
//                        const auto &d2 = j.second;
                        auto d2 = glm::length2(dp);
                        if (d2 > predator_size2)
                        {
                            auto proj = glm::dot(dp, a_vel_normalized);
                            if (proj > 0.f && proj < m && d2 - proj*proj < predator_size2)
                            {
                                b.alive = false;
                                a.vel = a_vel_normalized * (proj-param.predator_size);
                                hungry[i] = param.satiation;
                                break;
                            }
                        }
                        else
                        {
                            b.alive = false;
                            a.vel -= push_pred[i] * dt;
                            hungry[i] = param.satiation;
                            break;
                        }
                    }
                    a.pos += a.vel;
                    mag = glm::length(a.vel);
                    if (mag > v_max) a.vel *= param.v_max/mag;
                }
                else
                    a.pos += a.vel;

                hungry[i] -= dt;
            }
        }
    }
};

scene::BoidsScene::BoidsScene()
        : BaseScene(), pause(false),
          scene_param { 5000, 5, 10,
                        {30.f, 30.f, 30.f},
                        {-30.f, 0.f, -30.f}
          },
          boids{ 2.5f,  std::cos(static_cast<float>(M_PI)*.75f), .1f, .15f, .9f,
                 10.f, std::cos(static_cast<float>(M_PI)*.25f), .5f, .3f, .9f,
                 1.5f, // separation weight
                 1.f,  // alignment weight
                 .4f,  // cohesion weight
                 150.f,  // obstacle avoidance weight
                 10.f,   // escape force weight
                 2.f,   // chase force weight
                 .05f, // goal attraction weight
                 5.f,
                 false, {0.f, 0.f, 0.f}
        }
{
    pimpl = std::make_unique<impl>(scene_param, boids);
}

scene::BoidsScene::~BoidsScene()
{}

void scene::BoidsScene::init(Scene &scene)
{
    pimpl->genGLObjs();

    if (pimpl->agent_body_shader == nullptr)
        pimpl->agent_body_shader = std::make_unique<Shader>(pimpl->AGENT_VS, pimpl->AGENT_FS, pimpl->AGENT_GS);
    if (pimpl->agent_border_shader == nullptr)
        pimpl->agent_border_shader = std::make_unique<Shader>(pimpl->AGENT_VS, pimpl->AGENT_FS2, pimpl->AGENT_GS2);
    if (pimpl->predator_body_shader == nullptr)
        pimpl->predator_body_shader = std::make_unique<Shader>(pimpl->AGENT_VS, pimpl->AGENT_FS3, pimpl->AGENT_GS);
    if (pimpl->predator_border_shader == nullptr)
        pimpl->predator_border_shader = std::make_unique<Shader>(pimpl->AGENT_VS, pimpl->AGENT_FS2, pimpl->AGENT_GS2);

    if (pimpl->floor == nullptr)
        pimpl->floor = std::make_shared<item::Floor>();
    if (pimpl->skybox == nullptr)
        pimpl->skybox = std::make_unique<SkyBox>(ASSET_PATH "/texture/skybox/right.jpg",
                                                 ASSET_PATH "/texture/skybox/left.jpg",
                                                 ASSET_PATH "/texture/skybox/top.jpg",
                                                 ASSET_PATH "/texture/skybox/bottom.jpg",
                                                 ASSET_PATH "/texture/skybox/back.jpg",
                                                 ASSET_PATH "/texture/skybox/front.jpg");
    if (pimpl->goal_marker == nullptr)
        pimpl->goal_marker = std::make_shared<item::Ball>();

    pimpl->agent_body_shader->activate();
    pimpl->agent_body_shader->bind("GlobalAttributes", 0);
    pimpl->agent_body_shader->set("size", boids.agent_size);
    pimpl->agent_border_shader->activate();
    pimpl->agent_border_shader->bind("GlobalAttributes", 0);
    pimpl->agent_border_shader->set("size", boids.agent_size);
    pimpl->predator_body_shader->activate();
    pimpl->predator_body_shader->bind("GlobalAttributes", 0);
    pimpl->predator_body_shader->set("size", boids.predator_size);
    pimpl->predator_border_shader->activate();
    pimpl->predator_border_shader->bind("GlobalAttributes", 0);
    pimpl->predator_border_shader->set("size", boids.predator_size);

    glBindVertexArray(pimpl->vao[0]);
    glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[0]);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(impl::Agent), (void *)(0));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(impl::Agent), (void *)(3*sizeof(float)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 1, GL_BYTE, GL_FALSE, sizeof(impl::Agent), (void *)(6*sizeof(float)));
    glBindVertexArray(pimpl->vao[1]);
    glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[1]);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(impl::Agent), (void *)(0));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(impl::Agent), (void *)(3*sizeof(float)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 1, GL_BYTE, GL_FALSE, sizeof(impl::Agent), (void *)(6*sizeof(float)));

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    pimpl->agent_border_shader->activate(false);
}

void scene::BoidsScene::restart(Scene &scene)
{
    static auto first_run = true;
    if (first_run)
    {
        resetCamera();
        first_run = false;
    }
    App::instance()->scene.character.setForwardSpeed(Character::FORWARD_SP * 2.f);
    App::instance()->scene.character.setBackwardSpeed(Character::FORWARD_SP * 2.f);
    App::instance()->scene.character.setSidestepSpeed(Character::FORWARD_SP * 2.f);

    auto margin = 2.5f;

    pimpl->agents.clear();
    pimpl->predators.clear();
    pimpl->obstacles.clear();
    pimpl->hungry.clear();
    pimpl->max_obstacle_radius = 0.f;

    auto low_x = scene_param.lower_bound.x + margin;
    auto low_y = scene_param.lower_bound.y + margin;
    auto low_z = scene_param.lower_bound.z + margin;
    auto gap_x = (scene_param.upper_bound.x - margin) - low_x;
    auto gap_y = (scene_param.upper_bound.y - margin) - low_y;
    auto gap_z = (scene_param.upper_bound.z - margin) - low_z;
    for (decltype(scene_param.n_obstacles) i = 0; i < scene_param.n_obstacles; ++i)
    {
        auto x = rnd()*gap_x + low_z;
        auto y = rnd()*gap_z + low_z;
        auto r = .1f + rnd()*1.9f;

        auto collide = false;
        for (auto const &o : pimpl->obstacles)
        {
            if ((x-o.pos.x)*(x-o.pos.x)+(y-o.pos.y)*(y-o.pos.y) <= (r + o.radius)*(r + o.radius))
            {
                collide = true;
                break;
            }
        }
        if (!collide)
        {
            if (r > pimpl->max_obstacle_radius) pimpl->max_obstacle_radius = r;
            pimpl->obstacles.push_back({{x, y}, r});
        }
    }
    for (decltype(scene_param.n_agents) i = 0; i < scene_param.n_agents; ++i)
    {
        auto collide = true;
        float x, z;
        while (collide)
        {
            collide = false;
            x = rnd() * gap_x + low_x;
            z = rnd() * gap_z + low_z;
            for (auto const &o : pimpl->obstacles)
            {
                if (x*x + z*z < (o.radius+boids.agent_size)*(o.radius+boids.agent_size))
                {
                    collide = true;
                    break;
                }
            }
        }
        auto y = rnd() * gap_y + low_y;
        pimpl->agents.push_back({glm::vec3(x, y, z), glm::vec3(rnd_np(), rnd_np(), rnd_np()), 1});
    }
    for (decltype(scene_param.n_predators) i = 0; i < scene_param.n_predators; ++i)
    {
        auto collide = true;
        float x, z;
        while (collide)
        {
            collide = false;
            x = rnd() * gap_x + low_x;
            z = rnd() * gap_z + low_z;
            for (auto const &o : pimpl->obstacles)
            {
                if (x*x + z*z < (o.radius+boids.predator_size)*(o.radius+boids.predator_size))
                {
                    collide = true;
                    break;
                }
            }
        }
        auto y = rnd() * gap_y + low_y;
        pimpl->predators.push_back({glm::vec3(x, y, z), glm::vec3(rnd_np(), rnd_np(), rnd_np()), 1});
    }
    pimpl->hungry.resize(pimpl->predators.size(), boids.satiation);

    pimpl->obstacle_objs.clear();
    auto h = scene_param.upper_bound.y - scene_param.lower_bound.y;
    auto h_c = h*.5f + scene_param.lower_bound.y;
    for (auto const &o : pimpl->obstacles)
    {
        pimpl->obstacle_objs.emplace_back(
                new item::Pillar(glm::vec3(o.pos.x, h_c, -o.pos.y), o.radius,
                                 h));
        scene.add(pimpl->obstacle_objs.back());
    }
    pimpl->ob_tree.build(pimpl->obstacles);

    pimpl->floor->scale(glm::vec3(scene_param.upper_bound.x-scene_param.lower_bound.x,
                                  1.f,
                                  scene_param.upper_bound.z-scene_param.lower_bound.z));
    pimpl->floor->place(glm::vec3(scene_param.lower_bound.x,
                                  scene_param.lower_bound.y,
                                  scene_param.lower_bound.z));
    pimpl->floor->setGrid(scene_param.upper_bound.x-scene_param.lower_bound.x,
                          scene_param.upper_bound.z-scene_param.lower_bound.z);
    scene.add(pimpl->floor);

    pimpl->goal_marker->scale(glm::vec3(boids.has_goal ? .25f : 0.f));
    scene.add(pimpl->goal_marker);

    pimpl->need_upload = true;
}

void scene::BoidsScene::upload(Shader &scene_shader)
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

        glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[0]);
        glBufferData(GL_ARRAY_BUFFER, sizeof(impl::Agent)*pimpl->agents.size(), nullptr, GL_STREAM_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[1]);
        glBufferData(GL_ARRAY_BUFFER, sizeof(impl::Agent)*pimpl->predators.size(), nullptr, GL_STREAM_DRAW);

        pimpl->need_upload = false;
    }

    pimpl->cam_last_pos = App::instance()->scene.character.cam->pos();
}

void scene::BoidsScene::update(float dt)
{
    processInput(dt);
    if (pause) return;

    pimpl->update(dt);

    glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[0]);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(impl::Agent)*pimpl->agents.size(), pimpl->agents.data());
    glBindBuffer(GL_ARRAY_BUFFER, pimpl->vbo[1]);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(impl::Agent)*pimpl->predators.size(), pimpl->predators.data());

    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void scene::BoidsScene::render()
{
    pimpl->drawAgents();

    glBindVertexArray(0);
    pimpl->agent_border_shader->activate(false);

    pimpl->skybox->render();
    renderInfo();
}

void scene::BoidsScene::resetCamera()
{
    App::instance()->scene.character.reset(0.f, 50.f, 65.f, 0.f, 33.f);
    App::instance()->scene.character.setShootable(false);
    App::instance()->scene.character.setFloating(true);
}

void scene::BoidsScene::renderInfo()
{
    auto sum = std::accumulate(pimpl->agents.begin(), pimpl->agents.end(),
                               0, [](int s, const impl::Agent &a){
        return a.alive == 1 ? s + 1 : s;
    });
    App::instance()->text("Agents: " + std::to_string(sum) + " / " + std::to_string(scene_param.n_agents),
                          10, 10, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::LeftTop);
    sum = std::accumulate(pimpl->predators.begin(), pimpl->predators.end(),
                           0, [](int s, const impl::Agent &a){
            return a.alive == 1 ? s + 1 : s;
    });
    App::instance()->text("Predators: " + std::to_string(sum) + " / " + std::to_string(scene_param.n_predators),
                          10, 30, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::LeftTop);
    App::instance()->text("Obstacles: " + std::to_string(pimpl->obstacles.size()) + " / " + std::to_string(scene_param.n_obstacles),
                          10, 50, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::LeftTop);
    std::stringstream ss;
    if (boids.has_goal)
        ss << boids.goal.x << ", " << boids.goal.y << ", " << boids.goal.z;
    else
        ss << "Not Set";
    App::instance()->text("Goal: " + ss.str(),
                          10, 70, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::LeftTop);

    auto h = App::instance()->frameHeight() - 25;
    auto w = App::instance()->frameWidth() - 10;
    App::instance()->text("Press Right and Left Arrow to increase and decrease predators",
                          w, h, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::RightTop);
    h -= 20;
    App::instance()->text("Press Up and Down Arrow to increase and decrease boids",
                          w, h, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::RightTop);
    h -= 20;
    App::instance()->text("Press G to set a Goal Marker",
                          w, h, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::RightTop);
    h -= 20;
    App::instance()->text("Press C to cancel the Goal Marker",
                          w, h, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::RightTop);
    h -= 20;
    App::instance()->text("Hold H, J, K, L, U, M to move goal marker",
                          w, h, .4f,
                          glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                          Anchor::RightTop);
}

void scene::BoidsScene::processInput(float dt)
{
    static auto last_key = GLFW_KEY_UNKNOWN;
    auto window = App::instance()->window();
    static auto sum_dt = 0.f;
    static auto key_count = 0;
#define HOLD_KEY(Key)                                           \
    (last_key == Key && sum_dt > 0.01f && key_count == 10)

#define STICKY_KEY_CHECK(Key, Cmd)                              \
    if (glfwGetKey(window, Key) == GLFW_PRESS)                  \
    {                                                           \
        if (last_key != Key || sum_dt > 0.1f || HOLD_KEY(Key))  \
        {                                                       \
            { Cmd }                                             \
            sum_dt = 0; if (key_count < 10) ++key_count;        \
        }                                                       \
        else sum_dt += dt;                                      \
        if (last_key != Key)                                    \
        { last_key = Key; key_count = 0; }                      \
    }

#define INCREASE_N_AGENTS ++scene_param.n_agents;
#define DECREASE_N_AGENTS if (scene_param.n_agents>0) --scene_param.n_agents;
#define INCREASE_N_PREDATORS ++scene_param.n_predators;
#define DECREASE_N_PREDATORS if (scene_param.n_predators>0) --scene_param.n_predators;
#define INCREASE_N_OBSTACLES ++scene_param.n_obstacles;
#define DECREASE_N_OBSTACLES if (scene_param.n_obstacles>0) --scene_param.n_obstacles;
#define LEFT_GOAL_MARKER  boids.goal.x -= .5f; pimpl->goal_marker->place(glm::vec3(boids.goal.x, boids.goal.y, -boids.goal.z));
#define RIGHT_GOAL_MARKER boids.goal.x += .5f; pimpl->goal_marker->place(glm::vec3(boids.goal.x, boids.goal.y, -boids.goal.z));
#define FORWARD_GOAL_MARKER  boids.goal.z -= .5f; pimpl->goal_marker->place(glm::vec3(boids.goal.x, boids.goal.y, -boids.goal.z));
#define BACKWARD_GOAL_MARKER boids.goal.z += .5f; pimpl->goal_marker->place(glm::vec3(boids.goal.x, boids.goal.y, -boids.goal.z));
#define UP_GOAL_MARKER boids.goal.y += .5f; pimpl->goal_marker->place(glm::vec3(boids.goal.x, boids.goal.y, -boids.goal.z));
#define DOWN_GOAL_MARKER boids.goal.y -= .5f; pimpl->goal_marker->place(glm::vec3(boids.goal.x, boids.goal.y, -boids.goal.z));

    if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS)
        last_key = GLFW_KEY_P;
    else if (glfwGetKey(window, GLFW_KEY_B) == GLFW_PRESS)
        last_key = GLFW_KEY_B;
    else if (glfwGetKey(window, GLFW_KEY_G) == GLFW_PRESS)
        last_key = GLFW_KEY_G;
    else if (glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS)
        last_key = GLFW_KEY_C;
    else
    STICKY_KEY_CHECK(GLFW_KEY_UP, INCREASE_N_AGENTS)
    else
    STICKY_KEY_CHECK(GLFW_KEY_DOWN, DECREASE_N_AGENTS)
    else
    STICKY_KEY_CHECK(GLFW_KEY_LEFT, DECREASE_N_PREDATORS)
    else
    STICKY_KEY_CHECK(GLFW_KEY_RIGHT, INCREASE_N_PREDATORS)
    else
    STICKY_KEY_CHECK(GLFW_KEY_COMMA, DECREASE_N_OBSTACLES)
    else
    STICKY_KEY_CHECK(GLFW_KEY_PERIOD, INCREASE_N_OBSTACLES)
    else
    STICKY_KEY_CHECK(GLFW_KEY_H, LEFT_GOAL_MARKER)
    else
    STICKY_KEY_CHECK(GLFW_KEY_J, FORWARD_GOAL_MARKER)
    else
    STICKY_KEY_CHECK(GLFW_KEY_K, BACKWARD_GOAL_MARKER)
    else
    STICKY_KEY_CHECK(GLFW_KEY_L, RIGHT_GOAL_MARKER)
    else
    STICKY_KEY_CHECK(GLFW_KEY_U, UP_GOAL_MARKER)
    else
    STICKY_KEY_CHECK(GLFW_KEY_M, DOWN_GOAL_MARKER)
    else
    {
        if (last_key == GLFW_KEY_B)
            resetCamera();
        else if (last_key == GLFW_KEY_P)
            pause = !pause;
        else if (last_key == GLFW_KEY_G)
        {
            boids.has_goal = true;
            boids.goal = App::instance()->scene.cam.pos() + App::instance()->scene.cam.direction()*1.f;
            pimpl->goal_marker->place(boids.goal);
            boids.goal.z = -boids.goal.z;
            pimpl->goal_marker->scale(glm::vec3(.25f));
        }
        else if (last_key == GLFW_KEY_C)
        {
            boids.has_goal = false;
            pimpl->goal_marker->scale(glm::vec3(0.f));
        }
        last_key = GLFW_KEY_UNKNOWN;
    }
}
