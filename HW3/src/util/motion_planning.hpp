#ifndef PX_CG_PATH_FINDER_HPP
#define PX_CG_PATH_FINDER_HPP

// for quadratic programming used in ORCA
#include <CGAL/basic.h>
#include <CGAL/QP_models.h>
#include <CGAL/QP_functions.h>
#include <CGAL/MP_Float.h>

#include <list>
#include <vector>
#include <iterator>
#include <algorithm>
#include <queue>
#include <limits>
#include <cstring>
#include <iostream>
#include "util/random.hpp"

namespace px { namespace planning
{
template <typename T_vel, typename T_dir>
struct VelocityObstacle
{
    T_vel apex;  // move apex_true backward a little
    T_dir left_leg;
    T_dir right_leg;
    T_vel apex_true;

    VelocityObstacle(T_vel &&apex, T_dir &&left_leg, T_dir &&right_leg, T_vel &&apex_true)
            : apex(apex), left_leg(left_leg), right_leg(right_leg), apex_true(apex_true)
    {}
    VelocityObstacle(const T_vel &apex, const T_dir &left_leg, const T_dir &right_leg, const T_vel &apex_true)
            : apex(apex), left_leg(left_leg), right_leg(right_leg), apex_true(apex_true)
    {}
};
} }

template <typename T_vel, typename T_dir>
std::ostream& operator<<(std::ostream &os,
                         const px::planning::VelocityObstacle<T_vel, T_dir> &vo)
{
    os << "     Apex: "   << vo.apex_true.x << ", " << vo.apex_true.y
       << "\n Left Leg: " << vo.left_leg.x  << ", " << vo.left_leg.y
       << "\nRight Leg: " << vo.right_leg.x << ", " << vo.right_leg.y;
    return os;
};

namespace px { namespace planning
{

/**
 * AStar is an A* path finder
 * @tparam T type of distance, float or double
 * @tparam HValFn a functor to generate h value for a node
 * @param roadmap a 2D N-by-N vector of distances between two nodes.
 *                std::numeric_limits<T>::infinity for inaccessible path.
 * @param start the index of the start node
 * @param goal the index of the end node
 * @param path of node indices of the path from goal to start, empty if planning failed
 * @param h heuristic function
 */
template<typename T, class HValFn>
void AStar(const std::vector<std::vector<T> > &roadmap,
           const std::size_t start, const std::size_t goal,
           std::list<std::size_t> &path,
           HValFn h)
{
    auto n_v = roadmap.size();

    std::vector<bool> visited(n_v, false); // has been popped out from the open set
    std::vector<bool> waited(n_v, false);  // has been pushed into the open set

    std::vector<T> g_val(n_v, std::numeric_limits<float>::infinity());
    std::vector<T> f_val(n_v, std::numeric_limits<float>::infinity());
    std::vector<std::size_t> parent(n_v, std::size_t(-1));
    std::list<std::size_t> frontier;
    auto min_f = [&](const std::size_t &lhs, const std::size_t &rhs)
    {
        return f_val[lhs] < f_val[rhs];
    };

    auto success = false;
#ifndef NDEBUG
    auto n_expanded = 0;
    auto n_loaded = 0;
#endif
    auto u = start; // pick the start point firstly
    g_val[u] = T(0);
    f_val[u] = h(u);
    visited[u] = true;
    for (;;)
    {
        for (decltype(n_v) v = 1; v < n_v; ++v)
        {
            if (visited[v] ||
                roadmap[u][v] == std::numeric_limits<T>::infinity())
                continue;

            if (!waited[v])
            {
                frontier.push_back(v);
#ifndef NDEBUG
                ++n_loaded;
#endif
                waited[v] = true;
            }

            auto alt = g_val[u] + roadmap[u][v];
            if (alt < g_val[v])
            {
                parent[v] = u;
                g_val[v] = alt;
                f_val[v] = alt + h(v);
            }
        }

        if (frontier.empty())
            break;

        auto it = std::min_element(frontier.begin(), frontier.end(), min_f);
        u = *it;

#ifndef NDEBUG
        ++n_expanded;
#endif

        if (u == goal)
        {
            success = true;
            break;
        }

        visited[u] = true;
        frontier.erase(it);
    }
#ifndef NDEBUG
    std::cout << "Method: A*\n"
                 "Nodes Expanded: " << n_expanded << "\n"
                 "Nodes Loaded: " << n_loaded << std::endl;
#endif

    // build path
    path.clear();
    if (success)
    {
        u = goal; // build from the goal point to the start point
        do
        {
            path.push_back(u);
            u = parent[u];
        }
        while (parent[u] != std::size_t(-1));
        path.push_back(u);
    }
}

/**
 * uniformCost is a Dijkstra path finder
 * @tparam T type of distance, float or double
 * @param roadmap 2D N-by-N vector of distances between two nodes. std::numeric_limits<T>::infinity for inaccessible path.
 * @param start the index of the start node
 * @param goal the index of the end node
 * @param path list of node indices of the path from goal to start, empty if planning failed
 */
template<typename T>
void uniformCost(const std::vector<std::vector<T> > &roadmap,
                 const std::size_t start, const std::size_t goal,
                 std::list<std::size_t> &path)
{
    auto n_v = roadmap.size();

    std::vector<bool> visited(n_v, false); // has been popped out from the open set
    std::vector<bool> waited(n_v, false);  // has been pushed into the open set

    std::vector<std::size_t>  parent(n_v, std::size_t(-1));
    std::vector<T> g_val(n_v, std::numeric_limits<T>::infinity());

    std::list<std::size_t> frontier;
    auto min_g = [&](const std::size_t &lhs, const std::size_t &rhs)
    {
        return g_val[lhs] < g_val[rhs];
    };

    auto success = false;
#ifndef NDEBUG
    auto n_expanded = 0;
    auto n_loaded = 0;
#endif
    auto u = start; // pick the start point firstly
    g_val[u] = T(0);
    visited[u] = true;
    for (;;)
    {
        for (decltype(n_v) v = 1; v < n_v; ++v)
        {
            if (visited[v] ||
                roadmap[u][v] == std::numeric_limits<T>::infinity())
                continue;

            if (!waited[v])
            {
                frontier.push_back(v);
#ifndef NDEBUG
                ++n_loaded;
#endif
                waited[v] = true;
            }

            auto alt = g_val[u] + roadmap[u][v];
            if (alt < g_val[v])
            {
                parent[v] = u;
                g_val[v] = alt;
            }
        }

        if (frontier.empty())
            break;

        auto it = std::min_element(frontier.begin(), frontier.end(), min_g);
        u = *it;

#ifndef NDEBUG
        ++n_expanded;
#endif

        if (u == goal)
        {
            success = true;
            break;
        }

        visited[u] = true;
        frontier.erase(it);
    }
#ifndef NDEBUG
    std::cout << "Method: Uniform Cost Search\n"
            "Nodes Expanded: " << n_expanded << "\n"
                      "Nodes Loaded: " << n_loaded << std::endl;
#endif

    // build path
    path.clear();
    if (success)
    {
        u = goal; // build from the goal point to the start point
        do
        {
            path.push_back(u);
            u = parent[u];
        }
        while (parent[u] != std::size_t(-1));
        path.push_back(u);
    }
}

/**
 * smoothe tries to smoothe the path on place
 *
 * TODO try all possible combinations exhaustively if # of tries is large enough
 *
 * @tparam CollisionCheck collision check function
 * @param path indices of path nodes from goal to start
 * @param collide collision check function,
 *                which returns true if the link between two nodes on
 *                the path would cause any collision
 * @param tries # of max tries
 */
template <class CollisionChecker>
void smoothe(int tries,
             std::list<std::size_t> &path,
             CollisionChecker collide)
{
    for (auto i = 0; i < tries; ++i)
    {
        auto n = path.size();
        if (n < 3)
            break;
        else if (n == 3)
        {
            if (!collide(path.front(), path.back()))
                path.erase(std::next(path.begin()));
            break;
        }

        auto u = static_cast<int>(rnd()*n);
        auto v = static_cast<int>(rnd()*n);
        while (std::abs(v - u) < 2)
            v = static_cast<int>(rnd()*n);

        auto it = std::next(path.begin(), u);
        auto it_end = std::next(path.begin(), v);

        if (!collide(*it, *it_end))
        {
            if (u > v)
            {
                std::swap(u, v);
                it = it_end;
            }
            std::advance(it, 1);
            for (++u; u < v; ++u)
                it = path.erase(it);
        }
    }
}

template<typename T, typename T_milestone,
         class LineCollisionChecker,
         class DistanceFn,
         class PathFinder>
std::vector<std::vector<T> > PRM(std::vector<T_milestone> collision_free_milestones,
                                 const std::size_t start,
                                 const std::size_t goal,
                                 std::list<std::size_t> &path,
                                 DistanceFn dist,
                                 LineCollisionChecker collide,
                                 PathFinder pathfinder,
                                 std::size_t k = std::numeric_limits<std::size_t>::max())
{
    k = std::min(collision_free_milestones.size(), std::max(std::size_t(1), k));
    auto n = collision_free_milestones.size();
    std::vector<std::vector<T> >
    roadmap(n, std::vector<T>(n, std::numeric_limits<T>::infinity()));

    for (decltype(n) i = 0; i < n; ++i)
    {
        for (decltype(n) j = i+1; j < n; ++j)
        {
            if (!collide(collision_free_milestones[i], collision_free_milestones[j]))
            {
                roadmap[i][j] = dist(collision_free_milestones[i], collision_free_milestones[j]);
                roadmap[j][i] = roadmap[i][j];
            }
        }
    }
    if (k < n)
    {
        std::vector<T> knn_tmp(n);
        for (decltype(n) i = 0; i < n; ++i)
        {
            std::memcpy(knn_tmp.data(), roadmap[i].data(), sizeof(T)*roadmap[i].size());
            std::nth_element(knn_tmp.begin(), knn_tmp.begin()+k, knn_tmp.end(),
                             [](const T &lhs, const T &rhs)
                             {
                                 return lhs < rhs && lhs != T(0);
                             });
            if (knn_tmp[k-1] != std::numeric_limits<T>::infinity())
                std::replace_if(roadmap[i].begin(), roadmap[i].end(),
                                std::bind(std::greater<T>(), std::placeholders::_1, knn_tmp[k-1]),
                                std::numeric_limits<T>::infinity());
        }
    }
    pathfinder(roadmap, start, goal, path);
    return roadmap;
}

template<typename T, typename T_milestone,
        class DistanceFn,
        class LineCollisionChecker,
        class PathFinder>
std::vector<std::vector<T> > PRMLazy(std::vector<T_milestone> milestones,
                                     const std::size_t start,
                                     const std::size_t goal,
                                     std::list<std::size_t> &path,
                                     DistanceFn dist,
                                     LineCollisionChecker collide,
                                     PathFinder pathfinder)
{
    auto n = static_cast<int>(milestones.size());
    std::vector<std::vector<T> >
    roadmap(n, std::vector<T>(n, std::numeric_limits<T>::infinity()));

    for (auto i = 0; i < n; ++i)
    {
        for (auto j = i+1; j < n; ++j)
        {
            roadmap[i][j] = dist(milestones[i], milestones[j]);
            roadmap[j][i] = roadmap[i][j];
        }
    }

    auto failed = true;
    while (failed)
    {
        failed = false;
        pathfinder(roadmap, start, goal, path);
        if (path.empty()) break;

        for (auto it = path.begin(), it_end = std::next(path.end(), -1); it != it_end;)
        {
            auto it_next = std::next(it, 1);
            if (collide(milestones[*it], milestones[*it_next]))
            {
                roadmap[*it][*it_next] = std::numeric_limits<T>::infinity();
                roadmap[*it_next][*it] = std::numeric_limits<T>::infinity();
                failed = true;
                break;
            }
            it = it_next;
        }
    }

    return roadmap;
}

template<typename T, typename T_milestone,
        class DistanceFn,
        class LineCollisionChecker,
        class LineCollisionIntersect>
std::vector<std::vector<T> > RRT(std::vector<T_milestone> &milestones,
                                 const std::size_t start,
                                 const std::size_t goal,
                                 std::list<std::size_t> &path,
                                 DistanceFn dist,
                                 LineCollisionChecker collide,
                                 LineCollisionIntersect collide_from_to,
                                 T min_step_size,
                                 T max_step_size,
                                 std::function<void(T_milestone&)> resample = {})
{
    constexpr auto eps = T(1e-5);
    min_step_size += eps;

    auto n = milestones.size();
    std::vector<std::size_t> parent(n, std::size_t(-1));
    std::vector<std::vector<T> >
    roadmap(n, std::vector<T>(n, std::numeric_limits<T>::infinity()));

    for (decltype(n) i = 0; i < n; ++i)
    {
        if (i == start || i == goal) continue;

        auto shortest = dist(milestones[start], milestones[i]);
        if (shortest != T(0))
        {
            decltype(n) nn = 0;
            for (decltype(n) j = 0; j < i; ++j)
            {
                if (j == start || j == goal) continue;
                auto d = dist(milestones[j], milestones[i]);
                if (d == T(0))
                {
                    nn = decltype(n)(-1);
                    break;
                }
                else if (d < shortest)
                {
                    shortest = d;
                    nn = j;
                }
            }
            if (nn != decltype(n)(-1))
            {
                auto alpha = collide_from_to(milestones[nn], milestones[i]);
                auto d = alpha < 0 ? shortest : (shortest * alpha);
                if (d > min_step_size)
                {
                    if (d > max_step_size)
                        d = max_step_size;
                    else
                        d -= eps;

                    auto delta = milestones[i] - milestones[nn];
                    delta *= d / shortest;
                    milestones[i] = milestones[nn] + delta;

                    roadmap[nn][i] = d;
                    roadmap[i][nn] = d;
                    parent[i] = nn;

                    continue;
                }
            }
        }
        if (resample)
            resample(milestones[i--]);
        else
            std::swap(milestones[i--], milestones[--n]);

    }

    auto nearest = std::numeric_limits<T>::infinity();
    for (decltype(n) i = 0; i < n; ++i)
    {
        if (i != goal)
        {
            auto d = dist(milestones[goal], milestones[i]);
            if (d < nearest && !collide(milestones[goal], milestones[i]))
            {
                nearest = d;
                parent[goal] = i;
            }
        }
    }

    path.clear();
    if (parent[goal] != std::size_t(-1))
    {
        roadmap[goal][parent[goal]] = nearest;
        roadmap[parent[goal]][goal] = nearest;

        auto u = goal; // build from the goal point to the start point
        do
        {
            path.push_back(u);
            u = parent[u];
        }
        while (parent[u] != std::size_t(-1));
        path.push_back(u);
    }

    return roadmap;
};

template<typename T, typename T_milestone,
        class DistanceFn,
        class LineCollisionChecker,
        class LineCollisionIntersect>
std::vector<std::vector<T> > RRTStar(std::vector<T_milestone> &milestones,
                                     const std::size_t start,
                                     const std::size_t goal,
                                     std::list<std::size_t> &path,
                                     DistanceFn dist,
                                     LineCollisionChecker collide,
                                     LineCollisionIntersect collide_from_to,
                                     T min_step_size,
                                     T max_step_size,
                                     T rewire_radius,
                                     std::function<void(T_milestone&)> resample = {})
{
    constexpr auto eps = T(1e-5);
    min_step_size -= eps;

    auto n = milestones.size();
    std::vector<std::size_t> parent(n, std::size_t(-1));
    std::vector<std::size_t> child(n, std::size_t(-1));
    std::vector<T> cost(n, T(0));
    std::vector<std::vector<T> >
    roadmap(n, std::vector<T>(n, std::numeric_limits<T>::infinity()));

    for (decltype(n) i = 0; i < n; ++i)
    {
        if (i == start || i == goal) continue;

        auto shortest = dist(milestones[start], milestones[i]);
        if (shortest != T(0))
        {
            // look for the nearest neighbor
            decltype(n) nn = 0;
            for (decltype(n) j = 0; j < i; ++j)
            {
                if (j == start || j == goal) continue;
                auto d = dist(milestones[j], milestones[i]);
                if (d == T(0))
                {
                    nn = decltype(n)(-1); // repeated nodes
                    break;
                }
                else if (d < shortest)
                {
                    shortest = d;
                    nn = j;
                }
            }
            if (nn != decltype(n)(-1))
            {
                auto alpha = collide_from_to(milestones[nn], milestones[i]);
                auto d = alpha < 0 ? shortest : (shortest * alpha);
                if (d > min_step_size)
                {
                    if (d > max_step_size)
                        d = max_step_size;
                    else
                        d -= eps;
                    auto delta = milestones[i] - milestones[nn];
                    delta *= d / shortest;
                    // get new node
                    milestones[i] = milestones[nn] + delta;

                    auto cc = cost[nn] + d; // current cost via nearest node
                    if (d < rewire_radius) // there exists node in the rewire radius
                    {
                        std::vector<std::pair<decltype(n), T> > neighbors;
                        neighbors.emplace_back(nn, d);
                        // seek the optimal parent node in the neighbor region
                        for (decltype(n) j = 0; j < i; ++j)
                        {
                            if (j == goal || j == nn) continue;
                            auto tmp = dist(milestones[j], milestones[i]);
                            if (tmp < rewire_radius && !collide(milestones[j], milestones[i]))
                            {
                                auto c = cost[j] + tmp;
                                if (c < cc)
                                {
                                    d = tmp;
                                    cc = c;
                                    nn = j;
                                }
                                neighbors.emplace_back(j, tmp);
                            }
                        }
                        // rewire nodes in the rewire radius
                        // change the parent of nodes in the neighbor regions
                        // to the new node
                        // if it is better to go through the new node
                        for (const auto node: neighbors)
                        {
                            if (node.first == nn) continue;
                            auto c = cc + node.second;
                            if (c < cost[node.first])
                            {
                                roadmap[node.first][parent[node.first]] = std::numeric_limits<T>::infinity(); // remove the link between
                                roadmap[parent[node.first]][node.first] = std::numeric_limits<T>::infinity(); // the node of its current parent node
                                roadmap[node.first][i] = node.second; // link the node to
                                roadmap[i][node.first] = node.second; // the new node
                                auto d_cost = c - cost[node.first];
                                cost[node.first] = c; // update cost
                                child[parent[node.first]] = std::size_t(-1); // remove child node for the original parent node
                                parent[node.first] = i; // update the parent node
                                // update the cost of descendant nodes
                                auto u = node.first;
                                while (child[u] != std::size_t(-1))
                                {
                                    u = child[u];
                                    cost[u] += d_cost;
                                }
                            }
                        }
                    }

                    parent[i] = nn;
                    child[nn] = i;
                    cost[i] = cc;
                    roadmap[nn][i] = d;
                    roadmap[i][nn] = d;

                    continue;
                }
            }
        }
        if (resample)
            resample(milestones[i--]);
        else
            std::swap(milestones[i--], milestones[--n]);

    }

    auto nearest = std::numeric_limits<T>::infinity();
    for (decltype(n) i = 0; i < n; ++i)
    {
        if (i != goal)
        {
            auto d = dist(milestones[goal], milestones[i]);
            if (d < nearest && !collide(milestones[goal], milestones[i]))
            {
                nearest = d;
                parent[goal] = i;
            }
        }
    }

    path.clear();
    if (parent[goal] != std::size_t(-1))
    {
        roadmap[goal][parent[goal]] = nearest;
        roadmap[parent[goal]][goal] = nearest;

        auto u = goal; // build from the goal point to the start point
        do
        {
            path.push_back(u);
            u = parent[u];
        }
        while (parent[u] != std::size_t(-1));
        path.push_back(u);
    }

    return roadmap;
}


template <typename Vec3_pos, typename Vec2_vel, typename T_time, typename T_dist>
Vec2_vel TTC(const T_time t_h,
             const T_dist v_max,
             const T_time dt,
             const std::size_t agent_id,
             const std::vector<Vec3_pos> &ob_pos,
             const std::vector<Vec3_pos> &agent_pos,
             const std::vector<Vec2_vel> &agent_vel)
{
    assert(agent_pos.size() == agent_vel.size() && agent_id < agent_pos.size());

    constexpr T_time t_eps = T_time(1e-6);

    Vec2_vel acc; acc.x = 0; acc.y = 0;

    Vec2_vel w;
    auto ttc = [&](const Vec3_pos &b_pos, const Vec2_vel &dv,
                   const decltype(ob_pos.front().x) a)
    {
        // solve || (pos_b + v_b * t) - (pos_a + v_a *t) || = r_a + r_b
        // to get the time-to-collision
        auto r = b_pos.z + agent_pos[agent_id].z;
        w.x = b_pos.x - agent_pos[agent_id].x;
        w.y = b_pos.y - agent_pos[agent_id].y;

        auto w2 = w.x*w.x + w.y*w.y;
        auto c = w2 - r*r;
        if (c < 0) // collision already, push the agent away
        {
            r *= T_dist(1.05);
            if (w2 == 0)
            {
                if (dv.x == 0 && dv.y == 0)
                {
                    auto x = rnd_np();
                    auto y = rnd_np();
                    auto d = x*x + y*y;
                    r /= std::sqrt(d) * dt;
                    acc.x += x * r;
                    acc.y += y * r;
                }
                else
                {
                    acc += dv * (r / (std::sqrt(a)*dt));
                }
            }
            else
            {
                w2 = std::sqrt(w2);
                c = (r - w2) / (w2*dt);
                acc.x -= w.x * c;
                acc.y -= w.y * c;
            }
            return;
        }
        if (dv.x == 0 && dv.y == 0) return; // no collision if the agent does not move

        auto b = w.x*dv.x + w.y*dv.y;
        auto discriminant = b*b - a*c;
        if (discriminant < 0.f) return;

        auto t = (b - std::sqrt(discriminant)) / a;
        if (t < 0 && t > t_h) return;

        // avoidance direction (pos_a + v_a * t) - (pos_b + v_b * t)
        auto dir = dv * t - w;
        if (dir.x == 0 && dir.y == 0) return;
        dir /= dir.x*dir.x + dir.y*dir.y;
        if (dir.x * dv.y == dir.y * dv.x) // if avoidance force could make the agent stop,
        {                                 // then move along the tangent line a little randomly
            w.x = -dir.y; w.y = dir.x;
            dir = rnd_np() * w;
        }

        auto mag = (t_h - t) / (t + t_eps);

        acc.x += mag * dir.x;
        acc.y += mag * dir.y;
    };

    auto a = agent_vel[agent_id].x*agent_vel[agent_id].x
             + agent_vel[agent_id].y*agent_vel[agent_id].y;
    for (const auto &o : ob_pos)
        ttc(o, agent_vel[agent_id], a);
    for (std::size_t i = 0, n = agent_pos.size(); i < n; ++i)
    {
        if (i == agent_id) continue;
        auto dv = agent_vel[agent_id] - agent_vel[i];
        ttc(agent_pos[i], dv, dv.x*dv.x + dv.y*dv.y);
    }

    auto v = agent_vel[agent_id] + acc;
    auto d2 = v.x*v.x + v.y*v.y;
    if (d2 > v_max*v_max)
        v *= v_max / std::sqrt(d2);
    return v;
};

enum class VelocityObstacleMethod
{
    VO, RVO, HRVO
};
template<VelocityObstacleMethod VOMethod,
         typename Vec3_pos,  typename Vec2_vel,
         typename T_time,
         typename T_pos = Vec2_vel, typename T_dir = Vec2_vel>
std::vector<VelocityObstacle<T_pos, T_dir> >
VO2D(const T_time dt,
     const std::size_t agent_id,
     const std::vector<Vec3_pos> &ob_pos,
     const std::vector<Vec3_pos> &agent_pos,
     const std::vector<Vec2_vel> &agent_vel,
     const std::vector<Vec2_vel> &vel_pref = {}) // vel_pref needed for HRVO only
{
    assert(agent_id < agent_pos.size()
           && agent_pos.size() == agent_vel.size()
           && vel_pref.size() == agent_vel.size());

    constexpr auto apex_eps = 1e-6f; // move the apex of VO backward a little to avoid computation error

    std::vector<VelocityObstacle<T_pos, T_dir> > vo;
    vo.reserve(ob_pos.size() + agent_pos.size());

    T_pos apex, dp, apex_true;
    T_dir left_leg, right_leg;
    for (std::size_t n = ob_pos.size(), i = 0; i < n; ++i)
    {
        auto r  = ob_pos[i].z + agent_pos[agent_id].z;
        dp.x = ob_pos[i].x - agent_pos[agent_id].x;
        dp.y = ob_pos[i].y - agent_pos[agent_id].y;
        auto d = std::sqrt(dp.x*dp.x + dp.y*dp.y);
        if (d > r)
        {
            apex.x = 0; apex.y = 0;
            auto angle = std::atan2(dp.y, dp.x);
            auto theta = std::asin(r/d);
            left_leg.x  = std::cos(angle+theta);
            left_leg.y  = std::sin(angle+theta);
            right_leg.x = std::cos(angle-theta);
            right_leg.y = std::sin(angle-theta);

        }
        else // collision
        {
            apex = dp * ((d-r)/(dt*d));
            left_leg.x = -dp.y/d;
            left_leg.y = dp.x/d;
            right_leg.x = -left_leg.x;
            right_leg.y = -left_leg.y;
        }

        apex_true = apex;
        apex -= dp * (apex_eps/d);  // move the apex backward a little

        vo.emplace_back(apex, left_leg, right_leg, apex_true);
    }

    for (std::size_t n = agent_pos.size(), i = 0; i < n; ++i)
    {
        if (i == agent_id) continue;

        apex = agent_vel[i];

        auto r = agent_pos[i].z + agent_pos[agent_id].z;
        dp.x = ob_pos[i].x - agent_pos[agent_id].x;
        dp.y = ob_pos[i].y - agent_pos[agent_id].y;
        auto d = std::sqrt(dp.x*dp.x + dp.y*dp.y);

        if (d > r)
        {
            auto angle = std::atan2(dp.y, dp.x);
            auto theta = std::asin(r/d);
            left_leg.x  = std::cos(angle+theta);
            left_leg.y  = std::sin(angle+theta);
            right_leg.x = std::cos(angle-theta);
            right_leg.y = std::sin(angle-theta);

            if (VOMethod == VelocityObstacleMethod::RVO)
            {
                apex += agent_vel[agent_id];
                apex.x /= 2;
                apex.y /= 2;
            }
            else if (VOMethod == VelocityObstacleMethod::HRVO)
            {
                auto dv_pref = vel_pref[agent_id] - vel_pref[i];
                auto dv = agent_vel[agent_id] - agent_vel[i];
                if (dp.x*dv_pref.y - dp.y*dv_pref.x > 0)
                    apex += right_leg * ((dv.x*left_leg.y - dv.y*left_leg.x) / (2*std::sin(2*theta)));
                else
                    apex += left_leg * ((dv.x*right_leg.y - dv.y*right_leg.x) / (2*std::sin(2*theta)));
            }
        }
        else // collision
        {
            apex += dp * ((d-r)/(dt*d));
            if (VOMethod == VelocityObstacleMethod::RVO
                || VOMethod == VelocityObstacleMethod::HRVO)
            {
                apex += agent_vel[agent_id];
                apex.x /= 2;
                apex.y /= 2;
            }

            left_leg.x = -dp.y/d;
            left_leg.y = dp.x/d;
            right_leg.x = -left_leg.x;
            right_leg.y = -left_leg.y;
        }

        apex_true = apex;
        apex -= dp * (apex_eps/d); // move the apex backward a little

        vo.emplace_back(apex, left_leg, right_leg, apex_true);
    }

    return vo;
}

template<typename Vec2_vel, typename VelocityObstacle, typename T>
Vec2_vel clearPath(const T v_max,
                   const Vec2_vel &v_pref,
                   std::vector<VelocityObstacle> const &vos,
                   std::function<bool(Vec2_vel&)> v_verify = {}) // function to verify the candidate velocity
{
    std::vector<Vec2_vel> v_samples;
    const auto v_max2 = v_max * v_max;

    v_samples.reserve(vos.empty() ? 1:
                      vos.size()*6 + 2*vos.size()*(vos.size()-1));

    constexpr auto dot = [](const Vec2_vel &lhs, const Vec2_vel &rhs)
    {
        return lhs.x*rhs.x + rhs.y*rhs.y;
    };
    constexpr auto det = [](const Vec2_vel &lhs, const Vec2_vel &rhs)
    {
        return lhs.x*rhs.y - lhs.y*rhs.x;
    };
    constexpr auto len2 = [](const Vec2_vel &v)
    {
        return v.x*v.x + v.y*v.y;
    };

    auto d = v_pref.x*v_pref.x + v_pref.y*v_pref.y;
    v_samples.push_back(d > v_max2 ? v_pref * (v_max/std::sqrt(d)) : v_pref);
    for (auto const &vo : vos)
    {
        auto dv = v_pref - vo.apex;
        auto proj = dot(dv, vo.left_leg);
        if (proj > 0 && det(vo.left_leg, dv) < 0) // if v_pref is at the right (bottom)
        {                                         // side of the left leg
            auto sample = vo.apex + proj * vo.left_leg; // then sample the projection
            if (len2(sample) <= v_max2)             // of v_pref onto the left leg
                v_samples.push_back(sample);
        }
        proj = dot(dv, vo.right_leg);
        if (proj > 0 && det(vo.right_leg, dv) > 0) // if v_pref is at the left (up)
        {                                          // side of the right leg
            auto sample = vo.apex + proj * vo.right_leg; // then sample the projection
            if (len2(sample) <= v_max2)              // of v_pref onto the right leg
                v_samples.push_back(sample);
        }
    }
    for (auto const &vo : vos)
    {
        auto h = det(vo.apex, vo.left_leg);
        auto proj = v_max2 - h * h;
        if (proj > 0)
        {
            proj = std::sqrt(proj);
            auto t = proj - dot(vo.apex, vo.left_leg);
            if (t > 0) v_samples.push_back(vo.apex + t * vo.left_leg);
            t -= proj + proj;
            if (t > 0) v_samples.push_back(vo.apex + t * vo.left_leg);
        }

        h = det(vo.apex, vo.right_leg);
        proj = v_max2 - h * h;
        if (proj > 0)
        {
            proj = std::sqrt(proj);
            auto t = proj - dot(vo.apex, vo.right_leg);
            if (t > 0) v_samples.push_back(vo.apex + t * vo.right_leg);
            t -= proj + proj;
            if (t > 0) v_samples.push_back(vo.apex + t * vo.right_leg);
        }
    }
#define VO_SAMPLING_HELPER(I_LEG, J_LEG)                                    \
    area = det(vos[i].I_LEG, vos[j].J_LEG);                                 \
    if (area != 0) {                                                        \
        auto a = det(vos[j].apex-vos[i].apex,                               \
                     vos[j].J_LEG) / area;                                  \
        auto b = det(vos[j].apex-vos[i].apex,                               \
                     vos[i].I_LEG);                                         \
        if (a >= 0 && (b == 0 || (b<0&&area<0) || (b>0&&area>0))) {         \
            auto sample = vos[i].apex + a * vos[i].I_LEG;                   \
            if (len2(sample) <= v_max2)                                 \
                v_samples.push_back(sample);                                \
        }                                                                   \
    }
    float area;
    for (auto i = 0, tot = static_cast<int>(vos.size()); i < tot - 1; ++i)
    {
        for (auto j = i+1; j < tot; ++j)
        {
            VO_SAMPLING_HELPER(right_leg, right_leg);
            VO_SAMPLING_HELPER(left_leg,  right_leg);
            VO_SAMPLING_HELPER(right_leg, left_leg);
            VO_SAMPLING_HELPER(left_leg, left_leg);
        }
    }

    // check candidate velocities
    if (v_verify)
    {
        for (int i = 0, n = static_cast<int>(v_samples.size()); i < n; ++i)
        {
            if (!v_verify(v_samples[i]))
            {
                std::swap(v_samples[i--], v_samples.back());
                v_samples.pop_back();
                --n;
            }
        }
    }

    auto best = -1;
    Vec2_vel v;
    v.x = decltype(v.x)(0); v.y = decltype(v.y)(0);
    for (auto const &s : v_samples)
    {
        auto valid = true;

        auto count = 0;
        for (auto const &vo : vos)
        {
            auto dv = s - vo.apex_true;
            if (det(vo.right_leg, dv) > 0 && det(vo.left_leg, dv) < 0)
            {
                valid = false;
                if (count > best)
                {
                    v.x = s.x; v.y = s.y;
                    best = count;
                }
                break;
            }
            ++count;
        }
        if (valid)
        {
            v.x = s.x;
            v.y = s.y;
            break;
        }
    }

    return v;
};


template<typename Vec2_vel, typename Vec3_pos, typename T_time, typename T_dist>
Vec2_vel ORCA(const T_time tau,
              const T_dist v_max,
              const T_time dt,
              const std::size_t agent_id,
              const std::vector<Vec3_pos> &ob_pos,
              const std::vector<Vec3_pos> &agent_pos,
              const std::vector<Vec2_vel> &agent_vel,
              const std::vector<Vec2_vel> &vel_pref,
              const Vec2_vel &v_upper_bound = {std::numeric_limits<T_dist>::max(),
                                               std::numeric_limits<T_dist>::max()},
              const Vec2_vel &v_lower_bound = {std::numeric_limits<T_dist>::max(),
                                               std::numeric_limits<T_dist>::max()})
{
    assert(agent_id < agent_pos.size()
           && agent_pos.size() == agent_vel.size()
           && vel_pref.size() == agent_vel.size());
    assert(dt > 0);
//    assert(tau >= 1);

    auto &vel_opt = agent_vel;

    const auto inv_dt = 1 / dt;
    const auto inv_tau = T_dist(1) / tau;
    Vec2_vel u, dir;
    auto gen_orca_line = [&](Vec2_vel &dp, Vec2_vel &dv,
                             T_dist r)
    {
        auto r2 = r*r;
        auto d2 = dp.x*dp.x + dp.y*dp.y;

        if (d2 > r2)
        {
            auto w = dv - dp * inv_tau;
            auto w_len2 = w.x*w.x + w.y*w.y;
            auto cos = w.x*dp.x + w.y*dp.y;

            if (cos < 0 && cos*cos > r2 * w_len2)    // delta_v in the bottom circle, (dp/t, r/t)
            {                                        // u is from dv to the boundary of the circle
                auto w_len = std::sqrt(w_len2);      // along the radius direction
                w.x /= w_len; w.y /= w_len;
                u = w * (r*inv_tau - w_len);
                dir.x =  w.y;
                dir.y = -w.x;
            }
            else
            {
                auto s = std::sqrt(d2 - r2);
                if (dp.x*w.y - dp.y*w.x > 0) // delta_v at the left half region of the VO
                {                            // project to the left leg

                    dir.x = dp.x * s + r * (-dp.y);  // this move the apex along the backward of dp
                    dir.y = dp.y * s + r * dp.x;     // with distance ||dp|| - sqrt(||dp||^2 - r^2)
                }
                else
                {
                    dir.x = -dp.x * s + r * (-dp.y);
                    dir.y = -dp.y * s + r * dp.x;
                }
                dir.x /= d2;
                dir.y /= d2;
                u = dir * (dv.x*dir.x + dv.y*dir.y) - dv;
            }
        }
        else // collision
        {                                               // u is from dv to the boundary of the circle
            auto w = dv - dp * inv_dt;                  // along the radius direction
            auto w_len = std::sqrt(w.x*w.x + w.y*w.y);
            w.x /= w_len; w.y /= w_len;
            u = w * (r*inv_dt - w_len);
            dir.x =  w.y;
            dir.y = -w.x;
        }
    };

    Vec2_vel dp, dv, p;
    std::vector<std::pair<Vec2_vel, Vec2_vel> > lines;
    // <dir, point>, valid region is at the left side of the line
    dv.x = vel_opt[agent_id].x;
    dv.y = vel_opt[agent_id].y;
    for (std::size_t i = 0, n = ob_pos.size(); i < n; ++i)
    {
        dp.x = ob_pos[i].x - agent_pos[agent_id].x;
        dp.y = ob_pos[i].y - agent_pos[agent_id].y;

        gen_orca_line(dp, dv, agent_pos[agent_id].z + ob_pos[i].z);

        p.x = vel_opt[agent_id].x + u.x;
        p.y = vel_opt[agent_id].y + u.y;

        lines.emplace_back(dir, p);

    }
    for (std::size_t i = 0, n = agent_pos.size(); i < n; ++i)
    {
        if (i == agent_id) continue;

        dp.x = agent_pos[i].x - agent_pos[agent_id].x;
        dp.y = agent_pos[i].y - agent_pos[agent_id].y;
        dv.x = vel_opt[agent_id].x - vel_opt[i].x;
        dv.y = vel_opt[agent_id].y - vel_opt[i].y;

        gen_orca_line(dp, dv, agent_pos[agent_id].z + agent_pos[i].z);

        p.x = vel_opt[agent_id].x + u.x/2;
        p.y = vel_opt[agent_id].y + u.y/2;

        lines.emplace_back(dir, p);
    }


    CGAL::Quadratic_program<float> qp(CGAL::SMALLER);

    qp.set_d(0, 0, 2); qp.set_d(1, 1, 2);
    qp.set_c(0, -2*vel_pref[agent_id].x), qp.set_c(1, -2*vel_pref[agent_id].y);
    qp.set_c0(vel_pref[agent_id].x*vel_pref[agent_id].x + vel_pref[agent_id].y*vel_pref[agent_id].y);

    auto row = 0;
    for (const auto &l: lines)
    {
        // line: dir.y * x - dir.x * y - (dir.y*point.x - dir.x*point.y) = 0
        // valid region is the left side of the line
        qp.set_a(0, row, l.first.y); qp.set_a(1, row, -l.first.x);
        qp.set_b(row, l.first.y * l.second.x - l.first.x * l.second.y);
        ++row;
    }
    qp.set_u(0, true, v_upper_bound.x);
    qp.set_u(1, true, v_upper_bound.y);
    qp.set_l(0, true, v_lower_bound.x);
    qp.set_l(1, true, v_lower_bound.y);

    auto s = CGAL::solve_quadratic_program(qp, CGAL::MP_Float());
//    std::cout << "v_pref: " << vel_pref[agent_id].x << ", " << vel_pref[agent_id].y << std::endl;
//    std::cout << s << std::endl;

    return {static_cast<decltype(dv.x)>(CGAL::to_double(*(s.variable_values_begin()))),
            static_cast<decltype(dv.y)>(CGAL::to_double(*(++(s.variable_values_begin()))))};
}

} }

#endif
