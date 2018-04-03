#ifndef PX_CG_KD_TREE_HPP
#define PX_CG_KD_TREE_HPP

#include <vector>
#include <numeric>
#include <algorithm>

namespace px
{

template<typename T, typename D>
class KdTree
{
public:
    const unsigned int k;
    const unsigned int depth;

public:
    std::function<bool(const T &lhs, const T &rhs, std::size_t k)>
            islower = [this](const T &lhs, const T &rhs, std::size_t k)
    {
        return const_cast<T&>(lhs)[k] < const_cast<T&>(rhs)[k];
    };

    class Dist
    {
    public:
        unsigned int k;
        explicit Dist(unsigned int k) : k(k) {}

        D operator()(const T &lhs, const T &rhs)
        {
            auto d = static_cast<D>(0);
            for (typename std::remove_const<decltype(k)>::type i = 0; i < k; ++i)
                d += (const_cast<T&>(lhs)[i] - const_cast<T&>(rhs)[i]) * (const_cast<T&>(lhs)[i] - const_cast<T&>(rhs)[i]);
            return d;
        }
    };

    KdTree(unsigned int k, unsigned int depth = 32)
            : k(k), depth(depth)
    {}
    ~KdTree() = default;

    struct Node
    {
        std::size_t left;  // index of left child node in the vector nodes
        std::size_t right; // index of right child node in the vector nodes
        std::size_t split; //

        Node()
                : left(-1), right(-1), split(-1)
        {}
        Node(std::size_t left, std::size_t right, std::size_t split)
                : left(left), right(right), split(split)
        {}
    };

    void build(const std::vector<T> &points)
    {
        nodes.clear();
        if (points.empty()) return;

        auto count = points.size();
        std::vector<std::size_t> index(count);
        std::iota(index.begin(), index.end(), 0);

        auto m = count - 1;
        m |= m >> 1;
        m |= m >> 2;
        m |= m >> 4;
        m |= m >> 8;
        m |= m >> 16;
        ++m;
        nodes.reserve(std::min(m < 3 ? 1 : m-1, 2 * count - m / 2 - 1));

        // build recursively, root node is nodes.back()
        _build(count, index.data(), 0, points);

        // build by DFS, root node is nodes.front()
//        nodes.emplace_back();
//        struct Task
//        {
//            std::size_t begin, end, node, dim;
//        };
//        std::vector<Task> tasks(depth);
//        tasks[0] = {0, count - 1, 0, 0};
//
//        auto current_task = 0;
//        do
//        {
//            auto &task = tasks[current_task];
//            auto &node = nodes[task.node];
//            if (task.end - task.begin > 1)
//            {
//                auto n = (task.begin + task.end) / 2;
//                std::nth_element(index.begin() + task.begin,
//                                 index.begin() + n,
//                                 index.begin() + task.end + 1,
//                                 [&](std::size_t lhs, std::size_t rhs)
//                                 {
//                                     return islower(points[lhs], points[rhs], task.dim);
//                                 });
//                auto next_dim = task.dim == k - 1? 0 : task.dim + 1;
//
//                node.left = nodes.size();
//                node.right = node.left + 1;
//                node.split = index[n];
//
//                tasks[current_task+1] = {n + 1, task.end, node.right, next_dim};
//                tasks[current_task++] = {task.begin, n, node.left, next_dim};
//
//                nodes.emplace_back();
//                nodes.emplace_back();
//            }
//            else
//            {   // leaf node
//                node.left = index[task.begin];
//                if (task.begin != task.end)
//                    node.right = index[task.end];
//                --current_task;
//            }
//        } while (current_task != -1);

    }

protected:
    std::size_t _build(std::size_t size, std::size_t *index, std::size_t dim,
                       const std::vector<T> &points)
    {
        assert(size > 0);

        if (size == 1)
        {
            nodes.emplace_back(index[0], std::size_t(-1), std::size_t(-1));
        }
        else if (size == 2)
        {
            nodes.emplace_back(index[0], index[1], std::size_t(-1));
        }
        else
        {
            auto n = (size - 1) / 2;
            std::nth_element(index, index + n, index + size,
                             [&](std::size_t lhs, std::size_t rhs)
                             {
                                 return islower(points[lhs], points[rhs], dim);
                             });

            auto next_dim = dim == k - 1 ? 0 : dim + 1;
            auto split = index[n];

            nodes.emplace_back(_build(n+1, index, next_dim, points),
                               _build(size-(n+1), index+n+1, next_dim, points),
                               split);
        }
        return nodes.size()-1;
    }

public:
    template<typename T2, class DistHelper = Dist>
    std::size_t nearest(const T2 &p, const std::vector<T> &points,
                        const D &max_distance = std::numeric_limits<float>::max()) const
    {
        struct Task
        {
            T c;
            D dist;
            std::size_t node;
            std::size_t dim;
        };
        std::vector<Task> tasks(depth);
        tasks[0] = {p, static_cast<D>(0), 0, 0};

        auto distance = max_distance;
        Dist dist(k);
        std::size_t nearest;
        auto current_task = 0;
        do
        {
            auto &task = tasks[current_task];
            auto &n = nodes[task.node];
            if (task.dist > distance)
            {
                --current_task;
            }
            else if (n.split == std::size_t(-1))
            {
                auto d = dist(p, points[n.left]);
                if (d < distance)
                {
                    distance = d;
                    nearest = n.left;
                }
                if (n.right != std::size_t(-1))
                {
                    d = dist(p, points[n.right]);
                    if (d < distance)
                    {
                        distance = d;
                        nearest = n.right;
                    }
                }
                --current_task;
            }
            else
            {
                Task near, far;
                auto split = const_cast<T&>(points[n.split])[task.dim];
                auto next_dim = task.dim == k - 1? 0 : task.dim + 1;

                auto c = task.c;
                c[task.dim] = split;

                near.c = task.c;
                near.dim = next_dim;
                near.dist = static_cast<D>(0);

                far.c  = c;
                far.dim  = next_dim;
                far.dist  = dist(p, far.c);

                if (const_cast<T2&>(p)[task.dim] <= split)
                {
                    near.node = n.left;
                    far.node  = n.right;
                }
                else
                {
                    near.node = n.right;
                    far.node  = n.left;
                }

                tasks[current_task] = far;
                tasks[++current_task] = near;
            }
        }
        while (current_task != -1);

//        if (nodes.size() == 0) return -1;
//        auto nearest = std::size_t(-1);
//        auto ref_dist = max_distance;
//        _nearest<T2, DistHelper>(p, points, nodes.size()-1, 0,
//                                 p, ref_dist, nearest);
        return nearest;
    }

//protected:
//    template<typename T2, class DistHelper = Dist>
//    void _nearest(const T2 &p, const std::vector<T> &points,
//                  const std::size_t node, const std::size_t dim,
//                  T2 ref_point, D &ref_dist,
//                  std::size_t &nearest) const
//    {
//        DistHelper dist(k);
//        const auto &n = nodes[node];
//        if (n.split == std::size_t(-1))
//        {
//            auto d = dist(p, points[n.left]);
//            if (d < ref_dist)
//            {
//                ref_dist = d;
//                nearest = n.left;
//            }
//            if (n.right != std::size_t(-1))
//            {
//                d = dist(p, points[n.right]);
//                if (d < ref_dist)
//                {
//                    ref_dist = d;
//                    nearest = n.right;
//                }
//            }
//        }
//        else
//        {
//            auto split = const_cast<T&>(points[n.split])[dim];
//            auto next_dim = dim == k - 1? 0 : dim + 1;
//
//            if (const_cast<T2&>(p)[dim] <= split)
//            {
//                _nearest<T2, DistHelper>(p, points, n.left,  next_dim, ref_point, ref_dist,
//                         nearest);
//                ref_point[dim] = split;
//                if (dist(p, ref_point) <= ref_dist)
//                {
//                    _nearest<T2, DistHelper>(p, points, n.right, next_dim,
//                             ref_point, ref_dist,
//                             nearest);
//                }
//            }
//            else
//            {
//                _nearest<T2, DistHelper>(p, points, n.right, next_dim,
//                         ref_point, ref_dist,
//                         nearest);
//                ref_point[dim] = split;
//                if (dist(p, ref_point) <= ref_dist)
//                {
//                    _nearest<T2, DistHelper>(p, points, n.left, next_dim,
//                             ref_point, ref_dist,
//                             nearest);
//                }
//            }
//        }
//    }

public:
    template <typename T2, class DistHelper = Dist>
    std::vector<std::pair<std::size_t, D> >
        around(const T2 &p, const std::vector<T> &points, const D &radius) const
    {
        std::vector<std::pair<std::size_t, D> > neighbors;
        if (nodes.empty()) return neighbors;

        struct Task
        {
            T2 c;
            D dist;
            std::size_t node;
            std::size_t dim;
        };
        std::vector<Task> tasks(depth);
        tasks[0] = {p, static_cast<D>(0), nodes.size() - 1, 0};

        DistHelper dist(k);

        auto current_task = 0;
        do
        {
            auto &task = tasks[current_task];
            auto &n = nodes[task.node];
            if (task.dist > radius)
            {
                --current_task;
            }
            else if (n.split == std::size_t(-1))
            {
                auto d = dist(p, points[n.left]);
                if (d < radius)
                    neighbors.emplace_back(n.left, d);
                if (n.right != std::size_t(-1))
                {
                    d = dist(p, points[n.right]);
                    if (d < radius)
                        neighbors.emplace_back(n.right, d);
                }
                --current_task;
            }
            else
            {
                Task near, far;
                auto split = const_cast<T&>(points[n.split])[task.dim];
                auto next_dim = task.dim == k - 1? 0 : task.dim + 1;

                auto c = task.c;
                c[task.dim] = split;

                near.c = task.c;
                near.dim = next_dim;
                near.dist = static_cast<D>(0);

                far.c  = c;
                far.dim  = next_dim;
                far.dist  = dist(p, far.c);

                if (const_cast<T2&>(p)[task.dim] <= split)
                {
                    near.node = n.left;
                    far.node  = n.right;
                }
                else
                {
                    near.node = n.right;
                    far.node  = n.left;
                }

                tasks[current_task] = far;
                tasks[++current_task] = near;
            }
        }
        while (current_task != -1);
        return neighbors;

//        if (nodes.empty()) return neighbors;
//
//        _around<T2, DistHelper>(p, points, nodes.size()-1, 0,
//                                p, radius, neighbors);
//        return neighbors;
    }
//
//protected:
//    template <typename T2, class DistHelper>
//    void _around(const T2 &p, const std::vector<T> &points,
//            const std::size_t node, const std::size_t dim,
//            T2 ref_point, const D &radius,
//            std::vector<std::pair<std::size_t, D> > &neighbors) const
//    {
//        DistHelper dist(k);
//        const auto &n = nodes[node];
//        if (n.split == std::size_t(-1))
//        {
//            auto d = dist(p, points[n.left]);
//            if (d < radius)
//            {
//                neighbors.emplace_back(n.left, d);
//            }
//            if (n.right != std::size_t(-1))
//            {
//                d = dist(p, points[n.right]);
//                if (d < radius)
//                {
//                    neighbors.emplace_back(n.right, d);
//                }
//            }
//        }
//        else
//        {
//            auto split = const_cast<T&>(points[n.split])[dim];
//            auto next_dim = dim == k - 1? 0 : dim + 1;
//
//            if (const_cast<T2&>(p)[dim] <= split)
//            {
//                _around<T2, DistHelper>(p, points, n.left, next_dim,
//                                         ref_point, radius,
//                                         neighbors);
//
//                ref_point[dim] = split;
//                if (dist(p, ref_point) <= radius)
//                {
//                    _around<T2, DistHelper>(p, points, n.right, next_dim,
//                                         ref_point, radius,
//                                         neighbors);
//                }
//            }
//            else
//            {
//                _around<T2, DistHelper>(p, points, n.right, next_dim,
//                                        ref_point, radius,
//                                        neighbors);
//                ref_point[dim] = split;
//                if (dist(p, ref_point) <= radius)
//                {
//                    _around<T2, DistHelper>(p, points, n.left, next_dim,
//                                            ref_point, radius,
//                                            neighbors);
//                }
//            }
//        }
//    }

protected:
    std::vector<Node> nodes;
};
}

#endif // PX_CG_KD_TREE_HPP
