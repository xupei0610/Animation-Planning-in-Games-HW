# Motion Planning

This is the program collection for HW3 of CSci 5611 at UMN.

### Key Features

This collection implements a variety of motion planning algorithms, including

**Global Navigation Strategy**

+ Probability RoadMap (PRM) with uniform cost search (Dijkstra) and A\*
+ Lazy Probability RoadMap (LazyPRM)
+ Rapidly-exploring Random Trees (RRT)
+ Optimal Rapidly-exploring Random Trees (RRT\*)

**Interaction Technique**

+ Time-to-Collision (TTC) method
+ Velocity Obstacles (VO)
+ Reciprocal Velocity Obstacles (RVO)
+ Hybrid Reciprocal Velocity Obstacles (HRVO)
+ Optimal Reciprocal Collision Avoidance (ORCA)

**Boids** with

+ obstacle avoidance
+ navigation
+ chase for predators
+ flee for preys
+ collision avoidance to the user

There are also some trivial features, including

+ K-d Tree
+ path smoothing
+ fully 3D rendering with controllable camera
+ 3D navigation for boids
+ movable goal for boids at run time
+ multiple agents, obstacles in a scenario
+ dynamic obstacles (moving along pre-defined paths without intentive avoidance)


### Gallery

Control:

+ `esc`: menu
+ `a`, `w`, `s`, `d`: move left, forward, backward, right
+ `q`, `e`: turn left, right
+ `mouse move`: turn the forward direction
+ `r`: restart the simulation
+ `b`: reset the position of the camera
+ `p`: pause

#### Simple Road Map Scenario

[![Video Demo](./doc/roadmap_comp1.gif)](https://www.youtube.com/watch?v=WgelhGE1KLg&index=17&list=PLNoGwCJv2USFR9DVIukjJIZ5yAyFjtEF0&t=1m7s)

[![Video Demo](./doc/roadmap_comp2.gif)](https://www.youtube.com/watch?v=WgelhGE1KLg&index=17&list=PLNoGwCJv2USFR9DVIukjJIZ5yAyFjtEF0&t=1m17s)

Video Demo: [Click Me](https://www.youtube.com/watch?v=WgelhGE1KLg&index=17&list=PLNoGwCJv2USFR9DVIukjJIZ5yAyFjtEF0)

This is a scenairo with multiple randomly distributed, static, cylindrical obstacles, among which there must be an obstacle with radius 2 at the center of the playground. The playground is a 20x20 square region. A cylindrical agent with radius 0.5 is requested to travel to the point (9, 9) from (-9, 9).

Different path planning algorithms are available in this simulation. When using VO, ORCA and TTC, the agent will try to navigate to the goal position directly without the guidance of road map.
          
RVO and HRVO are not implemented in this simulation due to that they are exactly the same with V0 in the case.

Control:

+ `m`: switch between different planning algorithms 
+ `n`: switch path finding algorithm between uniform cost search and A\* for PRM and LazyPRM
+ `o`: toggle path smoothing for PRM, LazyPRM, RRT, RRT\*

#### Boids

[![Video Demo](./doc/boids_chase_flee1.gif)](https://www.youtube.com/watch?v=-7YDfdkaP5s&list=PLNoGwCJv2USFR9DVIukjJIZ5yAyFjtEF0&index=21)

Boids chase the goal marker (green ball): 

[![Video Demo](./doc/boids_navigation.gif)](https://www.youtube.com/watch?v=-7YDfdkaP5s&list=PLNoGwCJv2USFR9DVIukjJIZ5yAyFjtEF0&index=21&t=0m23s)

Boids fly around a pillar with collision avoidance:

[![Video Demo](./doc/boids_hover1.gif)](https://www.youtube.com/watch?v=-7YDfdkaP5s&list=PLNoGwCJv2USFR9DVIukjJIZ5yAyFjtEF0&index=21&t=1m4s)

Boids fly around with collision avoidance:

[![Video Demo](./doc/boids_hover2.gif)](https://www.youtube.com/watch?v=-7YDfdkaP5s&list=PLNoGwCJv2USFR9DVIukjJIZ5yAyFjtEF0&index=21&t=1m23s)

Boids flee from predators while doing navigation:

[![Video Demo](./doc/boids_chase_flee2.gif)](https://www.youtube.com/watch?v=-7YDfdkaP5s&list=PLNoGwCJv2USFR9DVIukjJIZ5yAyFjtEF0&index=21)

Video Demo: [Click Me](https://www.youtube.com/watch?v=-7YDfdkaP5s&list=PLNoGwCJv2USFR9DVIukjJIZ5yAyFjtEF0&index=21)

This simulation is a scenario with a 3D playground and multiple static cylinderical obstacles. Predators wander in the playground and chase preys. Preys flee from predators while wandering in the playgournd or navigate to the goal point if a goal point is set. While avoiding obstacles, boids also will avoid collision with the user controlled camera such that there usually is no boid flying towards the camera.

Control:

+ `g`: set a goal point at the current camera location
+ `h`, `j`, `k`, `l`, `u`, `m`: move the goal point
+ `c`: cancel the goal point (boids will wander in the scene)
+ `up`, `down`: increase, decrease the number of preys (need restart by `r`)
+ `left`, `right`: decrease, increase the amount of predators (need restart)
+ `,`, `.`: decrease, increase the amount of obstacles (need restart)

#### Interaction Techniques

This simulation compares the performance of interaction techniques including VO, RVO, HRVO, ORCA and TTC.

Three scenarios are implemented.

In the `circle` scenarios, 200 agents are spawned along a circle and each of them has a goal at the diagonal direction.

[![Circle Demo](./doc/multi_agent_circle.gif)](https://www.youtube.com/watch?v=RTmQkG8i0UQ&index=18&list=PLNoGwCJv2USFR9DVIukjJIZ5yAyFjtEF0&t=2m54s)

In the `blocks` scenarios, agents are distributed at the four corner of the playground. They are requested to go to the diagonal direction while four obstacles are distributed at the diagonal lines.

[![Blocks Demo](./doc/multi_agent_blocks.gif)](https://www.youtube.com/watch?v=cwZ2HaXCbSc&index=19&list=PLNoGwCJv2USFR9DVIukjJIZ5yAyFjtEF0&t=3m24s)

In the `passage` scenarios, two groups of agents are spawned at an opposite direction. They need to go to the other direction while four dynamic obstacles are pacing up and down along the path.

[![Passage Demo](./doc/multi_agent_passage.gif)](https://www.youtube.com/watch?v=tCQ7LpIgYs4&index=20&list=PLNoGwCJv2USFR9DVIukjJIZ5yAyFjtEF0&t=1m52s)

Video Demo: [Circle Demo](https://www.youtube.com/watch?v=RTmQkG8i0UQ&t=20s&list=PLNoGwCJv2USFR9DVIukjJIZ5yAyFjtEF0&index=18), [Blocks Demo](https://www.youtube.com/watch?v=cwZ2HaXCbSc&index=19&list=PLNoGwCJv2USFR9DVIukjJIZ5yAyFjtEF0), [Passage Demo](https://www.youtube.com/watch?v=tCQ7LpIgYs4&index=20&list=PLNoGwCJv2USFR9DVIukjJIZ5yAyFjtEF0)

Control:

+ `1`, `2`, `3`: change the scenairo
+ `m`: switch between different interaction techniques

### Usage

    mkdir build
    cd build
    cmake ..
    make

    ./roadmap
    ./boids
    ./multi_agent

Dependencies:
  
+ OpenGL 4.3+
+ GLM
+ GLEW
+ GLFW 3.2
+ OpenMP
+ FreeType
+ stbi\_image (already included in `lib`)

### Performance Analysis

#### Uniform Cost Search (Dijkstra) v.s. A\*

Uniform Cost Search (UCS) is in fact the method of Dijkstra with an early stop. UCS will stop when the goal node is picked from the open set, which is the same strategy of A\*. As a contrast, Dijkstra will try to find out the optimal path from the start node to all other nodes.
Therefore, it is unfair to directly compare the perofrmance of Dijkstra and A\*.

UCS is a special case of A\*. In A\*, a node, `n`, having the minimum value of `f(n)` will be firstly picked from the open set, where

    f(n) = g(n) + h(n)

`g(n)` is the distance from the start node to the node `n`, and `h(n)` is the heuristic value of the node `n`, which is an estimation of the distance from `n` to the goal node.

`g(n)` is the required not to overestimate the real distance from the node `n` to the goal node such that we must be able to find the optimal path to the goal node once the goal node has the minimum value of `f(n) = g(n) + 0`.

In UCS, a node is picked out from the open set if it has the minimum value of 

    f(n) = g(n)

It can be recognized that UCS picks nodes from the open set as a circle with an increasing radius and centered at the start node, while A\* trends to go towards the direction of the goal node with the guidance of the heuristic function.

In the simulation of Simple RoadMap Scenario, the agent is spawned at the left bottom corner and the goal node is at the right top corner. Therefore, UCS has to explore almost all nodes before it reaches the goal node.

In our tests, the simulation of Simple RoadMap Scenario with 500 sample nodes, UCS has to explore 490~500 nodes before finding the optimal path to the goal node, while A\* usually only needs to explore about 15~40 nodes.

The performance difference is significant for the LazyPRM algorithm, which needs to do path finding using UCS or A\* several times before finding the optimal, collision-free path.

UCS v.s. A\* via LazyPRM without path smoothing:

[![UCS and A\* Video Demo](./doc/ucs_a_comp.gif)](https://www.youtube.com/watch?v=PNHxbbWYvCI&list=PLNoGwCJv2USFR9DVIukjJIZ5yAyFjtEF0&index=23)

In this benchmark, LazyPRM uses the same milestones. They performs UCS and A\* several but same times to find the optimal path.

It is obvious that A\* is much faster than UCS.

#### K-d Tree v.s. Brute Force

K-d Tree is very attractive for nearest neighbor problems in theory, but another thing in practice. 

In theory, K-d Tree, in the worst case, has the same complexity with brute-force method to find nearest neighbors. However, K-d Tree has a huge cost for traversals, while the brute-force method using linear search has almost no extra cost.

Besides, in order to perform well, K-d Tree should be balanced. This means that K-d Tree has to be rebuilt frequently in highly dynamic environments, like the case of boids.

We only implement K-d Tree for finding nearest obstacles in order to avoid the cost of rebuilding the tree for agents.

The improvement is trivial in our implementation, since the number of obstacles are not so large to sense the performance difference.

We tried to introduce K-d Tree for neighbor search of boids. However, we only obtained a performance decrease. A uniform grid spatial data structure is expected to perform well for this case.  

#### PRM v.s. RRT v.s. RRT\*

From the aspect of ease of coding, PRM is easier for implementation.
PRM has three steps: collision-free sampling; roadmap building; path finding using UCS, A\* or other path finders.
As a contrast, everytime after simpling, RRT needs to find the nearest node, then pick a proper point on the path from the nearest node to the sampling road as the new node, and then link the nearest node to the new node.
The process is repeated after sampling several times or until it is reachable from the new node to the goal node.
RRT\* is more complex because we need to rewire the link for neighbor nodes of the new node.

The path generated by PRM usually is much more straight compared to RRT, since it builds roadmap for a node with all other nodes or several neighbor nodes, while RRT link a new node with only the nearest one.

RRT\* is able to generate straight path with the help of and at the cost of rewiring.

PRM v.s. RRT v.s. RRT\* without path smoothing:

[![PRM RRT RRT\* Comparison Video Demo](./doc/prm_rrt_rrtstar.gif)](https://www.youtube.com/watch?v=SCXu8B__Exo&list=PLNoGwCJv2USFR9DVIukjJIZ5yAyFjtEF0&index=24)

#### Path Smoothing

Path Smoothing is important, especially for RRT, as we talked above that RRT usually could lead a tortuous path.

In this program, we implement path smoothing by a simple way: randomly remove nodes from the original path, if the path without these nodes is still collision-free.

In some applications, path smoothing may be much more complex especially in the case where we want the turn to be smooth as well.

[![Path Smoothing Video Demo](./doc/path_smooth.gif)](https://www.youtube.com/watch?v=PYwMXThNwlc&t=0s&list=PLNoGwCJv2USFR9DVIukjJIZ5yAyFjtEF0&index=22)

#### Local Minima Examples




