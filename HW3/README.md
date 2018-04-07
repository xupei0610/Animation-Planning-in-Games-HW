# Motion Planning

This is the program collection for HW3 of CSci 5611 at UMN.

### Key Features

This collection implements a variety of motion planning algorithms, including

**Global Navigation Strategy**

+ Probability RoadMap (PRM) with uniform cost search (Dijkstra) and A\*
+ Lazy Probability RoadMap (LazyPRM)
+ Rapidly-exploring Random Trees (RRT)
+ Optimal Rapidly-expolring Random Trees (RRT\*)

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

[Video Demo](https://www.youtube.com/watch?v=WgelhGE1KLg&index=17&list=PLNoGwCJv2USFR9DVIukjJIZ5yAyFjtEF0)

This is a scenairo with multiple randomly distributed, static, cylindrical obstacles, among which there must be an obstacle with radius 2 at the center of the playground. The playground is a 20x20 square region. A cylindrical agent with radius 0.5 is requested to travel to the point (9, 9) from (-9, 9).

Different path planning algorithms are available in this simulation. When using VO, ORCA and TTC, the agent will try to navigate to the goal position directly without the guidance of road map.
          
RVO and HRVO are not implemented in this simulation due to that they are exactly the same with V0 in the case.

Control:

+ `m`: switch between different planning algorithms 
+ `n`: switch path finding algorithm between uniform cost search and A\* for PRM and LazyPRM

#### Boids

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
In the `blocks` scenarios, agents are distributed at the four corner of the playground. They are requested to go to the diagonal direction while four obstacles are distributed at the diagonal lines.
In the `passage` scenarios, two groups of agents are spawned at an opposite direction. They need to go to the other direction while four dynamic obstacles are pacing up and down along the path.

[Video Demo 1](https://www.youtube.com/watch?v=RTmQkG8i0UQ&t=20s&list=PLNoGwCJv2USFR9DVIukjJIZ5yAyFjtEF0&index=18)
[Video Demo 2](https://www.youtube.com/watch?v=cwZ2HaXCbSc&index=19&list=PLNoGwCJv2USFR9DVIukjJIZ5yAyFjtEF0)
[Video Demo 3](https://www.youtube.com/watch?v=tCQ7LpIgYs4&index=20&list=PLNoGwCJv2USFR9DVIukjJIZ5yAyFjtEF0)

Control:

+ `1`, `2`, `3`: change the scenairo
+ `m`: switch between different interaction techniques


Check-in Video: [click me](https://www.youtube.com/watch?v=krAFdUAnwME&list=PLNoGwCJv2USFR9DVIukjJIZ5yAyFjtEF0&index=16)
