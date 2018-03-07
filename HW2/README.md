# Physically Based Simulation

This is the program for HW2 of the course CSCI 5611, UMN.

## Features

##### Implementation

+ 3D Cloth Simulation using Mass-Spring Model
+ 2D Shallow Water

##### Cloth Simulation

+ CUDA Implementation
+ 200x200 with 10 iterations at 205 FPS using RK4 and CUDA
+ 50x50 with 10 iterations at 72 FPS using RK4 on CPU, ~370 FPS using Semi-implicit Euler
+ 1st-Order Integrator: Euler, Semi-implicit Euler
+ Higher-Order Integrator: Verlet, Velocity Verlet, Midpoint, RK4
+ Textured Cloth
+ Two-Way Coupling Object Interaction between ball and cloth for CPU implementation
+ One-Way Cloth-Sphere Interaction for GPU implementation
+ Wind Component of Drag Term

##### 2D Shallow Water Simulation

+ Thread-parallel Implementaion
+ CUDA Implementation
+ 1000x1000 with 10 iterations at 303 FPS using CUDA
+ 500x500 with 10 iterations at 35 FPS on CPU with OpenMP
+ User Interaction

##### User  Interface

+ 3D Implementation
+ 3D User Controlled Camera
+ Real-time User Interaction

## Gallery

##### Control

+ `w`, `a`, `s`, `d`: move
+ `q`, `e`: turn left/right
+ `mouse motion`: turn direction
+ `mouse scroll`: zoom in/out
+ `Esc`: toggle menu
+ `Spacebar`: Jump or Fly Up
+ `f`: toggle fly mode
+ `c`: reset camera
+ `r`: reset scene
+ `p`: freeze scene
+ `k`: switch between CPU and GPU mode

#### Cloth Simulation

<img src="./doc/curtain.gif" />

Curtain

<img src="./doc/flag.gif" />

Flag

<img src="./doc/cover.gif" />

Cover

Video demo: [click me](https://www.youtube.com/watch?v=NxPSItg767A&list=PLNoGwCJv2USFR9DVIukjJIZ5yAyFjtEF0&index=10)

CPU vs GPU: [click me](https://www.youtube.com/watch?v=gQZSnbV6_MY&list=PLNoGwCJv2USFR9DVIukjJIZ5yAyFjtEF0&index=12)

##### Control

+ hold `←`, `→`, `↑`, `↓`: control wind
+ `n`: reset wind
+ `m`: switch between integrators (only RK4 for GPU implementation)
+ `1`, `2`, `3`, `4`: change scenes
+ `b`: push ball in scene `1`


#### 2D Shallow Water

<img src="./doc/water.gif" />

Video demo: [click me](https://www.youtube.com/watch?v=Z7B2WFcbmLA&list=PLNoGwCJv2USFR9DVIukjJIZ5yAyFjtEF0&index=15)

##### Control

+ `g`: drop a drop

## Benchmark

#### Test Environment

	Ubuntu 16.04, G++ 5.4.1, CUDA 9.0, OpenGL 4.5, OpenBlas
	Intel i7-7700K, Nvidia GTX1080 Ti

#### Super Smash Integrators

<img src="./doc/bench.gif" />

**Runge-Kutta 4** v.s. **Verlet** v.s. **Velocity Verlet** v.s.
**Midpoint** vs. **Euler** v.s. **Semi-implicit Euler**

Video Demo: [click me](https://www.youtube.com/watch?v=bpruAYZ57aI&index=11&list=PLNoGwCJv2USFR9DVIukjJIZ5yAyFjtEF0)

#### CUDA Improvement

<img src="./doc/cloth_gpu.gif" />

200x200 with RK4 and 10 iterations at 205 FPS using CUDA v.s. 50x50 at 702 FPS on CPU

Video Demo: [click me](https://www.youtube.com/watch?v=gQZSnbV6_MY&list=PLNoGwCJv2USFR9DVIukjJIZ5yAyFjtEF0&index=12)

##### Cloth Simulation Benchmark

Measures: FPS

Workload:
+ structural + shear + bending springs;
+ drag-effect aerodynamics;
+ 2 way collision with a sphere for CPU, 1 Way for GPU;
+ RK4 and 10 iterations each frame with resolution 1920x1080

The 2 way collision largely reduces the performance of multi-threading due to synchronization.

|Nodes| 15x15 | 20x20 | 30x30 | 50x50 | 60x60 | 200x200 | 300x300 | 500x500 | 700x700 |
|----:|:-----:|:-----:|:-----:|:-----:|:-----:|:-------:|:-------:|:-------:|:-------:|
|CPU  |  543  |  536  |  284  |  72   |    20 |         |         |         |         |
|CUDA |       |       |       |  388  |   318 |   206   |    130  |   67    |    38   |

<img src="./doc/water_gpu.gif" />

1000x1000 with 10 iterations at 303 FPS using CUDA

Video Demo: [click me](https://www.youtube.com/watch?v=Z7B2WFcbmLA&list=PLNoGwCJv2USFR9DVIukjJIZ5yAyFjtEF0&index=15)

##### Shallow Water Benchmark

Measures: FPS; Workload: 10 iterations each frame with resolution 1920x1080

| Cells   | 50x50 | 100x100 | 300x300 | 500x500 | 700x700 | 1000x1000 |1500x1500|2000x2000|2500x2500|
|--------:|:-----:|:-------:|:-------:|:-------:|:-------:|:---------:|:-------:|:-------:|:-------:|
|1 Thread | 730   |   725   |    66   |    18   |         |           |         |         |         |
|4 Threads| 610   |   590   |   140   |    24   |         |           |         |         |         |
|6 Threads| 620   |   587   |   290   |    31   |   15    |           |         |         |         |
|8 Threads| 708   |   683   |   353   |    35   |   18    |           |         |         |         |
|CUDA     |       |         |         |         |         |     303   |   195   |    92   |   39    |



## Usage

	mkdir build
    cd build
    cmake ..
    make
    ./shallow_water
	./cloth

##### Dependencies

  + FreeType
  + stbi\_image (already included in the code)
  + GLM
  + GLEW
  + GLFW 3.2
  + OpenGL 4.3+
  + CUDA
  + OpenMP
  + CBlas


