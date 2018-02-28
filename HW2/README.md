# Physically Based Simulation

This is the program for HW2 of the course CSCI 5611, UMN.

## Features

##### Implementation

+ 3D Cloth Simulation using Mass-Spring Model
+ 2D Shallow Water

##### Cloth Simulation

+ CUDA Implementation
+ 200x200 with 10 iterations at ~270 FPS using RK4 and CUDA
+ 50x50 with 10 iterations at ~70 FPS using RK4 on CPU, ~370 FPS using Semi-implicit Euler
+ 1st-Order Integrator: Euler, Semi-implicit Euler
+ Higher-Order Integrator: Verlet, Velocity Verlet, Midpoint, RK4
+ Textured Cloth
+ Two-Way Coupling Object Interaction between ball and cloth for CPU implementation
+ One-Way Cloth-Sphere Interaction for GPU implementation
+ Wind Component of Drag Term

##### 2D Shallow Water Simulation

+ Thread-parallel Implementaion
+ CUDA Implementation
+ 1000x1000 with 10 iterations at ~500 FPS using CUDA
+ 500x500 with 10 iterations at ~38 FPS on CPU with OpenMP
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

Video demo: [click me](https://www.youtube.com/watch?v=k3QOb02JP5E&list=PLNoGwCJv2USFR9DVIukjJIZ5yAyFjtEF0&index=13)

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

200x200 with RK4 and 10 iterations at ~270 FPS using CUDA v.s. ~70 FPS on CPU

Video Demo: [click me](https://www.youtube.com/watch?v=gQZSnbV6_MY&list=PLNoGwCJv2USFR9DVIukjJIZ5yAyFjtEF0&index=12)

<img src="./doc/water_gpu.gif" />

1000x1000 with 10 iterations at ~500 FPS using CUDA

Video Demo: [click me](https://www.youtube.com/watch?v=uCJPIle3jkc&list=PLNoGwCJv2USFR9DVIukjJIZ5yAyFjtEF0&index=14)


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
  + CBlas (any)


