# Particle System

This is for HW1 of the course CSCI 5611 Animation and Motion Planning in Games, UMN.

This is bunch of programs implementing particle systems.

**Apply one day more to write README file.**

Too many features need to be documented. 

## Features

### Basic Features

Implementation of Bounce Ball, Fire, Fireworks, Water Fountain.

### Simulation

Fire, Smoke Simulation using Curl Noise

Water Simulation using the method of Smoothed Particle Hydrodynamics

N-body Galaxy Simulation (10,000 particles, 30 FPS)

### User Interface

3D Implementation of all scenes

3D User Controlled Camera

Real-time User Interaction with systems (pause/resume, control acceleration, adjust the amount of particles and ...)

Multiple Particle Systems in one scence

### Hardcore Technology

SIMD Implementation using Compute Shader and CUDA (see benchmark part)

Thread-Parallel Implementation using OpenMP

### Benchmark

500, 1000, 10,000, 100,000, 1,000,000, 3,000,000 Particles Benchmark for methods of TransformFeedback, Compute Shader, CUDA and CPU Implementation, and for Instancing Draw and Geometry Shader.

N-body Simulation Benchmark for Compute Shader and CUDA (CUDA win)


### Trivial Tricks

Textured and Translucent Particles

Particle Trails

Collision Detection with external obstcales (no collision detection among particles)

Snow

<img src="doc/demo.png" />
