
#ifndef COMPUTE_SHADER_WORK_GROUP_SIZE
#define COMPUTE_SHADER_WORK_GROUP_SIZE 256
#endif

#ifndef WATER_FOUNTAIN_MAX_PARTICLES
#define WATER_FOUNTAIN_MAX_PARTICLES 100000
#endif

#ifndef FIREWORKS_MAX_PARTICLES // for each firework
#define FIREWORKS_MAX_PARTICLES 10000
#endif
#ifndef FIREWORKS_TRAIL_RATE // number of particles left as trail each second
#define FIREWORKS_TRAIL_RATE 50
#endif
#ifndef FIREWORKS_AMOUNT
#define FIREWORKS_AMOUNT 6
#endif

#ifndef FIRE_MAX_PARTICLES
#define FIRE_MAX_PARTICLES 60000
#endif

#ifndef SNOW_MAX_PARTICLES
#define SNOW_MAX_PARTICLES 1000
#endif

#ifndef GALAXY_MAX_PARTICLES_CUDA
#define GALAXY_MAX_PARTICLES_CUDA 100000
#endif
#ifndef GALAXY_MAX_PARTICLES_COMPUTE_SHADER
#define GALAXY_MAX_PARTICLES_COMPUTE_SHADER 16000
#endif
#ifndef GALAXY_INIT_RADIUS
#define GALAXY_INIT_RADIUS 4
#endif

#ifndef BENCHMARK_OMP_THREADS
#define BENCHMARK_OMP_THREADS 8
#endif
#ifndef BENCHMARK_MAX_PARTICLES_SIMPLE_INSTANCING
#define BENCHMARK_MAX_PARTICLES_SIMPLE_INSTANCING 50000
#endif
#ifndef BENCHMARK_MAX_PARTICLES_SIMPLE_GEOMETRY_SHADER
#define BENCHMARK_MAX_PARTICLES_SIMPLE_GEOMETRY_SHADER 50000
#endif
#ifndef BENCHMARK_MAX_PARTICLES_TRANSFORM_FEEDBACK
#define BENCHMARK_MAX_PARTICLES_TRANSFORM_FEEDBACK 300000
#endif
#ifndef BENCHMARK_MAX_PARTICLES_CUDA
#define BENCHMARK_MAX_PARTICLES_CUDA 3000000
#endif
#ifndef BENCHMARK_MAX_PARTICLES_COMPUTE_SHADER
#define BENCHMARK_MAX_PARTICLES_COMPUTE_SHADER 3000000
#endif