cmake_minimum_required(VERSION 3.1)
project(hw1)
set(EXE_NAME hw1)

option(NFULLSCREEN "not open program in fullscreen mode" ON)

option(SCENE "BENCHMARK" ON)

# Setup Comipler Flags
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (MSVC)
    set(CMAKE_CXX_FLAGS "/W4 /O2")
else()
    set(CMAKE_CXX_FLAGS "-Wall -O3")
endif()

set(MSVC_CL_EXE "C:/Program Files (x86)/Microsoft Visual Studio/2017/Community/VC/Tools/MSVC/14.11.25503/bin/Hostx64/x64/CL.exe")

set(CMAKE_VERBOSE_MAKEFILE TRUE)
set(CMAKE_MACOSX_RPATH 0)

if (SCENE STREQUAL "BOUNCE_BALL")
    message(STATUS "Target Scene: Bounce Ball")
    add_definitions("-DBOUNCE_BALL_SCENE")
elseif (SCENE STREQUAL "WATER_FOUNTAIN")
    message(STATUS "Target Scene: Water Fountain")
    add_definitions("-DWATER_FOUNTAIN_SCENE")
elseif (SCENE STREQUAL "FIRE")
    message(STATUS "Target Scene: Fire")
    add_definitions("-DFIRE_SCENE")
elseif (SCENE STREQUAL "FIREWORKS")
    message(STATUS "Target Scene: Fireworks")
    add_definitions("-DFIREWORKS_SCENE")
elseif (SCENE STREQUAL "GALAXY")
    message(STATUS "Scene: Galaxy")
    add_definitions("-DGALAXY_SCENE")
else()
    message(STATUS "Target Scene: Benchmark")
    add_definitions("-DBENCHMARK_SCENE")
endif()

if (FULLSCREEN)
    add_definitions("-DFULLSCREEN=${FULLSCREEN}")
endif()

# Setup Dependencies
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
find_package(Freetype REQUIRED)
include_directories(${FREETYPE_INCLUDE_DIRS})
find_package(GLM REQUIRED)
find_package(glfw3 3.2 REQUIRED)
find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)
if (APPLE)
    include_directories(/usr/local/include)
    find_library(COCOA_LIBRARY Cocoa REQUIRED)
    find_library(IOKIT_LIBRARY IOKit REQUIRED)
    find_library(COREVID_LIBRARY CoreVideo REQUIRED)
endif()
#if (USE_OPENMP)
    find_package(OpenMP)
    if (OPENMP_FOUND)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
        set(PNET_USE_OPENMP ON)
    else()
        message (STATUS "No OpenMP support found.")
    endif()
#endif()
include_directories(${CMAKE_SOURCE_DIR}/lib/stb)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/include)
set(SOURCE_LIBRARIES
    ${CMAKE_THREAD_LIBS_INIT}
    ${FREETYPE_LIBRARIES}
    glfw
    ${GLEW_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${COCOA_LIBRARY} ${IOKIT_LIBRARY} ${COREVID_LIBRARY})

#if (USE_CUDA)
    find_package(CUDA 5.0.0 REQUIRED)
    include_directories(${CUDA_INCLUDE_DIRS})
    add_definitions("-DUSE_CUDA=${USE_CUDA}")
#endif()

# Setup Sources
include_directories(${CMAKE_SOURCE_DIR}/asset)
include_directories(${CMAKE_SOURCE_DIR}/src)
set(SOURCE_DIR ${CMAKE_SOURCE_DIR}/src)
file(GLOB SOURCE_FILES
     ${SOURCE_DIR}/*.cpp
     ${SOURCE_DIR}/shader/*.cpp
     ${SOURCE_DIR}/item/*.cpp
     ${SOURCE_DIR}/particle_system/*.cpp
     ${SOURCE_DIR}/scene/*.cpp)

file(GLOB CUDA_SOURCE_FILES
     ${SOURCE_DIR}/scene/*.cu)

#if (USE_CUDA)
    if (MSVC)
        set(CUDA_HOST_COMPILER MSVC_CL_EXE)
    endif()
#    set(CUDA_SEPARABLE_COMPILATION ON)
    set(CUDA_NVCC_FLAGS
        -std=c++11
        -use_fast_math
        -restrict
        --ptxas-options=-O3
        -O3
        #        --ptxas-options=-v
        #        -res-usage
        )
    cuda_add_executable(${EXE_NAME}
                        ${SOURCE_FILES}
                        ${CUDA_SOURCE_FILES}
                        ${SOURCE_DIR}/main.cpp)
#else ()
#    add_executable(${EXE_NAME}
#                   ${SOURCE_FILES}
#                   ${SOURCE_DIR}/main.cpp)
#endif ()
target_link_libraries(${EXE_NAME}
                      ${SOURCE_LIBRARIES}
                      ${CUDA_CUDART_LIBRARY})

#set(CMAKE_BUILD_TYPE "Release")
