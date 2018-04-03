cmake_minimum_required(VERSION 3.1)
project(hw3)

option(NFULLSCREEN "not open program in fullscreen mode" ON)

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

if (FULLSCREEN)
    add_definitions("-DFULLSCREEN=${FULLSCREEN}")
endif()

# Setup Dependencies
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
find_package(Freetype REQUIRED)
include_directories(${FREETYPE_INCLUDE_DIRS})
find_package(CGAL REQUIRED)
find_package(GLM REQUIRED)
find_package(glfw3 3.2 REQUIRED)
find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Threads)
if (APPLE)
    include_directories(/usr/local/include)
    find_library(COCOA_LIBRARY Cocoa REQUIRED)
    find_library(IOKIT_LIBRARY IOKit REQUIRED)
    find_library(COREVID_LIBRARY CoreVideo REQUIRED)
endif()
find_package(OpenMP)
if (OPENMP_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(PNET_USE_OPENMP ON)
else()
    message (STATUS "No OpenMP support found.")
endif()
include_directories(${CMAKE_SOURCE_DIR}/lib/stb)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/include)
set(SOURCE_LIBRARIES
    ${CMAKE_THREAD_LIBS_INIT}
    CGAL
    ${FREETYPE_LIBRARIES}
    glfw
    ${GLEW_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    ${COCOA_LIBRARY} ${IOKIT_LIBRARY} ${COREVID_LIBRARY})

# Setup Sources
include_directories(${CMAKE_SOURCE_DIR}/asset)
include_directories(${CMAKE_SOURCE_DIR}/src)
set(SOURCE_DIR ${CMAKE_SOURCE_DIR}/src)
file(GLOB SOURCE_FILES
     ${SOURCE_DIR}/*.cpp
     ${SOURCE_DIR}/shader/*.cpp
     ${SOURCE_DIR}/item/*.cpp
     ${SOURCE_DIR}/scene/*.cpp)
file(GLOB CUDA_SOURCE_FILES
     ${SOURCE_DIR}/scene/*.cu)

add_executable(roadmap
               ${SOURCE_FILES}
               ${SOURCE_DIR}/main.cpp)
add_executable(vo
               ${SOURCE_FILES}
               ${SOURCE_DIR}/main.cpp)
add_executable(ttc
               ${SOURCE_FILES}
               ${SOURCE_DIR}/main.cpp)
add_executable(boids
               ${SOURCE_FILES}
               ${SOURCE_DIR}/main.cpp)

target_compile_definitions(roadmap PRIVATE
                           ROADMAP_SCENE)
target_compile_definitions(vo PRIVATE
                           VO_SCENE)
target_compile_definitions(ttc PRIVATE
                           TTC_SCENE)
target_compile_definitions(boids PRIVATE
                           BOIDS_SCENE)

target_link_libraries(roadmap
                      ${SOURCE_LIBRARIES})
target_link_libraries(vo
                      ${SOURCE_LIBRARIES})
target_link_libraries(vo
                      ${SOURCE_LIBRARIES})
target_link_libraries(ttc
                      ${SOURCE_LIBRARIES})
target_link_libraries(boids
                      ${SOURCE_LIBRARIES})

#set(CMAKE_BUILD_TYPE "Release")

