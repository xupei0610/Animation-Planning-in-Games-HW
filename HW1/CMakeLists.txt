cmake_minimum_required(VERSION 3.2)
project(final)
set(EXE_NAME final)

option(USE_OPENMP "Use OpenMP or not." ON)

# Setup Comipler Flags
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (MSVC)
    set(CMAKE_CXX_FLAGS "/W4 /O2")
else()
    set(CMAKE_CXX_FLAGS "-Wall -O3")
endif()

set(CMAKE_VERBOSE_MAKEFILE TRUE)
set(CMAKE_MACOSX_RPATH 0)

# Setup Dependencies
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
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
if (USE_OPENMP)
    find_package(OpenMP)
    if (OPENMP_FOUND)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
        set(PNET_USE_OPENMP ON)
    else()
        message (STATUS "No OpenMP support found.")
    endif()
endif()
add_subdirectory(${CMAKE_SOURCE_DIR}/lib/assimp/)
include_directories(${CMAKE_SOURCE_DIR}/lib/assimp/include)
include_directories(${CMAKE_SOURCE_DIR}/lib/stb)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/include)
set(SOURCE_LIBRARIES
    ${CMAKE_THREAD_LIBS_INIT}
    ${FREETYPE_LIBRARIES}
    glfw
    ${GLEW_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${COCOA_LIBRARY} ${IOKIT_LIBRARY} ${COREVID_LIBRARY}
    assimp)

# Setup Sources
include_directories(${CMAKE_SOURCE_DIR}/asset)
include_directories(${CMAKE_SOURCE_DIR}/src)
set(SOURCE_DIR ${CMAKE_SOURCE_DIR}/src)
file(GLOB SOURCE_FILES
     ${SOURCE_DIR}/soil/*.c
     ${SOURCE_DIR}/*.cpp
     ${SOURCE_DIR}/shader/*.cpp
     ${SOURCE_DIR}/scene/*.cpp
     ${SOURCE_DIR}/item/*.cpp)

add_executable(${EXE_NAME}
               ${SOURCE_FILES}
               ${SOURCE_DIR}/main.cpp)
target_link_libraries(${EXE_NAME}
                      ${SOURCE_LIBRARIES})


