cmake_minimum_required(VERSION 3.20)
project(MissileSimProject
    VERSION 0.2.0
    LANGUAGES C CXX
    DESCRIPTION "6-DOF Missile Flight Simulation + Visualization"
)

# Shared settings (inherited by both subdirectories)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build type" FORCE)
endif()

# Subdirectories
# Order matters: sim_core must be defined before app links against it.
add_subdirectory(sim)
add_subdirectory(app)