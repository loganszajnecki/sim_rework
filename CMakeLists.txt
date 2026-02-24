cmake_minimum_required(VERSION 3.20)
project(MissileSim
    VERSION 0.1.0
    LANGUAGES CXX
    DESCRIPTION "6-DOF Missile Flight Simulation"
)

# C++ Standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler Warnings
add_compile_options(
    -Wall -Wextra -Wpedantic -Wshadow
    -Wnon-virtual-dtor -Wcast-align -Wunused
    -Woverloaded-virtual -Wconversion -Wsign-conversion
    -Wnull-dereference -Wdouble-promotion
)

# Build type defaults
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build type" FORCE)
endif()

# Header-only math/core library (INTERFACE target)
add_library(sim_core INTERFACE)
target_include_directories(sim_core INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Main executable
add_executable(missile_sim src/main.cpp)
target_link_libraries(missile_sim PRIVATE sim_core)

# Testing
option(BUILD_TESTS "Build unit tests" ON)
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()