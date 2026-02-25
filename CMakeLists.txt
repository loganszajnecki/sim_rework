cmake_minimum_required(VERSION 3.20)
project(MissileSim
    VERSION 0.1.0
    LANGUAGES CXX
    DESCRIPTION "6-DOF Missile Flight Simulation"
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(
    -Wall -Wextra -Wpedantic -Wshadow
    -Wnon-virtual-dtor -Wcast-align -Wunused
    -Woverloaded-virtual -Wconversion -Wsign-conversion
    -Wnull-dereference -Wdouble-promotion
)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build type" FORCE)
endif()

# ── Core library ──────────────────────────────────────────────
# Template/interface headers are header-only (in include/).
# Concrete model implementations are compiled from src/.
add_library(sim_core
    src/models/USStandard1976.cpp
)
target_include_directories(sim_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# ── Main executable ───────────────────────────────────────────
add_executable(missile_sim src/main.cpp)
target_link_libraries(missile_sim PRIVATE sim_core)

# ── Testing ───────────────────────────────────────────────────
option(BUILD_TESTS "Build unit tests" ON)
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()