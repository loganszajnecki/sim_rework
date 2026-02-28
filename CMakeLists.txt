cmake_minimum_required(VERSION 3.20)
project(MissileSim
    VERSION 0.2.0
    LANGUAGES CXX
    DESCRIPTION "6-DOF Missile Flight Simulation"
)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(
    -Wall -Wextra -Wpedantic -Wshadow
    -Wnon-virtual-dtor -Wcast-align -Wunused
    -Woverloaded-virtual -Wconversion
    -Wnull-dereference
)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build type" FORCE)
endif()

# External dependencies

# spdlog — runtime logging (fetched automatically)
include(FetchContent)
FetchContent_Declare(
    spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG        v1.14.1
)
FetchContent_MakeAvailable(spdlog)

# HDF5 — simulation data recording
find_package(HDF5 REQUIRED COMPONENTS CXX)

# ── Core library ──────────────────────────────────────────────
add_library(sim_core
    src/core/EOM.cpp
    src/core/DataRecorder.cpp
    src/models/USStandard1976.cpp
    src/models/FlatEarthGravity.cpp
    src/models/SolidRocketMotor.cpp
    src/models/SimpleAero.cpp
)
target_include_directories(sim_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${HDF5_INCLUDE_DIRS}
)
target_link_libraries(sim_core PUBLIC
    spdlog::spdlog
    ${HDF5_CXX_LIBRARIES}
    ${HDF5_LIBRARIES}
)

# ── Main executable ───────────────────────────────────────────
add_executable(missile_sim src/main.cpp)
target_link_libraries(missile_sim PRIVATE sim_core)

# ── Testing ───────────────────────────────────────────────────
option(BUILD_TESTS "Build unit tests" ON)
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()