# ==============================================================
# 6-DOF Missile Flight Simulation — Core Library + CLI
# ==============================================================
# Built as a subdirectory of the top-level project.
# Inherits C++20, build type, and compile_commands from parent.

# Compiler Warnings (per-target)
function(apply_sim_warnings target)
    target_compile_options(${target} PRIVATE
        -Wall -Wextra -Wpedantic -Wshadow
        -Wnon-virtual-dtor -Wcast-align -Wunused
        -Woverloaded-virtual -Wconversion
        -Wnull-dereference -Wdouble-promotion
    )
endfunction()

# External dependencies

include(FetchContent)

# spdlog - runtime logging
FetchContent_Declare(
    spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG        v1.14.1
)
FetchContent_MakeAvailable(spdlog)
# Enable debug/trace log macros in debug builds
target_compile_definitions(spdlog INTERFACE
    $<$<CONFIG:Debug>:SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_DEBUG>
    $<$<CONFIG:Release>:SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_INFO>
)

# pugixml - XML configuration parsing
FetchContent_Declare(
    pugixml
    GIT_REPOSITORY https://github.com/zeux/pugixml.git
    GIT_TAG        v1.14
)
FetchContent_MakeAvailable(pugixml)

# HDF5 - simulation data recording (system install)
find_package(HDF5 REQUIRED COMPONENTS CXX)

# Core library
add_library(sim_core
    src/core/EOM.cpp
    src/core/DataRecorder.cpp
    src/core/ConfigParser.cpp
    src/core/VehicleFactory.cpp
    src/core/SimRunner.cpp
    src/models/USStandard1976.cpp
    src/models/FlatEarthGravity.cpp
    src/models/SolidRocketMotor.cpp
    src/models/SimpleAero.cpp
    src/models/StationaryTarget.cpp
    src/models/ConstantVelocityTarget.cpp
    src/models/ManeuveringTarget.cpp
    src/models/IdealSeeker.cpp
    src/models/ProNavGuidance.cpp
    src/models/SimpleAutopilot.cpp
    src/models/FirstOrderActuator.cpp
)
target_include_directories(sim_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${HDF5_INCLUDE_DIRS}
)
target_link_libraries(sim_core PUBLIC
    spdlog::spdlog
    pugixml::pugixml
    ${HDF5_CXX_LIBRARIES}
    ${HDF5_LIBRARIES}
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

apply_sim_warnings(sim_core)
apply_sim_warnings(missile_sim)