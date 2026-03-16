cmake_minimum_required(VERSION 3.20)
project(MissileSimApp
    VERSION 0.1.0
    LANGUAGES C CXX
    DESCRIPTION "Missile Flight Simulator — Visualization Application"
)

# ── C++ Standard ──────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Compiler Warnings (per-target, not global) ───────────────
# Applied only to our own code, not to third-party dependencies.
function(apply_project_warnings target)
    target_compile_options(${target} PRIVATE
        -Wall -Wextra -Wpedantic -Wshadow
        -Wnon-virtual-dtor -Wcast-align -Wunused
        -Woverloaded-virtual -Wconversion
        -Wnull-dereference -Wdouble-promotion
    )
endfunction()

# ── Build type default ────────────────────────────────────────
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)

# ══════════════════════════════════════════════════════════════
# DEPENDENCIES
# ══════════════════════════════════════════════════════════════

# ── GLFW (windowing) ──────────────────────────────────────────
set(GLFW_BUILD_DOCS     OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL        OFF CACHE BOOL "" FORCE)

FetchContent_Declare(glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG        3.4
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(glfw)

# ── glad (OpenGL loader) ──────────────────────────────────────
# Uses the system-installed glad (v1) to generate a GL 4.1 Core loader.
# On Ubuntu/Debian: sudo apt install python3-glad  (if not already present)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(GLAD_OUT_DIR "${CMAKE_BINARY_DIR}/generated/glad")
if(NOT EXISTS "${GLAD_OUT_DIR}/include/glad/glad.h")
    message(STATUS "Generating glad GL 4.1 Core loader...")
    execute_process(
        COMMAND ${Python3_EXECUTABLE} -m glad
                --generator c
                --api "gl=4.1"
                --profile core
                --out-path "${GLAD_OUT_DIR}"
        RESULT_VARIABLE GLAD_RESULT
        ERROR_VARIABLE  GLAD_ERROR
        OUTPUT_VARIABLE GLAD_OUTPUT
    )
    if(NOT GLAD_RESULT EQUAL 0)
        message(FATAL_ERROR
            "Failed to generate glad loader.\n"
            "Install glad:  sudo apt install python3-glad\n"
            "Error: ${GLAD_ERROR}")
    endif()
    message(STATUS "glad loader generated in ${GLAD_OUT_DIR}")
endif()

add_library(glad_gl STATIC "${GLAD_OUT_DIR}/src/glad.c")
target_include_directories(glad_gl PUBLIC "${GLAD_OUT_DIR}/include")

# ── GLM (math) ────────────────────────────────────────────────
FetchContent_Declare(glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG        1.0.1
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(glm)

# ── ImGui (UI) ────────────────────────────────────────────────
# ImGui has no CMakeLists.txt, so we build it manually.
FetchContent_Declare(imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.91.8
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(imgui)

add_library(imgui STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
)
target_link_libraries(imgui PUBLIC glfw glad_gl)

# ── stb_image (texture loading) ───────────────────────────────
# Compiled in its own translation unit so project warnings don't apply.
FetchContent_Declare(stb
    GIT_REPOSITORY https://github.com/nothings/stb.git
    GIT_TAG        master
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(stb)

add_library(stb_image STATIC src/vis/stb_image_impl.c)
target_include_directories(stb_image PUBLIC ${stb_SOURCE_DIR})

# ══════════════════════════════════════════════════════════════
# APPLICATION TARGET
# ══════════════════════════════════════════════════════════════

set(APP_SOURCES
    # ── Entry point ───────────────────────────────────────────
    main.cpp

    # ── Application core ──────────────────────────────────────
    src/Application.cpp

    # ── Screens ───────────────────────────────────────────────
    src/screens/MainMenuScreen.cpp

    # ── Vis: shaders ──────────────────────────────────────────
    src/vis/shaders/ShaderProgram.cpp
    src/vis/shaders/EntityShader.cpp

    # ── Vis: rendering ────────────────────────────────────────
    src/vis/render/EntityRenderer.cpp
    src/vis/render/MasterRenderer.cpp

    # ── Vis: loaders / generators ─────────────────────────────
    src/vis/Loader.cpp
    src/vis/SphereGenerator.cpp
)

add_executable(missile_app ${APP_SOURCES})

target_include_directories(missile_app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(missile_app PRIVATE
    glfw
    glad_gl
    glm::glm
    imgui
    stb_image
)

# Apply strict warnings only to our own code.
apply_project_warnings(missile_app)

# ── Copy shader and resource files to build directory ─────────
# This ensures the app can find shaders/ and res/ when run from build/.
add_custom_command(TARGET missile_app POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E create_symlink
        ${CMAKE_CURRENT_SOURCE_DIR}/shaders
        $<TARGET_FILE_DIR:missile_app>/shaders
    COMMAND ${CMAKE_COMMAND} -E create_symlink
        ${CMAKE_CURRENT_SOURCE_DIR}/res
        $<TARGET_FILE_DIR:missile_app>/res
    COMMENT "Symlinking shaders/ and res/ to build directory"
)

# ── Print build info ──────────────────────────────────────────
message(STATUS "═══════════════════════════════════════════════")
message(STATUS "  Missile Flight Simulator App")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "═══════════════════════════════════════════════")