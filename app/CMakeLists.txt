# ==============================================================
# Missile Flight Simulator — Visualization Application
# ==============================================================
# Built as a subdirectory of the top-level project.
# Inherits C++20, build type, and compile_commands from parent.
# Links against sim_core (from missile_sim/) for simulation access.

# Compiler Warnings (per-target)
function(apply_app_warnings target)
    target_compile_options(${target} PRIVATE
        -Wall -Wextra -Wpedantic -Wshadow
        -Wnon-virtual-dtor -Wcast-align -Wunused
        -Woverloaded-virtual -Wconversion
        -Wnull-dereference -Wdouble-promotion
    )
endfunction()

include(FetchContent)

# ==============================================================
# GRAPHICS DEPENDENCIES (not needed by the sim)
# ==============================================================

# GLFW (windowing)
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

# glad (OpenGL loader)
# Uses the system-installed glad (v1) to generate a GL 4.1 Core loader.
# On Ubuntu/Debian: sudo apt install python3-glad
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

# GLM (math)
FetchContent_Declare(glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG        1.0.1
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(glm)

# ImGui (UI)
FetchContent_Declare(imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        docking
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

# stb_image (texture loading)
FetchContent_Declare(stb
    GIT_REPOSITORY https://github.com/nothings/stb.git
    GIT_TAG        master
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(stb)

add_library(stb_image STATIC src/vis/stb_image_impl.c)
target_include_directories(stb_image PUBLIC ${stb_SOURCE_DIR})

# ── tinyfiledialogs (native file dialogs) ─────────────────────
# Vendored from https://sourceforge.net/projects/tinyfiledialogs/
add_library(tinyfiledialogs STATIC third_party/tinyfiledialogs/tinyfiledialogs.c)
target_include_directories(tinyfiledialogs PUBLIC third_party/tinyfiledialogs)

# ==============================================================
# APPLICATION TARGET
# ==============================================================

set(APP_SOURCES
    # == Entry point ===========================================
    main.cpp

    # == Application core ======================================
    src/Application.cpp

    # == Screens ===============================================
    src/screens/MainMenuScreen.cpp
    src/screens/WorkspaceScreen.cpp

    # == Vis: shaders ==========================================
    src/vis/shaders/ShaderProgram.cpp
    src/vis/shaders/EntityShader.cpp
    src/vis/shaders/StarfieldShader.cpp
    src/vis/shaders/LineShader.cpp

    # == Vis: rendering ========================================
    src/vis/render/EntityRenderer.cpp
    src/vis/render/MasterRenderer.cpp
    src/vis/render/StarfieldRenderer.cpp
    src/vis/render/GridRenderer.cpp

    # == Vis: loaders / generators =============================
    src/vis/Loader.cpp
    src/vis/SphereGenerator.cpp
    src/vis/Framebuffer.cpp
)

add_executable(missile_app ${APP_SOURCES})

target_include_directories(missile_app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(missile_app PRIVATE
    sim_core        # simulation library (SimConfig, SimRunner, VehicleFactory, etc.)
    glfw
    glad_gl
    glm::glm
    imgui
    stb_image
    tinyfiledialogs
)

apply_app_warnings(missile_app)

# Symlink shaders/ and res/ into the build directory
add_custom_command(TARGET missile_app POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E create_symlink
        ${CMAKE_CURRENT_SOURCE_DIR}/shaders
        $<TARGET_FILE_DIR:missile_app>/shaders
    COMMAND ${CMAKE_COMMAND} -E create_symlink
        ${CMAKE_CURRENT_SOURCE_DIR}/res
        $<TARGET_FILE_DIR:missile_app>/res
    COMMENT "Symlinking shaders/ and res/ to build directory"
)

message(STATUS "═══════════════════════════════════════════════")
message(STATUS "  Missile Flight Simulator App")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "═══════════════════════════════════════════════")