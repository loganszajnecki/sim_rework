# Missile Flight Simulator — Visualization Application

## Quick Start

```bash
# 1. Fetch the Earth texture (one-time, requires internet)
python3 scripts/fetch_earth_texture.py

# 2. Configure and build
cd app
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)

# 3. Run (from the build directory so shaders/ symlink resolves)
cd build
./missile_app
```

## Prerequisites

- **CMake** >= 3.20
- **C++20** compiler (GCC 11+, Clang 14+)
- **Python 3** — required at configure time by glad2 to generate the OpenGL loader
- **OpenGL 4.1** capable GPU and drivers

All other dependencies (GLFW, glad, GLM, ImGui, stb_image) are fetched
automatically via CMake FetchContent.

## Architecture

```
app/
├── main.cpp                          Entry point
├── include/
│   ├── Application.hpp               Window + GL + ImGui + screen manager
│   ├── Screen.hpp                    Abstract screen interface
│   ├── screens/
│   │   └── MainMenuScreen.hpp        Landing page (rotating Earth)
│   └── vis/
│       ├── Camera.hpp                Perspective camera
│       ├── Loader.hpp                VAO/VBO/texture resource manager
│       ├── SphereGenerator.hpp       Procedural UV sphere mesh
│       ├── entities/                 Entity, Light
│       ├── models/                   RawModel, ModelTexture, TexturedModel
│       ├── render/                   EntityRenderer, MasterRenderer
│       └── shaders/                  ShaderProgram (base), EntityShader
├── src/                              Corresponding .cpp files
├── shaders/                          GLSL vertex/fragment shaders
├── res/                              Textures, OBJ models
└── scripts/                          Setup utilities
```

### Screen System

The application uses a screen-based state machine. `Application` owns the
GLFW window and ImGui context, and delegates frame logic to the active
`Screen`. Screens are registered by name and transitions happen via
`nextScreen()` return values.

| Screen          | Phase | Description                            |
|-----------------|-------|----------------------------------------|
| MainMenuScreen  | A     | Landing page with rotating Earth       |
| ConfigScreen    | B     | Vehicle/target/sim configuration GUI   |
| ViewerScreen    | C     | 3D trajectory replay + real-time view  |

### Rendering Pipeline

The entity rendering pipeline is layered:

1. **MasterRenderer** — per-frame orchestration (clear, set uniforms, batch)
2. **EntityRenderer** — bind model VAO/texture, upload transforms, draw
3. **EntityShader** — Phong lighting with texture sampling
4. **ShaderProgram** — RAII base class for GL shader programs

Entities reference shared `TexturedModel` objects (non-owning pointer),
allowing many instances of the same mesh/texture at different transforms.

### Coordinate System

- Rendering world: **Z-up** (+X right, +Y forward, +Z up)
- Simulation: **NED** (+X North, +Y East, +Z Down)
- Conversion between frames will be handled in later phases when sim data
  flows into the viewer.

## Controls (Main Menu)

- **ESC** — Exit
- Mouse/keyboard — ImGui buttons for navigation

## Troubleshooting

**Black screen / no Earth:**
Run `scripts/fetch_earth_texture.py` to download the Earth texture.
Without it, a solid blue sphere is used as fallback.

**Shader compile errors:**
Make sure you're running from a directory where `shaders/entity.vert`
is accessible. The build system creates a symlink from the build dir.

**glad2 Python error at configure time:**
glad2 requires Python 3 with the `jinja2` module. Install it:
```bash
pip install jinja2
```