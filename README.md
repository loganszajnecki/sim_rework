**6-DOF Missile Flight Simulation + Visualization Application - Architecture Summary**

---

## Project Structure

```
sim_rework/
├── CMakeLists.txt              <- Top-level: adds both subdirectories
├── .vscode/
│   └── settings.json           <- Unified IntelliSense (compile_commands.json)
├── sim/                        <- Simulation library + CLI executable
│   ├── CMakeLists.txt          <- Builds sim_core library + missile_sim executable
│   ├── include/
│   │   ├── math/               <- Vector3, Matrix3, Quaternion, Rotations (templates, header-only)
│   │   ├── core/               <- State, Integrator, Vehicle, EOM, SimConfig, ConfigParser,
│   │   │                          VehicleFactory, SimRunner, DataRecorder, Logger
│   │   └── models/             <- 10 abstract interfaces (I*.hpp) + 12 concrete implementations
│   ├── src/
│   │   ├── main.cpp            <- CLI entry point
│   │   ├── core/               <- EOM.cpp, ConfigParser.cpp, VehicleFactory.cpp, SimRunner.cpp, DataRecorder.cpp
│   │   └── models/             <- All concrete model .cpp files
│   ├── tests/                  <- Google Test cases
│   ├── inputs/
│   │   ├── default.xml         <- Unguided ballistic flight
│   │   └── guided.xml          <- Guided intercept against constant-velocity target
│   └── scripts/
│       └── plot_results.py     <- Python HDF5 reader + matplotlib validation plots
└── app/                        <- Visualization application (OpenGL + ImGui)
    ├── CMakeLists.txt          <- Links against sim_core + graphics dependencies
    ├── main.cpp                <- Entry point: creates Application, registers screens
    ├── include/
    │   ├── Application.hpp     <- Window + GL context + ImGui + screen manager + fade transitions
    │   ├── Screen.hpp          <- Abstract screen interface (enter/update/render/leave)
    │   ├── ImGuiConsoleSink.hpp<- Custom spdlog sink for ImGui console display
    │   ├── screens/
    │   │   ├── MainMenuScreen.hpp      <- Landing page (rotating Earth, starfield)
    │   │   ├── WorkspaceScreen.hpp     <- Dockable IDE workspace (viewport, properties, console)
    │   │   └── PlaceholderScreen.hpp   <- Stub screen for unbuilt screens
    │   └── vis/
    │       ├── Camera.hpp              <- Perspective camera (view/projection matrices)
    │       ├── Framebuffer.hpp         <- RAII FBO for offscreen rendering into ImGui panels
    │       ├── Loader.hpp              <- VAO/VBO/texture resource manager
    │       ├── SphereGenerator.hpp     <- Procedural UV sphere mesh
    │       ├── entities/
    │       │   ├── Entity.hpp          <- Renderable instance (transform + TexturedModel pointer)
    │       │   └── Light.hpp           <- Point/directional light
    │       ├── models/
    │       │   ├── RawModel.hpp        <- Mesh handle (VAO + index count)
    │       │   ├── ModelTexture.hpp    <- Material/texture descriptor
    │       │   └── TexturedModel.hpp   <- Mesh + material pair
    │       ├── render/
    │       │   ├── EntityRenderer.hpp  <- Batched textured entity drawing
    │       │   ├── MasterRenderer.hpp  <- High-level entity rendering orchestration
    │       │   ├── StarfieldRenderer.hpp <- Procedural starfield (point sprites)
    │       │   └── GridRenderer.hpp    <- Ground grid + coordinate axes
    │       └── shaders/
    │           ├── ShaderProgram.hpp   <- RAII base class for GL shader programs
    │           ├── EntityShader.hpp    <- Phong lighting shader
    │           ├── StarfieldShader.hpp <- Point sprite star shader
    │           └── LineShader.hpp      <- Colored line shader (grid, axes, trajectories)
    ├── src/
    │   ├── Application.cpp
    │   ├── screens/
    │   │   ├── MainMenuScreen.cpp
    │   │   └── WorkspaceScreen.cpp
    │   └── vis/
    │       ├── Loader.cpp
    │       ├── SphereGenerator.cpp
    │       ├── Framebuffer.cpp
    │       ├── stb_image_impl.c        <- Isolated stb_image compilation unit
    │       ├── shaders/
    │       │   ├── ShaderProgram.cpp
    │       │   ├── EntityShader.cpp
    │       │   ├── StarfieldShader.cpp
    │       │   └── LineShader.cpp
    │       └── render/
    │           ├── EntityRenderer.cpp
    │           ├── MasterRenderer.cpp
    │           ├── StarfieldRenderer.cpp
    │           └── GridRenderer.cpp
    ├── shaders/                        <- GLSL files
    │   ├── entity.vert / .frag         <- Phong lighting + texture
    │   ├── starfield.vert / .frag      <- Point sprite stars
    │   └── line.vert / .frag           <- Colored lines
    ├── res/
    │   └── earth_daymap.jpg            <- NASA Blue Marble texture
    ├── third_party/
    │   └── tinyfiledialogs/            <- Vendored native file dialog library
    │       ├── tinyfiledialogs.c
    │       └── tinyfiledialogs.h
    └── scripts/
        └── fetch_earth_texture.py      <- Downloads Earth texture from NASA
```

---

## Build System

Unified CMake build from project root:
```bash
cd sim_rework
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

Produces:
- `build/missile_sim/missile_sim` - CLI simulation executable
- `build/app/missile_app` - visualization application
- `build/compile_commands.json` - unified IntelliSense for both

Tests: `cd build && ./sim/tests/sim_tests`

Three CMakeLists.txt files:
- **Top-level** - sets C++20, build type, compile_commands; adds both subdirectories
- **missile_sim/** - builds `sim_core` static library + `missile_sim` executable; fetches spdlog, pugixml; finds system HDF5
- **app/** - builds `missile_app` executable; links against `sim_core`; fetches GLFW, glad (generated via python3-glad), GLM, ImGui (docking branch), stb_image; vendors tinyfiledialogs

Warning flags are per-target via `apply_sim_warnings()` and `apply_app_warnings()` functions - third-party code compiles without project warnings.

---

## Simulation Architecture (sim::core / sim::models)

**Namespaces:**
- `sim::math` - Vector3<T>, Matrix3<T>, Quaternion<T>, EulerAngles<T>, Rotations. Type aliases: Vec3d, Mat3d, Quatd. Row-major matrices. Quaternion `rotate()` = body-to-inertial. Euler order: 3-2-1 (yaw-pitch-roll), radians.
- `sim::core` - State (13-element: position NED, velocity body, Euler angles, body rates, mass), StateDerivative, Integrator (RK4 with C++20 concepts), Vehicle (aggregates all models via unique_ptr, Builder pattern), EOM (static compute method), SimConfig/ConfigParser (XML via pugixml, load + save + save_template), VehicleFactory (maps config strings to concrete classes), SimRunner (encapsulates full sim loop, returns SimResult), DataRecorder (HDF5 with run groups), Logger (spdlog wrapper with SIM_* macros).
- `sim::models` - All physics models behind abstract interfaces.

**Coordinate frames:**
- NED inertial: +X=North, +Y=East, +Z=Down
- Body: +X=nose, +Y=starboard, +Z=ventral
- Gravity in NED: [0, 0, +g]
- Thrust along body +X (forward)

**Model interfaces and concrete implementations:**
| Interface | Concrete | Notes |
|---|---|---|
| IAtmosphere | USStandard1976 | 7 layers, geometric altitude input |
| IGravity | ConstantGravity | Configurable g, default 9.80665 |
| IPropulsion | SolidRocketMotor | Constant thrust, linear mass depletion, full Mat3d inertia |
| IAerodynamics | SimpleAero | Constant coefficients: CA, CN_alpha, CY_beta, Cm_alpha, Cm_delta, Cn_beta, Cn_delta, Cl_delta, Cmq, Cnr, Clp |
| ITarget | StationaryTarget, ConstantVelocityTarget, ManeuveringTarget | Maneuvering: constant-g lateral turn |
| ISeeker | IdealSeeker | Perfect LOS geometry |
| IGuidance | ProNavGuidance | True PN: a_cmd = N x Vc x (OMEGA_los × uhat_los) |
| IAutopilot | SimpleAutopilot | Proportional + rate damping |
| IActuator | FirstOrderActuator | delta_dot= (delta_cmd - delta)/tau, rate + position limits |
| IWind | (none yet) | Interface exists, placeholder in EOM |

**EOM signal flow:**
1. Atmosphere -> density, speed of sound -> Mach, qbar
2. Gravity (NED) -> rotate to body
3. Propulsion -> thrust force/moment, mass flow, inertia
4. Actuator -> current fin deflections (if guided, else zero)
5. Aerodynamics -> body forces and moments
6. Translational kinematics: ṗ_NED = L_bi × V_body
7. Translational dynamics: V̇_body = g_body + (F_thrust + F_aero)/m - ω×V
8. Rotational kinematics: Euler_dot = H(φ,θ) × [p,q,r]
9. Rotational dynamics: ω̇ = I⁻¹(M_total - ω×(Iω))
10. Mass: ṁ = mass_flow_rate

**SimRunner:**
`SimResult SimRunner::run(SimConfig, DataRecorder*, run_id)` - builds Vehicle and State from config, runs the integration loop, handles three termination conditions: `closest_approach`, `ground_impact`, `timeout`. Returns SimResult with miss_distance, POCA time, impact speed, max altitude/speed/Mach. Supports `set_verbose(false)` and `set_logging(false)` for silent batch runs.

**ConfigParser:**
- `ConfigParser::load(filepath)` -> `SimConfig` (parses XML)
- `ConfigParser::save(cfg, filepath)` -> writes SimConfig to XML (round-trip compatible with load)
- `ConfigParser::save_template(filepath)` -> writes default XML template

**HDF5 output:** Each run writes to a group (run_0000, run_0001, ...) containing datasets: time, position[Nx3], velocity_body[Nx3], euler[Nx3], omega_body[Nx3], mass, alpha, beta, speed, mach, altitude, qbar. Attributes store config metadata, target parameters, and run results.

**Validated scenarios:**
- Unguided ballistic: 45 degree launch, 300 m/s, impacts at 35s/3581m range, 0.016% error vs analytic
- Guided intercept: PN against constant-velocity target (5km, -100 m/s), 0.034m miss at 220 m/s closing, 14.8s flight time

---

## Application Architecture (app:: / vis::)

**Screen System:**
The application uses a screen-based state machine. `Application` owns the GLFW window, OpenGL context, and ImGui context. It delegates frame logic to the active `Screen`. Screens are registered by name and transitions happen via `nextScreen()` return values with smooth fade-to-black animations (0.4s per half-transition, smooth-step easing).

| Screen | Description |
|---|---|
| MainMenuScreen | Landing page: rotating Earth, starfield, ImGui menu buttons |
| WorkspaceScreen | Dockable IDE: 3D viewport, properties panel, console, menu bar |
| PlaceholderScreen | Stub for unbuilt screens (viewer, settings) |

**Application class:**
- Creates GLFW window + OpenGL 4.1 context + ImGui (docking-enabled)
- Custom dark/military ImGui theme
- Screen registration, switching, and fade transitions
- Frame loop: poll events -> ImGui new frame -> update -> render -> fade overlay -> ImGui render -> swap

**MainMenuScreen:**
- Slowly rotating Earth globe (procedural UV sphere, NASA Blue Marble texture, Phong lighting, 23.4° axial tilt)
- Starfield background (4,000 procedural stars, point sprites, additive blending, power-curve brightness distribution)
- ImGui overlay: "New Simulation", "Load Results", "Settings", "Exit"

**WorkspaceScreen (the simulation IDE):**
- Full-screen ImGui DockSpace with programmatic default layout via DockBuilder API
- Menu bar: File (New/Open/Save/SaveAs/Exit), Edit, View (toggle panels, reset layout), Simulation (Run/Monte Carlo), Help
- **3D Viewport panel:** Renders to offscreen FBO (`Framebuffer` class), displayed as `ImGui::Image`. Ground grid + colored axes (X=red, Y=green, Z=blue). Camera at isometric angle. FBO resizes dynamically with panel.
- **Properties panel:** Full GUI editor for every field in `SimConfig`. Editable input fields for all doubles, combo dropdowns for model type selection, Vec3d editors for position/velocity, GNC enable/disable checkbox, dynamic target list with add/remove, inertia tensor editor with aerospace sign convention. Dirty indicator (`*`) in title.
- **Console panel:** Unified spdlog sink (`ImGuiConsoleSink`). All SIM_INFO/WARN/ERROR from the sim and app appear here with color-coded levels (gray=trace, blue=debug, white=info, yellow=warn, red=error). Filter toggles, auto-scroll, clear button, 5000-entry rolling history.

**SimConfig ↔ UI flow:**
- File -> New: creates default `SimConfig{}`
- File -> Open: native OS file dialog (tinyfiledialogs) -> `ConfigParser::load()` -> populates Properties
- File -> Save: `ConfigParser::save()` to last-used path
- File -> Save As: native save dialog -> `ConfigParser::save()` to chosen path
- Properties panel: direct two-way binding to `SimConfig` member fields
- All round-trip tested: save -> load preserves all values

**Simulation execution:**
- Simulation -> Run (or F5): launches `SimRunner::run()` on a `std::thread`
- Config is **copied** into the thread - UI remains fully interactive during the run
- `set_verbose(false)` suppresses stdout; `set_logging(true)` streams spdlog to console
- `SimResult` is stored via mutex-protected `std::optional` and picked up by the main thread
- Results summary displayed in Console: termination reason, flight time, miss distance, impact speed, max altitude/speed/Mach, HDF5 path
- Run menu is disabled while sim is running; ESC blocked during execution

**Rendering pipeline:**
Entity rendering (used by MainMenuScreen):
1. MasterRenderer - per-frame orchestration (projection + view + light uniforms, batch dispatch)
2. EntityRenderer - bind model VAO/texture, upload transform, glDrawElements
3. EntityShader - Phong lighting with texture sampling (derives from ShaderProgram)

All shaders derive from `ShaderProgram` base class (RAII, compile/link/destroy, uniform helpers). Derived shaders implement `bindAttributes()` and `getAllUniformLocations()`. Current shaders: EntityShader, StarfieldShader, LineShader.

`Loader` manages all GL resource lifetime (VAOs, VBOs, textures). Tracks every allocated object and cleans up in destructor.

**Coordinate system:**
- Rendering world: Z-up (+X right, +Y forward, +Z up)
- Simulation: NED (+X North, +Y East, +Z Down)
- Conversion between frames needed when sim data flows into the viewport (Phase C)

**Dependencies:**
- sim_core: spdlog (FetchContent), pugixml (FetchContent), HDF5 (system), Google Test (FetchContent)
- app: GLFW (FetchContent), glad (generated via python3-glad v1), GLM (FetchContent), ImGui docking branch (FetchContent), stb_image (FetchContent, isolated .c compilation), tinyfiledialogs (vendored)

**Key design decisions:**
- Private data members use trailing underscore
- Third-party code compiles as separate static libraries (no project warnings applied)
- `stb_image_impl.c` isolates stb_image implementation from project warning flags
- ImGui docking branch required (tagged releases don't include DockSpace/DockBuilder)
- `imgui_internal.h` wrapped with `#pragma GCC diagnostic` to suppress `-Wconversion`
- FBO rendering must happen **inside** the viewport panel method (after resize, before ImGui::Image) to avoid displaying a blank recreated texture

---

## Development Phases

**Complete:**
- Phase A: Application shell + landing page (window, GL context, ImGui, screen management, fade transitions, Earth globe, starfield)
- Phase B1: Workspace foundation (dockable layout, menu bar, FBO viewport, grid/axes, properties panel skeleton, console panel)
- Phase B2: Properties panel wired to SimConfig (all editable fields, file open/save with native dialogs, dirty tracking, GNC toggle, dynamic targets)
- Phase B3: Simulation execution (background thread, live console streaming, SimResult display, F5 shortcut)

**Next:**
- Phase C: Trajectory visualization (load HDF5 -> draw 3D trajectory lines in viewport, target path, playback controls, NED->Z-up coordinate conversion)
- Phase D: Text editor panel + vehicle geometry (STL/OBJ models, integrated C++ editor for model code, compile-and-reload workflow)
- Phase E: Monte Carlo (dispersions, batch execution, scatter plots, miss distance statistics)
- Phase F: Load Results (HDF5 -> workspace, resume editing)

**What's not built yet (sim side):** Monte Carlo dispatcher, wind model, realistic seeker (RF/IR), three-loop autopilot, tabulated aerodynamics, WGS-84 gravity, Dryden turbulence. All interface slots exist - each is a new concrete class behind an existing interface.