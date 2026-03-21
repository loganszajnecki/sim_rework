#include "screens/WorkspaceScreen.hpp"

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#include <imgui_internal.h>   // for DockBuilder API
#pragma GCC diagnostic pop

#include "core/Logger.hpp"    // sim logger — SIM_INFO, etc.
#include "core/ConfigParser.hpp"

#include <tinyfiledialogs.h>

#include <algorithm>
#include <cmath>
#include <cstring>

namespace app {

    // Panel ID constants
    // These must match the names passed to ImGui::Begin() for each panel.
    static const char* kViewportPanel   = "3D Viewport";
    static const char* kPropertiesPanel = "Properties";
    static const char* kConsolePanel    = "Console";
    static const char* kDockSpaceName   = "WorkspaceDockSpace";

    // Log level constants
	
    static ImVec4 levelColor(spdlog::level::level_enum level) {
        switch (level) {
            case spdlog::level::trace:    return {0.5f, 0.5f, 0.5f, 1.0f};
            case spdlog::level::debug:    return {0.6f, 0.7f, 0.8f, 1.0f};
            case spdlog::level::info:     return {0.8f, 0.9f, 1.0f, 1.0f};
            case spdlog::level::warn:     return {1.0f, 0.85f, 0.3f, 1.0f};
            case spdlog::level::err:      return {1.0f, 0.4f, 0.4f, 1.0f};
            case spdlog::level::critical: return {1.0f, 0.2f, 0.2f, 1.0f};
            default:                      return {1.0f, 1.0f, 1.0f, 1.0f};
        }
    }
    static bool shouldShow(spdlog::level::level_enum level,
                        bool trace, bool debug, bool info,
                        bool warn, bool error, bool critical) {
        switch (level) {
            case spdlog::level::trace:    return trace;
            case spdlog::level::debug:    return debug;
            case spdlog::level::info:     return info;
            case spdlog::level::warn:     return warn;
            case spdlog::level::err:      return error;
            case spdlog::level::critical: return critical;
            default:                      return true;
        }
    }

    // ImGui helpers for SimConfig editing

    /// Edit a double field. Returns true if value changed.
    static bool editDouble(const char* label, double* val, const char* fmt = "%.6g") {
        float width = ImGui::GetContentRegionAvail().x * 0.55f;
        ImGui::SetNextItemWidth(width);
        return ImGui::InputScalar(label, ImGuiDataType_Double, val, nullptr, nullptr, fmt);
    }

    /// Edit a Vec3d as 3 inline fields. Returns true if any changed.
    static bool editVec3(const char* label, sim::math::Vec3d* v,
                        const char* x_label, const char* y_label, const char* z_label) {
        bool changed = false;
        ImGui::TextUnformatted(label);
        ImGui::Indent();

        double x = v->x(), y = v->y(), z = v->z();
        float w = ImGui::GetContentRegionAvail().x * 0.55f;

        ImGui::SetNextItemWidth(w);
        if (ImGui::InputScalar(x_label, ImGuiDataType_Double, &x, nullptr, nullptr, "%.4g")) {
            *v = sim::math::Vec3d{x, y, z};
            changed = true;
        }
        ImGui::SetNextItemWidth(w);
        if (ImGui::InputScalar(y_label, ImGuiDataType_Double, &y, nullptr, nullptr, "%.4g")) {
            *v = sim::math::Vec3d{x, y, z};
            changed = true;
        }
        ImGui::SetNextItemWidth(w);
        if (ImGui::InputScalar(z_label, ImGuiDataType_Double, &z, nullptr, nullptr, "%.4g")) {
            *v = sim::math::Vec3d{x, y, z};
            changed = true;
        }

        ImGui::Unindent();
        return changed;
    }

    /// Combo box for string selection. Returns true if changed.
    static bool editCombo(const char* label, std::string* current,
                        const char* const items[], int count) {
        int idx = 0;
        for (int i = 0; i < count; ++i) {
            if (*current == items[i]) { idx = i; break; }
        }

        float width = ImGui::GetContentRegionAvail().x * 0.55f;
        ImGui::SetNextItemWidth(width);
        bool changed = ImGui::Combo(label, &idx, items, count);
        if (changed) {
            *current = items[idx];
        }
        return changed;
    }

    // Lifecycle

    WorkspaceScreen::WorkspaceScreen(const std::string& shaderDir,
                                    const std::string& resDir)
        : shaderDir_(shaderDir)
        , resDir_(resDir) {}

    WorkspaceScreen::~WorkspaceScreen() = default;

    void WorkspaceScreen::enter(GLFWwindow* window, int fbW, int fbH) {
        window_ = window;
        fbW_    = fbW;
        fbH_    = fbH;

        // Viewport camera
        float aspect = (fbW_ > 0 && fbH_ > 0)
                    ? static_cast<float>(fbW_) / static_cast<float>(fbH_)
                    : 16.0f / 9.0f;

        viewportCam_ = vis::Camera(
            glm::vec3(15.0f, -15.0f, 12.0f),   // eye: looking down at origin
            glm::vec3(0.0f, 0.0f, 0.0f),       // target: origin
            45.0f, aspect, 0.1f, 10000.0f       // wide far plane for large scenes
        );

        // FBO for viewport
        viewportFBO_ = vis::Framebuffer(viewportW_, viewportH_);

        // Grid
        grid_ = std::make_unique<vis::GridRenderer>(
            shaderDir_, 50.0f, 5.0f    // 50m extent, 5m spacing
        );

        // Console sink
	    consoleSink_ = std::make_shared<ImGuiConsoleSink>();
	    consoleSink_->set_pattern("[%H:%M:%S.%e] [%^%l%$] %v");
	    sim::core::Logger::get()->sinks().push_back(consoleSink_);
    	consoleHistory_.clear();

        // Default config.
        newConfig_();

        // Force dock layout rebuild on first frame.
        dockLayoutBuilt_ = false;
    	SIM_INFO("Workspace initialized.");
    }

    void WorkspaceScreen::leave() {
	    // Remove our sink from the sim logger
	    if (consoleSink_) {
	        auto& sinks = sim::core::Logger::get()->sinks();
	        sinks.erase(
	            std::remove(sinks.begin(), sinks.end(), consoleSink_),
	            sinks.end());
	        consoleSink_.reset();
	    }
        grid_.reset();
        viewportFBO_ = vis::Framebuffer();   // destroy FBO

        nextScreen_.clear();
        quit_ = false;
    }

    void WorkspaceScreen::onResize(int fbW, int fbH) {
        fbW_ = fbW;
        fbH_ = fbH;
    }

    // Update

    void WorkspaceScreen::update(float /*dt*/) {
        // ESC returns to main menu.
        if (glfwGetKey(window_, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
            nextScreen_ = "main_menu";
        }

        // Drain sink into history
        if (consoleSink_) {
            auto entries = consoleSink_->drain();
            for (auto& e : entries) {
                consoleHistory_.push_back(std::move(e));
            }
            // Trim history.
            while (consoleHistory_.size() > kMaxConsoleHistory) {
                consoleHistory_.pop_front();
            }
        }
    }

    // File operations

    void WorkspaceScreen::newConfig_() {
        config_ = sim::core::SimConfig{};
        configFilepath_.clear();
        configDirty_ = false;
        gncEnabled_ = config_.vehicle.seeker.has_value();
        SIM_INFO("New simulation configuration (defaults).");
    }

    void WorkspaceScreen::openConfig_() {
        const char* filters[] = {"*.xml"};
        const char* selected = tinyfd_openFileDialog(
            "Open Simulation Config",    // title
            "",                          // default path
            1,                           // number of filter patterns
            filters,                     // filter patterns
            "XML Config Files (*.xml)",  // filter description
            0                            // single select
        );

        if (!selected) return;   // user cancelled

        std::string path(selected);
        try {
            config_ = sim::core::ConfigParser::load(path);
            configFilepath_ = path;
            configDirty_ = false;
            gncEnabled_ = config_.vehicle.seeker.has_value();
            SIM_INFO("Loaded config: {}", path);
        } catch (const std::exception& e) {
            SIM_ERROR("Failed to load config: {}", e.what());
        }
    }

    void WorkspaceScreen::saveConfig_() {
        if (configFilepath_.empty()) {
            saveConfigAs_();
            return;
        }
        try {
            sim::core::ConfigParser::save(config_, configFilepath_);
            configDirty_ = false;
        } catch (const std::exception& e) {
            SIM_ERROR("Failed to save config: {}", e.what());
        }
    }

    void WorkspaceScreen::saveConfigAs_() {
        const char* filters[] = {"*.xml"};
        const char* selected = tinyfd_saveFileDialog(
            "Save Simulation Config",    // title
            "config.xml",                // default filename
            1,                           // number of filter patterns
            filters,                     // filter patterns
            "XML Config Files (*.xml)"   // filter description
        );

        if (!selected) return;   // user cancelled

        std::string path(selected);
        try {
            sim::core::ConfigParser::save(config_, path);
            configFilepath_ = path;
            configDirty_ = false;
        } catch (const std::exception& e) {
            SIM_ERROR("Failed to save config: {}", e.what());
        }
    }

    // Render

    void WorkspaceScreen::render() {
        // Clear the main framebuffer (behind ImGui).
        glClearColor(0.10f, 0.10f, 0.12f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // DockSpace
        // Create a full-screen dockable area with a menu bar.
        ImGuiViewport* vp = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(vp->WorkPos);
        ImGui::SetNextWindowSize(vp->WorkSize);
        ImGui::SetNextWindowViewport(vp->ID);

        ImGuiWindowFlags hostFlags =
            ImGuiWindowFlags_NoDocking
        | ImGuiWindowFlags_NoTitleBar
        | ImGuiWindowFlags_NoCollapse
        | ImGuiWindowFlags_NoResize
        | ImGuiWindowFlags_NoMove
        | ImGuiWindowFlags_NoBringToFrontOnFocus
        | ImGuiWindowFlags_NoNavFocus
        | ImGuiWindowFlags_MenuBar;

        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));

        ImGui::Begin("##WorkspaceHost", nullptr, hostFlags);
        ImGui::PopStyleVar(3);

        // Menu bar
        renderMenuBar_();

        // Create the DockSpace
        ImGuiID dockId = ImGui::GetID(kDockSpaceName);
        ImGui::DockSpace(dockId, ImVec2(0.0f, 0.0f),
                        ImGuiDockNodeFlags_PassthruCentralNode);

        // Set up default layout on first frame.
        if (!dockLayoutBuilt_) {
            setupDockLayout_();
            dockLayoutBuilt_ = true;
        }

        ImGui::End();  // WorkspaceHost

        // Panels
        if (showViewport_)   renderViewportPanel_();
        if (showProperties_) renderPropertiesPanel_();
        if (showConsole_)    renderConsolePanel_();
    }

    // Dock layout

    void WorkspaceScreen::setupDockLayout_() {
        ImGuiID dockId = ImGui::GetID(kDockSpaceName);

        // Clear any existing layout.
        ImGui::DockBuilderRemoveNode(dockId);
        ImGui::DockBuilderAddNode(dockId, ImGuiDockNodeFlags_DockSpace);
        ImGui::DockBuilderSetNodeSize(dockId, ImGui::GetMainViewport()->WorkSize);

        // Split: left 70% = viewport area, right 30% = properties.
        ImGuiID dockLeft, dockRight;
        ImGui::DockBuilderSplitNode(dockId, ImGuiDir_Right, 0.28f,
                                    &dockRight, &dockLeft);

        // Split left: top 75% = viewport, bottom 25% = console.
        ImGuiID dockViewport, dockConsole;
        ImGui::DockBuilderSplitNode(dockLeft, ImGuiDir_Down, 0.25f,
                                    &dockConsole, &dockViewport);

        // Dock windows into their nodes.
        ImGui::DockBuilderDockWindow(kViewportPanel,   dockViewport);
        ImGui::DockBuilderDockWindow(kPropertiesPanel, dockRight);
        ImGui::DockBuilderDockWindow(kConsolePanel,    dockConsole);

        ImGui::DockBuilderFinish(dockId);
    }

    // Menu bar

    void WorkspaceScreen::renderMenuBar_() {
        if (ImGui::BeginMenuBar()) {

            if (ImGui::BeginMenu("File")) {
                if (ImGui::MenuItem("New Simulation", "Ctrl+N")) { newConfig_(); }
                if (ImGui::MenuItem("Open Config...", "Ctrl+O")) { openConfig_(); }
                if (ImGui::MenuItem("Save Config", "Ctrl+S")) { saveConfig_(); }
                if (ImGui::MenuItem("Save Config As...")) { saveConfigAs_(); }
                ImGui::Separator();
                if (ImGui::MenuItem("Exit to Main Menu")) {
                    nextScreen_ = "main_menu";
                }
                if (ImGui::MenuItem("Quit", "Alt+F4")) {
                    quit_ = true;
                }
                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("Edit")) {
                if (ImGui::MenuItem("Undo", "Ctrl+Z", false, false)) {}
                if (ImGui::MenuItem("Redo", "Ctrl+Y", false, false)) {}
                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("View")) {
                ImGui::MenuItem(kViewportPanel,   nullptr, &showViewport_);
                ImGui::MenuItem(kPropertiesPanel, nullptr, &showProperties_);
                ImGui::MenuItem(kConsolePanel,    nullptr, &showConsole_);
                ImGui::Separator();
                if (ImGui::MenuItem("Reset Layout")) {
                    dockLayoutBuilt_ = false;
                }
                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("Simulation")) {
                if (ImGui::MenuItem("Run", "F5")) {
                	SIM_INFO("Run simulation (not yet implemented).");
                }
                if (ImGui::MenuItem("Run Monte Carlo")) {
                	SIM_INFO("Monte Carlo (not yet implemented).");
                }
                ImGui::Separator();
                if (ImGui::MenuItem("Stop", nullptr, false, false)) {}
                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("Help")) {
                if (ImGui::MenuItem("About")) {
                	SIM_INFO("Missile Flight Simulator v3.1.0-alpha");
                }
                ImGui::EndMenu();
            }

            ImGui::EndMenuBar();
        }
    }

    // 3D Viewport

    void WorkspaceScreen::renderSceneToFBO_() {
        viewportFBO_.bind();

        glClearColor(0.08f, 0.08f, 0.10f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);

        grid_->render(viewportCam_);

        viewportFBO_.unbind();
    }

    void WorkspaceScreen::renderViewportPanel_() {
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
        ImGui::Begin(kViewportPanel, &showViewport_);

        // Get the panel's content size and resize FBO if needed.
        ImVec2 panelSize = ImGui::GetContentRegionAvail();
        int panelW = static_cast<int>(panelSize.x);
        int panelH = static_cast<int>(panelSize.y);

        if (panelW > 0 && panelH > 0) {
            viewportFBO_.resize(panelW, panelH);
            viewportCam_.setAspect(static_cast<float>(panelW) /
                                static_cast<float>(panelH));
            viewportW_ = panelW;
            viewportH_ = panelH;

            // Render the 3D scene into the FBO (after resize, before display).
            renderSceneToFBO_();

            // Restore the main framebuffer viewport for ImGui.
            glBindFramebuffer(GL_FRAMEBUFFER, 0);
            glViewport(0, 0, fbW_, fbH_);

            // Display the FBO color texture as an ImGui image.
            ImTextureID texId = static_cast<ImTextureID>(viewportFBO_.colorTexture());
            ImGui::Image(texId, panelSize, ImVec2(0, 1), ImVec2(1, 0));
        }

        ImGui::End();
        ImGui::PopStyleVar();
    }

    // Properties panel

    void WorkspaceScreen::renderPropertiesPanel_() {
        ImGui::Begin(kPropertiesPanel, &showProperties_);

        // Title + dirty indicator.
        const char* dirtyMark = configDirty_ ? " *" : "";
        std::string title = configFilepath_.empty()
                            ? "New Simulation" + std::string(dirtyMark)
                            : configFilepath_ + dirtyMark;
        ImGui::TextColored(ImVec4(0.7f, 0.8f, 1.0f, 1.0f), "%s", title.c_str());
        ImGui::Separator();
        ImGui::Dummy(ImVec2(0, 4));

        renderInitialConditions_();
        renderVehicle_();
        renderGNC_();
        renderTargets_();
        renderSimSettings_();

        ImGui::End();
    }

    void WorkspaceScreen::renderInitialConditions_() {
        if (!ImGui::CollapsingHeader("Initial Conditions", ImGuiTreeNodeFlags_DefaultOpen))
            return;

        ImGui::Indent();
        auto& ic = config_.initial_conditions;

        if (editVec3("Position (NED, m)", &ic.position, "North##ic", "East##ic", "Down##ic"))
            configDirty_ = true;
        if (editDouble("Speed (m/s)##ic",  &ic.speed))     configDirty_ = true;
        if (editDouble("Pitch (deg)##ic",  &ic.pitch_deg)) configDirty_ = true;
        if (editDouble("Yaw (deg)##ic",    &ic.yaw_deg))   configDirty_ = true;
        if (editDouble("Roll (deg)##ic",   &ic.roll_deg))  configDirty_ = true;

        ImGui::Unindent();
    }

    void WorkspaceScreen::renderVehicle_() {
        if (!ImGui::CollapsingHeader("Vehicle"))
            return;

        ImGui::Indent();
        auto& veh = config_.vehicle;

        if (editDouble("Ref Area (m^2)", &veh.ref_area))    configDirty_ = true;
        if (editDouble("Ref Length (m)", &veh.ref_length))   configDirty_ = true;

        ImGui::Dummy(ImVec2(0, 4));

        // Atmosphere
        if (ImGui::TreeNode("Atmosphere")) {
            static const char* atmoTypes[] = {"us_standard_1976"};
            if (editCombo("Type##atmo", &veh.atmosphere.type, atmoTypes, 1))
                configDirty_ = true;
            ImGui::TreePop();
        }

        // Gravity
        if (ImGui::TreeNode("Gravity")) {
            static const char* gravTypes[] = {"constant"};
            if (editCombo("Type##grav", &veh.gravity.type, gravTypes, 1))
                configDirty_ = true;
            if (editDouble("g (m/s^2)##grav", &veh.gravity.g))
                configDirty_ = true;
            ImGui::TreePop();
        }

        renderPropulsion_();
        renderAerodynamics_();

        ImGui::Unindent();
    }

    void WorkspaceScreen::renderPropulsion_() {
        if (!ImGui::TreeNode("Propulsion"))
            return;

        auto& p = config_.vehicle.propulsion;

        static const char* propTypes[] = {"solid_rocket"};
        if (editCombo("Type##prop", &p.type, propTypes, 1)) configDirty_ = true;

        if (editDouble("Thrust (N)",       &p.thrust))    configDirty_ = true;
        if (editDouble("Burn Time (s)",    &p.burn_time)) configDirty_ = true;
        if (editDouble("Total Mass (kg)",  &p.total_mass)) configDirty_ = true;
        if (editDouble("Prop Mass (kg)",   &p.prop_mass)) configDirty_ = true;

        ImGui::Dummy(ImVec2(0, 2));
        if (editVec3("CG Body (m)", &p.cg_body, "CG X##prop", "CG Y##prop", "CG Z##prop"))
            configDirty_ = true;

        // Inertia tensor - display the 6 unique elements.
        ImGui::Dummy(ImVec2(0, 2));
        ImGui::TextUnformatted("Inertia (kg*m^2)");
        ImGui::Indent();

        double Ixx = p.inertia(0,0), Iyy = p.inertia(1,1), Izz = p.inertia(2,2);
        double Ixy = std::abs(p.inertia(0,1));
        double Ixz = std::abs(p.inertia(0,2));
        double Iyz = std::abs(p.inertia(1,2));

        bool changed = false;
        if (editDouble("Ixx##inertia", &Ixx)) changed = true;
        if (editDouble("Iyy##inertia", &Iyy)) changed = true;
        if (editDouble("Izz##inertia", &Izz)) changed = true;
        if (editDouble("Ixy##inertia", &Ixy)) changed = true;
        if (editDouble("Ixz##inertia", &Ixz)) changed = true;
        if (editDouble("Iyz##inertia", &Iyz)) changed = true;

        if (changed) {
            // Rebuild with aerospace sign convention (negated off-diags).
            p.inertia = sim::math::Mat3d{
                Ixx, -Ixy, -Ixz,
                -Ixy,  Iyy, -Iyz,
                -Ixz, -Iyz,  Izz
            };
            configDirty_ = true;
        }

        ImGui::Unindent();
        ImGui::TreePop();
    }

    void WorkspaceScreen::renderAerodynamics_() {
        if (!ImGui::TreeNode("Aerodynamics"))
            return;

        auto& a = config_.vehicle.aerodynamics;

        static const char* aeroTypes[] = {"simple"};
        if (editCombo("Type##aero", &a.type, aeroTypes, 1)) configDirty_ = true;

        ImGui::Dummy(ImVec2(0, 2));
        ImGui::TextDisabled("Force Coefficients");
        if (editDouble("CA##aero",        &a.CA))        configDirty_ = true;
        if (editDouble("CN_alpha##aero",  &a.CN_alpha))  configDirty_ = true;
        if (editDouble("CY_beta##aero",   &a.CY_beta))   configDirty_ = true;

        ImGui::Dummy(ImVec2(0, 2));
        ImGui::TextDisabled("Moment Coefficients");
        if (editDouble("Cm_alpha##aero",  &a.Cm_alpha))  configDirty_ = true;
        if (editDouble("Cn_beta##aero",   &a.Cn_beta))   configDirty_ = true;
        if (editDouble("Cl_delta##aero",  &a.Cl_delta))  configDirty_ = true;
        if (editDouble("Cm_delta##aero",  &a.Cm_delta))  configDirty_ = true;
        if (editDouble("Cn_delta##aero",  &a.Cn_delta))  configDirty_ = true;

        ImGui::Dummy(ImVec2(0, 2));
        ImGui::TextDisabled("Damping Derivatives");
        if (editDouble("Cmq##aero", &a.Cmq)) configDirty_ = true;
        if (editDouble("Cnr##aero", &a.Cnr)) configDirty_ = true;
        if (editDouble("Clp##aero", &a.Clp)) configDirty_ = true;

        ImGui::TreePop();
    }

    void WorkspaceScreen::renderGNC_() {
        if (!ImGui::CollapsingHeader("GNC Chain"))
            return;

        ImGui::Indent();

        // Master enable toggle.
        if (ImGui::Checkbox("Enable GNC", &gncEnabled_)) {
            configDirty_ = true;
            if (gncEnabled_) {
                // Create default GNC configs if not present.
                if (!config_.vehicle.seeker.has_value())
                    config_.vehicle.seeker = sim::core::SeekerConfig{};
                if (!config_.vehicle.guidance.has_value())
                    config_.vehicle.guidance = sim::core::GuidanceConfig{};
                if (!config_.vehicle.autopilot.has_value())
                    config_.vehicle.autopilot = sim::core::AutopilotConfig{};
                if (!config_.vehicle.actuator.has_value())
                    config_.vehicle.actuator = sim::core::ActuatorConfig{};
            } else {
                config_.vehicle.seeker.reset();
                config_.vehicle.guidance.reset();
                config_.vehicle.autopilot.reset();
                config_.vehicle.actuator.reset();
            }
        }

        if (!gncEnabled_) {
            ImGui::TextDisabled("  GNC disabled — unguided flight.");
            ImGui::Unindent();
            return;
        }

        // Seeker
        if (ImGui::TreeNode("Seeker")) {
            auto& s = config_.vehicle.seeker.value();
            static const char* seekTypes[] = {"ideal"};
            if (editCombo("Type##seeker", &s.type, seekTypes, 1)) configDirty_ = true;
            if (editDouble("Min Range (m)##seeker", &s.min_range)) configDirty_ = true;
            ImGui::TreePop();
        }

        // Guidance
        if (ImGui::TreeNode("Guidance")) {
            auto& g = config_.vehicle.guidance.value();
            static const char* guidTypes[] = {"pro_nav"};
            if (editCombo("Type##guid", &g.type, guidTypes, 1)) configDirty_ = true;
            if (editDouble("Nav Ratio (N)##guid", &g.nav_ratio)) configDirty_ = true;
            ImGui::TreePop();
        }

        // Autopilot
        if (ImGui::TreeNode("Autopilot")) {
            auto& ap = config_.vehicle.autopilot.value();
            static const char* apTypes[] = {"simple"};
            if (editCombo("Type##ap", &ap.type, apTypes, 1)) configDirty_ = true;
            if (editDouble("Kp##ap", &ap.Kp))                  configDirty_ = true;
            if (editDouble("Kd##ap", &ap.Kd))                  configDirty_ = true;
            if (editDouble("Kd Roll##ap", &ap.Kd_roll))        configDirty_ = true;
            if (editDouble("Max Deflection (rad)##ap", &ap.max_deflection)) configDirty_ = true;
            ImGui::TreePop();
        }

        // Actuator
        if (ImGui::TreeNode("Actuator")) {
            auto& act = config_.vehicle.actuator.value();
            static const char* actTypes[] = {"first_order"};
            if (editCombo("Type##act", &act.type, actTypes, 1)) configDirty_ = true;
            if (editDouble("Time Constant (s)##act", &act.time_constant)) configDirty_ = true;
            if (editDouble("Max Deflection (rad)##act", &act.max_deflection)) configDirty_ = true;
            if (editDouble("Max Rate (rad/s)##act", &act.max_rate)) configDirty_ = true;
            ImGui::TreePop();
        }

        ImGui::Unindent();
    }

    void WorkspaceScreen::renderTargets_() {
        if (!ImGui::CollapsingHeader("Targets"))
            return;

        ImGui::Indent();
        auto& targets = config_.targets;

        // Add/remove buttons.
        if (ImGui::SmallButton("+ Add Target")) {
            targets.emplace_back();
            configDirty_ = true;
        }

        for (size_t i = 0; i < targets.size(); ++i) {
            ImGui::PushID(static_cast<int>(i));
            auto& tgt = targets[i];

            std::string label = "Target " + std::to_string(i);
            if (ImGui::TreeNode(label.c_str())) {
                // Remove button.
                ImGui::SameLine();
                if (ImGui::SmallButton("Remove")) {
                    targets.erase(targets.begin() + static_cast<long>(i));
                    configDirty_ = true;
                    ImGui::TreePop();
                    ImGui::PopID();
                    break;  // indices shifted, bail out of this frame
                }

                static const char* tgtTypes[] = {"stationary", "constant_velocity", "maneuvering"};
                if (editCombo("Type##tgt", &tgt.type, tgtTypes, 3)) configDirty_ = true;

                if (editVec3("Position (NED, m)##tgt", &tgt.position,
                            "North##tgt", "East##tgt", "Down##tgt"))
                    configDirty_ = true;

                if (tgt.type != "stationary") {
                    if (editVec3("Velocity (NED, m/s)##tgt", &tgt.velocity,
                                "V North##tgt", "V East##tgt", "V Down##tgt"))
                        configDirty_ = true;
                }

                if (tgt.type == "maneuvering") {
                    if (editDouble("Maneuver G##tgt", &tgt.maneuver_g))
                        configDirty_ = true;
                    if (editDouble("Maneuver Start (s)##tgt", &tgt.maneuver_start))
                        configDirty_ = true;
                }

                ImGui::TreePop();
            }
            ImGui::PopID();
        }

        if (targets.empty()) {
            ImGui::TextDisabled("  No targets defined.");
        }

        ImGui::Unindent();
    }

    void WorkspaceScreen::renderSimSettings_() {
        if (!ImGui::CollapsingHeader("Simulation Settings"))
            return;

        ImGui::Indent();

        // Integrator
        if (ImGui::TreeNode("Integrator")) {
            auto& integ = config_.integrator;
            static const char* integTypes[] = {"rk4"};
            if (editCombo("Type##integ", &integ.type, integTypes, 1)) configDirty_ = true;
            if (editDouble("dt (s)##integ", &integ.dt)) configDirty_ = true;
            ImGui::TreePop();
        }

        // Recorder
        if (ImGui::TreeNode("Recorder")) {
            auto& rec = config_.recorder;
            // Filepath as text input.
            char buf[256];
            std::strncpy(buf, rec.filepath.c_str(), sizeof(buf) - 1);
            buf[sizeof(buf) - 1] = '\0';
            float w = ImGui::GetContentRegionAvail().x * 0.55f;
            ImGui::SetNextItemWidth(w);
            if (ImGui::InputText("Filepath##rec", buf, sizeof(buf))) {
                rec.filepath = buf;
                configDirty_ = true;
            }
            if (editDouble("Interval (s)##rec", &rec.interval)) configDirty_ = true;
            ImGui::TreePop();
        }

        // Stop conditions
        if (ImGui::TreeNode("Stop Conditions")) {
            auto& stop = config_.stop;
            if (editDouble("Max Time (s)##stop", &stop.max_time)) configDirty_ = true;
            if (editDouble("Min Altitude (m)##stop", &stop.min_altitude)) configDirty_ = true;
            if (editDouble("POCA Range Gate (m)##stop", &stop.poca_range_gate)) configDirty_ = true;
            ImGui::TreePop();
        }

        ImGui::Unindent();
    }

    // Console panel

    void WorkspaceScreen::renderConsolePanel_() {
        ImGui::Begin(kConsolePanel, &showConsole_);

        // Toolbar
        if (ImGui::SmallButton("Clear")) {
            consoleHistory_.clear();
        }
        ImGui::SameLine();
        ImGui::TextDisabled("(%d)", static_cast<int>(consoleHistory_.size()));

        ImGui::SameLine();
        ImGui::Spacing();
        ImGui::SameLine();

        // Level filter toggles.
        ImGui::PushStyleColor(ImGuiCol_Text, levelColor(spdlog::level::info));
        ImGui::SameLine(); ImGui::Checkbox("Info", &showInfo_);
        ImGui::PopStyleColor();

        ImGui::PushStyleColor(ImGuiCol_Text, levelColor(spdlog::level::warn));
        ImGui::SameLine(); ImGui::Checkbox("Warn", &showWarn_);
        ImGui::PopStyleColor();

        ImGui::PushStyleColor(ImGuiCol_Text, levelColor(spdlog::level::err));
        ImGui::SameLine(); ImGui::Checkbox("Error", &showError_);
        ImGui::PopStyleColor();

        ImGui::PushStyleColor(ImGuiCol_Text, levelColor(spdlog::level::debug));
        ImGui::SameLine(); ImGui::Checkbox("Debug", &showDebug_);
        ImGui::PopStyleColor();

        ImGui::SameLine();
        ImGui::Checkbox("Auto-scroll", &consoleAutoScroll_);

        ImGui::Separator();

        // Log entries
        ImGui::BeginChild("##ConsoleScroll", ImVec2(0, 0), false,
                        ImGuiWindowFlags_HorizontalScrollbar);

        for (const auto& entry : consoleHistory_) {
            if (!shouldShow(entry.level, showTrace_, showDebug_, showInfo_,
                            showWarn_, showError_, showCritical_)) {
                continue;
            }

            ImGui::PushStyleColor(ImGuiCol_Text, levelColor(entry.level));
            ImGui::TextUnformatted(entry.text.c_str());
            ImGui::PopStyleColor();
        }

        // Auto-scroll to bottom on new messages.
        if (consoleAutoScroll_ && ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 10.0f) {
            ImGui::SetScrollHereY(1.0f);
        }

        ImGui::EndChild();
        ImGui::End();
    }

} // namespace app