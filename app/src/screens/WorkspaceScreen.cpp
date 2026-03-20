#include "screens/WorkspaceScreen.hpp"

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#include <imgui_internal.h>   // for DockBuilder API
#pragma GCC diagnostic pop

#include "core/Logger.hpp"    // sim logger — SIM_INFO, etc.

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

    // ── Dock layout ──────────────────────────────────────────────

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

    // ── Menu bar ─────────────────────────────────────────────────

    void WorkspaceScreen::renderMenuBar_() {
        if (ImGui::BeginMenuBar()) {

            if (ImGui::BeginMenu("File")) {
                if (ImGui::MenuItem("New Simulation", "Ctrl+N")) {
                	SIM_INFO("New simulation created (defaults).");
                }
                if (ImGui::MenuItem("Open Config...", "Ctrl+O")) {
                	SIM_INFO("Open config (not yet implemented).");
                }
                if (ImGui::MenuItem("Save Config", "Ctrl+S")) {
                	SIM_INFO("Save config (not yet implemented).");
                }
                if (ImGui::MenuItem("Save Config As...")) {
                	SIM_INFO("Save config as (not yet implemented).");
                }
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

        ImGui::TextColored(ImVec4(0.7f, 0.8f, 1.0f, 1.0f),
                        "Simulation Configuration");
        ImGui::Separator();
        ImGui::Dummy(ImVec2(0, 4));

        // Collapsing headers for each config section.
        // These will be filled with editable fields in B2.

        if (ImGui::CollapsingHeader("Initial Conditions",
                ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::TextDisabled("  (fields coming in Phase B2)");
        }

        if (ImGui::CollapsingHeader("Vehicle")) {
            ImGui::TextDisabled("  Reference area, length");

            if (ImGui::TreeNode("Propulsion")) {
                ImGui::TextDisabled("    Thrust, burn time, masses, inertia");
                ImGui::TreePop();
            }
            if (ImGui::TreeNode("Aerodynamics")) {
                ImGui::TextDisabled("    Force/moment coefficients, damping");
                ImGui::TreePop();
            }
            if (ImGui::TreeNode("Atmosphere")) {
                ImGui::TextDisabled("    Model selection");
                ImGui::TreePop();
            }
            if (ImGui::TreeNode("Gravity")) {
                ImGui::TextDisabled("    Model selection, g value");
                ImGui::TreePop();
            }
        }

        if (ImGui::CollapsingHeader("GNC Chain")) {
            if (ImGui::TreeNode("Seeker")) {
                ImGui::TextDisabled("    Type, min range");
                ImGui::TreePop();
            }
            if (ImGui::TreeNode("Guidance")) {
                ImGui::TextDisabled("    Type, nav ratio");
                ImGui::TreePop();
            }
            if (ImGui::TreeNode("Autopilot")) {
                ImGui::TextDisabled("    Gains, limits");
                ImGui::TreePop();
            }
            if (ImGui::TreeNode("Actuator")) {
                ImGui::TextDisabled("    Time constant, rate/pos limits");
                ImGui::TreePop();
            }
        }

        if (ImGui::CollapsingHeader("Target")) {
            ImGui::TextDisabled("  Type, position, velocity, maneuver");
        }

        if (ImGui::CollapsingHeader("Simulation Settings")) {
            ImGui::TextDisabled("  Integrator, recorder, stop conditions");
        }

        ImGui::End();
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