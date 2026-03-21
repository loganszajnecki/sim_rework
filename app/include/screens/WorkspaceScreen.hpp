#pragma once

#include "Screen.hpp"
#include "ImGuiConsoleSink.hpp"

#include "vis/Camera.hpp"
#include "vis/Framebuffer.hpp"
#include "vis/render/GridRenderer.hpp"

#include "core/SimConfig.hpp"    // sim config structs

#include <memory>
#include <string>
#include <vector>
#include <deque>

struct GLFWwindow;

namespace app {

    /**
     * @brief The main workspace screen - simulation IDE.
     *
     * Provides a dockable ImGui layout with:
     *   - 3D Viewport (renders to FBO, displayed as ImGui::Image)
     *   - Properties panel (will host SimConfig editing in B2)
     *   - Console panel (text log, will show sim output in B3)
     *   - Menu bar (File, Edit, View, Simulation, Help)
     *
     * The viewport uses an offscreen Framebuffer so the 3D scene
     * can be rendered inside a resizable, dockable ImGui panel
     * alongside other panels - the same approach used by Unity,
     * Unreal, and Roblox Studio editors.
     *
     * The default dock layout is set up programmatically on first
     * entry via ImGui's DockBuilder API.
     */
    class WorkspaceScreen : public Screen {
    public:
        WorkspaceScreen(const std::string& shaderDir,
                        const std::string& resDir);
        ~WorkspaceScreen() override;

        const char* name() const override { return "Workspace"; }

        void enter(GLFWwindow* window, int fbW, int fbH) override;
        void update(float dt) override;
        void render() override;
        void leave() override;
        void onResize(int fbW, int fbH) override;

        bool wantsQuit() const override { return quit_; }
        std::string nextScreen() const override { return nextScreen_; }

    private:
        // Docking layout
        void setupDockLayout_();
        bool dockLayoutBuilt_ = false;

        // Menu bar
        void renderMenuBar_();

        // Panels
        void renderViewportPanel_();
        void renderPropertiesPanel_();
        void renderConsolePanel_();

        // Properties sub-sections
        void renderInitialConditions_();
        void renderVehicle_();
        void renderPropulsion_();
        void renderAerodynamics_();
        void renderGNC_();
        void renderTargets_();
        void renderSimSettings_();

        // 3D Viewport rendering
        void renderSceneToFBO_();

        // File operations
        void newConfig_();
        void openConfig_();
        void saveConfig_();
        void saveConfigAs_();

        // Paths
        std::string shaderDir_;
        std::string resDir_;

        // Window state
        GLFWwindow* window_ = nullptr;
        int fbW_ = 0, fbH_ = 0;

        // Simulation config (the central data model)
        sim::core::SimConfig config_;
        std::string configFilepath_;     // "" = unsaved new config
        bool configDirty_ = false;       // has unsaved changes
        bool gncEnabled_  = false;       // controls optional GNC fields

        // 3D Viewport
        vis::Framebuffer                    viewportFBO_;
        vis::Camera                         viewportCam_;
        std::unique_ptr<vis::GridRenderer>  grid_;
        int viewportW_ = 800, viewportH_ = 600;

        // Console (unified spdlog sink)
	    std::shared_ptr<ImGuiConsoleSink>  consoleSink_;
	    std::deque<ConsoleEntry>           consoleHistory_;
	    bool consoleAutoScroll_ = true;
	    static constexpr size_t kMaxConsoleHistory = 5000;

        // Console filter
	    bool showTrace_    = false;
	    bool showDebug_    = false;
	    bool showInfo_     = true;
	    bool showWarn_     = true;
	    bool showError_    = true;
	    bool showCritical_ = true;

        // UI state
        bool        quit_       = false;
        std::string nextScreen_;

        // Panel visibility (toggled via View menu)
        bool showViewport_   = true;
        bool showProperties_ = true;
        bool showConsole_    = true;
    };

} // namespace app