#pragma once

#include "Screen.hpp"

#include "vis/Camera.hpp"
#include "vis/Framebuffer.hpp"
#include "vis/render/GridRenderer.hpp"

#include <memory>
#include <string>
#include <vector>

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

        // 3D Viewport rendering
        void renderSceneToFBO_();

        // Console helpers
        void logMessage_(const std::string& msg);

        // Paths
        std::string shaderDir_;
        std::string resDir_;

        // Window state
        GLFWwindow* window_ = nullptr;
        int fbW_ = 0, fbH_ = 0;

        // 3D Viewport
        vis::Framebuffer                    viewportFBO_;
        vis::Camera                         viewportCam_;
        std::unique_ptr<vis::GridRenderer>  grid_;
        int viewportW_ = 800, viewportH_ = 600;

        // Console
        std::vector<std::string> consoleLog_;
        bool consoleScrollToBottom_ = false;

        // UI state
        bool        quit_       = false;
        std::string nextScreen_;

        // Panel visibility (toggled via View menu)
        bool showViewport_   = true;
        bool showProperties_ = true;
        bool showConsole_    = true;
    };

} // namespace app