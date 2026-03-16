#pragma once

#include "Screen.hpp"

#include "vis/Camera.hpp"
#include "vis/entities/Entity.hpp"
#include "vis/entities/Light.hpp"
#include "vis/models/TexturedModel.hpp"
#include "vis/render/MasterRenderer.hpp"
#include "vis/Loader.hpp"

#include <memory>
#include <string>

namespace app {
    
    /**
     * @brief Main menu / landing page screen.
     * 
     * Displays a slowly rotating, textured Earth globe with directional
     * lighting (sun), rendered against a dark background. ImGui
     * buttons overlay the left side of the screen. 
     * 
     *   - New Simulation  → transitions to ConfigScreen
     *   - Load Results    → transitions to ViewerScreen
     *   - Settings        → (stubbed)
     *   - Exit            → closes the application
     * The Earth rotates around the Z axis (Z-up world) at a configurable
     * rate. The camera is fixed, looking at the globe from a slight angle.
     *
     * Visual style: dark background with subtle blue tint, Earth
     * illuminated from the upper-right. Inspired by KSP main menu.
     */
    class MainMenuScreen : public Screen
    {
    public:
        /// @param shaderDir Path to the shaders/ directory.
        /// @param resDir    Path to the res/ directory (textures).
        MainMenuScreen(const std::string& shaderDir,
                       const std::string& resDir);
 
        ~MainMenuScreen() override;

        const char* name() const override { return "Main Menu"; }

        void enter(GLFWwindow* window, int fbW, int fbH) override;
        void update(float dt) override;
        void render() override;
        void leave() override;
        void onResize(int fbW, int fbH) override;

        bool wantsQuit() const override { return quit_; }
        std::string nextScreen() const override { return nextScreen_; }
    
    private:
        void renderImGui_();

        // Paths 
        std::string shaderDir_;
        std::string resDir_;
        
        // GL state (owned, created in enter())
        GLFWwindow* window_ = nullptr;
        int fbW_ = 0, fbH_ = 0;

        vis::Loader loader_;
        vis::Camera camera_;
        vis::Light  sun_;
        vis::TexturedModel  earthModel_;
        vis::Entity         earthEntity_;

        std::unique_ptr<vis::MasterRenderer> master_;

        // Animation
        float earthRotation_ = 0.0f; // degrees, current Z rotation
        static constexpr float kRotationRate_ = 5.0f; // degrees per second

        // UI State
        bool quit_ = false;
        std::string nextScreen_ = "";
    };

} // namespace app