#pragma once

#include <string> 

// Forward declare GLFWwindow to avoid pulling GL/GLFW into every screen header. 
struct GLFWwindow;

namespace app {

    /**
     * @brief Abstract base class for appplication screens.
     * 
     * Each screen represents a distinct view/state of the application
     * (main menu, configuration, 3D viewer, etc.). The Application 
     * class manages a stack of screens and delegates the frame loop
     * to the active (top) screen.
     * 
     * Lifecycle:
     *   1. enter()  - called once when the screen becomes active.
     *   2. update() - called every frame (input, logic, animation).
     *   3. render() - called every frame (draw 3D scene + ImGui)
     *   4. leave()  - called once when the screen is deactivated or popped.
     * 
     * Screens recieve the GLFW window pointer for input queries and 
     * the framebuffer dimensions for viewport/aspect ratio.
     */
    class Screen 
    {
    public:
        virtual ~Screen() = default;

        /// Human readable name for logging/debug
        [[nodiscard]] virtual const char* name() const = 0;

        /// Called once when this screen becomes the active screen
        virtual void enter(GLFWwindow* window, int fbW, int fbH) = 0;

        /// Called every frame: handle input, advance animation, update state.
        virtual void update(float dt) = 0;

        /// Called every frame: draw 3D content and ImGui overlays.
        virtual void render() = 0;

        /// Called whyen this screen is about to be deactivated or destroyed.
        virtual void leave() = 0;

        /// Called when the framebuffer is resized.
        virtual void onResize(int fbW, int fbH) = 0;

        /// Return true if this screen wants the application to quit
        [[nodiscard]] virtual bool wantsQuit() const { return false; }

        /// Return the name of the screen to transition to, or "" for none.
        /// The Application checks this after update() and performs the switch.
        [[nodiscard]] virtual std::string nextScreen() const { return ""; }
    };
    
} // namespace app
