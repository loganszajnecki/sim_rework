#pragma once
 
#include <memory>
#include <string>
#include <unordered_map>
#include <functional>

struct GLFWwindow;

namespace app {

    class Screen;

    /**
     * @brief Top-level application controller. 
     * 
     * Responsibilities:
     *   - Creates and owns the GLFW window + OpenGL context.
     *   - Initializes and shuts down ImGui.
     *   - Manages named Screen objects and transitions between them.
     *   - Runs the main loop: poll events -> update -> render -> swap.
     * 
     * Lifetime:
     *   - init() creates the window and GL context (RAII-safe).
     *   - run() enters the main loop (blocks until quit).
     *   - Destructor cleans up ImGui, GLFW, and all screens.
     * 
     *
     */
    class Application
    {
    public:
        struct Config {
            int width = 1920;
            int height = 1080;
            bool vsync = true;
            const char* title = "Missile Flight Simulator";
        };

        Application() = default;
        ~Application();

        Application(const Application&)            = delete;
        Application& operator=(const Application&) = delete;

        /// Initialize window, GL context, and ImGui. Returns false on failure.
        [[nodiscard]] bool init(const Config& cfg);
        [[nodiscard]] bool init() { return init(Config{}); }

        /// Register a screen by name. Ownership transfers to Application.
        void registerScreen(const std::string& name,
                            std::unique_ptr<Screen> screen);

        /// Switch to a named screen (calls leave() on current, enter() on new).
        void switchTo(const std::string& name);
    
        /// Enter the main loop. Blocks until the window is closed or quit.
        void run();
    
        /// Access the GLFW window (for screens that need it during construction).
        [[nodiscard]] GLFWwindow* window() const { return window_; }
    
        /// Current framebuffer dimensions.
        [[nodiscard]] int framebufferWidth()  const { return fbW_; }
        [[nodiscard]] int framebufferHeight() const { return fbH_; }

    private:
        void beginFrame_();
        void endFrame_();
        void shutdown_();

        /// Perform the actual screen switch (no fade, immediate).
        void switchImmediate_(const std::string& name);
    
        /// Render the fade overlay. Called after screen render, before ImGui finalize.
        void renderFadeOverlay_();

        /// Update fade state machine. Returns true if a deferred switch happened.
        bool updateFade_(float dt);

        // GLFW / GL state
        GLFWwindow* window_      = nullptr;
        bool        initialized_ = false;
        int         fbW_ = 0, fbH_ = 0;
    
        // Screen management
        std::unordered_map<std::string, std::unique_ptr<Screen>> screens_;
        Screen* active_ = nullptr;
        std::string activeName_;

        // ── Fade transition ──────────────────────────────────────
        enum class FadeState { idle, fading_out, fading_in };
        FadeState   fadeState_    = FadeState::idle;
        float       fadeAlpha_    = 0.0f;    // 0 = transparent, 1 = fully black
        float       fadeDuration_ = 0.4f;    // seconds per half-transition
        std::string pendingScreen_;          // screen to switch to after fade-out
    
        // Timing
        double lastTime_ = 0.0;
    };

} // namespace app