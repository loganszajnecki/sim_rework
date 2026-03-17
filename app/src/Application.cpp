#include "Application.hpp"
#include "Screen.hpp"

#include <iostream>
#include <stdexcept>
#include <cmath>
#include <algorithm>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

namespace app {

    // Easing function

    /// Smooth-step ease-in-out: slow start and end, fast middle.
    static float easeInOut(float t) {
        t = std::clamp(t, 0.0f, 1.0f);
        return t * t * (3.0f - 2.0f * t);
    }

    // Lifecycle
    Application::~Application() {
        shutdown_();
    }

    bool Application::init(const Config& cfg) {
        if (initialized_) return true;

        // GLFW
        if (!glfwInit()) {
            std::cerr << "Application: glfwInit() failed\n";
            return false;
        }

        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    #ifdef __APPLE__
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    #endif
        glfwWindowHint(GLFW_SAMPLES, 4);   // MSAA

        window_ = glfwCreateWindow(cfg.width, cfg.height, cfg.title,
                                nullptr, nullptr);
        if (!window_) {
            std::cerr << "Application: glfwCreateWindow() failed\n";
            glfwTerminate();
            return false;
        }

        glfwMakeContextCurrent(window_);
        glfwSwapInterval(cfg.vsync ? 1 : 0);

        // glad (OpenGL loader)
        if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress))) {
            std::cerr << "Application: gladLoadGLLoader() failed\n";
            glfwDestroyWindow(window_);
            glfwTerminate();
            window_ = nullptr;
            return false;
        }

        std::cout << "OpenGL " << GLVersion.major << "."
                << GLVersion.minor << " loaded\n";

        // Framebuffer size
        glfwGetFramebufferSize(window_, &fbW_, &fbH_);
        glViewport(0, 0, fbW_, fbH_);

        // OpenGL defaults
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_MULTISAMPLE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        // ImGui
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGuiIO& io = ImGui::GetIO();
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

        // Dark theme as base, then customize.
        ImGui::StyleColorsDark();
        ImGuiStyle& style = ImGui::GetStyle();
        style.WindowRounding   = 6.0f;
        style.FrameRounding    = 4.0f;
        style.GrabRounding     = 4.0f;
        style.WindowBorderSize = 0.0f;
        style.FrameBorderSize  = 0.0f;
        style.WindowPadding    = ImVec2(12, 12);
        style.FramePadding     = ImVec2(8, 4);
        style.ItemSpacing      = ImVec2(8, 6);

        // Subtle color overrides for a space/military aesthetic.
        ImVec4* colors = style.Colors;
        colors[ImGuiCol_WindowBg]        = ImVec4(0.06f, 0.06f, 0.08f, 0.85f);
        colors[ImGuiCol_Button]          = ImVec4(0.18f, 0.22f, 0.30f, 1.00f);
        colors[ImGuiCol_ButtonHovered]   = ImVec4(0.28f, 0.35f, 0.48f, 1.00f);
        colors[ImGuiCol_ButtonActive]    = ImVec4(0.15f, 0.20f, 0.30f, 1.00f);
        colors[ImGuiCol_TitleBg]         = ImVec4(0.06f, 0.06f, 0.08f, 1.00f);
        colors[ImGuiCol_TitleBgActive]   = ImVec4(0.10f, 0.12f, 0.16f, 1.00f);
        colors[ImGuiCol_Header]          = ImVec4(0.18f, 0.22f, 0.30f, 1.00f);
        colors[ImGuiCol_HeaderHovered]   = ImVec4(0.28f, 0.35f, 0.48f, 1.00f);
        colors[ImGuiCol_FrameBg]         = ImVec4(0.12f, 0.14f, 0.18f, 1.00f);
        colors[ImGuiCol_FrameBgHovered]  = ImVec4(0.18f, 0.22f, 0.28f, 1.00f);

        ImGui_ImplGlfw_InitForOpenGL(window_, true);
        ImGui_ImplOpenGL3_Init("#version 410");

        // Timing
        lastTime_ = glfwGetTime();
        initialized_ = true;

        std::cout << "Application initialized: " << cfg.width << "x"
                << cfg.height << "\n";
        return true;
    }

    void Application::shutdown_() {
        // Leave current screen.
        if (active_) {
            active_->leave();
            active_ = nullptr;
        }

        // Destroy all screens (releases their GL resources).
        screens_.clear();

        // ImGui teardown.
        if (initialized_) {
            ImGui_ImplOpenGL3_Shutdown();
            ImGui_ImplGlfw_Shutdown();
            ImGui::DestroyContext();
        }

        // GLFW teardown.
        if (window_) {
            glfwDestroyWindow(window_);
            window_ = nullptr;
        }
        if (initialized_) {
            glfwTerminate();
        }

        initialized_ = false;
    }

    // Screen management

    void Application::registerScreen(const std::string& name,
                                    std::unique_ptr<Screen> screen) {
        screens_[name] = std::move(screen);
    }

    void Application::switchTo(const std::string& name) {
        if (screens_.find(name) == screens_.end()) {
            std::cerr << "Application: unknown screen '" << name << "'\n";
            return;
        }

        // If no active screen yet (first switch), do it immediately with a fade-in.
        if (!active_) {
            switchImmediate_(name);
            fadeState_ = FadeState::fading_in;
            fadeAlpha_ = 1.0f;
            return;
        }

        // If already fading, ignore the request.
        if (fadeState_ != FadeState::idle) return;

        // Begin fade-out, defer the actual switch.
        pendingScreen_ = name;
        fadeState_      = FadeState::fading_out;
        fadeAlpha_      = 0.0f;
    }

    void Application::switchImmediate_(const std::string& name) {
        auto it = screens_.find(name);
        if (it == screens_.end()) return;

        if (active_) {
            active_->leave();
        }

        active_     = it->second.get();
        activeName_ = name;

        glfwGetFramebufferSize(window_, &fbW_, &fbH_);
        active_->enter(window_, fbW_, fbH_);

        std::cout << "Screen: " << active_->name() << "\n";
    }

    // Fade transition

    bool Application::updateFade_(float dt) {
        if (fadeState_ == FadeState::idle) return false;

        float speed = 1.0f / fadeDuration_;

        if (fadeState_ == FadeState::fading_out) {
            fadeAlpha_ += speed * dt;
            if (fadeAlpha_ >= 1.0f) {
                fadeAlpha_ = 1.0f;
                // Screen is fully black — do the switch.
                switchImmediate_(pendingScreen_);
                pendingScreen_.clear();
                fadeState_ = FadeState::fading_in;
                return true;
            }
        } else if (fadeState_ == FadeState::fading_in) {
            fadeAlpha_ -= speed * dt;
            if (fadeAlpha_ <= 0.0f) {
                fadeAlpha_ = 0.0f;
                fadeState_ = FadeState::idle;
            }
        }

        return false;
    }

    void Application::renderFadeOverlay_() {
        if (fadeState_ == FadeState::idle) return;

        float alpha = easeInOut(fadeAlpha_);

        // Full-screen black overlay using ImGui.
        ImGui::SetNextWindowPos(ImVec2(0, 0));
        ImGui::SetNextWindowSize(ImVec2(static_cast<float>(fbW_),
                                        static_cast<float>(fbH_)));

        ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration
                            | ImGuiWindowFlags_NoInputs
                            | ImGuiWindowFlags_NoNav
                            | ImGuiWindowFlags_NoBackground
                            | ImGuiWindowFlags_NoBringToFrontOnFocus;

        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0, 0, 0, 0));

        ImGui::Begin("##FadeOverlay", nullptr, flags);

        // Draw a filled rectangle covering the entire window.
        ImDrawList* draw = ImGui::GetWindowDrawList();
        ImU32 color = IM_COL32(0, 0, 0, static_cast<int>(alpha * 255.0f));
        draw->AddRectFilled(ImVec2(0, 0),
                            ImVec2(static_cast<float>(fbW_),
                                static_cast<float>(fbH_)),
                            color);

        ImGui::End();
        ImGui::PopStyleColor();
        ImGui::PopStyleVar();
    }

    // Main loop

    void Application::run() {
        if (!initialized_ || !active_) {
            std::cerr << "Application: not initialized or no active screen\n";
            return;
        }

        while (!glfwWindowShouldClose(window_)) {
            // Timing
            double now = glfwGetTime();
            auto dt = static_cast<float>(now - lastTime_);
            lastTime_ = now;

            // Clamp dt to avoid spiral-of-death on window drag, etc.
            if (dt > 0.1f) dt = 0.1f;

            // Framebuffer size (may change from callback)
            int newW = 0, newH = 0;
            glfwGetFramebufferSize(window_, &newW, &newH);
            if (newW > 0 && newH > 0) {
                glViewport(0, 0, newW, newH);
                if (newW != fbW_ || newH != fbH_) {
                    fbW_ = newW;
                    fbH_ = newH;
                    active_->onResize(fbW_, fbH_);
                }
            }

            // Fade
        	updateFade_(dt);

            // Frame
            beginFrame_();
            active_->update(dt);
            active_->render();
            renderFadeOverlay_();
            endFrame_();

            // Screen transitions
            if (active_->wantsQuit()) {
                glfwSetWindowShouldClose(window_, GLFW_TRUE);
            }

            // Only accept transitions when not already fading.
            if (fadeState_ == FadeState::idle) {
                std::string next = active_->nextScreen();
                if (!next.empty()) {
                    switchTo(next);
                }
            }
        }
    }

    void Application::beginFrame_() {
        glfwPollEvents();

        // Start ImGui frame.
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
    }

    void Application::endFrame_() {
        // Render ImGui draw data.
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window_);
    }

} // namespace app