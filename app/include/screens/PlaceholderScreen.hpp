#pragma once

#include "Screen.hpp"

#include <string>
#include <glad/glad.h>
#include <imgui.h>
#include <GLFW/glfw3.h>

namespace app {

/**
 * @brief Temporary placeholder screen for testing transitions.
 *
 * Displays a dark background with a label and a "Back" button.
 * Will be replaced by real screens in later phases.
 */
class PlaceholderScreen : public Screen {
public:
    explicit PlaceholderScreen(const std::string& label)
        : label_(label) {}

    const char* name() const override { return label_.c_str(); }

    void enter(GLFWwindow* window, int fbW, int fbH) override {
        window_ = window;
        fbW_ = fbW;
        fbH_ = fbH;
    }

    void update(float /*dt*/) override {
        if (glfwGetKey(window_, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
            next_ = "main_menu";
        }
    }

    void render() override {
        glClearColor(0.04f, 0.04f, 0.06f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Centered label + back button.
        ImGui::SetNextWindowPos(
            ImVec2(static_cast<float>(fbW_) * 0.5f,
                   static_cast<float>(fbH_) * 0.5f),
            ImGuiCond_Always, ImVec2(0.5f, 0.5f));
        ImGui::SetNextWindowSize(ImVec2(300, 140), ImGuiCond_Always);

        ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize
                               | ImGuiWindowFlags_NoMove
                               | ImGuiWindowFlags_NoCollapse
                               | ImGuiWindowFlags_NoTitleBar;

        ImGui::Begin(("##" + label_).c_str(), nullptr, flags);

        // Center the label.
        float textW = ImGui::CalcTextSize(label_.c_str()).x;
        ImGui::SetCursorPosX((300.0f - textW) * 0.5f);
        ImGui::TextColored(ImVec4(0.7f, 0.8f, 1.0f, 1.0f), "%s", label_.c_str());

        ImGui::Dummy(ImVec2(0, 8));
        ImGui::Separator();
        ImGui::Dummy(ImVec2(0, 12));

        float btnW = 276.0f;
        ImGui::SetCursorPosX((300.0f - btnW) * 0.5f);
        if (ImGui::Button("Back to Main Menu", ImVec2(btnW, 36))) {
            next_ = "main_menu";
        }

        ImGui::End();
    }

    void leave() override { next_.clear(); }
    void onResize(int fbW, int fbH) override { fbW_ = fbW; fbH_ = fbH; }
    std::string nextScreen() const override { return next_; }

private:
    std::string label_;
    std::string next_;
    GLFWwindow* window_ = nullptr;
    int fbW_ = 0, fbH_ = 0;
};

} // namespace app