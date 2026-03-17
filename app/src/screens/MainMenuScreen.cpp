#include "screens/MainMenuScreen.hpp"
 
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
 
#include "vis/SphereGenerator.hpp"

namespace app {

	MainMenuScreen::MainMenuScreen(const std::string& shaderDir,
        const std::string &resDir) 
    : shaderDir_(shaderDir)
    , resDir_(resDir) {}

    MainMenuScreen::~MainMenuScreen() = default;

    // Lifecycle
    void MainMenuScreen::enter(GLFWwindow* window, int fbW, int fbH) {
        window_ = window;
        fbW_    = fbW;
        fbH_    = fbH;
    
        // Camera
        // Position the camera so the Earth fills roughly the right
        // two-thirds of the screen, with the menu on the left.
        float aspect = (fbW_ > 0 && fbH_ > 0)
                    ? static_cast<float>(fbW_) / static_cast<float>(fbH_)
                    : 16.0f / 9.0f;
    
        camera_ = vis::Camera(
            glm::vec3(2.5f, -4.0f, 1.5f),    // eye: offset right and slightly above
            glm::vec3(0.3f,  0.0f, 0.0f),    // look at: slightly right of center
            45.0f, aspect, 0.1f, 100.0f
        );
    
        // Sun
        // Directional light from upper-right (far away ~ directional).
        sun_ = vis::Light(
            glm::vec3(50.0f, -30.0f, 40.0f),
            glm::vec3(1.0f, 0.98f, 0.95f)   // slightly warm white
        );
    
        // Earth
        // Generate a UV sphere and load it into a VAO.
        auto sphereData = vis::SphereGenerator::generate(1.0f, 80, 40);
        auto rawModel = loader_.loadToVAO(sphereData.positions,
                                        sphereData.texCoords,
                                        sphereData.normals,
                                        sphereData.indices);
    
        // Load Earth texture (or fallback to solid blue).
        std::string earthTexPath = resDir_ + "/earth_daymap.jpg";
        GLuint earthTex = loader_.loadTexture(earthTexPath);
        if (earthTex == 0) {
            std::string fallback = resDir_ + "/earth_daymap.png";
            earthTex = loader_.loadTexture(fallback);
        }
        if (earthTex == 0) {
            // No texture found — create a solid blue sphere as fallback.
            earthTex = loader_.createSolidTexture(30, 80, 180);
        }
    
        vis::ModelTexture earthMat(earthTex);
        earthMat.shineDamper  = 20.0f;
        earthMat.reflectivity = 0.15f;   // subtle specular on oceans
    
        earthModel_ = vis::TexturedModel(rawModel, earthMat);
        earthEntity_ = vis::Entity(&earthModel_,
                                glm::vec3(1.25f, 0.5f, 0.0f),  // origin
                                glm::vec3(0.0f, 0.0f, 0.0f),  // no initial rotation
                                1.8f);                        // scale
    
        // Renderer
        master_ = std::make_unique<vis::MasterRenderer>(
            shaderDir_ + "/entity.vert",
            shaderDir_ + "/entity.frag"
        );
    	master_->setSkyColor(glm::vec3(0.005f, 0.005f, 0.01f));  // very dark space
        
        // Starfield
        starfield_ = std::make_unique<vis::StarfieldRenderer>(
            shaderDir_, 16000, 10.0f);     // 4000 stars on a sphere of radius 80
    }

    void MainMenuScreen::leave() {
    	starfield_.reset();
        master_.reset();
        loader_.cleanup();
    }

    void MainMenuScreen::onResize(int fbW, int fbH) {
        fbW_ = fbW;
        fbH_ = fbH;
        if (fbW_ > 0 && fbH_ > 0) {
            camera_.setAspect(static_cast<float>(fbW_) / static_cast<float>(fbH_));
        }
    }

    // Per frame
    void MainMenuScreen::update(float dt) {
        // Slowly rotate the Earth.
        earthRotation_ += kRotationRate_ * dt;
        if (earthRotation_ > 360.0f) earthRotation_ -= 360.0f;
    
        // Apply rotation around Z axis (Z-up world = Earth's polar axis).
        earthEntity_.rotation.z = earthRotation_;
    
        // Tilt Earth ~23.4 deg (axial tilt) around the X axis.
        earthEntity_.rotation.x = 23.4f;
    
        // Handle ESC key.
        if (glfwGetKey(window_, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
            quit_ = true;
        }
    }

    void MainMenuScreen::render() {
        // 3D scene
        glClearColor(0.005f, 0.005f, 0.01f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    	
        // Starfield behind everything
        starfield_->render(camera_);

        master_->processEntity(earthEntity_);
        master_->render(sun_, camera_);
    
        // ImGui overlay
        renderImGui_();
    }

    void MainMenuScreen::renderImGui_() {
        // Dimensions of the menu panel
        float panelWidth  = 320.0f;
        float panelHeight = 340.0f;
        float panelX      = 40.0f;
        float panelY      = static_cast<float>(fbH_) * 0.5f - panelHeight * 0.5f;
        
        ImGui::SetNextWindowPos(ImVec2(panelX, panelY), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(panelWidth, panelHeight), ImGuiCond_Always);

        ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize
                               | ImGuiWindowFlags_NoMove
                               | ImGuiWindowFlags_NoCollapse
                               | ImGuiWindowFlags_NoTitleBar
                               | ImGuiWindowFlags_NoScrollbar;

        ImGui::Begin("##MainMenu", nullptr, flags);

        // Title
        ImGui::Dummy(ImVec2(0, 8));
    
        // Center the title text.
        const char* title = "MISSILE FLIGHT SIMULATOR";
        float titleWidth = ImGui::CalcTextSize(title).x;
        ImGui::SetCursorPosX((panelWidth - titleWidth) * 0.5f);
        ImGui::TextColored(ImVec4(0.7f, 0.8f, 1.0f, 1.0f), "%s", title);
    
        ImGui::Dummy(ImVec2(0, 4));
        ImGui::Separator();
        ImGui::Dummy(ImVec2(0, 16));
    
        // Buttons
        float btnWidth  = panelWidth - 24.0f;
        float btnHeight = 40.0f;
    
        // Center buttons horizontally.
        float btnX = (panelWidth - btnWidth) * 0.5f;
    
        ImGui::SetCursorPosX(btnX);
        if (ImGui::Button("New Simulation", ImVec2(btnWidth, btnHeight))) {
            nextScreen_ = "config";
        }
    
        ImGui::Dummy(ImVec2(0, 6));
        ImGui::SetCursorPosX(btnX);
        if (ImGui::Button("Load Results", ImVec2(btnWidth, btnHeight))) {
            nextScreen_ = "viewer";
        }
    
        ImGui::Dummy(ImVec2(0, 6));
        ImGui::SetCursorPosX(btnX);
        if (ImGui::Button("Settings", ImVec2(btnWidth, btnHeight))) {
            // Stubbed for now — will open a settings screen.
        }
    
        ImGui::Dummy(ImVec2(0, 6));
        ImGui::SetCursorPosX(btnX);
        if (ImGui::Button("Exit", ImVec2(btnWidth, btnHeight))) {
            quit_ = true;
        }
    
        // Version footer
        ImGui::Dummy(ImVec2(0, 12));
        const char* ver = "v3.0.0-alpha";
        float verWidth = ImGui::CalcTextSize(ver).x;
        ImGui::SetCursorPosX((panelWidth - verWidth) * 0.5f);
        ImGui::TextDisabled("%s", ver);
    
        ImGui::End();
    }

} // namespace app