#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace vis {

    /**
     * @brief Perspective camera for 3D rendering.
     * 
     * Stores position + orientation and generates view and projection
     * matrices for the shader pipeline. 
     * 
     * The camera uses a look-at model: it has a position and a target
     * point it faces. The up vector defaults to +Z (Z-up world)
     * 
     * For the landing page, the camera is static. 
     * TODO: Add CameraController for orbit/pan/zoom camera control during flyout.
     */
    class Camera 
    {
    public:
        Camera() = default;

        Camera(const glm::vec3& position,
               const glm::vec3& target,
               float fovDeg, float aspect, float nearClip, float farClip)
            : position_(position), target_(target), fovDeg_(fovDeg)
            , aspect_(aspect), near_(nearClip), far_(farClip) {}
        
        // Matrices
        [[nodiscard]] glm::mat4 viewMatrix() const {
            return glm::lookAt(position_, target_, up_);
        }

        [[nodiscard]] glm::mat4 projectionMatrix() const {
            return glm::perspective(glm::radians(fovDeg_), aspect_, near_, far_);
        }

        // Setters

        void setPosition(const glm::vec3& p)       { position_ = p; }
        void setTarget(const glm::vec3& t)         { target_   = t; }
        void setUp(const glm::vec3& u)             { up_       = u; }
        void setFov(float deg)                     { fovDeg_   = deg; }
        void setAspect(float a)                    { aspect_   = a; }
        void setClipPlanes(float n, float f)       { near_ = n; far_ = f; }

        // Getters
        [[nodiscard]] const glm::vec3& position() const { return position_; }
        [[nodiscard]] const glm::vec3& target()   const { return target_; }
        [[nodiscard]] float fov()    const { return fovDeg_; }
        [[nodiscard]] float aspect() const { return aspect_; }
        [[nodiscard]] float nearClip() const { return near_; }
        [[nodiscard]] float farClip()  const { return far_; }

    private:
        glm::vec3 position_{0.0f, -5.0f, 2.0f};
        glm::vec3 target_{0.0f, 0.0f, 0.0f};
        glm::vec3 up_{0.0f, 0.0f, 1.0f};       // Z-up world.
    
        float fovDeg_ = 45.0f;
        float aspect_ = 16.0f / 9.0f;
        float near_   = 0.1f;
        float far_    = 1000.0f;
    };

} // namespace vis