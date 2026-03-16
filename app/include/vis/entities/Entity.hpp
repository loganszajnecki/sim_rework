#pragma once

#include <glm/glm.hpp>
#include "vis/models/TexturedModel.hpp"

namespace vis {

    /** 
     * @brief A single renderable instance of a TexturedModel
     * 
     * Stores transform data (position, rotation, scale) and a non-owning 
     * pointer to a shared TexturedModel.
     * 
     * Coordinate system:
     *   - Rendering world is Z-up (+X right, +Y forward, +Z up).
     *   - Rotation in degrees, applied in XYZ order.
     *   - Position in meters.
     * 
     * Ownership:
     *   The Entity does NOT own its TexturedModel. The caller must ensure
     *   the TexturedModel outlives any Entity that references it. 
     */
    struct Entity {
        TexturedModel* model = nullptr;  ///< Non-owning.
        glm::vec3 position{0.0f};        ///< World position (meters, Z-up).
        glm::vec3 rotation{0.0f};        ///< Euler angles in degrees (rotX, rotY, rotZ).
        float     scale = 1.0f;          ///< Uniform scale factor.

        Entity() = default;
        Entity(TexturedModel* m, const glm::vec3& pos,
               const glm::vec3& rot, float s)
            : model(m), position(pos), rotation(rot), scale(s) {}

        void translate(const glm::vec3 d) { position += d; }
        void rotate(const glm::vec3& d)    { rotation += d; }
    };
    
} // namespace vis