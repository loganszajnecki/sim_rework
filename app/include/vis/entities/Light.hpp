#pragma once

#include <glm/glm.hpp>

namespace vis {

    /**
     * @brief Simple light source for Phong shading.
     *
     * Currently treated as a point light (position + color).
     * For the landing page sun, place it far away to approximate directional.
     */
    struct Light {
        glm::vec3 position{0.0f, 0.0f, 0.0f};
        glm::vec3 color{1.0f, 1.0f, 1.0f};

        Light() = default;
        Light(const glm::vec3& pos, const glm::vec3& col)
            : position(pos), color(col) {}
    };

} // namespace vis