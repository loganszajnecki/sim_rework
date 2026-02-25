#pragma once

#include "math/Vector3.hpp"
#include <memory>

namespace sim::models {

/**
 * @brief Abstract gravity model interface.
 *
 * Computes the gravitational acceleration vector at a given position
 * in the inertial (NED) frame. Concrete implementations might include
 * constant-g, WGS-84 ellipsoidal, or J2 perturbation models.
 */
class IGravity {
public:
    virtual ~IGravity() = default;

    /**
     * @param position_ned  Vehicle position in the NED inertial frame (m)
     * @return Gravitational acceleration in the NED frame (m/s^2)
     *         For flat earth: [0, 0, +g] since NED +Z is down
     */
    [[nodiscard]] virtual sim::math::Vec3d compute(
        const sim::math::Vec3d& position_ned) const = 0;
};

} // namespace sim::models