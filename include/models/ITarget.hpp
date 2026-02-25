#pragma once

#include "math/Vector3.hpp"
#include <memory>

namespace sim::models {

/**
 * @brief Target state at a given time.
 */
struct TargetState {
    sim::math::Vec3d position;     // NED inertial frame (m)
    sim::math::Vec3d velocity;     // NED inertial frame (m/s)
    sim::math::Vec3d acceleration; // NED inertial frame (m/s^2)
};

/**
 * @brief Abstract target kinematics interface.
 *
 * Provides the target's position, velocity, and acceleration as a
 * function of time. Concrete implementations: stationary, constant-
 * velocity, weaving/maneuvering, or driven by recorded trajectory data.
 */
class ITarget {
public:
    virtual ~ITarget() = default;

    /// Get target state at time t
    [[nodiscard]] virtual TargetState compute(double t) const = 0;
};

} // namespace sim::models