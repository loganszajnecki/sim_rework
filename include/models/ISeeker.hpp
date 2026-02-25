#pragma once

#include "math/Vector3.hpp"
#include "math/Matrix3.hpp"
#include <memory>

namespace sim::models {

/**
 * @brief Seeker measurement output.
 */
struct SeekerOutput {
    sim::math::Vec3d los_unit;    // Line-of-sight unit vector, body frame
    sim::math::Vec3d los_rate;    // LOS rate vector (rad/s), body frame
    double range{0.0};            // Range to target (m)
    double range_rate{0.0};       // Closing velocity (m/s, negative = closing)
    bool track_valid{false};      // True if seeker has a valid track
};

/**
 * @brief Abstract seeker / sensor model interface.
 *
 * Models the missile's target tracking sensor. Takes the missile
 * and target states, produces line-of-sight measurements that
 * feed into the guidance law. Concrete implementations can add
 * noise, gimbal limits, field-of-view constraints, and track logic.
 */
class ISeeker {
public:
    virtual ~ISeeker() = default;

    /**
     * @param missile_pos       Missile position, NED (m)
     * @param missile_vel       Missile velocity, NED (m/s)
     * @param target_pos        Target position, NED (m)
     * @param target_vel        Target velocity, NED (m/s)
     * @param body_to_inertial  DCM or quaternion (passed as body-to-inertial DCM)
     * @param dt                Time step since last update (s)
     */
    [[nodiscard]] virtual SeekerOutput compute(
        const sim::math::Vec3d& missile_pos,
        const sim::math::Vec3d& missile_vel,
        const sim::math::Vec3d& target_pos,
        const sim::math::Vec3d& target_vel,
        const sim::math::Mat3d& body_to_inertial,
        double dt) = 0;
};

} // namespace sim::models