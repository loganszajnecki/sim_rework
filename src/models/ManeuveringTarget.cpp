#include "models/ManeuveringTarget.hpp"
#include <cmath>

namespace sim::models 
{
    ManeuveringTarget::ManeuveringTarget(
        const sim::math::Vec3d& position,
        const sim::math::Vec3d& velocity,
        double maneuver_g,
        double maneuver_start)
        : position_(position)
        , velocity_(velocity)
        , maneuver_accel_(maneuver_g * 9.80665)
        , maneuver_start_(maneuver_start)
    {
        // Compute maneuver direction: perpendicular to the velocity in the
        // horizontal plane. Rotate the horizontal velocity component by 90 
        // deg about the down axis
        double vn = velocity_.x();
        double ve = velocity_.y();
        double horiz_speed = std::sqrt(vn * vn + ve * ve);

        if (horiz_speed > 1e-6) {
            maneuver_dir_ = sim::math::Vec3d{-ve / horiz_speed, vn / horiz_speed, 0.0};
        } else {
            // Velocity is purely vertical; default maneuver to east
            maneuver_dir_ = sim::math::Vec3d{0.0, 1.0, 0.0};
        }
    }

    TargetState ManeuveringTarget::compute(double t) const {
        if (t < maneuver_start_) {
            // Phase 1: straight line flight
            return {
                position_ + velocity_ * t,
                velocity_,
                sim::math::Vec3d::zero()
            };
        }

        // Phase 2: constant g lateral maneuver
        sim::math::Vec3d pos_at_start = position_ + velocity_ * maneuver_start_;

        double dt = t - maneuver_start_;
        sim::math::Vec3d accel = maneuver_dir_ * maneuver_accel_;

        // p(t) = p(t_start) + v0 * dt + 0.5 * a * dt^2
        sim::math::Vec3d pos = pos_at_start + velocity_ * dt + accel * (0.5 * dt * dt);

        // v(t) = v0 + a * dt
        sim::math::Vec3d vel = velocity_ + accel * dt;

        return {pos, vel, accel};
    }

} // namespace sim::models