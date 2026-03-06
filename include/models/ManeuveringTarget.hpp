#pragma once

#include "models/ITarget.hpp"

namespace sim::models 
{
    /**
     * @brief Target that flies straight, then executes a constant g-maneuver.
     * 
     * Phase 1 (t < maneuver_start): straight line flight at constant velocity.
     * Phase 2 (t >= maneuver_start): applies a constant lateral acceleration
     *   perpendicular to the initial velocity vector in the horizontal plane.
     * 
     * The maneuver direction is computed automatically as the horizontal
     * perpendicular to the initial velocity (rotated 90 deg about the Down axis).
     * 
     * Parameters:
     *   - position:       Initial position in NED (m)
     *   - velocity:       Initial velocity in NED (m/s)
     *   - maneuver_g:     Lateral acceleration magnitude (g's)
     *   - maneuver_start: Time to begin maneuvering (s)
     */
    class ManeuveringTarget : public ITarget {
    public:
        ManeuveringTarget(const sim::math::Vec3d& position,
                        const sim::math::Vec3d& velocity,
                        double maneuver_g,
                        double maneuver_start);

        [[nodiscard]] TargetState compute(double t) const override;

    private:
        sim::math::Vec3d position_;
        sim::math::Vec3d velocity_;
        double maneuver_accel_;         // Lateral acceleration magnitude (m/s^2)
        double maneuver_start_;         // Time maneuver begins (s)
        sim::math::Vec3d maneuver_dir_; // Unit vector: maneuver direction
    };

} // namespace sim::models