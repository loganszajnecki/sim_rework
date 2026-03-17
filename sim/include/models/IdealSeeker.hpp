#pragma once

#include "models/ISeeker.hpp"

namespace sim::models {
    /**
     * @brief Ideal (truth) seeker model.
     *
     * Provides perfect line-of-sight measurements with no noise, no
     * gimbal limits, no field-of-view constraints, and no track latency.
     * Serves as the baseline for validating guidance algorithms before
     * introducing realistic sensor effects.
     *
     * Computes from first principles:
     *   - LOS unit vector:  u_hat = (p_tgt - p_msl) / |p_tgt - p_msl|
     *   - Range:            r = |p_tgt - p_msl|
     *   - Range rate:       rdot = (dot(u,V_rel))   (negative = closing)
     *   - LOS rate:         LOS_dot = (R × V_rel) / r^2
     *
     * All outputs are provided in both inertial (NED) and body frames.
     * The guidance law typically uses body-frame LOS rate.
     *
     * The 'track_valid' flag is true whenever range is above a minimum
     * threshold (prevents division by zero at intercept).
     */
    class IdealSeeker : public ISeeker 
    {
    public:
        /// Minimum range for valid track (m). Below this, track is lost
        explicit IdealSeeker(double min_range = 1.0);

        [[nodiscard]] SeekerOutput compute(
            const sim::math::Vec3d& missile_pos,
            const sim::math::Vec3d& missile_vel,
            const sim::math::Vec3d& target_pos,
            const sim::math::Vec3d& target_vel,
            const sim::math::Mat3d& body_to_inertial,
            double dt) override;

        /// Look angle: angle between missile body x and LOS (rad)
        /// Valid after the most recent compute() call.
        [[nodiscard]] double look_angle() const { return look_angle_; }

        /// LOS elevation angle in NED frame (rad, positive up).
        /// Valid after the most recent compute() call.
        [[nodiscard]] double los_elevation() const { return los_elevation_; }

        /// LOS azimuth angle in NED frame (rad, from North, positive clockwise).
        /// Valid after the most recent compute() call.
        [[nodiscard]] double los_azimuth() const { return los_azimuth_; }

    private:
        double min_range_;
        double look_angle_{0.0};
        double los_elevation_{0.0};
        double los_azimuth_{0.0};
    };

} // namespace sim::models