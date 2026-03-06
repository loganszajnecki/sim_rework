#pragma once

#include "models/IGuidance.hpp"

namespace sim::models 
{
    /**
     * @brief True Proportional Navigation (TPN) guidance law.
     *
     * Commands an acceleration perpendicular to the line of sight,
     * proportional to the closing velocity and the LOS rotation rate.
     *
     * 3D formulation:
     *   a_cmd = N * Vc * (LOS_RATE × u_hat)
     *
     * Where:
     *   N        - Navigation ratio (dimensionless, typically 3–5)
     *   Vc       - Closing velocity (m/s), equals -range_rate
     *   LOS_RATE - LOS rate vector from the seeker (rad/s)
     *   u_hat    - LOS unit vector from the seeker
     *
     * The cross product LOS_RATE × u_hat produces an acceleration that:
     *   - Is perpendicular to the LOS (no wasted effort along range)
     *   - Lies in the plane of LOS rotation
     *
     * This naturally handles 3D engagements without decomposing into
     * separate pitch/yaw planes. When LOS_RATE = 0, the missile is on a
     * collision course and no correction is needed.
     *
     * Notes on N:
     *   N = 3 : theoretical minimum for a non-maneuvering target
     *   N = 4 : common design choice, good balance of performance
     *   N = 5 : aggressive, fast convergence, high acceleration demand
     *
     * The output acceleration is in the body frame, matching the
     * convention expected by the autopilot.
     */
    class ProNavGuidance : public IGuidance 
    {
    public:
        /// @param nav_ratio  Navigation ratio N (default: 4)
        explicit ProNavGuidance(double nav_ratio = 4.0);

        [[nodiscard]] GuidanceCommand compute(
            const SeekerOutput& seeker_data,
            double missile_speed) const override;

    private:
        double nav_ratio_;
    };

} // namespace sim::models