#pragma once

#include "models/IAutopilot.hpp"

namespace sim::models {
    /**
     * @brief Simple proportional autopilot with rate damping. 
     * 
     * Converts guidance acceleration commands to fin deflections using
     * a proportional gain with body-rate feedback for stability. 
     * 
     *   delta_pitch = Kp * a_cmd_z - Kd * q
     *   delta_yaw   = Kp * a_cmd_y - Kd * r
     *   delta_roll  = -Kd_roll * p         (roll damper only, no roll command)
     * 
     * Where:
     *   a_cmd_y, a_cmd_z — lateral acceleration commands (body frame, m/s^2)
     *   p, q, r          — body angular rates (rad/s)
     *   Kp               — command gain (rad / (m/s^2))
     *   Kd               — rate damping gain (rad / (rad/s))
     *   Kd_roll          — roll damping gain (rad / (rad/s))
     * 
     * Fin deflections are clambed to a configurable maximum to represent 
     * physical limits of the control surfaces.
     * 
     * This is a placeholder for downstream integration. A proper three-loop
     * autopilot (acceleration, rate, position loops with gain scheduling)
     * will replace this later. The key insight is that PN guidance is 
     * robust enough to work with an imperfect autopilot - the outer
     * guidance loop compensates for inner-loop tracking errors. 
     * 
     * Mapping convention:
     *   - Positive a_cmd_z (body ventral) -> positive pitch fin deflection
     *     -> nose down moment -> generates a positive normal force
     *   - Positive a_cmd_y (body starboard) -> positive yaw fin deflection
     *     -> nose-right moment -> generates a positive side force
     */
    class SimpleAutopilot : public IAutopilot
    {
    public:
        struct Gains {
            double Kp{0.001};         // Command gain: fin rad per m/s&2 accel
            double Kd{0.5};           // Pitch/yaw rate damping: fin rad per rad/s
            double Kd_roll{0.2};      // Roll rate damping: fin rad per rad/s
            double max_deflection{0.4};  // Max fin deflection (rad, ~23 degrees)
        };

        SimpleAutopilot();
        explicit SimpleAutopilot(const Gains& gains);

        [[nodiscard]] sim::math::Vec3d compute(
            const GuidanceCommand& guidance_cmd,
            const sim::math::Vec3d& omega_body,
            const sim::math::Vec3d& velocity_body,
            double dynamic_pressure,
            double dt) override;
        
    private:
        Gains gains_;
    };

} // namespace sim::models