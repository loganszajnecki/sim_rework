#include "models/SimpleAutopilot.hpp"
#include <cmath>
#include <algorithm>

namespace sim::models {
    
    SimpleAutopilot::SimpleAutopilot()
        : gains_{} {}

    SimpleAutopilot::SimpleAutopilot(const Gains& gains)
        : gains_(gains) {}

    sim::math::Vec3d SimpleAutopilot::compute(
        const GuidanceCommand& guidance_cmd,
        const sim::math::Vec3d& omega_body,
        const sim::math::Vec3d& /*velocity_body*/,
        double /*dynamic_pressure*/,
        double /*dt*/)
    {
        double p = omega_body.x(); // roll rate
        double q = omega_body.y(); // pitch rate
        double r = omega_body.z(); // yaw rate

        // Guidance acceleration commands in body frame
        double a_cmd_y = guidance_cmd.accel_cmd.y(); // starboard
        double a_cmd_z = guidance_cmd.accel_cmd.z(); // ventral

        // Pitch channel
        // Positive a_cmd_z -> need nose down pitching moment
        // Rate damping opposes current pitch rate
        double delta_pitch = gains_.Kp * a_cmd_z - gains_.Kd * q;

        // Yaw channel
        // Positive a_cmd_y -> need nose-right yawing moment
        // Rate damping opposes current yaw rate
        double delta_yaw = gains_.Kp * a_cmd_y - gains_.Kd * r;

        // Roll channel
        // No roll command from PN guidance; just damp any roll rate
        double delta_roll = -gains_.Kd_roll * p;

        // Clamp to physical limits
        double max = gains_.max_deflection;
        delta_roll  = std::clamp(delta_roll,  -max, max);
        delta_pitch = std::clamp(delta_pitch, -max, max);
        delta_yaw   = std::clamp(delta_yaw,   -max, max);

        // Return as [roll, pitch, yaw] fin deflections (rad)
        return {delta_roll, delta_pitch, delta_yaw};
    }

} // namespace sim::models