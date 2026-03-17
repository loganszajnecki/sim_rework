#include "models/ProNavGuidance.hpp"
#include <cmath>

namespace sim::models {
    
    ProNavGuidance::ProNavGuidance(double nav_ratio)
        : nav_ratio_(nav_ratio) {}

    GuidanceCommand ProNavGuidance::compute(
        const SeekerOutput& seeker_data,
        double /*missile_speed*/) const
    {
        GuidanceCommand cmd;

        // No valid track -> zero command
        // TODO: this isnt always true. Waypoint planning exists
        // You don't always need to be in track to command accels.
        // For now, we will assume so.
        if (!seeker_data.track_valid) {
            cmd.accel_cmd = sim::math::Vec3d::zero();
            return cmd;
        }

        // Closing velocity: positive when range is decreasing
        double Vc = -seeker_data.range_rate;

        // If target is opening (Vc < 0), no useful guidance
        if (Vc < 1e-6) {
            cmd.accel_cmd = sim::math::Vec3d::zero();
            return cmd;
        }

        // True Proportional Navigation:
        //   a_cmd = N * Vc * (LOS_RATE x u_hat)
        //
        // The cross product produces an acceleration vector that is:
        //   - Perpendicular to the LOS (no axial component)
        //   - In the plane of LOS rotation
        //   - With magnitude equal to the scalar LOS rate
        //
        // All vectors are in the body frame to match seeker output
        cmd.accel_cmd = seeker_data.los_rate.cross(seeker_data.los_unit)
                      * (Vc*nav_ratio_);

        return cmd;
    }

} // namespace sim::models