#include "models/IdealSeeker.hpp"
#include <cmath>
#include <algorithm>

namespace sim::models 
{
    using namespace sim::math;

    IdealSeeker::IdealSeeker(double min_range)
        : min_range_(min_range) {}

    SeekerOutput IdealSeeker::compute(
        const Vec3d& missile_pos,
        const Vec3d& missile_vel,
        const Vec3d& target_pos,
        const Vec3d& target_vel,
        const Mat3d& body_to_inertial,
        double /*dt*/)
    {
        SeekerOutput out;

        // LOS Geometry in inertial (NED) frame

        // Relative position: target minus missile
        Vec3d R = target_pos - missile_pos;
        double range = R.magnitude();

        // Check if minimum range for valid track
        if (range < min_range_) {
            out.track_valid = false;
            out.range = range;
            return out;
        }

        out.track_valid = true;
        out.range = range;

        // LOS unit vector in NED
        Vec3d los_ned = R / range;

        // Relative velocity in NED: target velocity minus missile velocity
        Vec3d V_rel = target_vel - missile_vel;

        // Range rate: projection of relative velocity onto LOS
        // Negative means closing (missile approaching target)
        out.range_rate = V_rel.dot(los_ned);

        // LOS rate vector in NED
        // LOS_dot = (R x V_rel) / r^2
        //
        // This is the angular velocity of the LOS vector. It captures
        // how the line of sight is rotating in inertial space. This is
        // the quantity that pronav guidance uses.
        Vec3d los_rate_ned = R.cross(V_rel) / (range * range);
        
        // Transform to body frame

        // Inertial-to-body DCM is the transpose of body-to-inertial
        Mat3d L_ib = body_to_inertial.transposed();

        out.los_unit = L_ib * los_ned;
        out.los_rate = L_ib * los_rate_ned;

        // Diagnostic angles

        // Look angle: angle between body X-axis (nose) and LOS
        // cos(look_angle) = los_body · [1, 0, 0] = los_body.x()
        double cos_look = std::clamp(out.los_unit.x(), -1.0, 1.0);
        look_angle_ = std::acos(cos_look);

        // LOS azimuth in NED: angle from North, positive clockwise (East)
        los_azimuth_ = std::atan2(los_ned.y(), los_ned.x());

        // LOS elevation in NED: angle above horizontal, positive up
        // In NED, Down is +Z, so up is -Z
        double horiz = std::sqrt(los_ned.x() * los_ned.x() + los_ned.y() * los_ned.y());
        los_elevation_ = std::atan2(-los_ned.z(), horiz);

        return out;
    }

} // namespace sim::models