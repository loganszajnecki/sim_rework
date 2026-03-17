#include "models/StationaryTarget.hpp"

namespace sim::models {
    StationaryTarget::StationaryTarget(const sim::math::Vec3d& position)
        : position_(position) {}

    TargetState StationaryTarget::compute(double t) const {
        return {
            position_,
            sim::math::Vec3d::zero(),
            sim::math::Vec3d::zero()
        };
    }

} // namespace sim::models