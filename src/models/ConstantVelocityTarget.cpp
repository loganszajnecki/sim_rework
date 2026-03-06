#include "models/ConstantVelocityTarget.hpp"

namespace sim::models 
{
    ConstantVelocityTarget::ConstantVelocityTarget(
        const sim::math::Vec3d& position,
        const sim::math::Vec3d& velocity)
        : position_(position), velocity_(velocity) {}

    TargetState ConstantVelocityTarget::compute(double t) const {
        return {
            position_ + velocity_ * t,
            velocity_,
            sim::math::Vec3d::zero()
        };
    }
    
} // namespace sim::models