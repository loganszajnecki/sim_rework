#pragma once

#include "models/ITarget.hpp"

namespace sim::models {
    /**
     * @brief Target moving at constant velocity.
     * 
     * Position evolves linearly: p(t) = p0 + v0*t
     * Acceleration is zero for all time.
     * 
     * Typical use: incoming aircraft, ballistic debris, or any
     * non-maneuvering threat o a straight line trajectory.
     */
    class ConstantVelocityTarget : public ITarget 
    {
    public:
        ConstantVelocityTarget(const sim::math::Vec3d& position,
                            const sim::math::Vec3d& velocity);

        [[nodiscard]] TargetState compute(double t) const override;

    private:
        sim::math::Vec3d position_;
        sim::math::Vec3d velocity_;
    };

} // namespace sim::models