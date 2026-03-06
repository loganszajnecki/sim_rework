#pragma once

#include "models/ITarget.hpp"

namespace sim::models
{
    /**
     * @brief Stationary (non-moving) target.
     * 
     * Returns a fixed position with zero velocity and acceleration
     * for all time. Useful for ground targets, waypoints, or as a 
     * baseline for validating seeker and guidance logic.
     */
    class StationaryTarget : public ITarget {
    public:
        explicit StationaryTarget(const sim::math::Vec3d& position);

        [[nodiscard]] TargetState compute(double t) const override;
    
    private:
        sim::math::Vec3d position_;
    };

} // namespace sim::models