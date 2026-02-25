#pragma once

#include "models/IGravity.hpp"

namespace sim::models {

/**
 * @brief Flat-earth constant gravity model.
 *
 * Returns [0, 0, +g] in the NED frame (gravity acts in the +Z/down direction).
 * Sufficient for short-range engagements where altitude variation is small
 * relative to earth radius. The gravity magnitude can be overridden at
 * construction time for planetary or testing scenarios.
 */
class FlatEarthGravity : public IGravity {
public:
    /// Default: standard gravity 9.80665 m/s^2
    explicit FlatEarthGravity(double g = 9.80665);

    [[nodiscard]] sim::math::Vec3d compute(
        const sim::math::Vec3d& position_ned) const override;

private:
    double g_;
};

} // namespace sim::models