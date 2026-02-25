#pragma once

#include "math/Vector3.hpp"
#include <memory>

namespace sim::models {

/**
 * @brief Abstract fin actuator model interface.
 *
 * Models the dynamics of fin deflection including rate limits,
 * position limits, and optional second-order lag.
 *
 * Usage:
 *   std::unique_ptr<IActuator> fins = std::make_unique<RateLimitedActuator>(config);
 *   fins->command(commanded_deflections);
 *   fins->update(dt);
 *   auto actual = fins->current_deflections();
 */
class IActuator {
public:
    virtual ~IActuator() = default;

    /// Set the commanded deflections (from autopilot)
    virtual void command(const sim::math::Vec3d& cmd_deflections) = 0;

    /// Advance actuator state by dt (applies rate/position limits)
    virtual void update(double dt) = 0;

    /// Get actual (rate-limited) deflections
    [[nodiscard]] virtual sim::math::Vec3d current_deflections() const = 0;
};

} // namespace sim::models