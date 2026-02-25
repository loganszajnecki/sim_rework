#pragma once

#include "math/Vector3.hpp"
#include <memory>

namespace sim::models {

/**
 * @brief Abstract wind and turbulence model interface.
 *
 * Returns the wind velocity at a given position and time in the
 * inertial (NED) frame. The wind vector is subtracted from the
 * inertial velocity to obtain airspeed, which feeds into aero
 * calculations (Mach, dynamic pressure, alpha, beta).
 *
 * Concrete implementations: no-wind, constant wind, altitude-
 * dependent profiles, or stochastic turbulence (Dryden/von Karman).
 */
class IWind {
public:
    virtual ~IWind() = default;

    /**
     * @param position_ned  Vehicle position in NED (m)
     * @param t             Simulation time (s), needed for turbulence models
     * @return Wind velocity in the NED inertial frame (m/s)
     */
    [[nodiscard]] virtual sim::math::Vec3d compute(
        const sim::math::Vec3d& position_ned,
        double t) const = 0;
};

} // namespace sim::models