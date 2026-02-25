#pragma once

#include "math/Vector3.hpp"
#include "models/ISeeker.hpp"
#include <memory>

namespace sim::models {

/**
 * @brief Guidance command output.
 */
struct GuidanceCommand {
    sim::math::Vec3d accel_cmd;  // Commanded acceleration, body frame (m/s^2)
                                  // Typically lateral (y,z), axial may be zero
};

/**
 * @brief Abstract guidance law interface.
 *
 * Takes seeker measurements and missile state, computes the
 * acceleration command needed to intercept the target. Concrete
 * implementations: proportional navigation (PN), augmented PN,
 * command-to-line-of-sight (CLOS), pursuit guidance, etc.
 *
 * Sits in the signal chain between the seeker and the autopilot:
 *   Seeker → Guidance → Autopilot → Actuator
 */
class IGuidance {
public:
    virtual ~IGuidance() = default;

    /**
     * @param seeker_data  Current seeker measurements (LOS, range, etc.)
     * @param missile_speed Current missile speed (m/s)
     */
    [[nodiscard]] virtual GuidanceCommand compute(
        const SeekerOutput& seeker_data,
        double missile_speed) const = 0;
};

} // namespace sim::models