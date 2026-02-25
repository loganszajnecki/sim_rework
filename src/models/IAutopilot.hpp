#pragma once

#include "math/Vector3.hpp"
#include "models/IGuidance.hpp"
#include <memory>

namespace sim::models {

/**
 * @brief Abstract autopilot / flight controller interface.
 *
 * The inner-loop controller that translates guidance acceleration
 * commands into fin deflection commands. Concrete implementations:
 * PID controller, gain-scheduled classical controller, etc.
 *
 * Sits in the signal chain between guidance and the actuators:
 *   Guidance -> Autopilot -> Actuator -> Aerodynamics
 */
class IAutopilot {
public:
    virtual ~IAutopilot() = default;

    /**
     * @param guidance_cmd   Commanded acceleration from guidance law
     * @param omega_body     Current body angular rates [p,q,r] (rad/s)
     * @param velocity_body  Current body velocity [u,v,w] (m/s)
     * @param dynamic_pressure Current q=0.5*rho*v^2 (Pa)
     * @param dt             Time step (s), for integral/derivative terms
     * @return Fin deflection commands [roll, pitch, yaw] (rad)
     */
    [[nodiscard]] virtual sim::math::Vec3d compute(
        const GuidanceCommand& guidance_cmd,
        const sim::math::Vec3d& omega_body,
        const sim::math::Vec3d& velocity_body,
        double dynamic_pressure,
        double dt) = 0;
};

} // namespace sim::models