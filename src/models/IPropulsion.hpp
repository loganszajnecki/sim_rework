#pragma once

#include "math/Vector3.hpp"
#include "math/Matrix3.hpp"
#include <memory>

namespace sim::models
{
    /**
     * @brief Propulsion output at a given time.
     */
    struct PropulsionOutput {
        sim::math::Vec3d force;     // Thrust vector, body frame (N)
        sim::math::Vec3d moment;    // Thrust-induced moment about CG, body frame (N*m)
        double mass_flow_rate{0.0}; // dm/dt (kg/s, negative = mass leaving)
    };

    /**
     * @brief Mass properties that change as propellant burns.
     */
    struct MassProperties {
        double mass{0.0};                             // Total vehicle mass (kg)
        sim::math::Vec3d cg_body;                     // CG location in body frame (m)
        sim::math::Mat3d inertia;                     // Inertia tensor about CG (kg·m²)
    };

    /**
     * @brief Abstract propulsion model interface.
     *
     * Provides thrust, mass flow, and time-varying mass properties.
     * Owned via std::unique_ptr.
     *
     * Usage:
     *   std::unique_ptr<IPropulsion> motor = std::make_unique<SolidRocketMotor>(config);
     *   auto thrust = motor->compute_thrust(t);
     *   auto mass   = motor->compute_mass_properties(t);
     */
    class IPropulsion {
    public:
        virtual ~IPropulsion() = default;

        /// Compute thrust force, moment, and mass flow at time t
        [[nodiscard]] virtual PropulsionOutput compute_thrust(double t) const = 0;

        /// Compute current mass, CG, and inertia tensor at time t
        [[nodiscard]] virtual MassProperties compute_mass_properties(double t) const = 0;

        /// Is the motor still burning at time t?
        [[nodiscard]] virtual bool is_burning(double t) const = 0;
    };

} // namespace sim::models