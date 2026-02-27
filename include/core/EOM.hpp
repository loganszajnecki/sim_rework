#pragma once

#include "core/State.hpp"
#include "core/Vehicle.hpp"

namespace sim::core
{
    /**
     * @brief 6-DOF Equations of Motion.
     * 
     * Computes the full state derivative by querying all subsystem models
     * through the Vehicle. This is the function the integratir calls at 
     * every time step.
     * 
     * Signal flow:
     *   1. Atmosphere   -> density, speed of sound, Mach, dynamic pressure
     *   2. Wind         -> airspeed correction -> alpha, beta (not implemented yet)
     *   3. Gravitry     -> gravity in NED -> rotate to body frame
     *   4. Propulsion   -> thrust force/moment, mass flow, inertia tensor
     *   5. Actuator     -> current fin deflections (not yet implemented)
     *   6. Aerodynamics -> body-frame forces and moments
     *
     */
    class EOM
    {
    public:
        /**
         * @brief Compute the state derivative.
         * 
         * @param t        Current simulation time (s)
         * @param state    Current state vector
         * @param vehicle  Vehicle containing all subsystem models
         * @return Full state derivative for the integrator
         */
        [[nodiscard]] static StateDerivative compute(
            double t,
            const State& state,
            const Vehicle& vehicle);
    };
}