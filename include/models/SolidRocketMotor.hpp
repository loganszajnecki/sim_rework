#pragma once

#include "models/IPropulsion.hpp"

namespace sim::models
{
    /**
     * @brief Simple solid rocket motor model.
     *
     * Produces constant thrust along the body +X axis for a fixed burn
     * duration, then zero thrust. Mass decreases linearly during the burn.
     * Inertia tensor scales linearly with mass ratio during the burn.
     *
     * Configuration parameters:
     *   - thrust:       Constant thrust magnitude during burn (N)
     *   - burn_time:    Duration of burn (s)
     *   - total_mass:   Initial total vehicle mass (kg)
     *   - prop_mass:    Propellant mass (kg), consumed during burn
     *   - inertia:      Full 3×3 inertia tensor at launch (kg·m^2)
     *                    Can be diagonal for simple shapes or full for
     *                    CAD-derived geometry with products of inertia.
     *   - cg_body:      CG location in body frame at launch (m)
     */
    class SolidRocketMotor : public IPropulsion
    {
    public:
        struct Config {
            double thrust{5000.0};       // N
            double burn_time{3.0};       // s
            double total_mass{100.0};    // kg
            double prop_mass{30.0};      // kg

            // Full inertia tensor at launch (kg·m^2)
            // Default: axisymmetric body (Iyy = Izz), no products of inertia
            sim::math::Mat3d inertia{
                0.5, 0.0, 0.0,
                0.0, 10.0, 0.0,
                0.0, 0.0, 10.0
            };

            // CG location in body frame at launch (m)
            sim::math::Vec3d cg_body{1.0, 0.0, 0.0};
        };

        explicit SolidRocketMotor(const Config& cfg);

        [[nodiscard]] PropulsionOutput compute_thrust(double t) const override;
        [[nodiscard]] MassProperties compute_mass_properties(double t) const override;
        [[nodiscard]] bool is_burning(double t) const override;
    private:
        Config cfg_;
        double mass_flow_rate_;  // kg/s (magnitude, positive value)
    };

} // namespace sim::models
