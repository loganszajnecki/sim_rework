#pragma once

#include "math/Vector3.hpp"
#include <memory>

namespace sim::models
{
    /**
     * Aerodynamic forces and moments in the body frame
     */
    struct AeroOutput {
        sim::math::Vec3d force;   // [Fx, Fy, Fz] body frame (N)
        sim::math::Vec3d moment;  // [L, M, N] body frame (N*m) 
    };

    /**
     * @brief Abstract aerodynamics model interface.
     *
     * Concrete implementations compute body-frame forces and moments
     * from the current flight condition. Owned via std::unique_ptr.
     *
     * Usage:
     *   std::unique_ptr<IAerodynamics> aero = std::make_unique<TabulatedAero>(config);
     *   auto result = aero->compute(mach, alpha, beta, omega, fin_deflections, qbar);
     */
    class IAerodynamics {
    public:
        virtual ~IAerodynamics() = default;

        /**
         * @param mach              Mach number
         * @param alpha_rad         Angle of attack (rad)
         * @param beta_rad          Sideslip angle (rad)
         * @param omega_body        Body angular rates [p,q,r] (rad/s)
         * @param fin_deflections   Control surface deflections (rad)
         * @param dynamic_pressure  q = 0.5*rho*v^2 (Pa)
         * @param ref_area          Reference area (m^2)
         * @param ref_length        Reference length / diameter (m)
         */
        [[nodiscard]] virtual AeroOutput compute(
            double mach, 
            double alpha_rad,
            double beta_rad,
            const sim::math::Vec3d& omega_body,
            const sim::math::Vec3d& fin_deflections,
            double dynamic_pressure,
            double ref_area,
            double ref_length) const = 0;
    };

} // namespace sim::models