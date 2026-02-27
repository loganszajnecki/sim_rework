#pragma once

#include "models/IAerodynamics.hpp"

namespace sim::models {

    /**
     * @brief Simple constant-coefficient aerodynamics model.
     *
     * Computes body-frame forces and moments from constant aerodynamic
     * coefficients. Suitable for initial framework validation before
     * tabulated or higher-fidelity aero models are available.
     *
     * Force coefficients (body frame):
     *   CA  — axial force coefficient (drag along +X)
     *   CN  — normal force coefficient (per radian of alpha)
     *   CY  — side force coefficient (per radian of beta)
     *
     * Moment coefficients (body frame):
     *   Cm  — pitching moment coefficient (per radian of alpha)
     *   Cn  — yawing moment coefficient (per radian of beta)
     *   Cl  — rolling moment coefficient (per radian of fin deflection)
     *
     * Damping derivatives (per rad/s, normalized by V/d):
     *   Cmq — pitch damping
     *   Cnr — yaw damping
     *   Clp — roll damping
     *
     * Forces:  F = q * S_ref * C
     * Moments: M = q * S_ref * d_ref * C
     */
    class SimpleAero : public IAerodynamics
    {
    public:
        struct Coefficients {
            // Static coefficients
            double CA{0.3};       // Axial (drag)
            double CN_alpha{10.0}; // Normal force slope (per rad)
            double CY_beta{-10.0}; // Side force slope (per rad)

            // Static moment coefficients
            double Cm_alpha{-3.0};  // Pitch moment slope (per rad, negative = stable)
            double Cn_beta{3.0};    // Yaw moment slope (per rad, positive = stable)
            double Cl_delta{-0.5};  // Roll moment due to fin deflection (per rad)

            // Damping derivatives (normalized by d / (2V))
            double Cmq{-20.0};     // Pitch damping (negative = damping)
            double Cnr{-20.0};     // Yaw damping (negative = damping)
            double Clp{-5.0};      // Roll damping (negative = damping)
        };

        SimpleAero();
        explicit SimpleAero(const Coefficients& coeffs);

        [[nodiscard]] AeroOutput compute(
            double mach,
            double alpha_rad,
            double beta_rad,
            const sim::math::Vec3d& omega_body,
            const sim::math::Vec3d& fin_deflections,
            double dynamic_pressure,
            double ref_area,
            double ref_length) const override;
    private:
        Coefficients coeffs_;
    };



} // namespace sim::models