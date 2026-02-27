#include "models/SimpleAero.hpp"
#include <cmath>

namespace sim::models {

    SimpleAero::SimpleAero() : coeffs_{} {}

    SimpleAero::SimpleAero(const Coefficients& coeffs) : coeffs_(coeffs) {}

    AeroOutput SimpleAero::compute(
        double /*mach*/,
        double alpha_rad,
        double beta_rad,
        const sim::math::Vec3d& omega_body,
        const sim::math::Vec3d& fin_deflections,
        double dynamic_pressure,
        double ref_area,
        double ref_length) const
    {
        AeroOutput out;

        double qS = dynamic_pressure * ref_area;
        double qSd = qS * ref_length;

        // Body angular rates
        double p = omega_body.x();  // roll rate
        double q = omega_body.y();  // pitch rate
        double r = omega_body.z();  // yaw rate

        // Compute airspeed for damping normalization
        // Damping terms are normalized by d/(2V), but we receive q = .5*rho*v^2
        // So q * d/(2V) = 0.5*rho*v^2 * d/(2V) = 1/4*rho*V*d
        // Rather than needing V explicitly, we can express damping as:
        //   M_damp = qSd * Cmq * (q * d / (2V))
        // We need V. We can get it from q if we had density, but we don't here.
        // Instead, use the convention that omega is pre-normalized: hat_rate = rate * d/(2V)
        // For simplicity in this model, we apply damping directly with a fixed
        // normalization. The caller can set coefficients accordingly.
        //
        // For now, damping terms use: C_damp = C_deriv * rate * d_ref / (2 * V)
        // Since we don't have V, we approximate using omega directly scaled by d_ref.
        // This is valid when coefficients are tuned to this convention.

        // ── Forces (body frame) ──────────────────────────────────
        // Axial: opposes forward motion (acts in -X)
        double FA = -qS * coeffs_.CA;

        // Normal: due to angle of attack (acts in -Z for positive alpha)
        double FN = -qS * coeffs_.CN_alpha * alpha_rad;

        // Side: due to sideslip (acts in +Y for positive beta with negative CY_beta)
        double FY = qS * coeffs_.CY_beta * beta_rad;

        out.force = {FA, FY, FN};

        // ── Moments (body frame) ─────────────────────────────────
        // Pitching moment: due to alpha + pitch damping
        double Cm = coeffs_.Cm_alpha * alpha_rad + coeffs_.Cmq * q * ref_length;

        // Yawing moment: due to beta + yaw damping
        double Cn = coeffs_.Cn_beta * beta_rad + coeffs_.Cnr * r * ref_length;

        // Rolling moment: due to fin deflection + roll damping
        double Cl = coeffs_.Cl_delta * fin_deflections.x() + coeffs_.Clp * p * ref_length;

        out.moment = {qSd * Cl, qSd * Cm, qSd * Cn};

        return out;
    }

} // namespace sim::models