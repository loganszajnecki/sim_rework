#include "models/USStandard1976.hpp"
#include <cmath>
#include <algorithm>
#include <utility>

namespace sim::models 
{
    AtmosphericState USStandard1976::compute(double altitude_m) const {
        // Clamp to valid range
        altitude_m = std::clamp(altitude_m, -2000.0, 86000.0);

        // Convert geometric altitude to geopotential altitude
        // h_hp = (r_earth * h_geo) / (r_earth + h_geo);
        double h_gp = (kEarthRadius * altitude_m) / (kEarthRadius + altitude_m);
        
        // Determine layer and compute T, P
        auto [T, P] = compute_tp(h_gp);

        // Density from ideal gas law: rho = P / (R * T)
        double rho = P / (kR * T);

        // Speed of sound: a = sqrt(gamma * R * T)
        double a = std::sqrt(kGamma * kR * T);

        return {T, P, rho, a};
    }

    std::pair<double, double> USStandard1976::compute_tp(double h_gp) const {
        // Find the layer containing this altitude
        int layer = kNumLayers - 1;
        for (int i = kNumLayers - 1; i > 0; --i) {
            if (h_gp < kH[i]) {
                layer = i - 1;
            }
        }

        double dh = h_gp - kH[layer];
        double lambda = kLambda[layer];
        double T_base = kT[layer];
        double P_base = kP[layer];

        // Temperature: linear lapse rate
        double T = T_base + lambda * dh;

        // Pressure: depends on whether the lapse rate is zero
        double P;
        if (std::abs(lambda) < 1e-10) {
            // Isothermal layer: P = P_base * exp(-g0 * dh / (R * T))
            P = P_base * std::exp(-kG0 * dh / (kR * T_base));
        } else {
            // Gradient layer: P = P_base * (T / T_base) ^ (-g0 / (λ * R))
            double exponent = -kG0 / (lambda * kR);
            P = P_base * std::pow(T / T_base, exponent);
        }

        return {T, P};
    }


} // namespace sim::models