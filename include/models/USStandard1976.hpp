#pragma once

#include "models/IAtmosphere.hpp"
#include <cmath>
#include <algorithm>

namespace sim::models
{
    /**
     * @brief US Standard Atmosphere 1976 Implementation
     * 
     * Computes temperature, pressure, density, and speed of sound as
     * functions of geometric altitude. Implements seven layers from 
     * sea level to 86 km using the standard lapse rate model.
     * 
     * Layer definitions (geopotential altitude)
     *   Layer 0: 0 - 11 km      Troposphere     lambda = 06.5 K/km
     *   Layer 1: 11 - 20 km     Tropopause      lambda =  0.0 K/km (isothermal)
     *   Layer 2: 20 - 32 km     Stratosphere 1  lambda = +1.0 K/km
     *   Layer 3: 32 - 47 km     Stratosphere 2  lambda = +2.8 K/km
     *   Layer 4: 47 - 51 km     Stratopause     lambda =  0.0 K/km (isothermal)
     *   Layer 5: 51 - 71 km     Mesosphere 1    lambda = -2.8 K/km
     *   Layer 6: 71 - 84.852 km Mesosphere 2    lambda = -2.0 K/km
     * 
     * Reference: "U.S. Standard Atmosphere, 1976", NOAA/NASA/USAF
     * 
     * Input: geometric altitude in meters (MSL)
     * Valid range: -2000m to 86000 m (clamped outside this range)
     */
    class USStandard1976 : public IAtmosphere
    {
    public:
        [[nodiscard]] AtmosphericState compute(double altitude_m) const override;
    
    private:
        // Physical constants
        static constexpr double kG0          = 9.80665;      // Standard gravity (m/s^2)
        static constexpr double kR           = 287.05287;    // Specific gas constant, dry air (J/(kg*K))
        static constexpr double kGamma       = 1.4;          // Ratio of specific heats
        static constexpr double kEarthRadius = 6356766.0;    // Effective earth radius (m)

        // Layer data
        static constexpr int kNumLayers = 7;

        // Base geopotential altitudes for each layer (m)
        static constexpr double kH[kNumLayers] = {
            0.0, 11000.0, 20000.0, 32000.0, 47000.0, 51000.0, 71000.0
        };

        // Temperature lapse rates for each layer (K/m)
        static constexpr double kLambda[kNumLayers] = {
            -0.0065, 0.0, 0.001, 0.0028, 0.0, -0.0028, -0.002
        };

        // Base temperatures at the bottom of each layer (K)
        // Computed: T[i+1] = T[i] + lambda[i] * (H[i+1] - H[i])
        static constexpr double kT[kNumLayers] = {
            288.15,    // Layer 0: sea level
            216.65,    // Layer 1: 288.15 + (-0.0065)(11000)
            216.65,    // Layer 2: isothermal, unchanged
            228.65,    // Layer 3: 216.65 + (0.001)(12000)
            270.65,    // Layer 4: 228.65 + (0.0028)(15000)
            270.65,    // Layer 5: isothermal, unchanged
            214.65     // Layer 6: 270.65 + (-0.0028)(20000)
        };

        // Base pressures at the bottom of each layer (Pa)
        // Pre-computed from the hydrostatic equation applied layer by layer
        static constexpr double kP[kNumLayers] = {
            101325.0,       // Layer 0: sea level
            22632.06397,    // Layer 1: at 11 km
            5474.88867,     // Layer 2: at 20 km
            868.01874,      // Layer 3: at 32 km
            110.90630,      // Layer 4: at 47 km
            66.93887,       // Layer 5: at 51 km
            3.95642         // Layer 6: at 71 km
        };

        /// Compute temperature and pressure at a given geopotential altitude
        [[nodiscard]] std::pair<double,double> compute_tp(double h_gp) const;
    };


} // namespace sim::models