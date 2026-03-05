#pragma once

#include "math/Vector3.hpp"
#include "math/Matrix3.hpp"
#include <string>
#include <vector>
#include <optional>

namespace sim::core
{
    /**
     * @brief Complete simulation configuration, populated from XML.
     * 
     * All parameters needed to construct a Vehicle, set initial conditions,
     * and run the simulation. Designed to be extended for Monte Carlo
     * dispersions, target definitions, and GNC configurations.
     */

    // Integrator
    struct IntegratorConfig {
        std::string type{"rk4"};
        double dt{0.001};  // Integration timestep (s)
    };

    // Data Recorder
    struct RecorderConfig {
        std::string filepath{"sim_output.h5"};
        double interval{0.01};  // Recording interval (s)
    };

    // Stop conditions
    struct StopConfig {
        double max_time{300.0};     // Maximum simulation time (s)
        double min_altitude{0.0};   // Ground impact altitude (m)
        // Potential TODO: add threat intercept handling config here
    };

    // Initial Conditions
    struct InitialConditions {
        math::Vec3d position{0.0, 0.0, 0.0};     // NED (m)
        double speed{30.0};                      // Body-frame speed (m/s)
        double pitch_deg{45.0};                  // Launch pitch angle (deg)
        double yaw_deg{0.0};                     // Launch yaw angle (deg)
        double roll_deg{0.0};                    // Launch roll angle (deg)
    };

    // Model configurations
    struct AtmosphereConfig {
        std::string type{"us_standard_1976"};
    };

    struct GravityConfig {
        std::string type{"constant"};
        double g{9.80665};          // Gravity magnitude (m/s^2)
    };

    struct PropulsionConfig {
        std::string type{"solid_rocket"};
        double thrust{5000.0};      // N
        double burn_time{3.0};      // s
        double total_mass{100.0};   // kg
        double prop_mass{30.0};     // kg

        // Inertia tensor at launch (kg·m^2)
        // Default: axisymmetric body
        math::Mat3d inertia{
            0.5, 0.0, 0.0,
            0.0, 10.0, 0.0,
            0.0, 0.0, 10.0
        };

        // CG in body frame (m)
        math::Vec3d cg_body{1.0, 0.0, 0.0};
    };

    struct AerodynamicsConfig {
        std::string type{"simple"};

        // Static coefficients
        double CA{0.3};
        double CN_alpha{10.0};
        double CY_beta{-10.0};

        // Moment coefficients
        double Cm_alpha{-3.0};
        double Cn_beta{3.0};
        double Cl_delta{-0.5};

        // Damping derivatives
        double Cmq{-20.0};
        double Cnr{-20.0};
        double Clp{-5.0};
    };

    // Vehicle
    struct VehicleConfig {
        double ref_area{0.05};      // Reference area (m²)
        double ref_length{0.2};     // Reference length (m)

        AtmosphereConfig atmosphere;
        GravityConfig gravity;
        PropulsionConfig propulsion;
        AerodynamicsConfig aerodynamics;
    };

    // Threat definition
    struct TargetConfig {
        std::string type{"stationary"};              // stationary, constant_velocity, maneuvering
        math::Vec3d position{5000.0, 0.0, -500.0};   // Initial position NED (m)
        math::Vec3d velocity{0.0, 0.0, 0.0};         // Initial velocity NED (m/s)
        double maneuver_g{0.0};                      // Lateral acceleration (g's, for maneuvering)
        double maneuver_start{0.0};                  // Time maneuver begins (s)
    };
    
    // Top-level simulation configuration
    struct SimConfig {
        IntegratorConfig integrator;
        RecorderConfig recorder;
        StopConfig stop;
        InitialConditions initial_conditions;
        VehicleConfig vehicle;
        std::vector<TargetConfig> targets;      // Zero or more targets
    };
    
} // namespace sim::core