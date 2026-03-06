#include "core/VehicleFactory.hpp"
#include "core/Logger.hpp"

#include "models/USStandard1976.hpp"
#include "models/FlatEarthGravity.hpp"
#include "models/SolidRocketMotor.hpp"
#include "models/SimpleAero.hpp"
#include "models/StationaryTarget.hpp"
#include "models/ConstantVelocityTarget.hpp"
#include "models/ManeuveringTarget.hpp"

#include <stdexcept>
#include <cmath>

namespace sim::core {

    using namespace sim::math;
    using namespace sim::models;

    //  Atmosphere factory

    static std::unique_ptr<IAtmosphere> make_atmosphere(const AtmosphereConfig& cfg) {
        if (cfg.type == "us_standard_1976") {
            SIM_DEBUG("Creating US Standard Atmosphere 1976");
            return std::make_unique<USStandard1976>();
        }

        throw std::runtime_error("Unknown atmosphere type: " + cfg.type);
    }

    // Gravity factory

    static std::unique_ptr<IGravity> make_gravity(const GravityConfig& cfg) {
        if (cfg.type == "constant") {
            SIM_DEBUG("Creating constant gravity: g={} m/s^2", cfg.g);
            return std::make_unique<FlatEarthGravity>(cfg.g);
        }

        throw std::runtime_error("Unknown gravity type: " + cfg.type);
    }

    // Propulsion factory

    static std::unique_ptr<IPropulsion> make_propulsion(const PropulsionConfig& cfg) {
        if (cfg.type == "solid_rocket") {
            SolidRocketMotor::Config motor_cfg;
            motor_cfg.thrust     = cfg.thrust;
            motor_cfg.burn_time  = cfg.burn_time;
            motor_cfg.total_mass = cfg.total_mass;
            motor_cfg.prop_mass  = cfg.prop_mass;
            motor_cfg.inertia    = cfg.inertia;
            motor_cfg.cg_body    = cfg.cg_body;

            SIM_DEBUG("Creating solid rocket motor: F={} N, tb={}s, m0={} kg",
                    cfg.thrust, cfg.burn_time, cfg.total_mass);
            return std::make_unique<SolidRocketMotor>(motor_cfg);
        }

        throw std::runtime_error("Unknown propulsion type: " + cfg.type);
    }

    // Aerodynamics factory

    static std::unique_ptr<IAerodynamics> make_aerodynamics(const AerodynamicsConfig& cfg) {
        if (cfg.type == "simple") {
            SimpleAero::Coefficients coeffs;
            coeffs.CA       = cfg.CA;
            coeffs.CN_alpha = cfg.CN_alpha;
            coeffs.CY_beta  = cfg.CY_beta;
            coeffs.Cm_alpha = cfg.Cm_alpha;
            coeffs.Cn_beta  = cfg.Cn_beta;
            coeffs.Cl_delta = cfg.Cl_delta;
            coeffs.Cmq      = cfg.Cmq;
            coeffs.Cnr      = cfg.Cnr;
            coeffs.Clp      = cfg.Clp;

            SIM_DEBUG("Creating simple aero: CA={}, CN_a={}, Cm_a={}",
                    cfg.CA, cfg.CN_alpha, cfg.Cm_alpha);
            return std::make_unique<SimpleAero>(coeffs);
        }

        throw std::runtime_error("Unknown aerodynamics type: " + cfg.type);
    }

    // Public API

    Vehicle VehicleFactory::build_vehicle(const VehicleConfig& cfg) {
        SIM_INFO("Building vehicle: S_ref={} m^2, d_ref={} m", cfg.ref_area, cfg.ref_length);

        return Vehicle::Builder()
            .set_atmosphere(make_atmosphere(cfg.atmosphere))
            .set_gravity(make_gravity(cfg.gravity))
            .set_propulsion(make_propulsion(cfg.propulsion))
            .set_aerodynamics(make_aerodynamics(cfg.aerodynamics))
            .set_reference(cfg.ref_area, cfg.ref_length)
            .build();
    }

    State VehicleFactory::build_initial_state(const SimConfig& cfg) {
        const auto& ic = cfg.initial_conditions;

        State state;
        state.position = ic.position;

        // Convert speed + angles to body-frame velocity
        // At launch, body X-axis points along the velocity vector
        state.velocity_body = Vec3d{ic.speed, 0.0, 0.0};

        // Convert degrees to radians for Euler angles
        constexpr double deg2rad = M_PI / 180.0;
        state.euler = EulerAnglesd{
            ic.roll_deg * deg2rad,
            ic.pitch_deg * deg2rad,
            ic.yaw_deg * deg2rad
        };

        state.omega_body = Vec3d::zero();
        state.mass = cfg.vehicle.propulsion.total_mass;

        SIM_INFO("Initial state: speed={:.1f} m/s, pitch={:.1f} deg, yaw={:.1f} deg, mass={:.1f} kg",
                ic.speed, ic.pitch_deg, ic.yaw_deg, state.mass);

        return state;
    }

    // Target Factory
    static std::unique_ptr<ITarget> make_target(const TargetConfig& cfg) {
        if (cfg.type == "stationary") {
            SIM_DEBUG("Creating stationary target at [{}, {}, {}]",
                      cfg.position.x(), cfg.position.y(), cfg.position.z());
            return std::make_unique<StationaryTarget>(cfg.position);
        }

        if (cfg.type == "constant_velocity") {
            SIM_DEBUG("Creating constant-velocity target at [{},{},{}], vel=[{},{},{}]",
                       cfg.position.x(), cfg.position.y(), cfg.position.z(),
                       cfg.velocity.x(), cfg.velocity.y(), cfg.velocity.z());
            return std::make_unique<ConstantVelocityTarget>(cfg.position, cfg.velocity);
        }
        
        if (cfg.type == "maneuvering") {
            SIM_DEBUG("Creating maneuvering target: {}g at t={}s",
                      cfg.maneuver_g, cfg.maneuver_start);
            return std::make_unique<ManeuveringTarget>(
                cfg.position, cfg.velocity, cfg.maneuver_g, cfg.maneuver_start);
        }

        throw std::runtime_error("Unknown target type: " + cfg.type);
    }

    std::vector<std::unique_ptr<sim::models::ITarget>> 
    VehicleFactory::build_targets(const std::vector<TargetConfig>& configs) 
    {
        std::vector<std::unique_ptr<sim::models::ITarget>> targets;
        targets.reserve(configs.size());

        for (const auto& cfg : configs) {
            targets.push_back(make_target(cfg));
        }

        SIM_INFO("Built {} target(s)", targets.size());
        return targets;
    }

} // namespace sim::core