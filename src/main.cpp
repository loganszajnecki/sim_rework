#include <iostream>
#include <iomanip>
#include <cmath>
#include <functional>
#include <fstream>

#include "math/Vector3.hpp"
#include "math/Rotations.hpp"
#include "core/State.hpp"
#include "core/Integrator.hpp"
#include "core/Vehicle.hpp"
#include "core/EOM.hpp"
#include "models/USStandard1976.hpp"
#include "models/FlatEarthGravity.hpp"
#include "models/SolidRocketMotor.hpp"
#include "models/SimpleAero.hpp"

using namespace sim::math;
using namespace sim::core;
using namespace sim::models;

int main() {
    std::cout << "===============================================\n"
              << "  6-DOF Missile Simulation — Powered Flight\n"
              << "===============================================\n\n";

    // =========== Motor configuration =========
    SolidRocketMotor::Config motor_cfg;
    motor_cfg.thrust     = 5000.0;   // N
    motor_cfg.burn_time  = 3.0;      // s
    motor_cfg.total_mass = 100.0;    // kg
    motor_cfg.prop_mass  = 30.0;     // kg
    motor_cfg.inertia    = Mat3d{
        0.5, 0.0, 0.0,
        0.0, 10.0, 0.0,
        0.0, 0.0, 10.0
    };
    motor_cfg.cg_body = {1.0, 0.0, 0.0};

    // =========== Aero configuration ==========
    SimpleAero::Coefficients aero_coeffs;
    aero_coeffs.CA       = 0.3;
    aero_coeffs.CN_alpha = 10.0;
    aero_coeffs.CY_beta  = -10.0;
    aero_coeffs.Cm_alpha = -3.0;    // statically stable
    aero_coeffs.Cn_beta  = 3.0;
    aero_coeffs.Cl_delta = -0.5;
    aero_coeffs.Cmq      = -20.0;
    aero_coeffs.Cnr      = -20.0;
    aero_coeffs.Clp      = -5.0;

    // =========== Build the vehicle =========== 
    auto vehicle = Vehicle::Builder()
        .set_atmosphere(std::make_unique<USStandard1976>())
        .set_gravity(std::make_unique<FlatEarthGravity>())
        .set_propulsion(std::make_unique<SolidRocketMotor>(motor_cfg))
        .set_aerodynamics(std::make_unique<SimpleAero>(aero_coeffs))
        .set_reference(0.05, 0.2)  // S_ref = 0.05 m^2, d_ref = 0.2 m
        .build();

    // =========== Initial Conditions ========== 
    constexpr double launch_angle = 45.0 * M_PI / 180.0;
    constexpr double rail_speed   = 30.0;  // m/s at rail exit
    constexpr double dt           = 0.001; // 1 kHz integration

    State state;
    state.position      = Vec3d{0.0, 0.0, 0.0};
    state.velocity_body = Vec3d{rail_speed, 0.0, 0.0};
    state.euler         = EulerAnglesd{0.0, launch_angle, 0.0};
    state.omega_body    = Vec3d::zero();
    state.mass          = motor_cfg.total_mass;
   
    std::cout << "Initial state:\n" << state << "\n\n";
    std::ofstream results("results.csv");
    results << "time,"
        << "n,e,d,"
        << "u,v,w,"
        << "phi,theta,psi,"
        << "p,q,r,"
        << "mass,"
        << "V,alpha,beta,"
        << "Vn,Ve,Vd\n";

    // =============== Integrator ================
    auto integrator = make_integrator<State, StateDerivative>("rk4");
    
    // Wrap EOM::compute to match the integrators expected signature
    auto derivatives = [&vehicle](double t, const State& s) -> StateDerivative {
        return EOM::compute(t, s, vehicle);
    };
    
    // ============= Simulation Loop ============= 
    double t = 0.0;
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "  Time(s)   North(m)   Alt(m)   Speed(m/s)   Mach   Pitch(deg)  Mass(kg)\n"
              << "  ========================================================================\n";
    
    auto print_row = [&]() {
        double altitude = -state.position.z();
        auto atmo = vehicle.atmosphere().compute(altitude);
        double V = state.speed();
        double mach = (V > 1e-6) ? V / atmo.speed_of_sound : 0.0;

        std::cout << "  " << std::setw(7) << t
                  << "  " << std::setw(9) << state.position.x()
                  << "  " << std::setw(7) << altitude
                  << "  " << std::setw(11) << V
                  << "  " << std::setw(5) << std::setprecision(3) << mach
                  << "  " << std::setw(9) << std::setprecision(3)
                  << state.euler.theta * 180.0 / M_PI
                  << "  " << std::setw(7) << std::setprecision(1) << state.mass
                  << "\n" << std::setprecision(3);

        // Derived quantities
        Vec3d vel_i = state.velocity_intertial();
        double alpha = state.alpha();
        double beta  = state.beta();

        // Log full 13-state + derived
        results << t << ","

                // Position (NED)
                << state.position.x() << ","
                << state.position.y() << ","
                << state.position.z() << ","

                // Body velocity
                << state.velocity_body.x() << ","
                << state.velocity_body.y() << ","
                << state.velocity_body.z() << ","

                // Euler angles (rad)
                << state.euler.phi   << ","
                << state.euler.theta << ","
                << state.euler.psi   << ","

                // Angular rates (rad/s)
                << state.omega_body.x() << ","
                << state.omega_body.y() << ","
                << state.omega_body.z() << ","

                // Mass
                << state.mass << ","

                // Derived body metrics
                << state.speed() << ","
                << alpha << ","
                << beta << ","

                // Inertial velocity
                << vel_i.x() << ","
                << vel_i.y() << ","
                << vel_i.z() << "\n";

    };

    print_row();

    while (true) {
        state = integrator->step(derivatives, t, state, dt);
        t += dt;

        double altitude = -state.position.z();

        print_row();

        // Impact detection
        if (t > 0.5 && altitude <= 0.0) {
            print_row();
            std::cout << "\n  *** Impact at t = " << t << " s ***\n";
            std::cout << "  Range: " << state.position.x() << " m\n";
            std::cout << "  Final speed: " << state.speed() << " m/s\n";
            std::cout << "  Final mass: " << state.mass << " kg\n";
            break;
        }

        if (t > 300.0) {
            std::cout << "\n  *** Timeout at t = 300 s ***\n";
            break;
        }
    }

    std::cout << std::endl;
    results.close();
    return 0;
}