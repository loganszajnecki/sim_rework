#include <iostream>
#include <iomanip>
#include <cmath>
#include <functional>

#include "math/Vector3.hpp"
#include "math/Rotations.hpp"
#include "core/State.hpp"
#include "core/Integrator.hpp"
#include "core/Vehicle.hpp"
#include "core/EOM.hpp"
#include "core/Logger.hpp"
#include "core/DataRecorder.hpp"
#include "models/USStandard1976.hpp"
#include "models/FlatEarthGravity.hpp"
#include "models/SolidRocketMotor.hpp"
#include "models/SimpleAero.hpp"

using namespace sim::math;
using namespace sim::core;
using namespace sim::models;

/// Compute derived flight quantities from the current state
struct FlightData {
    double altitude;
    double speed;
    double alpha;
    double beta;
    double mach;
    double qbar;
};

static FlightData compute_flight_data(const State& state, const Vehicle& vehicle)
{
    FlightData fd;
    fd.altitude = -state.position.z();

    auto atmo = vehicle.atmosphere().compute(fd.altitude);
    Vec3d airspeed = state.velocity_body; //no wind for now
    fd.speed = airspeed.magnitude();

    if (fd.speed > 1e-6) {
        fd.alpha = std::atan2(airspeed.z(), airspeed.x());
        fd.beta  = std::asin(std::clamp(airspeed.y() / fd.speed, -1.0, 1.0));
        fd.mach  = fd.speed / atmo.speed_of_sound;
        fd.qbar  = 0.5 * atmo.density * fd.speed * fd.speed;
    } else {
        fd.alpha = 0.0;
        fd.beta  = 0.0;
        fd.mach  = 0.0;
        fd.qbar  = 0.0;
    }
    return fd;
}

int main() {
    // Initialize logger
    Logger::init("missile_sim.log", spdlog::level::info);
    SIM_INFO("===============================================");
    SIM_INFO("  6-DOF Missile Simulation — Powered Flight");
    SIM_INFO("===============================================");

    // Motor configuration
    SolidRocketMotor::Config motor_cfg;
    motor_cfg.thrust     = 5000.0;
    motor_cfg.burn_time  = 3.0;
    motor_cfg.total_mass = 100.0;
    motor_cfg.prop_mass  = 30.0;
    motor_cfg.inertia    = Mat3d{
        0.5, 0.0, 0.0,
        0.0, 10.0, 0.0,
        0.0, 0.0, 10.0
    };
    motor_cfg.cg_body = {1.0, 0.0, 0.0};

    // Aero configuration
    SimpleAero::Coefficients aero_coeffs;
    aero_coeffs.CA       = 0.3;
    aero_coeffs.CN_alpha = 10.0;
    aero_coeffs.CY_beta  = -10.0;
    aero_coeffs.Cm_alpha = -3.0;
    aero_coeffs.Cn_beta  = 3.0;
    aero_coeffs.Cl_delta = -0.5;
    aero_coeffs.Cmq      = -20.0;
    aero_coeffs.Cnr      = -20.0;
    aero_coeffs.Clp      = -5.0;

    // Build the vehicle
    auto vehicle = Vehicle::Builder()
        .set_atmosphere(std::make_unique<USStandard1976>())
        .set_gravity(std::make_unique<FlatEarthGravity>())
        .set_propulsion(std::make_unique<SolidRocketMotor>(motor_cfg))
        .set_aerodynamics(std::make_unique<SimpleAero>(aero_coeffs))
        .set_reference(0.05, 0.2)
        .build();
    
    SIM_INFO("Vehicle built: S_ref={} m^2, d_ref={} m", vehicle.ref_area, vehicle.ref_length);

    // Initial conditions
    constexpr double launch_angle = 45.0 * M_PI / 180.0;
    constexpr double rail_speed   = 30.0;
    constexpr double dt           = 0.001;

    State state;
    state.position      = Vec3d{0.0, 0.0, 0.0};
    state.velocity_body  = Vec3d{rail_speed, 0.0, 0.0};
    state.euler         = EulerAnglesd{0.0, launch_angle, 0.0};
    state.omega_body    = Vec3d::zero();
    state.mass          = motor_cfg.total_mass;

    SIM_INFO("IC: pitch={:.1f} deg, speed={:.1f} m/s, mass={:.1f} kg",
             launch_angle * 180.0 / M_PI, rail_speed, state.mass);

    DataRecorder::Config rec_cfg;
    rec_cfg.filepath = "sim_output.h5";
    rec_cfg.record_interval = 0.01;  // 100 Hz
    DataRecorder recorder(rec_cfg);

    recorder.begin_run(0);
    recorder.write_attribute("launch_angle_deg", launch_angle * 180.0 / M_PI);
    recorder.write_attribute("rail_speed_mps", rail_speed);
    recorder.write_attribute("dt", dt);
    recorder.write_attribute("motor_thrust_N", motor_cfg.thrust);
    recorder.write_attribute("motor_burn_time_s", motor_cfg.burn_time);

    // Integrator
    auto integrator = make_integrator<State, StateDerivative>("rk4");
    auto derivatives = [&vehicle](double t, const State& s) -> StateDerivative {
        return EOM::compute(t, s, vehicle);
    };

    // ── Console output header ────────────────────────────────
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "\n  Time(s)   North(m)   Alt(m)   Speed(m/s)   Mach  "
              << " Pitch(deg)  Mass(kg)\n"
              << "  ═══════════════════════════════════════════════════"
              << "═══════════════════\n";

    auto print_row = [&](double t, const State& s, const FlightData& fd) {
        std::cout << "  " << std::setw(7) << t
                  << "  " << std::setw(9) << s.position.x()
                  << "  " << std::setw(7) << fd.altitude
                  << "  " << std::setw(11) << fd.speed
                  << "  " << std::setw(5) << fd.mach
                  << "  " << std::setw(10) << s.euler.theta * 180.0 / M_PI
                  << "  " << std::setw(7) << std::setprecision(1) << s.mass
                  << "\n" << std::setprecision(3);
    };

    // Simulation loop
    double t = 0.0;
    int step_count = 0;
    constexpr int print_interval = 1000;
    bool motor_burnout_logged = false;

    // Record and print initial state
    auto fd = compute_flight_data(state, vehicle);
    recorder.record_now(t, state, fd.altitude, fd.alpha, fd.beta, fd.mach, fd.speed, fd.qbar);
    print_row(t, state, fd);

    while (true) {
        // Check for motor burnout event
        if (!motor_burnout_logged && !vehicle.propulsion().is_burning(t)) {
            SIM_INFO("Motor burnout at t={:.3f}s, speed={:.1f} m/s, alt={:.0f} m",
                     t, fd.speed, fd.altitude);
            motor_burnout_logged = true;
        }

        state = integrator->step(derivatives, t, state, dt);
        t += dt;
        step_count++;
        
        fd = compute_flight_data(state, vehicle);

        // Record to HDF5
        recorder.record(t, state, fd.altitude, fd.alpha, fd.beta, fd.mach, fd.speed, fd.qbar);    

        // Console output at fixed step interval
        if (step_count % print_interval == 0) {
            print_row(t, state, fd);
        }

        // Impact detection
        if (t > 0.5 && fd.altitude <= 0.0) {
            recorder.record_now(t, state, fd.altitude, fd.alpha, fd.beta,
                                fd.mach, fd.speed, fd.qbar);
            print_row(t, state, fd);

            SIM_INFO("Impact at t={:.3f}s, range={:.1f}m, speed={:.1f} m/s",
                     t, state.position.x(), fd.speed);
            std::cout << "\n  *** Impact at t = " << t << " s ***\n";
            std::cout << "  Range: " << state.position.x() << " m\n";
            std::cout << "  Final speed: " << fd.speed << " m/s\n";
            std::cout << "  Final mass: " << state.mass << " kg\n";
            break;
        }

        if (t > 300.0) {
            SIM_WARN("Simulation timeout at t=300s");
            std::cout << "\n  *** Timeout at t = 300 s ***\n";
            break;
        }
    }
    recorder.end_run();
    SIM_INFO("Simulation complete. Data written to {}", rec_cfg.filepath);
    std::cout << std::endl;
    return 0;
}