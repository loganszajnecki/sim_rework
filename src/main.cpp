#include <iostream>
#include <iomanip>
#include <cmath>
#include <string>

#include "math/Vector3.hpp"
#include "math/Rotations.hpp"
#include "core/State.hpp"
#include "core/Integrator.hpp"
#include "core/Vehicle.hpp"
#include "core/EOM.hpp"
#include "core/Logger.hpp"
#include "core/DataRecorder.hpp"
#include "core/SimConfig.hpp"
#include "core/ConfigParser.hpp"
#include "core/VehicleFactory.hpp"

using namespace sim::math;
using namespace sim::core;

/// Derived flight quantities computed each step
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

static void print_usage(const char* prog) {
    std::cerr << "Usage:\n"
              << "  " << prog << " <config.xml>            Run simulation\n"
              << "  " << prog << " --generate-template     Write default config to default.xml\n";
}

int main(int argc, char* argv[]) {
    // Initialize Logger
    Logger::init("missile_sim.log", spdlog::level::info);

    // Parse command line
    if (argc < 2) {
        print_usage(argv[0]);
        return 1;
    }

    std::string arg = argv[1];

    if (arg == "--generate-template") {
        ConfigParser::save_template("default.xml");
        std::cout << "Template configuration written to default.xml\n";
        return 0;
    }

    // Load configuration
    SIM_INFO("===============================================");
    SIM_INFO("  6-DOF Missile Simulation");
    SIM_INFO("===============================================");

    SimConfig cfg;
    try {
        cfg = ConfigParser::load(arg);
    } catch (const std::exception& e) {
        SIM_ERROR("Failed to load config: {}", e.what());
        return 1;
    }

    // Build vehicle and initial state from config
    Vehicle vehicle = VehicleFactory::build_vehicle(cfg.vehicle);
    State state = VehicleFactory::build_initial_state(cfg);

    // Set up integrator
    auto integrator = make_integrator<State, StateDerivative>(cfg.integrator.type);
    double dt = cfg.integrator.dt;

    auto derivatives = [&vehicle](double t, const State& s) -> StateDerivative {
        return EOM::compute(t, s, vehicle);
    };

    // Set up data recorder
    DataRecorder::Config rec_cfg;
    rec_cfg.filepath = cfg.recorder.filepath;
    rec_cfg.record_interval = cfg.recorder.interval;
    DataRecorder recorder(rec_cfg);

    recorder.begin_run(0);
    recorder.write_attribute("config_file", arg);
    recorder.write_attribute("dt", dt);
    recorder.write_attribute("launch_pitch_deg", cfg.initial_conditions.pitch_deg);
    recorder.write_attribute("launch_speed_mps", cfg.initial_conditions.speed);

    // ── Console output header ────────────────────────────────
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "\nInitial state:\n" << state << "\n\n";
    std::cout << "  Time(s)   North(m)   Alt(m)   Speed(m/s)   Mach  "
              << " Pitch(deg)  Mass(kg)\n"
              << "  ==================================================="
              << "===================\n";
    
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
    int print_interval = static_cast<int>(std::round(1.0 / dt)); // print every ~1 second
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

        // Console output
        if (step_count % print_interval == 0) {
            print_row(t, state, fd);
        }

        // Impact detection
        if (t > 0.5 && fd.altitude <= cfg.stop.min_altitude) {
            recorder.record_now(t, state, fd.altitude, fd.alpha, fd.beta,
                                fd.mach, fd.speed, fd.qbar);
            print_row(t, state, fd);

            SIM_INFO("Impact at t={:.3f}s, range={:.1f}m, speed={:.1f} m/s",
                     t, state.position.x(), fd.speed);
            std::cout << "\n  *** Impact at t = " << t << " s ***\n"
                      << "  Range: " << state.position.x() << " m\n"
                      << "  Final speed: " << fd.speed << " m/s\n"
                      << "  Final mass: " << state.mass << " kg\n";
            break;
        }

        // Timeout
        if (t > cfg.stop.max_time) {
            SIM_WARN("Simulation timeout at t={:.1f}s", cfg.stop.max_time);
            std::cout << "\n  *** Timeout at t = " << t << " s ***\n";
            break;
        }
    }

    recorder.end_run();
    SIM_INFO("Simulation complete. Data written to {}", cfg.recorder.filepath);

    std::cout << std::endl;
    return 0;
}