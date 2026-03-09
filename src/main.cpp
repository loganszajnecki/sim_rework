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
    Vehicle vehicle = VehicleFactory::build_from_config(cfg);
    State state = VehicleFactory::build_initial_state(cfg);
    bool guided = vehicle.has_gnc();

    if (guided) {
        SIM_INFO("Mode: GUIDED FLIGHT");
        SIM_DEBUG("Parsed autopilot config: Kp={}, Kd={}, Kd_roll={}, max_defl={}",
                  cfg.vehicle.autopilot->Kp, cfg.vehicle.autopilot->Kd,
                  cfg.vehicle.autopilot->Kd_roll, cfg.vehicle.autopilot->max_deflection);
        SIM_DEBUG("Parsed guidance config: type={}, N={}",
                  cfg.vehicle.guidance->type, cfg.vehicle.guidance->nav_ratio);
        SIM_DEBUG("Parsed seeker config: type={}, min_range={}",
                  cfg.vehicle.seeker->type, cfg.vehicle.seeker->min_range);
        SIM_DEBUG("Parsed actuator config: type={}, tau={}, max_defl={}, max_rate={}",
                  cfg.vehicle.actuator->type, cfg.vehicle.actuator->time_constant,
                  cfg.vehicle.actuator->max_deflection, cfg.vehicle.actuator->max_rate);
    } else {
        SIM_INFO("Mode: UNGUIDED FLIGHT");
    }

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
    if (!cfg.targets.empty()) {
        const auto& tgt = cfg.targets.front();
        recorder.write_attribute("target_type", tgt.type);
        recorder.write_attribute("target_pos_north", tgt.position.x());
        recorder.write_attribute("target_pos_east", tgt.position.y());
        recorder.write_attribute("target_pos_down", tgt.position.z());
        recorder.write_attribute("target_vel_north", tgt.velocity.x());
        recorder.write_attribute("target_vel_east", tgt.velocity.y());
        recorder.write_attribute("target_vel_down", tgt.velocity.z());
    }
    recorder.write_attribute("config_file", arg);
    recorder.write_attribute("dt", dt);
    recorder.write_attribute("guided", guided ? 1 : 0);
    recorder.write_attribute("launch_pitch_deg", cfg.initial_conditions.pitch_deg);
    recorder.write_attribute("launch_speed_mps", cfg.initial_conditions.speed);

    // Console output header
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "\nInitial state:\n" << state << "\n\n";

    if (guided) {
        std::cout << "  Time(s)   North(m)   Alt(m)   Speed(m/s)  Mach"
                  << "   Pitch(deg)  Range(m)   Miss(m)\n"
                  << "  ==============================================="
                  << "==============================\n";
    } else {
        std::cout << "  Time(s)   North(m)   Alt(m)   Speed(m/s)  Mach"
                  << "   Pitch(deg)  Mass(kg)\n"
                  << "  ==============================================="
                  << "===================\n";
    }

    // Simulation loop
    double t = 0.0;
    int step_count = 0;
    int print_interval = static_cast<int>(std::round(1.0 / dt));
    bool motor_burnout_logged = false;
    double miss_distance = 1e30;
    double range_to_target = 1e30;
    double prev_range = 1e30;

    auto print_row = [&](double t_now, const State& s, const FlightData& fd) {
        if (guided) {
            std::cout << "  " << std::setw(7) << t_now
                      << "  " << std::setw(9) << s.position.x()
                      << "  " << std::setw(7) << fd.altitude
                      << "  " << std::setw(10) << fd.speed
                      << "  " << std::setw(5) << fd.mach
                      << "  " << std::setw(10) << s.euler.theta * 180.0 / M_PI
                      << "  " << std::setw(8) << std::setprecision(1) << range_to_target
                      << "  " << std::setw(7) << miss_distance
                      << "\n" << std::setprecision(3);
        } else {
            std::cout << "  " << std::setw(7) << t_now
                      << "  " << std::setw(9) << s.position.x()
                      << "  " << std::setw(7) << fd.altitude
                      << "  " << std::setw(10) << fd.speed
                      << "  " << std::setw(5) << fd.mach
                      << "  " << std::setw(10) << s.euler.theta * 180.0 / M_PI
                      << "  " << std::setw(7) << std::setprecision(1) << s.mass
                      << "\n" << std::setprecision(3);
        }
    };

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

        // GNC Chain, runs once per major timestep
        if (guided) {
            // Missile NED velocity for the seeker
            Mat3d L_bi = dcm::body_to_inertial(state.euler);
            Vec3d msl_vel_ned = L_bi * state.velocity_body;

            // Target state
            auto tgt = vehicle.target().compute(t);

            // Range tracking for intercept detection
            range_to_target = (tgt.position - state.position).magnitude();
            miss_distance = std::min(miss_distance, range_to_target);

            // Seeker
            auto seeker_out = vehicle.seeker().compute(
                state.position, msl_vel_ned,
                tgt.position, tgt.velocity,
                L_bi, dt);

            // Guidance
            auto guidance_cmd = vehicle.guidance().compute(
                seeker_out, fd.speed);

            // Autopilot
            auto fin_cmd = vehicle.autopilot().compute(
                guidance_cmd, state.omega_body, state.velocity_body,
                fd.qbar, dt);

            // Actuator: command and update
            vehicle.actuator().command(fin_cmd);
            vehicle.actuator().update(dt);

            // GNC debug trace (first 5 seconds only)
            if (t < 5.0 && step_count % 1000 == 0) {
                SIM_DEBUG("=== GNC trace at t={:.3f}s ===", t);
                SIM_DEBUG("  Target pos: [{:.1f}, {:.1f}, {:.1f}]",
                          tgt.position.x(), tgt.position.y(), tgt.position.z());
                SIM_DEBUG("  Range: {:.1f} m, Rdot: {:.1f} m/s",
                          seeker_out.range, seeker_out.range_rate);
                SIM_DEBUG("  LOS rate: [{:.6f}, {:.6f}, {:.6f}] mag={:.6f}",
                          seeker_out.los_rate.x(), seeker_out.los_rate.y(),
                          seeker_out.los_rate.z(), seeker_out.los_rate.magnitude());
                SIM_DEBUG("  Track valid: {}", seeker_out.track_valid);
                SIM_DEBUG("  Guidance accel cmd: [{:.2f}, {:.2f}, {:.2f}] mag={:.2f} m/s²",
                          guidance_cmd.accel_cmd.x(), guidance_cmd.accel_cmd.y(),
                          guidance_cmd.accel_cmd.z(), guidance_cmd.accel_cmd.magnitude());
                SIM_DEBUG("  Autopilot fin cmd: [{:.4f}, {:.4f}, {:.4f}] rad",
                          fin_cmd.x(), fin_cmd.y(), fin_cmd.z());
                SIM_DEBUG("  Actuator actual:   [{:.4f}, {:.4f}, {:.4f}] rad",
                          vehicle.actuator().current_deflections().x(),
                          vehicle.actuator().current_deflections().y(),
                          vehicle.actuator().current_deflections().z());
            }
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

        // Termination conditions

        // Intercept detection (guided): range increasing after closest approach
        if (guided && range_to_target > prev_range && prev_range < 100.0) {
            recorder.record_now(t, state, fd.altitude, fd.alpha, fd.beta,
                                fd.mach, fd.speed, fd.qbar);
            print_row(t, state, fd);

            SIM_INFO("Intercept at t={:.3f}s, miss distance={:.2f} m", t, miss_distance);
            std::cout << "\n  *** Intercept at t = " << t << " s ***\n"
                      << "  Miss distance: " << miss_distance << " m\n"
                      << "  Final speed: " << fd.speed << " m/s\n"
                      << "  Final range: " << range_to_target << " m\n";
            break;
        }
        prev_range = range_to_target;

        // Ground impact
        if (t > 0.5 && fd.altitude <= cfg.stop.min_altitude) {
            recorder.record_now(t, state, fd.altitude, fd.alpha, fd.beta,
                                fd.mach, fd.speed, fd.qbar);
            print_row(t, state, fd);

            SIM_INFO("Ground impact at t={:.3f}s, range={:.1f}m, speed={:.1f} m/s",
                     t, state.position.x(), fd.speed);
            std::cout << "\n  *** Ground impact at t = " << t << " s ***\n"
                      << "  Range: " << state.position.x() << " m\n"
                      << "  Final speed: " << fd.speed << " m/s\n"
                      << "  Final mass: " << state.mass << " kg\n";
            if (guided) {
                std::cout << "  Miss distance: " << miss_distance << " m\n";
            }
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