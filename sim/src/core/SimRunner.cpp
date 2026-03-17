#include "core/SimRunner.hpp"
#include "core/Integrator.hpp"
#include "core/EOM.hpp"
#include "core/VehicleFactory.hpp"
#include "core/Logger.hpp"
#include "math/Rotations.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <iomanip>

namespace sim::core {

    using namespace sim::math;

    // ======== Flight Data computation ======== 
    SimRunner::FlightData SimRunner::compute_flight_data(
        const State& state, const Vehicle& vehicle) const 
    {
        FlightData fd;
        fd.altitude = -state.position.z();

        auto atmo = vehicle.atmosphere().compute(fd.altitude);
        Vec3d airspeed = state.velocity_body;
        fd.speed = airspeed.magnitude();

        if (fd.speed > 1e-6) {
            fd.alpha = std::atan2(airspeed.z(), airspeed.x());
            fd.beta  = std::asin(std::clamp(airspeed.y() / fd.speed, -1.0, 1.0));
            fd.mach  = fd.speed / atmo.speed_of_sound;
            fd.qbar  = 0.5 * atmo.density * fd.speed * fd.speed;
        }

        return fd;
    }

    // ======== GNC Chain ======== 
    void SimRunner::run_gnc(double t, double dt,
                            const State& state,
                            const FlightData& fd,
                            Vehicle& vehicle)
    {
        Mat3d L_bi = dcm::body_to_inertial(state.euler);
        Vec3d msl_vel_ned = L_bi * state.velocity_body;

        auto tgt = vehicle.target().compute(t);

        auto seeker_out = vehicle.seeker().compute(
            state.position, msl_vel_ned,
            tgt.position, tgt.velocity,
            L_bi, dt);

        auto guidance_cmd = vehicle.guidance().compute(
            seeker_out, fd.speed);

        auto fin_cmd = vehicle.autopilot().compute(
            guidance_cmd, state.omega_body, state.velocity_body,
            fd.qbar, dt);

        vehicle.actuator().command(fin_cmd);
        vehicle.actuator().update(dt);
    }

    // ============= Main Simulation Loop =============
    SimResult SimRunner::run(const SimConfig& cfg,
                             DataRecorder* recorder,
                             int run_id)
    {
        SimResult result;
        result.run_id = run_id;

        // Build vehicle and initial state
        Vehicle vehicle = VehicleFactory::build_from_config(cfg);
        State state = VehicleFactory::build_initial_state(cfg);
        bool guided = vehicle.has_gnc();

        if (logging_) {
            SIM_INFO("Run {}: {} mode, speed={:.0f} m/s, pitch={:.0f} deg",
                    run_id, guided ? "GUIDED" : "UNGUIDED",
                    cfg.initial_conditions.speed,
                    cfg.initial_conditions.pitch_deg);
        }

        // Set up integrator
        auto integrator = make_integrator<State, StateDerivative>(cfg.integrator.type);
        double dt = cfg.integrator.dt;

        auto derivatives = [&vehicle](double t, const State& s) -> StateDerivative {
            return EOM::compute(t, s, vehicle);
        };

        // Data recorder setup
        if (recorder) {
            recorder->begin_run(run_id);
            recorder->write_attribute("run_id", run_id);
            recorder->write_attribute("dt", dt);
            recorder->write_attribute("guided", guided ? 1 : 0);
            recorder->write_attribute("launch_pitch_deg", cfg.initial_conditions.pitch_deg);
            recorder->write_attribute("launch_speed_mps", cfg.initial_conditions.speed);

            if (!cfg.targets.empty()) {
                const auto& tgt = cfg.targets.front();
                recorder->write_attribute("target_type", tgt.type);
                recorder->write_attribute("target_pos_north", tgt.position.x());
                recorder->write_attribute("target_pos_east", tgt.position.y());
                recorder->write_attribute("target_pos_down", tgt.position.z());
                recorder->write_attribute("target_vel_north", tgt.velocity.x());
                recorder->write_attribute("target_vel_east", tgt.velocity.y());
                recorder->write_attribute("target_vel_down", tgt.velocity.z());
            }
        }

        // Console header
        if (verbose_) {
            std::cout << std::fixed << std::setprecision(3);
            if (guided) {
                std::cout << "  Time(s)   North(m)   Alt(m)   Speed(m/s)  Mach"
                        << "   Pitch(deg)  Range(m)   Miss(m)\n"
                        << "  ═══════════════════════════════════════════════"
                        << "══════════════════════════════\n";
            } else {
                std::cout << "  Time(s)   North(m)   Alt(m)   Speed(m/s)  Mach"
                        << "   Pitch(deg)  Mass(kg)\n"
                        << "  ═══════════════════════════════════════════════"
                        << "═══════════════════\n";
            }
        }

        // Simulation loop
        double t = 0.0;
        int step_count = 0;
        int print_interval = static_cast<int>(std::round(1.0 / dt));
        bool motor_burnout_logged = false;
        double miss_distance = 1e30;
        double miss_time = 0.0;
        double range_to_target = 1e30;
        double prev_range = 1e30;

        auto fd = compute_flight_data(state, vehicle);

        // Compute initial range for guided
        if (guided) {
            auto tgt = vehicle.target().compute(0.0);
            range_to_target = (tgt.position - state.position).magnitude();
            prev_range = range_to_target;
            miss_distance = range_to_target;
        }

        // Record initial state
        if (recorder) {
            recorder->record_now(t, state, fd.altitude, fd.alpha, fd.beta,
                                fd.mach, fd.speed, fd.qbar);
        }

        // Console print helper
        auto print_row = [&](double t_now) {
            if (!verbose_) return;
            if (guided) {
                std::cout << "  " << std::setw(7) << t_now
                        << "  " << std::setw(9) << state.position.x()
                        << "  " << std::setw(7) << fd.altitude
                        << "  " << std::setw(10) << fd.speed
                        << "  " << std::setw(5) << fd.mach
                        << "  " << std::setw(10) << state.euler.theta * 180.0 / M_PI
                        << "  " << std::setw(8) << std::setprecision(1) << range_to_target
                        << "  " << std::setw(7) << miss_distance
                        << "\n" << std::setprecision(3);
            } else {
                std::cout << "  " << std::setw(7) << t_now
                        << "  " << std::setw(9) << state.position.x()
                        << "  " << std::setw(7) << fd.altitude
                        << "  " << std::setw(10) << fd.speed
                        << "  " << std::setw(5) << fd.mach
                        << "  " << std::setw(10) << state.euler.theta * 180.0 / M_PI
                        << "  " << std::setw(7) << std::setprecision(1) << state.mass
                        << "\n" << std::setprecision(3);
            }
        };

        print_row(t);

        while (true) {
            // Motor burnout event
            if (logging_ && !motor_burnout_logged && !vehicle.propulsion().is_burning(t)) {
                SIM_INFO("Run {}: burnout at t={:.3f}s, speed={:.1f} m/s, alt={:.0f} m",
                        run_id, t, fd.speed, fd.altitude);
                motor_burnout_logged = true;
            }

            // GNC chain
            if (guided) {
                run_gnc(t, dt, state, fd, vehicle);

                auto tgt = vehicle.target().compute(t);
                range_to_target = (tgt.position - state.position).magnitude();
                if (range_to_target < miss_distance) {
                    miss_distance = range_to_target;
                    miss_time = t;
                }
            }

            // Integrate
            state = integrator->step(derivatives, t, state, dt);
            t += dt;
            step_count++;

            fd = compute_flight_data(state, vehicle);

            // Track statistics
            result.max_altitude = std::max(result.max_altitude, fd.altitude);
            result.max_speed = std::max(result.max_speed, fd.speed);
            result.max_mach = std::max(result.max_mach, fd.mach);

            // Record
            if (recorder) {
                recorder->record(t, state, fd.altitude, fd.alpha, fd.beta,
                                fd.mach, fd.speed, fd.qbar);
            }

            // Console output
            if (step_count % print_interval == 0) {
                print_row(t);
            }

            // Termination checks

            // POCA detection (guided): range increasing after closest approach,
            // but only after the missile has gotten within the range gate.
            // This prevents false triggers during early guidance transients.
            if (guided && range_to_target > prev_range
                && prev_range < cfg.stop.poca_range_gate)
            {
                result.termination = TerminationReason::closest_approach;
                result.final_time = t;
                result.miss_distance = miss_distance;
                result.miss_time = miss_time;
                result.impact_speed = fd.speed;
                result.final_state = state;

                if (recorder) {
                    recorder->record_now(t, state, fd.altitude, fd.alpha, fd.beta,
                                        fd.mach, fd.speed, fd.qbar);
                }
                print_row(t);

                if (logging_) {
                    SIM_INFO("Run {}: POCA at t={:.3f}s, miss={:.2f} m, speed={:.1f} m/s",
                            run_id, t, miss_distance, fd.speed);
                }
                if (verbose_) {
                    std::cout << "\n  *** Closest approach at t = " << t << " s ***\n"
                            << "  Miss distance: " << miss_distance << " m\n"
                            << "  Final speed: " << fd.speed << " m/s\n";
                }
                break;
            }
            prev_range = range_to_target;

            // Ground impact
            if (t > 0.5 && fd.altitude <= cfg.stop.min_altitude) {
                result.termination = TerminationReason::ground_impact;
                result.final_time = t;
                result.miss_distance = miss_distance;
                result.miss_time = miss_time;
                result.impact_speed = fd.speed;
                result.final_state = state;

                if (recorder) {
                    recorder->record_now(t, state, fd.altitude, fd.alpha, fd.beta,
                                        fd.mach, fd.speed, fd.qbar);
                }
                print_row(t);

                if (logging_) {
                    SIM_INFO("Run {}: ground impact at t={:.3f}s, range={:.1f}m, speed={:.1f} m/s",
                            run_id, t, state.position.x(), fd.speed);
                }
                if (verbose_) {
                    std::cout << "\n  *** Ground impact at t = " << t << " s ***\n"
                            << "  Range: " << state.position.x() << " m\n"
                            << "  Final speed: " << fd.speed << " m/s\n"
                            << "  Final mass: " << state.mass << " kg\n";
                    if (guided) {
                        std::cout << "  Miss distance: " << miss_distance << " m\n";
                    }
                }
                break;
            }

            // Timeout
            if (t > cfg.stop.max_time) {
                result.termination = TerminationReason::timeout;
                result.final_time = t;
                result.miss_distance = miss_distance;
                result.miss_time = miss_time;
                result.impact_speed = fd.speed;
                result.final_state = state;

                if (logging_) {
                    SIM_WARN("Run {}: timeout at t={:.1f}s", run_id, cfg.stop.max_time);
                }
                if (verbose_) {
                    std::cout << "\n  *** Timeout at t = " << t << " s ***\n";
                }
                break;
            }
        }

        // Finalize recording with run metadata
        if (recorder) {
            recorder->write_attribute("termination",
                result.termination == TerminationReason::closest_approach ? "closest_approach" :
                result.termination == TerminationReason::ground_impact ? "ground_impact" :
                result.termination == TerminationReason::timeout ? "timeout" : "unknown");
            recorder->write_attribute("miss_distance", result.miss_distance);
            recorder->write_attribute("miss_time", result.miss_time);
            recorder->write_attribute("final_time", result.final_time);
            recorder->write_attribute("impact_speed", result.impact_speed);
            recorder->write_attribute("max_altitude", result.max_altitude);
            recorder->write_attribute("max_speed", result.max_speed);
            recorder->write_attribute("max_mach", result.max_mach);
            recorder->end_run();
        }

        return result;
    }

} // namespace sim::core