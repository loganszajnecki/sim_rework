#pragma once

#include "core/State.hpp"
#include <string>
#include <vector>
#include <array>
#include <memory>

// Forward declare HDF5 types to keep the header clean.
// Users of DataRecorder don't need to include H5Cpp.h
namespace H5 {
    class H5File;
    class Group;
}

namespace sim::core
{
    /**
     * @brief HDF5 simulation data recorder.
     *
     * Records simulation state at configurable intervals to an HDF5 file.
     * Designed for both single runs and Monte Carlo runs.
     *
     * HDF5 structure:
     *   sim_output.h5
     *   |-- run_0000/
     *   |   |-- time          [N]
     *   │   |-- position      [N × 3]    (NED, meters)
     *   │   |-- velocity_body [N × 3]    (body frame, m/s)
     *   │   |-- euler         [N × 3]    (phi, theta, psi in radians)
     *   │   |-- omega_body    [N × 3]    (body rates, rad/s)
     *   │   |-- mass          [N]        (kg)
     *   │   |-- alpha         [N]        (angle of attack, rad)
     *   │   |-- beta          [N]        (sideslip, rad)
     *   │   |-- speed         [N]        (total speed, m/s)
     *   │   |-- mach          [N]        (Mach number)
     *   │   |-- altitude      [N]        (meters MSL)
     *   │   |-- qbar          [N]        (dynamic pressure, Pa)
     *   │   |-- @attributes: run_id, initial conditions, etc.
     *   |-- run_0001/
     *   │   |-- ...
     *
     * Usage:
     *   DataRecorder::Config cfg;
     *   cfg.filepath = "output.h5";
     *   cfg.record_interval = 0.01;  // 100 Hz
     *   DataRecorder recorder(cfg);
     *
     *   recorder.begin_run(0);
     *   // ... simulation loop ...
     *   recorder.record(t, state, altitude, alpha, beta, mach, speed, qbar);
     *   // ... end of loop ...
     *   recorder.end_run();
     */
    class DataRecorder
    {
    public:
        struct Config {
            std::string filepath{"sim_output.h5"};
            double record_interval{0.01};  // seconds between records (100 Hz default)
        };

        explicit DataRecorder(const Config& cfg);
        ~DataRecorder();

        // Non-copyable, but movable
        DataRecorder(const DataRecorder&) = delete;
        DataRecorder& operator=(const DataRecorder&) = delete;
        DataRecorder(DataRecorder&&) noexcept;
        DataRecorder& operator=(DataRecorder&&) noexcept;

        /**
         * @brief Begin a new simulation run.
         * @param run_id Run index (used for group naming: run_0000, run_0001, ...)
         */
        void begin_run(int run_id = 0);

        /**
         * @brief Record the current state (if the record interval has elapsed).
         *
         * Call this every integration step. The recorder internally tracks
         * timing and only stores data at the configured interval.
         *
         * @param t        Current simulation time (s)
         * @param state    Current state vector
         * @param altitude Altitude MSL (m), typically -state.position.z()
         * @param alpha    Angle of attack (rad)
         * @param beta     Sideslip angle (rad)
         * @param mach     Mach number
         * @param speed    Total speed (m/s)
         * @param qbar     Dynamic pressure (Pa)
         */
        void record(double t, const State& state,
                    double altitude, double alpha, double beta,
                    double mach, double speed, double qbar);

        /**
         * @brief Force a record regardless of the interval timer.
         * Useful for capturing exact impact or event states.
         */
        void record_now(double t, const State& state,
                        double altitude, double alpha, double beta,
                        double mach, double speed, double qbar);

        /**
         * @brief Finalize the current run, flushing all buffered data to disk.
         */
        void end_run();

        // Run-level metadata
        void write_attribute(const std::string& name, double value);
        void write_attribute(const std::string& name, int value);
        void write_attribute(const std::string& name, const std::string& value);
    
    private:
        Config cfg_;
        double last_record_time_{-1e30};

        // HDF5 handles (pimpl to avoid exposing H5Cpp.h in header)
        struct Impl;
        std::unique_ptr<Impl> impl_;

        // In-memory buffers - flused to HDF5 on end_run()
        std::vector<double> buf_time_;
        std::vector<std::array<double, 3>> buf_position_;
        std::vector<std::array<double, 3>> buf_velocity_body_;
        std::vector<std::array<double, 3>> buf_euler_;
        std::vector<std::array<double, 3>> buf_omega_body_;
        std::vector<double> buf_mass_;
        std::vector<double> buf_alpha_;
        std::vector<double> buf_beta_;
        std::vector<double> buf_speed_;
        std::vector<double> buf_mach_;
        std::vector<double> buf_altitude_;
        std::vector<double> buf_qbar_;

        /// Append current values to all buffers
        void buffer_state(double t, const State& state,
                          double altitude, double alpha, double beta,
                          double mach, double speed, double qbar);

        /// Write all buffers to the current run group
        void flush_buffers();

        /// Clear all buffers for the next run
        void clear_buffers();
    };

} // namespace sim::core