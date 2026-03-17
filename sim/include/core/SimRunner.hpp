#pragma once

// Contains a bunch of structs that configure the sim
#include "core/SimConfig.hpp"

// Contains the operators/structs needed to store 6DOF state vector
#include "core/State.hpp"  

// Aggregates/owns all unique pointers to all subsystem models
#include "core/Vehicle.hpp"

// Class that encapsulates HDF5 recording capability
#include "core/DataRecorder.hpp"

#include <string>

namespace sim::core {
    /**
     * @brief Termination reason for a simulation run.
     *
     * Describes the geometric event that ended the run, not whether
     * the engagement was successful. Success assessment (kill/no-kill)
     * is an analysis-layer decision based on miss_distance vs a
     * warhead lethal radius, which varies by scenario.
     */
    enum class TerminationReason {
        closest_approach,   // Range diverging after point of closest approach
        ground_impact,      // Altitude below minimum
        timeout,            // Exceeded max simulation time
        unknown
    };

    /**
     * @brief Results from a single simulation run.
     *
     * Contains everything needed for post-processing, Monte Carlo
     * statistics, or visualization setup.
     */
    struct SimResult {
        TerminationReason termination{TerminationReason::unknown};
        double final_time{0.0};
        double miss_distance{1e30};     // Minimum range to target at POCA (m)
        double miss_time{0.0};          // Time of closest approach (s)
        double impact_speed{0.0};       // Speed at termination (m/s)
        double max_altitude{0.0};       // Peak altitude during flight (m)
        double max_speed{0.0};          // Peak speed during flight (m/s)
        double max_mach{0.0};           // Peak Mach number
        State final_state;              // State at termination
        int run_id{0};                  // Run index (for Monte Carlo)
    };

    /**
     * @brief Encapsulates the full simulation loop.
     *
     * Extracts the sim loop from main.cpp into a callable unit.
     * Used by:
     *   - main.cpp for single runs
     *   - Monte Carlo dispatcher for batch runs
     *   - TODO: Future real-time visualizer for live execution
     *
     * Usage:
     *   SimRunner runner;
     *   SimResult result = runner.run(cfg);
     *
     *   // With recording:
     *   SimResult result = runner.run(cfg, &recorder, run_id);
     *
     *   // Silent (no console output):
     *   runner.set_verbose(false);
     *   SimResult result = runner.run(cfg);
     */
    class SimRunner
    {
    public:
        SimRunner() = default;

        /**
         * @brief Execute a single simulation run.
         * 
         * Builds the Vehicle and initial state from config, runs the
         * integration loop with optional GNC, and returns results.
         * 
         * @param cfg       Full simulation configuration
         * @param recorder  Optional data recorder (nullptr = no recording)
         * @param run_id    Run index for the recorder group name
         * @return SimResult with termination info and statistics
         */
        [[nodiscard]] SimResult run(const SimConfig& cfg,
                                    DataRecorder* recorder = nullptr,
                                    int run_id = 0);

        /// Enable/disable console output during the run
        void set_verbose(bool verbose) { verbose_ = verbose; }

        /// Enable/disable progress logging (SIM_INFO events)
        void set_logging(bool logging) { logging_ = logging; }
    
    private:
        bool verbose_{true};
        bool logging_{true};

        /// Derived flight quantities computed each step
        struct FlightData {
            double altitude{0.0};
            double speed{0.0};
            double alpha{0.0};
            double beta{0.0};
            double mach{0.0};
            double qbar{0.0};
        };

        /// Run the GNC chain for one timestep
        void run_gnc(double t, double dt, 
                    const State& state,
                    const FlightData& fd,
                    Vehicle& vehicle);
        
        /// Compute derived flight quantities
        [[nodiscard]] FlightData compute_flight_data(
            const State& state, const Vehicle& vehicle) const;
    };
    
} // namespace sim::core
