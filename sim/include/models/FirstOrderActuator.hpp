#pragma once

#include "models/IActuator.hpp"

namespace sim::models {
    /**
     * @brief First-order lag actuator with rate and position limits.
     *
     * Models fin servo dynamics as a first-order system:
     *   delta_dot = (delta_cmd - delta) / tau
     *
     * Subject to:
     *   - Position limits: |delta| <= delta_max
     *   - Rate limits:     |delta_dot| <= delta_dot_max
     *
     * The time constant tau determines how quickly the fins respond.
     * Typical values for tactical missiles:
     *   tau = 0.01–0.05 s  (10–50 ms)
     *   delta_max = 0.4 rad  (~23 degrees)
     *   delta_dot_max = 10 rad/s (~570 deg/s)
     *
     * The actuator maintains internal state (current deflections),
     * updated each timestep via update(dt). This introduces phase
     * lag into the control loop that affects stability margins.
     *
     * For Phase 2 validation, the first-order model is sufficient.
     * A second-order actuator (with natural frequency and damping)
     * can replace it in Phase 5 without changing the interface.
     *
     * The Vec3d convention is [roll, pitch, yaw] matching the
     * autopilot output.
     */
    class FirstOrderActuator : public IActuator 
    {
    public:
        struct Config {
            double time_constant{0.02};     // tau (s)
            double max_deflection{0.4};     // delta_max (rad)
            double max_rate{10.0};          // delta_dot_max (rad/s)
        };

        FirstOrderActuator();
        explicit FirstOrderActuator(const Config& cfg);

        void command(const sim::math::Vec3d& cmd_deflections) override;
        void update(double dt) override;
        [[nodiscard]] sim::math::Vec3d current_deflections() const override;

        /// Reset actuator to zero deflection and zero command
        void reset();

    private:
        Config cfg_;
        sim::math::Vec3d cmd_;       // Commanded deflection (rad)
        sim::math::Vec3d current_;   // Current actual deflection (rad)

        /// Apply first-order lag with rate and position limits to a single channel
        double step_channel(double current, double commanded, double dt) const;
    };

} // namespace sim::models