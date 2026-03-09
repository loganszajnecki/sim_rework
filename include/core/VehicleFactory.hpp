#pragma once

#include "core/Vehicle.hpp"
#include "core/SimConfig.hpp"
#include "core/State.hpp"
#include "models/ITarget.hpp"
#include <vector>
#include <memory>

namespace sim::core 
{
    /**
     * @brief Factory for constructing a Vehicle and initial State from configuration.
     *
     * Maps the string-based "type" fields in the config to concrete model classes.
     * New model types are registered here as they are implemented.
     *
     * Usage:
     *   auto cfg = ConfigParser::load("scenario.xml");
     *   auto vehicle = VehicleFactory::build_vehicle(cfg.vehicle);
     *   auto state = VehicleFactory::build_initial_state(cfg);
     */
    class VehicleFactory
    {
    public:
        /**
         * @brief Construct a fully configured Vehicle from the complete sim config.
         *
         * Builds all physics models, and if GNC + target are present,
         * wires the full guided flight chain. Uses the first target.
         */
        [[nodiscard]] static Vehicle build_from_config(const SimConfig& cfg);

        /**
         * @brief Construct a Vehicle from the vehicle configuration.
         * @param cfg  Vehicle configuration section
         * @return Fully assembled Vehicle with all models wired in
         * @throws std::runtime_error for unknown model types
         */
        [[nodiscard]] static Vehicle build_vehicle(const VehicleConfig& cfg);

        /**
         * @brief Construct the initial State from the full simulation config.
         * @param cfg  Full simulation configuration
         * @return State populated with initial conditions
         */
        [[nodiscard]] static State build_initial_state(const SimConfig& cfg);

        /**
         * @brief Construct target models from configuration.
         * @param configs Vector of target configurations from XML
         * @return Vector of concrete target model instances
         * @throws std::runtime_error for unknown target types
         */
        [[nodiscard]] static std::vector<std::unique_ptr<sim::models::ITarget>>
            build_targets(const std::vector<TargetConfig>& configs);
    };

} // namespace sim::core