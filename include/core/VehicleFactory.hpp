#pragma once

#include "core/Vehicle.hpp"
#include "core/SimConfig.hpp"
#include "core/State.hpp"

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
    };

} // namespace sim::core