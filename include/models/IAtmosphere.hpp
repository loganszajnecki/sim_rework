#pragma once

#include <memory>

namespace sim::models
{
    /**
     * @brief Atmospheric properties at a given altitude
     * 
     * Bundled into a struct so the atmosphere model returns all
     * properties in a single call 
     */
    struct AtmosphericState {
        double temperature{288.15};    // K
        double pressure{101325.0};     // Pa
        double density{1.225};         // kg/m^3
        double speed_of_sound{340.29}; // m/s
    };

    /**
     * @brief Abstract atmosphere model interface
     * 
     * Concrete implementations (e.g., US Standard 1976, custom tables)
     * inherit from this and are owned via std::unique_ptr in the 
     * Vehicle or Simulation class.
     * 
     * Usage:
     *   std::unique_ptr<IAtmosphere> atmo = std::make_unique<USStandardAtmosphere>();
     *   auto state = atmo->compute(-altitude_down);
     */
    class IAtmosphere {
    public:
        virtual ~IAtmosphere() = default;

        /// Compute atmospheric properties at the given geometric altitude (m, MSL)
        [[nodiscard]] virtual AtmosphericState compute(double altitude_m) const = 0;
    };

} // namespace sim::models