#pragma once

#include "models/IAtmosphere.hpp"
#include "models/IAerodynamics.hpp"
#include "models/IPropulsion.hpp"
#include "models/IActuator.hpp"
#include "models/IGravity.hpp"
#include "models/IWind.hpp"
#include "models/ITarget.hpp"
#include "models/ISeeker.hpp"
#include "models/IGuidance.hpp"
#include "models/IAutopilot.hpp"
#include <memory>
#include <stdexcept>

namespace sim::core
{
    /**
     * @brief Aggregates all vehicle and environment subsystem models
     * 
     * Each subsystem is owned via std::unique_ptr, allowing concrete
     * implementations to be swapped without modifying the simulation loop.
     * 
     * Signal chain for guided flight:
     *   Target -> Seeker -> Guidance -> Autopilot -> Actuator -> Aerodynamics
     * 
     * Environment models feeding in:
     *   Atmosphere (density, Mach) -> Aerodynamics
     *   Gravity -> Translational EOM
     *   Wind -> Airspeed computation
     *   Propulsion -> Forces + mass properties
     * 
     * Usage:
     *   auto vehicle = Vehicle::Builder()
     *       .set_atmosphere(std::make_unique<USStandard1976>())
     *       .set_gravity(std::make_unique<ConstantGravity>())
     *       .set_wind(std::make_unique<NoWind>())
     *       .set_aerodynamics(std::make_unique<TabulatedAero>(deck))
     *       .set_propulsion(std::make_unique<SolidRocketMotor>(curve))
     *       .set_actuator(std::make_unique<RateLimitedActuator>(limits))
     *       .set_seeker(std::make_unique<IdealSeeker>())
     *       .set_guidance(std::make_unique<ProportionalNavigation>(N))
     *       .set_autopilot(std::make_unique<PIDAutopilot>(gains))
     *       .set_target(std::make_unique<ConstantVelocityTarget>(pos, vel))
     *       .set_reference(ref_area, ref_length)
     *       .build();
     */
    class Vehicle
    {
    public:
        // ==================================================================== 
        // Environment model access
        // ====================================================================
        [[nodiscard]] const models::IAtmosphere& atmosphere() const {
        if (!atmosphere_) throw std::runtime_error("Atmosphere model not set");
            return *atmosphere_;
        }

        [[nodiscard]] const models::IGravity& gravity() const {
            if (!gravity_) throw std::runtime_error("Gravity model not set");
            return *gravity_;
        }

        [[nodiscard]] const models::IWind& wind() const {
            if (!wind_) throw std::runtime_error("Wind model not set");
            return *wind_;
        }

        // ==================================================================== 
        // Vehicle model access
        // ====================================================================
        [[nodiscard]] const models::IAerodynamics& aerodynamics() const {
        if (!aerodynamics_) throw std::runtime_error("Aerodynamics model not set");
            return *aerodynamics_;
        }

        [[nodiscard]] const models::IPropulsion& propulsion() const {
            if (!propulsion_) throw std::runtime_error("Propulsion model not set");
            return *propulsion_;
        }

        [[nodiscard]] models::IActuator& actuator() {
            if (!actuator_) throw std::runtime_error("Actuator model not set");
            return *actuator_;
        }

        // ==================================================================== 
        // GNC model access
        // ====================================================================
        [[nodiscard]] models::ISeeker& seeker() {
        if (!seeker_) throw std::runtime_error("Seeker model not set");
            return *seeker_;
        }

        [[nodiscard]] const models::IGuidance& guidance() const {
            if (!guidance_) throw std::runtime_error("Guidance model not set");
            return *guidance_;
        }

        [[nodiscard]] models::IAutopilot& autopilot() {
            if (!autopilot_) throw std::runtime_error("Autopilot model not set");
            return *autopilot_;
        }

        // ==================================================================== 
        // Target model access
        // ====================================================================
        [[nodiscard]] const models::ITarget& target() const {
        if (!target_) throw std::runtime_error("Target model not set");
            return *target_;
        }

        // ==================================================================== 
        // Reference geometry
        // ====================================================================
        double ref_area{0.0};     // Reference area for aero coefficients (m^2)
        double ref_length{0.0};   // Reference length for moment coefficients (m)

        // ==================================================================== 
        // Builder class
        // Nested class inside Vehicle scope. Accessed via Vehicle::Builder.
        // Builder serves no purpose other than to construct vehicles, 
        // so keeping it inside the scope of Vehicle makes sense.
        // ====================================================================
        class Builder
        {
        public:
            // Environment
            Builder& set_atmosphere(std::unique_ptr<models::IAtmosphere> m) {
                atmosphere_ = std::move(m); return *this;
            }
            Builder& set_gravity(std::unique_ptr<models::IGravity> m) {
                gravity_ = std::move(m); return *this;
            }
            Builder& set_wind(std::unique_ptr<models::IWind> m) {
                wind_ = std::move(m); return *this;
            }

            // Vehicle
            Builder& set_aerodynamics(std::unique_ptr<models::IAerodynamics> m) {
                aerodynamics_ = std::move(m); return *this;
            }
            Builder& set_propulsion(std::unique_ptr<models::IPropulsion> m) {
                propulsion_ = std::move(m); return *this;
            }
            Builder& set_actuator(std::unique_ptr<models::IActuator> m) {
                actuator_ = std::move(m); return *this;
            }

            // GNC
            Builder& set_seeker(std::unique_ptr<models::ISeeker> m) {
                seeker_ = std::move(m); return *this;
            }
            Builder& set_guidance(std::unique_ptr<models::IGuidance> m) {
                guidance_ = std::move(m); return *this;
            }
            Builder& set_autopilot(std::unique_ptr<models::IAutopilot> m) {
                autopilot_ = std::move(m); return *this;
            }

            // Target
            Builder& set_target(std::unique_ptr<models::ITarget> m) {
                target_ = std::move(m); return *this;
            }

            // Reference geometry
            Builder& set_reference(double area, double length) {
                ref_area_ = area; ref_length_ = length; return *this;
            }

            [[nodiscard]] Vehicle build() {
                Vehicle v;
                v.atmosphere_   = std::move(atmosphere_);
                v.gravity_      = std::move(gravity_);
                v.wind_         = std::move(wind_);
                v.aerodynamics_ = std::move(aerodynamics_);
                v.propulsion_   = std::move(propulsion_);
                v.actuator_     = std::move(actuator_);
                v.seeker_       = std::move(seeker_);
                v.guidance_     = std::move(guidance_);
                v.autopilot_    = std::move(autopilot_);
                v.target_       = std::move(target_);
                v.ref_area      = ref_area_;
                v.ref_length    = ref_length_;
                return v;
            }
        
        private:
            std::unique_ptr<models::IAtmosphere>   atmosphere_;
            std::unique_ptr<models::IGravity>      gravity_;
            std::unique_ptr<models::IWind>         wind_;
            std::unique_ptr<models::IAerodynamics> aerodynamics_;
            std::unique_ptr<models::IPropulsion>   propulsion_;
            std::unique_ptr<models::IActuator>     actuator_;
            std::unique_ptr<models::ISeeker>       seeker_;
            std::unique_ptr<models::IGuidance>     guidance_;
            std::unique_ptr<models::IAutopilot>    autopilot_;
            std::unique_ptr<models::ITarget>       target_;
            double ref_area_{0.0};
            double ref_length_{0.0};
        };
    
    
    
    private:
        std::unique_ptr<models::IAtmosphere>   atmosphere_;
        std::unique_ptr<models::IGravity>      gravity_;
        std::unique_ptr<models::IWind>         wind_;
        std::unique_ptr<models::IAerodynamics> aerodynamics_;
        std::unique_ptr<models::IPropulsion>   propulsion_;
        std::unique_ptr<models::IActuator>     actuator_;
        std::unique_ptr<models::ISeeker>       seeker_;
        std::unique_ptr<models::IGuidance>     guidance_;
        std::unique_ptr<models::IAutopilot>    autopilot_;
        std::unique_ptr<models::ITarget>       target_;
    };

} // namespace sim::core