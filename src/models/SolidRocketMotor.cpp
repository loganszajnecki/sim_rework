#include "models/SolidRocketMotor.hpp"
#include <algorithm>

namespace sim::models
{
    SolidRocketMotor::SolidRocketMotor(const Config& cfg)
        : cfg_(cfg), mass_flow_rate_(cfg.prop_mass/cfg.burn_time) {}

    bool SolidRocketMotor::is_burning(double t) const {
        return t >= 0.0 && t < cfg_.burn_time;
    }

    PropulsionOutput SolidRocketMotor::compute_thrust(double t) const {
        PropulsionOutput out;

        if (is_burning(t)) {
            // Constant force along body +X axis
            out.force = {cfg_.thrust, 0.0, 0.0};

            // No thrust offset from CG, no thrust vectoring, moment is 0
            out.moment = sim::math::Vec3d::zero();

            // Mass leaving the vehicle
            out.mass_flow_rate = -mass_flow_rate_;
        } else {
            out.force  = sim::math::Vec3d::zero();
            out.moment = sim::math::Vec3d::zero();
            out.mass_flow_rate = 0.0;
        }

        return out;
    }

    MassProperties SolidRocketMotor::compute_mass_properties(double t) const {
        MassProperties mp;

        // Propellant consumed so far
        double burned = is_burning(t) ? mass_flow_rate_ * t : cfg_.prop_mass;
        burned = std::clamp(burned, 0.0, cfg_.prop_mass);

        // Current mass
        mp.mass = cfg_.total_mass - burned;

        // CG: Simplified - stays fixed (symmetric burn)
        mp.cg_body = cfg_.cg_body;

        // Inertia: scale full tensor linearly with mass ratio.
        // This is a simplification; real motors have nonlinear inertia profiles.
        // Using a full Mat3d preserves off-diagonal products of inertia from
        // CAD-derived tensors, scaling them uniformly with mass depletion.
        double mass_ratio = mp.mass / cfg_.total_mass;
        mp.inertia = mass_ratio * cfg_.inertia;

        return mp;
    }



} // namespace sim::models