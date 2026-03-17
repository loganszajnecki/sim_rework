#include "models/FirstOrderActuator.hpp"
#include <cmath>
#include <algorithm>

namespace sim::models {
    FirstOrderActuator::FirstOrderActuator()
        : cfg_{}
        , cmd_(sim::math::Vec3d::zero())
        , current_(sim::math::Vec3d::zero()) {}

    FirstOrderActuator::FirstOrderActuator(const Config& cfg)
        : cfg_(cfg)
        , cmd_(sim::math::Vec3d::zero())
        , current_(sim::math::Vec3d::zero()) {}

    void FirstOrderActuator::command(const sim::math::Vec3d& cmd_deflections) {
        cmd_ = cmd_deflections;
    }

    void FirstOrderActuator::update(double dt) {
        double new_roll  = step_channel(current_.x(), cmd_.x(), dt);
        double new_pitch = step_channel(current_.y(), cmd_.y(), dt);
        double new_yaw   = step_channel(current_.z(), cmd_.z(), dt);
        current_ = {new_roll, new_pitch, new_yaw};
    }

    sim::math::Vec3d FirstOrderActuator::current_deflections() const {
        return current_;
    }

    void FirstOrderActuator::reset() {
        cmd_ = sim::math::Vec3d::zero();
        current_ = sim::math::Vec3d::zero();
    }

    double FirstOrderActuator::step_channel(double current, double commanded, double dt) const {
        // Clamp command to position limits
        commanded = std::clamp(commanded, -cfg_.max_deflection, cfg_.max_deflection);

        // First-order lag
        double error = commanded - current;
        double rate = error / cfg_.time_constant;

        // Apply rate limit
        rate = std::clamp(rate, -cfg_.max_rate, cfg_.max_rate);

        // Integrate
        double result = current + rate * dt;

        // Clamp to position limits
        result = std::clamp(result, -cfg_.max_deflection, cfg_.max_deflection);

        return result;
    }

} // namespace sim::models