#include <gtest/gtest.h>
#include <cmath>
#include <memory>

#include "models/FirstOrderActuator.hpp"

using namespace sim::models;
using namespace sim::math;

class FirstOrderActuatorTest : public ::testing::Test {
protected:
    FirstOrderActuator::Config cfg;

    void SetUp() override {
        cfg.time_constant = 0.02;      // 20 ms
        cfg.max_deflection = 0.4;      // rad
        cfg.max_rate = 10.0;           // rad/s
    }

    /// Run the actuator for a given duration at a fixed timestep
    void run_for(FirstOrderActuator& act, double duration, double dt = 0.001) {
        int steps = static_cast<int>(duration / dt + 0.5);
        for (int i = 0; i < steps; ++i) {
            act.update(dt);
        }
    }
};

// ═══════════════════════════════════════════════════════════════
//  Initial state
// ═══════════════════════════════════════════════════════════════

TEST_F(FirstOrderActuatorTest, StartsAtZero) {
    FirstOrderActuator act(cfg);
    auto d = act.current_deflections();
    EXPECT_NEAR(d.x(), 0.0, 1e-10);
    EXPECT_NEAR(d.y(), 0.0, 1e-10);
    EXPECT_NEAR(d.z(), 0.0, 1e-10);
}

// ═══════════════════════════════════════════════════════════════
//  First-order lag response
// ═══════════════════════════════════════════════════════════════

TEST_F(FirstOrderActuatorTest, ApproachesCommandedValue) {
    FirstOrderActuator act(cfg);
    act.command(Vec3d{0.0, 0.2, 0.0});

    // Run for many time constants — should be very close to target
    run_for(act, 0.2);  // 10τ

    EXPECT_NEAR(act.current_deflections().y(), 0.2, 0.001);
}

TEST_F(FirstOrderActuatorTest, TimeConstantBehavior) {
    // For a first-order system, at t = τ the response should be
    // approximately 63.2% of the step command (1 - e^(-1))
    //
    // Using a large time constant and no rate limiting for clean test
    FirstOrderActuator::Config slow_cfg;
    slow_cfg.time_constant = 0.1;       // 100 ms
    slow_cfg.max_deflection = 1.0;
    slow_cfg.max_rate = 1000.0;         // effectively unlimited
    FirstOrderActuator act(slow_cfg);

    double cmd_val = 0.5;
    act.command(Vec3d{0.0, cmd_val, 0.0});

    // Run for exactly one time constant
    double dt = 0.0001;  // Very small dt for accuracy
    run_for(act, 0.1, dt);

    double expected = cmd_val * (1.0 - std::exp(-1.0));  // 63.2%
    EXPECT_NEAR(act.current_deflections().y(), expected, 0.005);
}

TEST_F(FirstOrderActuatorTest, SettlesWithinFiveTimeConstants) {
    FirstOrderActuator act(cfg);
    act.command(Vec3d{0.0, 0.3, 0.0});

    // 5τ = 0.1 s, should be within 0.7% of final value
    run_for(act, 5.0 * cfg.time_constant);

    double expected = 0.3 * (1.0 - std::exp(-5.0));  // 99.3%
    EXPECT_NEAR(act.current_deflections().y(), expected, 0.005);
}

// ═══════════════════════════════════════════════════════════════
//  Rate limiting
// ═══════════════════════════════════════════════════════════════

TEST_F(FirstOrderActuatorTest, RateLimitCapsSpeed) {
    // Large step command with tight rate limit
    FirstOrderActuator::Config rate_cfg;
    rate_cfg.time_constant = 0.001;     // Very fast lag (wants to move instantly)
    rate_cfg.max_deflection = 1.0;
    rate_cfg.max_rate = 5.0;            // 5 rad/s limit
    FirstOrderActuator act(rate_cfg);

    act.command(Vec3d{0.0, 0.5, 0.0});

    // After 0.01 s at max rate of 5 rad/s: max travel = 0.05 rad
    run_for(act, 0.01);

    EXPECT_LE(act.current_deflections().y(), 0.055);  // slight tolerance for dt effects
    EXPECT_GE(act.current_deflections().y(), 0.045);
}

TEST_F(FirstOrderActuatorTest, RateLimitBothDirections) {
    FirstOrderActuator::Config rate_cfg;
    rate_cfg.time_constant = 0.001;
    rate_cfg.max_deflection = 1.0;
    rate_cfg.max_rate = 5.0;
    FirstOrderActuator act(rate_cfg);

    // Command negative
    act.command(Vec3d{0.0, -0.5, 0.0});
    run_for(act, 0.01);

    EXPECT_GE(act.current_deflections().y(), -0.055);
    EXPECT_LE(act.current_deflections().y(), -0.045);
}

// ═══════════════════════════════════════════════════════════════
//  Position limiting
// ═══════════════════════════════════════════════════════════════

TEST_F(FirstOrderActuatorTest, PositionClamps) {
    FirstOrderActuator act(cfg);

    // Command beyond max deflection
    act.command(Vec3d{0.0, 1.0, 0.0});  // 1.0 > 0.4 max
    run_for(act, 0.5);  // Long enough to settle

    EXPECT_NEAR(act.current_deflections().y(), cfg.max_deflection, 0.001);
}

TEST_F(FirstOrderActuatorTest, NegativePositionClamps) {
    FirstOrderActuator act(cfg);
    act.command(Vec3d{0.0, -1.0, 0.0});
    run_for(act, 0.5);

    EXPECT_NEAR(act.current_deflections().y(), -cfg.max_deflection, 0.001);
}

// ═══════════════════════════════════════════════════════════════
//  Channel independence
// ═══════════════════════════════════════════════════════════════

TEST_F(FirstOrderActuatorTest, ChannelsAreIndependent) {
    FirstOrderActuator act(cfg);
    act.command(Vec3d{0.1, 0.2, 0.3});
    run_for(act, 0.2);

    auto d = act.current_deflections();
    EXPECT_NEAR(d.x(), 0.1, 0.001);
    EXPECT_NEAR(d.y(), 0.2, 0.001);
    EXPECT_NEAR(d.z(), 0.3, 0.001);
}

TEST_F(FirstOrderActuatorTest, CanCommandDifferentDirections) {
    FirstOrderActuator act(cfg);
    act.command(Vec3d{-0.1, 0.15, -0.2});
    run_for(act, 0.2);

    auto d = act.current_deflections();
    EXPECT_NEAR(d.x(), -0.1, 0.001);
    EXPECT_NEAR(d.y(), 0.15, 0.001);
    EXPECT_NEAR(d.z(), -0.2, 0.001);
}

// ═══════════════════════════════════════════════════════════════
//  Command changes
// ═══════════════════════════════════════════════════════════════

TEST_F(FirstOrderActuatorTest, TracksChangingCommands) {
    FirstOrderActuator act(cfg);

    // Step to 0.2
    act.command(Vec3d{0.0, 0.2, 0.0});
    run_for(act, 0.2);
    EXPECT_NEAR(act.current_deflections().y(), 0.2, 0.001);

    // Step back to 0
    act.command(Vec3d{0.0, 0.0, 0.0});
    run_for(act, 0.2);
    EXPECT_NEAR(act.current_deflections().y(), 0.0, 0.001);
}

TEST_F(FirstOrderActuatorTest, ReverseCommand) {
    FirstOrderActuator act(cfg);

    act.command(Vec3d{0.0, 0.3, 0.0});
    run_for(act, 0.2);
    EXPECT_NEAR(act.current_deflections().y(), 0.3, 0.001);

    // Reverse to negative
    act.command(Vec3d{0.0, -0.3, 0.0});
    run_for(act, 0.5);
    EXPECT_NEAR(act.current_deflections().y(), -0.3, 0.001);
}

// ═══════════════════════════════════════════════════════════════
//  Reset
// ═══════════════════════════════════════════════════════════════

TEST_F(FirstOrderActuatorTest, ResetClearsState) {
    FirstOrderActuator act(cfg);
    act.command(Vec3d{0.1, 0.2, 0.3});
    run_for(act, 0.2);

    act.reset();

    auto d = act.current_deflections();
    EXPECT_NEAR(d.x(), 0.0, 1e-10);
    EXPECT_NEAR(d.y(), 0.0, 1e-10);
    EXPECT_NEAR(d.z(), 0.0, 1e-10);
}

// ═══════════════════════════════════════════════════════════════
//  Interface
// ═══════════════════════════════════════════════════════════════

TEST_F(FirstOrderActuatorTest, WorksThroughInterface) {
    std::unique_ptr<IActuator> iface = std::make_unique<FirstOrderActuator>(cfg);
    iface->command(Vec3d{0.0, 0.2, 0.0});

    for (int i = 0; i < 200; ++i) {
        iface->update(0.001);
    }

    EXPECT_NEAR(iface->current_deflections().y(), 0.2, 0.001);
}