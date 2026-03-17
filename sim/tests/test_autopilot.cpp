#include <gtest/gtest.h>
#include <cmath>
#include <memory>

#include "models/SimpleAutopilot.hpp"

using namespace sim::models;
using namespace sim::math;

class SimpleAutopilotTest : public ::testing::Test {
protected:
    SimpleAutopilot::Gains gains;

    void SetUp() override {
        gains.Kp = 0.001;
        gains.Kd = 0.5;
        gains.Kd_roll = 0.2;
        gains.max_deflection = 0.4;
    }

    GuidanceCommand make_cmd(double ay, double az) {
        GuidanceCommand cmd;
        cmd.accel_cmd = Vec3d{0.0, ay, az};
        return cmd;
    }
};

// ═══════════════════════════════════════════════════════════════
//  Zero command, zero rates → zero deflection
// ═══════════════════════════════════════════════════════════════

TEST_F(SimpleAutopilotTest, ZeroInputsZeroOutput) {
    SimpleAutopilot ap(gains);
    auto cmd = make_cmd(0.0, 0.0);
    auto delta = ap.compute(cmd, Vec3d::zero(), Vec3d{300, 0, 0}, 50000.0, 0.01);

    EXPECT_NEAR(delta.x(), 0.0, 1e-10);  // roll
    EXPECT_NEAR(delta.y(), 0.0, 1e-10);  // pitch
    EXPECT_NEAR(delta.z(), 0.0, 1e-10);  // yaw
}

// ═══════════════════════════════════════════════════════════════
//  Pitch channel
// ═══════════════════════════════════════════════════════════════

TEST_F(SimpleAutopilotTest, PositiveAzProducesPitchDeflection) {
    SimpleAutopilot ap(gains);
    auto cmd = make_cmd(0.0, 100.0);  // 100 m/s² ventral command
    auto delta = ap.compute(cmd, Vec3d::zero(), Vec3d{300, 0, 0}, 50000.0, 0.01);

    // delta_pitch = Kp * a_cmd_z = 0.001 * 100 = 0.1 rad
    EXPECT_NEAR(delta.y(), 0.1, 1e-6);
    EXPECT_NEAR(delta.z(), 0.0, 1e-10);  // yaw unaffected
}

TEST_F(SimpleAutopilotTest, PitchRateDamping) {
    SimpleAutopilot ap(gains);
    auto cmd = make_cmd(0.0, 0.0);  // no guidance command
    Vec3d omega{0.0, 0.5, 0.0};     // positive pitch rate (nose up)

    auto delta = ap.compute(cmd, omega, Vec3d{300, 0, 0}, 50000.0, 0.01);

    // delta_pitch = -Kd * q = -0.5 * 0.5 = -0.25 rad (opposes pitch rate)
    EXPECT_NEAR(delta.y(), -0.25, 1e-6);
}

TEST_F(SimpleAutopilotTest, PitchCommandAndDampingCombined) {
    SimpleAutopilot ap(gains);
    auto cmd = make_cmd(0.0, 200.0);   // strong ventral command
    Vec3d omega{0.0, 0.3, 0.0};        // pitch rate

    auto delta = ap.compute(cmd, omega, Vec3d{300, 0, 0}, 50000.0, 0.01);

    // delta_pitch = Kp * az - Kd * q = 0.001 * 200 - 0.5 * 0.3 = 0.2 - 0.15 = 0.05
    EXPECT_NEAR(delta.y(), 0.05, 1e-6);
}

// ═══════════════════════════════════════════════════════════════
//  Yaw channel
// ═══════════════════════════════════════════════════════════════

TEST_F(SimpleAutopilotTest, PositiveAyProducesYawDeflection) {
    SimpleAutopilot ap(gains);
    auto cmd = make_cmd(100.0, 0.0);  // 100 m/s² starboard command
    auto delta = ap.compute(cmd, Vec3d::zero(), Vec3d{300, 0, 0}, 50000.0, 0.01);

    // delta_yaw = Kp * a_cmd_y = 0.001 * 100 = 0.1 rad
    EXPECT_NEAR(delta.z(), 0.1, 1e-6);
    EXPECT_NEAR(delta.y(), 0.0, 1e-10);  // pitch unaffected
}

TEST_F(SimpleAutopilotTest, YawRateDamping) {
    SimpleAutopilot ap(gains);
    auto cmd = make_cmd(0.0, 0.0);
    Vec3d omega{0.0, 0.0, 0.4};  // positive yaw rate

    auto delta = ap.compute(cmd, omega, Vec3d{300, 0, 0}, 50000.0, 0.01);

    // delta_yaw = -Kd * r = -0.5 * 0.4 = -0.2 rad
    EXPECT_NEAR(delta.z(), -0.2, 1e-6);
}

// ═══════════════════════════════════════════════════════════════
//  Roll channel
// ═══════════════════════════════════════════════════════════════

TEST_F(SimpleAutopilotTest, RollDampingOnly) {
    SimpleAutopilot ap(gains);
    auto cmd = make_cmd(0.0, 0.0);
    Vec3d omega{0.5, 0.0, 0.0};  // positive roll rate

    auto delta = ap.compute(cmd, omega, Vec3d{300, 0, 0}, 50000.0, 0.01);

    // delta_roll = -Kd_roll * p = -0.2 * 0.5 = -0.1 rad
    EXPECT_NEAR(delta.x(), -0.1, 1e-6);
}

TEST_F(SimpleAutopilotTest, RollNotAffectedByGuidance) {
    SimpleAutopilot ap(gains);
    auto cmd = make_cmd(100.0, 100.0);  // strong lateral commands
    auto delta = ap.compute(cmd, Vec3d::zero(), Vec3d{300, 0, 0}, 50000.0, 0.01);

    // Roll should be zero — no roll command from PN
    EXPECT_NEAR(delta.x(), 0.0, 1e-10);
}

// ═══════════════════════════════════════════════════════════════
//  Channel independence
// ═══════════════════════════════════════════════════════════════

TEST_F(SimpleAutopilotTest, ChannelsAreIndependent) {
    SimpleAutopilot ap(gains);
    auto cmd = make_cmd(50.0, 80.0);
    Vec3d omega{0.3, 0.2, 0.1};

    auto delta = ap.compute(cmd, omega, Vec3d{300, 0, 0}, 50000.0, 0.01);

    double exp_roll  = -gains.Kd_roll * 0.3;
    double exp_pitch = gains.Kp * 80.0 - gains.Kd * 0.2;
    double exp_yaw   = gains.Kp * 50.0 - gains.Kd * 0.1;

    EXPECT_NEAR(delta.x(), exp_roll, 1e-6);
    EXPECT_NEAR(delta.y(), exp_pitch, 1e-6);
    EXPECT_NEAR(delta.z(), exp_yaw, 1e-6);
}

// ═══════════════════════════════════════════════════════════════
//  Saturation
// ═══════════════════════════════════════════════════════════════

TEST_F(SimpleAutopilotTest, DeflectionClampsToMax) {
    SimpleAutopilot ap(gains);
    // Very large command that would exceed max_deflection
    auto cmd = make_cmd(0.0, 1000.0);  // Kp * 1000 = 1.0 rad >> 0.4 max
    auto delta = ap.compute(cmd, Vec3d::zero(), Vec3d{300, 0, 0}, 50000.0, 0.01);

    EXPECT_NEAR(delta.y(), gains.max_deflection, 1e-6);
}

TEST_F(SimpleAutopilotTest, NegativeDeflectionClampsToMin) {
    SimpleAutopilot ap(gains);
    auto cmd = make_cmd(0.0, -1000.0);
    auto delta = ap.compute(cmd, Vec3d::zero(), Vec3d{300, 0, 0}, 50000.0, 0.01);

    EXPECT_NEAR(delta.y(), -gains.max_deflection, 1e-6);
}

TEST_F(SimpleAutopilotTest, AllChannelsClampIndependently) {
    SimpleAutopilot ap(gains);
    auto cmd = make_cmd(1000.0, 1000.0);
    Vec3d omega{10.0, 0.0, 0.0};  // large roll rate

    auto delta = ap.compute(cmd, omega, Vec3d{300, 0, 0}, 50000.0, 0.01);

    EXPECT_NEAR(delta.x(), -gains.max_deflection, 1e-6);  // roll clamped
    EXPECT_NEAR(delta.y(), gains.max_deflection, 1e-6);   // pitch clamped
    EXPECT_NEAR(delta.z(), gains.max_deflection, 1e-6);   // yaw clamped
}

// ═══════════════════════════════════════════════════════════════
//  Custom gains
// ═══════════════════════════════════════════════════════════════

TEST_F(SimpleAutopilotTest, CustomGainsAreApplied) {
    SimpleAutopilot::Gains custom;
    custom.Kp = 0.005;
    custom.Kd = 1.0;
    custom.Kd_roll = 0.5;
    custom.max_deflection = 0.3;

    SimpleAutopilot ap(custom);
    auto cmd = make_cmd(0.0, 50.0);
    Vec3d omega{0.1, 0.2, 0.0};

    auto delta = ap.compute(cmd, omega, Vec3d{300, 0, 0}, 50000.0, 0.01);

    double exp_roll  = -0.5 * 0.1;
    double exp_pitch = 0.005 * 50.0 - 1.0 * 0.2;
    EXPECT_NEAR(delta.x(), exp_roll, 1e-6);
    EXPECT_NEAR(delta.y(), exp_pitch, 1e-6);
}

// ═══════════════════════════════════════════════════════════════
//  Interface
// ═══════════════════════════════════════════════════════════════

TEST_F(SimpleAutopilotTest, WorksThroughInterface) {
    std::unique_ptr<IAutopilot> iface = std::make_unique<SimpleAutopilot>(gains);
    auto cmd = make_cmd(0.0, 100.0);
    auto delta = iface->compute(cmd, Vec3d::zero(), Vec3d{300, 0, 0}, 50000.0, 0.01);
    EXPECT_NEAR(delta.y(), 0.1, 1e-6);
}