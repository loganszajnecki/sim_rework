#include <gtest/gtest.h>
#include <cmath>
#include <memory>

#include "models/ProNavGuidance.hpp"

using namespace sim::models;
using namespace sim::math;

class ProNavTest : public ::testing::Test {
protected:
    ProNavGuidance guidance{4.0};  // N = 4

    /// Build a SeekerOutput manually for controlled testing
    SeekerOutput make_seeker(const Vec3d& los_unit, const Vec3d& los_rate,
                             double range, double range_rate,
                             bool valid = true) {
        SeekerOutput out;
        out.los_unit = los_unit;
        out.los_rate = los_rate;
        out.range = range;
        out.range_rate = range_rate;
        out.track_valid = valid;
        return out;
    }
};

// ═══════════════════════════════════════════════════════════════
//  Zero command cases
// ═══════════════════════════════════════════════════════════════

TEST_F(ProNavTest, CollisionCourseZeroCommand) {
    // Head-on, no LOS rate → already on intercept, no correction needed
    auto seeker = make_seeker(
        Vec3d{1.0, 0.0, 0.0},   // LOS along body X
        Vec3d::zero(),            // Zero LOS rate = collision course
        5000.0,                   // Range
        -500.0);                  // Closing at 500 m/s

    auto cmd = guidance.compute(seeker, 300.0);
    EXPECT_NEAR(cmd.accel_cmd.magnitude(), 0.0, 1e-10);
}

TEST_F(ProNavTest, InvalidTrackZeroCommand) {
    auto seeker = make_seeker(Vec3d{1.0, 0.0, 0.0}, Vec3d{0, 0, 0.01},
                              5000.0, -500.0, false);

    auto cmd = guidance.compute(seeker, 300.0);
    EXPECT_NEAR(cmd.accel_cmd.magnitude(), 0.0, 1e-10);
}

TEST_F(ProNavTest, OpeningTargetZeroCommand) {
    // Positive range_rate = target moving away
    auto seeker = make_seeker(Vec3d{1.0, 0.0, 0.0}, Vec3d{0, 0, 0.01},
                              5000.0, +100.0);

    auto cmd = guidance.compute(seeker, 300.0);
    EXPECT_NEAR(cmd.accel_cmd.magnitude(), 0.0, 1e-10);
}

// ═══════════════════════════════════════════════════════════════
//  2D analytic validation
// ═══════════════════════════════════════════════════════════════

TEST_F(ProNavTest, Analytic2DHorizontalCrossing) {
    // Missile flying North, target crossing East.
    // In body frame (aligned with NED for simplicity):
    //   LOS unit: [1, 0, 0] (target ahead)
    //   LOS rate: [0, 0, ω] about Down axis (target drifting right)
    //   Range rate: -500 m/s (closing)
    //
    // a_cmd = N * Vc * (ω_LOS × û_LOS)
    //       = 4 * 500 * ([0,0,ω] × [1,0,0])
    //       = 4 * 500 * [0*0 - ω*0, ω*1 - 0*0, 0*0 - 0*1]
    //       = 4 * 500 * [0, ω, 0]
    //       = [0, 2000ω, 0]
    //
    // Acceleration is in +Y (body starboard = East), steering toward target.

    double omega = 0.02;  // rad/s
    double rdot = -500.0;
    double Vc = 500.0;

    auto seeker = make_seeker(
        Vec3d{1.0, 0.0, 0.0},
        Vec3d{0.0, 0.0, omega},
        5000.0, rdot);

    auto cmd = guidance.compute(seeker, 300.0);

    double expected_ay = 4.0 * Vc * omega;
    EXPECT_NEAR(cmd.accel_cmd.x(), 0.0, 1e-6);
    EXPECT_NEAR(cmd.accel_cmd.y(), expected_ay, 1e-6);
    EXPECT_NEAR(cmd.accel_cmd.z(), 0.0, 1e-6);
}

TEST_F(ProNavTest, Analytic2DVerticalCrossing) {
    // Target drifting above: LOS rate about Y axis
    //   ω_LOS = [0, -ω, 0]  (rotation about -Y lifts LOS upward)
    //   û_LOS = [1, 0, 0]
    //
    //   ω × û = [0,-ω,0] × [1,0,0] = [-ω*0-0*0, 0*1-0*0, 0*0-(-ω)*1]
    //         = [0, 0, ω]
    //
    //   a_cmd = N * Vc * [0, 0, ω] → push nose down (body +Z = ventral)

    double omega = 0.01;
    double rdot = -400.0;
    double Vc = 400.0;

    auto seeker = make_seeker(
        Vec3d{1.0, 0.0, 0.0},
        Vec3d{0.0, -omega, 0.0},
        3000.0, rdot);

    auto cmd = guidance.compute(seeker, 300.0);

    double expected_az = 4.0 * Vc * omega;
    EXPECT_NEAR(cmd.accel_cmd.x(), 0.0, 1e-6);
    EXPECT_NEAR(cmd.accel_cmd.y(), 0.0, 1e-6);
    EXPECT_NEAR(cmd.accel_cmd.z(), expected_az, 1e-6);
}

// ═══════════════════════════════════════════════════════════════
//  3D engagement
// ═══════════════════════════════════════════════════════════════

TEST_F(ProNavTest, ThreeDimensionalEngagement) {
    // LOS rate has both Y and Z components (combined pitch and yaw drift)
    double omega_y = -0.01;
    double omega_z = 0.015;
    double Vc = 600.0;

    auto seeker = make_seeker(
        Vec3d{1.0, 0.0, 0.0},
        Vec3d{0.0, omega_y, omega_z},
        8000.0, -Vc);

    auto cmd = guidance.compute(seeker, 300.0);

    // ω × û = [0, ωy, ωz] × [1, 0, 0] = [ωy*0-ωz*0, ωz*1-0*0, 0*0-ωy*1]
    //       = [0, ωz, -ωy]
    Vec3d expected = Vec3d{0.0, omega_z, -omega_y} * (4.0 * Vc);

    EXPECT_NEAR(cmd.accel_cmd.x(), expected.x(), 1e-6);
    EXPECT_NEAR(cmd.accel_cmd.y(), expected.y(), 1e-6);
    EXPECT_NEAR(cmd.accel_cmd.z(), expected.z(), 1e-6);
}

// ═══════════════════════════════════════════════════════════════
//  Command properties
// ═══════════════════════════════════════════════════════════════

TEST_F(ProNavTest, CommandPerpendicularToLOS) {
    // For any LOS rate, the command should be perpendicular to LOS
    auto seeker = make_seeker(
        Vec3d{1.0, 0.0, 0.0},
        Vec3d{0.0, -0.01, 0.02},
        5000.0, -500.0);

    auto cmd = guidance.compute(seeker, 300.0);

    // Dot product of command with LOS should be zero
    double dot = cmd.accel_cmd.dot(seeker.los_unit);
    EXPECT_NEAR(dot, 0.0, 1e-6);
}

TEST_F(ProNavTest, CommandPerpendicularToLOSOffAxis) {
    // LOS not along body X
    Vec3d los = Vec3d{0.8, 0.3, -0.2};
    los = los / los.magnitude();  // normalize

    auto seeker = make_seeker(los, Vec3d{0.001, -0.005, 0.003},
                              4000.0, -400.0);

    auto cmd = guidance.compute(seeker, 300.0);

    double dot = cmd.accel_cmd.dot(los);
    EXPECT_NEAR(dot, 0.0, 1e-4);
}

TEST_F(ProNavTest, CommandScalesWithNavRatio) {
    auto seeker = make_seeker(Vec3d{1, 0, 0}, Vec3d{0, 0, 0.02},
                              5000.0, -500.0);

    ProNavGuidance pn3(3.0);
    ProNavGuidance pn5(5.0);

    auto cmd3 = pn3.compute(seeker, 300.0);
    auto cmd5 = pn5.compute(seeker, 300.0);

    // Ratio of magnitudes should be 5/3
    double ratio = cmd5.accel_cmd.magnitude() / cmd3.accel_cmd.magnitude();
    EXPECT_NEAR(ratio, 5.0 / 3.0, 1e-6);
}

TEST_F(ProNavTest, CommandScalesWithClosingVelocity) {
    auto seeker_fast = make_seeker(Vec3d{1, 0, 0}, Vec3d{0, 0, 0.02},
                                    5000.0, -500.0);  // Vc = 500
    auto seeker_slow = make_seeker(Vec3d{1, 0, 0}, Vec3d{0, 0, 0.02},
                                    5000.0, -250.0);  // Vc = 250

    auto cmd_fast = guidance.compute(seeker_fast, 300.0);
    auto cmd_slow = guidance.compute(seeker_slow, 300.0);

    double ratio = cmd_fast.accel_cmd.magnitude() / cmd_slow.accel_cmd.magnitude();
    EXPECT_NEAR(ratio, 2.0, 1e-6);
}

// ═══════════════════════════════════════════════════════════════
//  Interface
// ═══════════════════════════════════════════════════════════════

TEST_F(ProNavTest, WorksThroughInterface) {
    std::unique_ptr<IGuidance> iface = std::make_unique<ProNavGuidance>(4.0);
    auto seeker = make_seeker(Vec3d{1, 0, 0}, Vec3d{0, 0, 0.01},
                              5000.0, -500.0);
    auto cmd = iface->compute(seeker, 300.0);
    EXPECT_GT(cmd.accel_cmd.magnitude(), 0.0);
}