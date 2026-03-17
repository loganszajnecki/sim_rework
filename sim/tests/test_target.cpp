#include <gtest/gtest.h>
#include <cmath>
#include <memory>

#include "models/StationaryTarget.hpp"
#include "models/ConstantVelocityTarget.hpp"
#include "models/ManeuveringTarget.hpp"

using namespace sim::models;
using namespace sim::math;

// ═══════════════════════════════════════════════════════════════
//  Stationary Target
// ═══════════════════════════════════════════════════════════════

TEST(StationaryTarget, PositionIsConstant) {
    Vec3d pos{5000.0, 0.0, -500.0};
    StationaryTarget tgt(pos);

    auto s0 = tgt.compute(0.0);
    auto s1 = tgt.compute(10.0);
    auto s2 = tgt.compute(100.0);

    EXPECT_TRUE(s0.position.approx_equal(pos));
    EXPECT_TRUE(s1.position.approx_equal(pos));
    EXPECT_TRUE(s2.position.approx_equal(pos));
}

TEST(StationaryTarget, VelocityIsZero) {
    StationaryTarget tgt(Vec3d{1000.0, 2000.0, -300.0});
    auto s = tgt.compute(5.0);

    EXPECT_TRUE(s.velocity.approx_equal(Vec3d::zero()));
    EXPECT_TRUE(s.acceleration.approx_equal(Vec3d::zero()));
}

TEST(StationaryTarget, WorksThroughInterface) {
    std::unique_ptr<ITarget> iface = std::make_unique<StationaryTarget>(
        Vec3d{5000.0, 0.0, -500.0});
    auto s = iface->compute(0.0);
    EXPECT_NEAR(s.position.x(), 5000.0, 1e-10);
}

// ═══════════════════════════════════════════════════════════════
//  Constant Velocity Target
// ═══════════════════════════════════════════════════════════════

TEST(ConstantVelocityTarget, PositionEvolvesLinearly) {
    Vec3d pos{10000.0, 1000.0, -3000.0};
    Vec3d vel{-200.0, 0.0, 0.0};  // Incoming from north at 200 m/s
    ConstantVelocityTarget tgt(pos, vel);

    auto s0 = tgt.compute(0.0);
    EXPECT_TRUE(s0.position.approx_equal(pos));

    auto s10 = tgt.compute(10.0);
    Vec3d expected = pos + vel * 10.0;  // [8000, 1000, -3000]
    EXPECT_TRUE(s10.position.approx_equal(expected));
}

TEST(ConstantVelocityTarget, VelocityIsConstant) {
    Vec3d vel{-200.0, 50.0, 10.0};
    ConstantVelocityTarget tgt(Vec3d::zero(), vel);

    auto s0 = tgt.compute(0.0);
    auto s5 = tgt.compute(5.0);
    auto s20 = tgt.compute(20.0);

    EXPECT_TRUE(s0.velocity.approx_equal(vel));
    EXPECT_TRUE(s5.velocity.approx_equal(vel));
    EXPECT_TRUE(s20.velocity.approx_equal(vel));
}

TEST(ConstantVelocityTarget, AccelerationIsZero) {
    ConstantVelocityTarget tgt(Vec3d{1000, 0, -500}, Vec3d{-100, 0, 0});
    auto s = tgt.compute(5.0);
    EXPECT_TRUE(s.acceleration.approx_equal(Vec3d::zero()));
}

TEST(ConstantVelocityTarget, RangeToOriginDecreases) {
    // Target approaching the origin
    Vec3d pos{5000.0, 0.0, 0.0};
    Vec3d vel{-200.0, 0.0, 0.0};
    ConstantVelocityTarget tgt(pos, vel);

    double range_0 = tgt.compute(0.0).position.magnitude();
    double range_10 = tgt.compute(10.0).position.magnitude();
    double range_20 = tgt.compute(20.0).position.magnitude();

    EXPECT_GT(range_0, range_10);
    EXPECT_GT(range_10, range_20);
}

TEST(ConstantVelocityTarget, WorksThroughInterface) {
    std::unique_ptr<ITarget> iface = std::make_unique<ConstantVelocityTarget>(
        Vec3d{10000, 0, -3000}, Vec3d{-200, 0, 0});
    auto s = iface->compute(10.0);
    EXPECT_NEAR(s.position.x(), 8000.0, 1e-6);
}

// ═══════════════════════════════════════════════════════════════
//  Maneuvering Target
// ═══════════════════════════════════════════════════════════════

TEST(ManeuveringTarget, StraightBeforeManeuverStart) {
    Vec3d pos{10000.0, 0.0, -3000.0};
    Vec3d vel{-200.0, 0.0, 0.0};
    ManeuveringTarget tgt(pos, vel, 5.0, 10.0);  // 5g maneuver at t=10

    auto s5 = tgt.compute(5.0);
    Vec3d expected = pos + vel * 5.0;  // [9000, 0, -3000]
    EXPECT_TRUE(s5.position.approx_equal(expected));
    EXPECT_TRUE(s5.velocity.approx_equal(vel));
    EXPECT_TRUE(s5.acceleration.approx_equal(Vec3d::zero()));
}

TEST(ManeuveringTarget, AccelerationAfterManeuverStart) {
    Vec3d pos{10000.0, 0.0, -3000.0};
    Vec3d vel{-200.0, 0.0, 0.0};
    ManeuveringTarget tgt(pos, vel, 5.0, 10.0);

    auto s15 = tgt.compute(15.0);

    // Acceleration should be nonzero and perpendicular to velocity
    EXPECT_GT(s15.acceleration.magnitude(), 1.0);

    // Acceleration magnitude should be 5g
    double expected_accel = 5.0 * 9.80665;
    EXPECT_NEAR(s15.acceleration.magnitude(), expected_accel, 0.01);
}

TEST(ManeuveringTarget, ManeuverDirectionIsPerpendicular) {
    // Target flying south (negative North)
    Vec3d vel{-200.0, 0.0, 0.0};
    ManeuveringTarget tgt(Vec3d::zero(), vel, 3.0, 0.0);

    auto s1 = tgt.compute(1.0);

    // Maneuver direction should be perpendicular to velocity in horizontal plane
    // For vel = [-200, 0, 0], perp = [0, -200, 0] normalized = [0, -1, 0]
    // So acceleration should be in the East/West direction
    EXPECT_NEAR(s1.acceleration.x(), 0.0, 1e-6);
    EXPECT_NEAR(s1.acceleration.z(), 0.0, 1e-6);
    EXPECT_GT(std::abs(s1.acceleration.y()), 1.0);
}

TEST(ManeuveringTarget, ManeuverCausesLateralDisplacement) {
    Vec3d pos{10000.0, 0.0, -3000.0};
    Vec3d vel{-200.0, 0.0, 0.0};  // Flying south
    ManeuveringTarget tgt(pos, vel, 5.0, 0.0);  // Maneuver immediately

    // After some time, should have lateral (East) displacement
    auto s5 = tgt.compute(5.0);

    // Without maneuver, East position would be 0
    // With 5g lateral, displacement = 0.5 * a * t^2 = 0.5 * 49.03 * 25 ≈ 613 m
    EXPECT_GT(std::abs(s5.position.y()), 100.0);
}

TEST(ManeuveringTarget, ContinuityAtManeuverStart) {
    Vec3d pos{10000.0, 0.0, -3000.0};
    Vec3d vel{-200.0, 0.0, 0.0};
    ManeuveringTarget tgt(pos, vel, 5.0, 10.0);

    // State just before and just after maneuver start should be nearly identical
    auto s_before = tgt.compute(9.999);
    auto s_after = tgt.compute(10.001);

    double pos_diff = (s_after.position - s_before.position).magnitude();
    double vel_diff = (s_after.velocity - s_before.velocity).magnitude();

    // Position and velocity should be continuous
    EXPECT_LT(pos_diff, 1.0);    // Less than 1 meter
    EXPECT_LT(vel_diff, 0.1);    // Less than 0.1 m/s
}

TEST(ManeuveringTarget, VelocityChangesUnderManeuver) {
    Vec3d vel{-200.0, 0.0, 0.0};
    ManeuveringTarget tgt(Vec3d::zero(), vel, 5.0, 0.0);

    auto s0 = tgt.compute(0.0);
    auto s5 = tgt.compute(5.0);

    // Velocity magnitude should increase (additional lateral component)
    EXPECT_GT(s5.velocity.magnitude(), s0.velocity.magnitude());

    // Original velocity component should be unchanged
    EXPECT_NEAR(s5.velocity.x(), vel.x(), 1e-6);
}

TEST(ManeuveringTarget, ZeroGReducesToConstantVelocity) {
    Vec3d pos{5000.0, 0.0, -1000.0};
    Vec3d vel{-150.0, 50.0, 0.0};
    ManeuveringTarget tgt(pos, vel, 0.0, 0.0);  // 0g = no maneuver

    auto s10 = tgt.compute(10.0);
    Vec3d expected = pos + vel * 10.0;
    EXPECT_TRUE(s10.position.approx_equal(expected, 1e-6));
    EXPECT_TRUE(s10.velocity.approx_equal(vel, 1e-6));
}

TEST(ManeuveringTarget, WorksThroughInterface) {
    std::unique_ptr<ITarget> iface = std::make_unique<ManeuveringTarget>(
        Vec3d{10000, 0, -3000}, Vec3d{-200, 0, 0}, 5.0, 10.0);
    auto s = iface->compute(15.0);
    EXPECT_GT(s.acceleration.magnitude(), 1.0);
}