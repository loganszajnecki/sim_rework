#include <gtest/gtest.h>
#include <cmath>
#include <memory>

#include "models/IdealSeeker.hpp"
#include "math/Rotations.hpp"

using namespace sim::models;
using namespace sim::math;

class IdealSeekerTest : public ::testing::Test {
protected:
    IdealSeeker seeker;

    // Identity DCM: body frame aligned with NED
    Mat3d I = Mat3d::identity();

    // Missile at origin, no velocity (unless overridden)
    Vec3d msl_pos = Vec3d::zero();
    Vec3d msl_vel = Vec3d::zero();
};

// ═══════════════════════════════════════════════════════════════
//  Basic geometry
// ═══════════════════════════════════════════════════════════════

TEST_F(IdealSeekerTest, TargetDirectlyNorth) {
    // Target 1000m North, body aligned with NED
    Vec3d tgt_pos{1000.0, 0.0, 0.0};
    Vec3d tgt_vel = Vec3d::zero();

    auto out = seeker.compute(msl_pos, msl_vel, tgt_pos, tgt_vel, I, 0.01);

    EXPECT_TRUE(out.track_valid);
    EXPECT_NEAR(out.range, 1000.0, 1e-6);
    EXPECT_NEAR(out.range_rate, 0.0, 1e-6);

    // With identity DCM, body X = North, so LOS in body = [1,0,0]
    EXPECT_NEAR(out.los_unit.x(), 1.0, 1e-6);
    EXPECT_NEAR(out.los_unit.y(), 0.0, 1e-6);
    EXPECT_NEAR(out.los_unit.z(), 0.0, 1e-6);

    // Look angle should be zero (target on boresight)
    EXPECT_NEAR(seeker.look_angle(), 0.0, 1e-6);
}

TEST_F(IdealSeekerTest, TargetDirectlyEast) {
    Vec3d tgt_pos{0.0, 500.0, 0.0};
    Vec3d tgt_vel = Vec3d::zero();

    auto out = seeker.compute(msl_pos, msl_vel, tgt_pos, tgt_vel, I, 0.01);

    EXPECT_NEAR(out.range, 500.0, 1e-6);

    // With identity DCM: body Y = East, so LOS = [0,1,0] in body
    EXPECT_NEAR(out.los_unit.x(), 0.0, 1e-6);
    EXPECT_NEAR(out.los_unit.y(), 1.0, 1e-6);

    // Look angle = 90 degrees (target broadside)
    EXPECT_NEAR(seeker.look_angle(), M_PI / 2.0, 1e-6);
}

TEST_F(IdealSeekerTest, TargetAbove) {
    // Target 1000m above (NED: -Z = up)
    Vec3d tgt_pos{0.0, 0.0, -1000.0};
    Vec3d tgt_vel = Vec3d::zero();

    auto out = seeker.compute(msl_pos, msl_vel, tgt_pos, tgt_vel, I, 0.01);

    EXPECT_NEAR(out.range, 1000.0, 1e-6);

    // With identity DCM: body Z = Down, target is Up → LOS body = [0,0,-1]
    EXPECT_NEAR(out.los_unit.z(), -1.0, 1e-6);

    // Elevation should be +90 degrees (straight up)
    EXPECT_NEAR(seeker.los_elevation(), M_PI / 2.0, 1e-3);
}

// ═══════════════════════════════════════════════════════════════
//  Range rate (closing velocity)
// ═══════════════════════════════════════════════════════════════

TEST_F(IdealSeekerTest, HeadOnClosingVelocity) {
    // Missile at origin moving North at 300 m/s
    // Target 5000m North moving South at 200 m/s
    msl_vel = Vec3d{300.0, 0.0, 0.0};
    Vec3d tgt_pos{5000.0, 0.0, 0.0};
    Vec3d tgt_vel{-200.0, 0.0, 0.0};

    auto out = seeker.compute(msl_pos, msl_vel, tgt_pos, tgt_vel, I, 0.01);

    // Range rate = V_rel · los = (-200 - 300) · [1,0,0] = -500 m/s
    // Negative = closing
    EXPECT_NEAR(out.range_rate, -500.0, 1e-6);
}

TEST_F(IdealSeekerTest, TailChaseClosingVelocity) {
    // Missile at origin moving North at 300 m/s
    // Target 3000m North moving North at 200 m/s (missile faster)
    msl_vel = Vec3d{300.0, 0.0, 0.0};
    Vec3d tgt_pos{3000.0, 0.0, 0.0};
    Vec3d tgt_vel{200.0, 0.0, 0.0};

    auto out = seeker.compute(msl_pos, msl_vel, tgt_pos, tgt_vel, I, 0.01);

    // Range rate = (200 - 300) · [1,0,0] = -100 m/s (still closing)
    EXPECT_NEAR(out.range_rate, -100.0, 1e-6);
}

TEST_F(IdealSeekerTest, RecedingTarget) {
    // Target moving away faster than missile
    msl_vel = Vec3d{100.0, 0.0, 0.0};
    Vec3d tgt_pos{1000.0, 0.0, 0.0};
    Vec3d tgt_vel{300.0, 0.0, 0.0};

    auto out = seeker.compute(msl_pos, msl_vel, tgt_pos, tgt_vel, I, 0.01);

    // Range rate = (300 - 100) · [1,0,0] = +200 m/s (opening)
    EXPECT_NEAR(out.range_rate, 200.0, 1e-6);
}

// ═══════════════════════════════════════════════════════════════
//  LOS rate
// ═══════════════════════════════════════════════════════════════

TEST_F(IdealSeekerTest, HeadOnCollisionCourseZeroLosRate) {
    // Pure head-on: no lateral motion → LOS rate = 0
    msl_vel = Vec3d{300.0, 0.0, 0.0};
    Vec3d tgt_pos{5000.0, 0.0, 0.0};
    Vec3d tgt_vel{-200.0, 0.0, 0.0};

    auto out = seeker.compute(msl_pos, msl_vel, tgt_pos, tgt_vel, I, 0.01);

    EXPECT_NEAR(out.los_rate.magnitude(), 0.0, 1e-10);
}

TEST_F(IdealSeekerTest, CrossingTargetNonzeroLosRate) {
    // Target moving purely East (crossing)
    Vec3d tgt_pos{5000.0, 0.0, 0.0};
    Vec3d tgt_vel{0.0, 100.0, 0.0};

    auto out = seeker.compute(msl_pos, msl_vel, tgt_pos, tgt_vel, I, 0.01);

    // LOS rate should be nonzero - target is crossing the LOS
    EXPECT_GT(out.los_rate.magnitude(), 0.001);
}

TEST_F(IdealSeekerTest, AnalyticLosRate2D) {
    // Analytic validation: missile stationary, target crossing
    //
    // Missile at origin, target at (R, 0, 0) moving in +Y at speed V_t
    // LOS rate = (R × V_rel) / r²
    //   R = [R, 0, 0], V_rel = [0, V_t, 0]
    //   R × V_rel = [0, 0, R*V_t]
    //   |ω| = R*V_t / R² = V_t / R

    double R = 5000.0;
    double V_t = 100.0;

    Vec3d tgt_pos{R, 0.0, 0.0};
    Vec3d tgt_vel{0.0, V_t, 0.0};

    auto out = seeker.compute(msl_pos, msl_vel, tgt_pos, tgt_vel, I, 0.01);

    double expected_los_rate = V_t / R;  // 0.02 rad/s
    EXPECT_NEAR(out.los_rate.magnitude(), expected_los_rate, 1e-8);

    // The LOS rate should be about the Z-axis (Down in NED)
    // R × V_rel = [0, 0, +R*V_t], positive Z = clockwise viewed from above
    EXPECT_NEAR(out.los_rate.z(), expected_los_rate, 1e-8);
    EXPECT_NEAR(out.los_rate.x(), 0.0, 1e-10);
    EXPECT_NEAR(out.los_rate.y(), 0.0, 1e-10);
}

TEST_F(IdealSeekerTest, AnalyticLosRateWithOffset) {
    // Target at (1000, 100, 0), missile at origin, both stationary except
    // missile moving North at 300 m/s.
    //
    // R = [1000, 100, 0], r = sqrt(1000² + 100²) = 1004.988
    // V_rel = [0, 0, 0] - [300, 0, 0] = [-300, 0, 0]
    // R × V_rel = [1000, 100, 0] × [-300, 0, 0]
    //           = [100*0 - 0*0, 0*(-300) - 1000*0, 1000*0 - 100*(-300)]
    //           = [0, 0, 30000]
    // ω = [0, 0, 30000] / r² = [0, 0, 30000 / 1009975.025]

    msl_vel = Vec3d{300.0, 0.0, 0.0};
    Vec3d tgt_pos{1000.0, 100.0, 0.0};
    Vec3d tgt_vel = Vec3d::zero();

    double r = tgt_pos.magnitude();
    double expected_omega_z = 30000.0 / (r * r);

    auto out = seeker.compute(msl_pos, msl_vel, tgt_pos, tgt_vel, I, 0.01);

    EXPECT_NEAR(out.los_rate.z(), expected_omega_z, 1e-8);
}

// ═══════════════════════════════════════════════════════════════
//  Body frame rotation
// ═══════════════════════════════════════════════════════════════

TEST_F(IdealSeekerTest, PitchedBodyFrame) {
    // Missile pitched up 45 degrees
    // Body X points northeast-up in NED
    EulerAnglesd euler{0.0, M_PI / 4.0, 0.0};
    Mat3d L_bi = dcm::body_to_inertial(euler);

    // Target directly North at 1000 m
    Vec3d tgt_pos{1000.0, 0.0, 0.0};
    Vec3d tgt_vel = Vec3d::zero();

    auto out = seeker.compute(msl_pos, msl_vel, tgt_pos, tgt_vel, L_bi, 0.01);

    // LOS in NED is [1, 0, 0] (North)
    // Body frame pitched 45° up: body X = [cos45, 0, -sin45] in NED
    // LOS in body: rotate NED [1,0,0] by L_ib
    // For 45° pitch: los_body should have equal X and Z components
    // los_body.x = cos(45°) ≈ 0.707, los_body.z = sin(45°) ≈ 0.707
    EXPECT_NEAR(out.los_unit.x(), std::cos(M_PI / 4.0), 1e-6);
    EXPECT_NEAR(out.los_unit.y(), 0.0, 1e-6);
    EXPECT_NEAR(out.los_unit.z(), std::sin(M_PI / 4.0), 1e-6);

    // Look angle should be 45 degrees
    EXPECT_NEAR(seeker.look_angle(), M_PI / 4.0, 1e-6);
}

// ═══════════════════════════════════════════════════════════════
//  Diagnostic angles
// ═══════════════════════════════════════════════════════════════

TEST_F(IdealSeekerTest, AzimuthAndElevation) {
    // Target northeast and above: (1000, 1000, -1000)
    Vec3d tgt_pos{1000.0, 1000.0, -1000.0};
    Vec3d tgt_vel = Vec3d::zero();

    seeker.compute(msl_pos, msl_vel, tgt_pos, tgt_vel, I, 0.01);

    // Azimuth: atan2(1000, 1000) = 45 degrees
    EXPECT_NEAR(seeker.los_azimuth(), M_PI / 4.0, 1e-6);

    // Elevation: atan2(1000, sqrt(1000² + 1000²)) = atan2(1000, 1414.2)
    // ≈ 35.26 degrees
    double expected_el = std::atan2(1000.0, std::sqrt(2.0) * 1000.0);
    EXPECT_NEAR(seeker.los_elevation(), expected_el, 1e-3);
}

// ═══════════════════════════════════════════════════════════════
//  Edge cases
// ═══════════════════════════════════════════════════════════════

TEST_F(IdealSeekerTest, TrackLostBelowMinRange) {
    // Target very close (below default 1.0m threshold)
    Vec3d tgt_pos{0.5, 0.0, 0.0};
    Vec3d tgt_vel = Vec3d::zero();

    auto out = seeker.compute(msl_pos, msl_vel, tgt_pos, tgt_vel, I, 0.01);

    EXPECT_FALSE(out.track_valid);
    EXPECT_NEAR(out.range, 0.5, 1e-6);
}

TEST_F(IdealSeekerTest, CustomMinRange) {
    IdealSeeker seeker_10m(10.0);

    // Target at 5m - below custom threshold
    Vec3d tgt_pos{5.0, 0.0, 0.0};
    auto out = seeker_10m.compute(msl_pos, msl_vel, tgt_pos, Vec3d::zero(), I, 0.01);
    EXPECT_FALSE(out.track_valid);

    // Target at 15m - above custom threshold
    tgt_pos = Vec3d{15.0, 0.0, 0.0};
    out = seeker_10m.compute(msl_pos, msl_vel, tgt_pos, Vec3d::zero(), I, 0.01);
    EXPECT_TRUE(out.track_valid);
}

TEST_F(IdealSeekerTest, WorksThroughInterface) {
    std::unique_ptr<ISeeker> iface = std::make_unique<IdealSeeker>();
    Vec3d tgt_pos{5000.0, 0.0, -1000.0};
    auto out = iface->compute(msl_pos, msl_vel, tgt_pos, Vec3d::zero(), I, 0.01);
    EXPECT_TRUE(out.track_valid);
    EXPECT_NEAR(out.range, std::sqrt(5000.0*5000.0 + 1000.0*1000.0), 1e-3);
}