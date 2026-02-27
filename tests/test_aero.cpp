#include <gtest/gtest.h>
#include <cmath>
#include <memory>

#include "models/SimpleAero.hpp"

using namespace sim::models;
using namespace sim::math;

class SimpleAeroTest : public ::testing::Test {
protected:
    SimpleAero::Coefficients coeffs;
    double qbar = 50000.0;   // dynamic pressure (Pa)
    double S = 0.05;         // reference area (m^2)
    double d = 0.2;          // reference length (m)

    void SetUp() override {
        coeffs.CA = 0.3;
        coeffs.CN_alpha = 10.0;
        coeffs.CY_beta = -10.0;
        coeffs.Cm_alpha = -3.0;
        coeffs.Cn_beta = 3.0;
        coeffs.Cl_delta = -0.5;
        coeffs.Cmq = -20.0;
        coeffs.Cnr = -20.0;
        coeffs.Clp = -5.0;
    }
};

// ── Zero angle of attack / sideslip ──────────────────────────

TEST_F(SimpleAeroTest, ZeroAlphaBetaOnlyDrag) {
    SimpleAero aero(coeffs);
    auto out = aero.compute(2.0, 0.0, 0.0, Vec3d::zero(), Vec3d::zero(), qbar, S, d);

    double qS = qbar * S;
    // Only axial force (drag), no normal/side force
    EXPECT_NEAR(out.force.x(), -qS * 0.3, 1e-6);
    EXPECT_NEAR(out.force.y(), 0.0, 1e-10);
    EXPECT_NEAR(out.force.z(), 0.0, 1e-10);
}

TEST_F(SimpleAeroTest, ZeroConditionsZeroMoments) {
    SimpleAero aero(coeffs);
    auto out = aero.compute(2.0, 0.0, 0.0, Vec3d::zero(), Vec3d::zero(), qbar, S, d);

    EXPECT_NEAR(out.moment.x(), 0.0, 1e-10);
    EXPECT_NEAR(out.moment.y(), 0.0, 1e-10);
    EXPECT_NEAR(out.moment.z(), 0.0, 1e-10);
}

// ── Angle of attack effects ──────────────────────────────────

TEST_F(SimpleAeroTest, PositiveAlphaProducesNormalForce) {
    SimpleAero aero(coeffs);
    double alpha = 0.1;  // ~5.7 degrees
    auto out = aero.compute(2.0, alpha, 0.0, Vec3d::zero(), Vec3d::zero(), qbar, S, d);

    double qS = qbar * S;
    // Normal force: -qS * CN_alpha * alpha (acts in -Z for positive alpha)
    EXPECT_NEAR(out.force.z(), -qS * coeffs.CN_alpha * alpha, 1e-6);
}

TEST_F(SimpleAeroTest, PositiveAlphaProducesNegativePitchMoment) {
    SimpleAero aero(coeffs);
    double alpha = 0.1;
    auto out = aero.compute(2.0, alpha, 0.0, Vec3d::zero(), Vec3d::zero(), qbar, S, d);

    // Cm_alpha is negative (statically stable), so positive alpha -> negative Cm -> restoring
    EXPECT_LT(out.moment.y(), 0.0);
}

// ── Sideslip effects ─────────────────────────────────────────

TEST_F(SimpleAeroTest, PositiveBetaProducesSideForce) {
    SimpleAero aero(coeffs);
    double beta = 0.05;
    auto out = aero.compute(2.0, 0.0, beta, Vec3d::zero(), Vec3d::zero(), qbar, S, d);

    double qS = qbar * S;
    EXPECT_NEAR(out.force.y(), qS * coeffs.CY_beta * beta, 1e-6);
}

TEST_F(SimpleAeroTest, PositiveBetaProducesYawMoment) {
    SimpleAero aero(coeffs);
    double beta = 0.05;
    auto out = aero.compute(2.0, 0.0, beta, Vec3d::zero(), Vec3d::zero(), qbar, S, d);

    // Cn_beta positive (stable), positive beta -> positive yaw moment (restoring)
    EXPECT_GT(out.moment.z(), 0.0);
}

// ── Force scaling ────────────────────────────────────────────

TEST_F(SimpleAeroTest, ForcesScaleWithDynamicPressure) {
    SimpleAero aero(coeffs);
    auto out1 = aero.compute(2.0, 0.1, 0.0, Vec3d::zero(), Vec3d::zero(), qbar, S, d);
    auto out2 = aero.compute(2.0, 0.1, 0.0, Vec3d::zero(), Vec3d::zero(), 2.0 * qbar, S, d);

    EXPECT_NEAR(out2.force.x(), 2.0 * out1.force.x(), 1e-6);
    EXPECT_NEAR(out2.force.z(), 2.0 * out1.force.z(), 1e-6);
}

TEST_F(SimpleAeroTest, ForcesScaleWithRefArea) {
    SimpleAero aero(coeffs);
    auto out1 = aero.compute(2.0, 0.1, 0.0, Vec3d::zero(), Vec3d::zero(), qbar, S, d);
    auto out2 = aero.compute(2.0, 0.1, 0.0, Vec3d::zero(), Vec3d::zero(), qbar, 2.0 * S, d);

    EXPECT_NEAR(out2.force.x(), 2.0 * out1.force.x(), 1e-6);
}

// ── Zero dynamic pressure → zero forces ──────────────────────

TEST_F(SimpleAeroTest, ZeroQBarZeroForces) {
    SimpleAero aero(coeffs);
    auto out = aero.compute(2.0, 0.1, 0.05, Vec3d{0.1, 0.2, 0.3}, Vec3d{0.01, 0, 0}, 0.0, S, d);

    EXPECT_NEAR(out.force.magnitude(), 0.0, 1e-10);
    EXPECT_NEAR(out.moment.magnitude(), 0.0, 1e-10);
}

// ── Interface ────────────────────────────────────────────────

TEST_F(SimpleAeroTest, WorksThroughInterface) {
    std::unique_ptr<IAerodynamics> iface = std::make_unique<SimpleAero>(coeffs);
    auto out = iface->compute(2.0, 0.0, 0.0, Vec3d::zero(), Vec3d::zero(), qbar, S, d);
    EXPECT_LT(out.force.x(), 0.0);  // drag is negative
}