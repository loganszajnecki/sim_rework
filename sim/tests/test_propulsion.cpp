#include <gtest/gtest.h>
#include <cmath>
#include <memory>

#include "models/SolidRocketMotor.hpp"

using namespace sim::models;
using namespace sim::math;

class SolidRocketTest : public ::testing::Test {
protected:
    SolidRocketMotor::Config cfg;
    
    void SetUp() override {
        cfg.thrust = 5000.0;
        cfg.burn_time = 3.0;
        cfg.total_mass = 100.0;
        cfg.prop_mass = 30.0;
        cfg.inertia = sim::math::Mat3d{
            0.5, 0.0, 0.0,
            0.0, 10.0, 0.0,
            0.0, 0.0, 10.0
        };
        cfg.cg_body = {1.0, 0.0, 0.0};
    }
};

// ── Burning phase ────────────────────────────────────────────

TEST_F(SolidRocketTest, IsBurningDuringBurn) {
    SolidRocketMotor motor(cfg);
    EXPECT_TRUE(motor.is_burning(0.0));
    EXPECT_TRUE(motor.is_burning(1.5));
    EXPECT_TRUE(motor.is_burning(2.999));
}

TEST_F(SolidRocketTest, NotBurningAfterBurnout) {
    SolidRocketMotor motor(cfg);
    EXPECT_FALSE(motor.is_burning(3.0));
    EXPECT_FALSE(motor.is_burning(10.0));
}

TEST_F(SolidRocketTest, ThrustAlongBodyX) {
    SolidRocketMotor motor(cfg);
    auto out = motor.compute_thrust(1.0);
    EXPECT_DOUBLE_EQ(out.force.x(), 5000.0);
    EXPECT_DOUBLE_EQ(out.force.y(), 0.0);
    EXPECT_DOUBLE_EQ(out.force.z(), 0.0);
}

TEST_F(SolidRocketTest, ZeroThrustAfterBurnout) {
    SolidRocketMotor motor(cfg);
    auto out = motor.compute_thrust(5.0);
    EXPECT_TRUE(out.force.approx_equal(Vec3d::zero()));
    EXPECT_DOUBLE_EQ(out.mass_flow_rate, 0.0);
}

TEST_F(SolidRocketTest, MassFlowRate) {
    SolidRocketMotor motor(cfg);
    auto out = motor.compute_thrust(1.0);
    // 30 kg propellant / 3 s burn = 10 kg/s, negative = mass leaving
    EXPECT_DOUBLE_EQ(out.mass_flow_rate, -10.0);
}

// ── Mass properties ──────────────────────────────────────────

TEST_F(SolidRocketTest, InitialMass) {
    SolidRocketMotor motor(cfg);
    auto mp = motor.compute_mass_properties(0.0);
    EXPECT_DOUBLE_EQ(mp.mass, 100.0);
}

TEST_F(SolidRocketTest, MassDepletionDuringBurn) {
    SolidRocketMotor motor(cfg);
    // At t=1.5 s: half the propellant burned = 15 kg consumed
    auto mp = motor.compute_mass_properties(1.5);
    EXPECT_NEAR(mp.mass, 85.0, 0.01);
}

TEST_F(SolidRocketTest, FinalMassAfterBurnout) {
    SolidRocketMotor motor(cfg);
    auto mp = motor.compute_mass_properties(10.0);
    // total_mass - prop_mass = 100 - 30 = 70 kg
    EXPECT_NEAR(mp.mass, 70.0, 0.01);
}

TEST_F(SolidRocketTest, TotalImpulse) {
    SolidRocketMotor motor(cfg);
    // Integrate thrust over burn: constant 5000 N * 3 s = 15000 N*s
    double dt = 0.001;
    double impulse = 0.0;
    for (double t = 0.0; t < cfg.burn_time; t += dt) {
        impulse += motor.compute_thrust(t).force.x() * dt;
    }
    EXPECT_NEAR(impulse, 15000.0, 10.0);
}

TEST_F(SolidRocketTest, InertiaDecreasesDuringBurn) {
    SolidRocketMotor motor(cfg);
    auto mp0 = motor.compute_mass_properties(0.0);
    auto mp1 = motor.compute_mass_properties(1.5);
    auto mp2 = motor.compute_mass_properties(10.0);

    // Iyy should decrease as mass decreases
    EXPECT_GT(mp0.inertia(1,1), mp1.inertia(1,1));
    EXPECT_GT(mp1.inertia(1,1), mp2.inertia(1,1));
}

// ── Interface ────────────────────────────────────────────────

TEST_F(SolidRocketTest, WorksThroughInterface) {
    std::unique_ptr<IPropulsion> iface = std::make_unique<SolidRocketMotor>(cfg);
    EXPECT_TRUE(iface->is_burning(1.0));
    EXPECT_FALSE(iface->is_burning(5.0));
    EXPECT_DOUBLE_EQ(iface->compute_thrust(1.0).force.x(), 5000.0);
}