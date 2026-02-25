#include <gtest/gtest.h>
#include <cmath>

#include "core/State.hpp"
#include "core/Integrator.hpp"

using namespace sim::core;
using namespace sim::math;

static auto make_freefall_derivatives(double g) {
    return [g](double /*t*/, const State& s) -> StateDerivative {
        StateDerivative ds;
        auto L_ib = dcm::inertial_to_body(s.euler);
        ds.d_velocity_body = L_ib * Vec3d{0, 0, g};
        ds.d_position = dcm::body_to_inertial(s.euler) * s.velocity_body;
        ds.d_euler      = Vec3d::zero();
        ds.d_omega_body = Vec3d::zero();
        ds.d_mass       = 0.0;
        return ds;
    };
}

static State make_initial_state() {
    State s;
    s.position      = Vec3d::zero();
    s.velocity_body = Vec3d::zero();
    s.euler         = EulerAnglesd{0, 0, 0};
    s.omega_body    = Vec3d::zero();
    s.mass          = 10.0;
    return s;
}

TEST(Integrator, FreeFallMatchesAnalytic) {
    constexpr double g = 9.80665, dt = 0.001, t_end = 5.0;
    auto state = make_initial_state();
    auto integrator = make_integrator<State, StateDerivative>("rk4");
    auto deriv = make_freefall_derivatives(g);

    double t = 0.0;
    while (t < t_end) {
        state = integrator->step(deriv, t, state, dt);
        t += dt;
    }

    double z_expected = 0.5 * g * t_end * t_end;
    double w_expected = g * t_end;

    EXPECT_NEAR(state.position.z(), z_expected, z_expected * 1e-6);
    EXPECT_NEAR(state.velocity_body.z(), w_expected, w_expected * 1e-6);
    EXPECT_NEAR(state.position.x(), 0.0, 1e-10);
    EXPECT_NEAR(state.position.y(), 0.0, 1e-10);
    EXPECT_DOUBLE_EQ(state.mass, 10.0);
}

TEST(Integrator, RK4FourthOrderConvergence) {
    constexpr double g = 9.80665, t_end = 2.0;

    auto deriv = [g](double /*t*/, const State& s) -> StateDerivative {
        StateDerivative ds;
        auto L_ib = dcm::inertial_to_body(s.euler);
        auto L_bi = dcm::body_to_inertial(s.euler);
        ds.d_position = L_bi * s.velocity_body;
        Vec3d gravity_body = L_ib * Vec3d{0, 0, g};
        ds.d_velocity_body = gravity_body - s.omega_body.cross(s.velocity_body);
        ds.d_euler = dcm::euler_kinematic_matrix(s.euler) * s.omega_body;
        ds.d_omega_body = Vec3d::zero();
        ds.d_mass = 0.0;
        return ds;
    };

    auto run_sim = [&](double dt) -> double {
        State state;
        state.position = Vec3d::zero();
        state.velocity_body = Vec3d{300, 0, 0};
        state.euler = EulerAnglesd{0, 0.7854, 0};  // 45 degrees
        state.omega_body = Vec3d::zero();
        state.omega_body = Vec3d{0.1, 0.05, 0.02};  // small body rates (rad/s)
        state.mass = 1.0;

        auto integrator = make_integrator<State, StateDerivative>("rk4");
        double t = 0.0;
        int steps = static_cast<int>(t_end / dt);
        for (int i = 0; i < steps; ++i) {
            state = integrator->step(deriv, t, state, dt);
            t += dt;
        }
        return state.position.x();
    };

    // Use three step sizes and compare convergence
    double r1 = run_sim(0.04);
    double r2 = run_sim(0.02);
    double r3 = run_sim(0.01);

    double error_coarse = std::abs(r1 - r2);
    double error_fine   = std::abs(r2 - r3);
    double ratio = error_coarse / error_fine;

    // 4th order: halving step -> 16x error reduction. Allow > 12.
    EXPECT_GT(ratio, 12.0);
}