#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <fstream>

#include "math/Vector3.hpp"
#include "math/Matrix3.hpp"
#include "math/Rotations.hpp"
#include "core/State.hpp"
#include "core/Integrator.hpp"

using namespace sim::math;
using namespace sim::core;

int main() {
    std::cout << "═══════════════════════════════════════════════\n"
              << "  6-DOF Missile Simulation — Scaffold Demo\n"
              << "═══════════════════════════════════════════════\n\n";

    constexpr double g     = 9.80665;
    constexpr double dt    = 0.01;
    constexpr double V0    = 300.0;
    constexpr double pitch = 45.0 * M_PI / 180.0;

    State state;
    state.position      = Vec3d{0, 0, 0};
    state.velocity_body = Vec3d{V0, 0, 0};
    state.euler         = EulerAnglesd{0, pitch, 0};
    state.omega_body    = Vec3d::zero();
    state.mass          = 100.0;

    std::cout << "Initial state:\n" << state << "\n\n";

    auto compute_derivatives = [g](double /*t*/, const State& s) -> StateDerivative {
        StateDerivative ds;
        auto L_bi = dcm::body_to_inertial(s.euler);
        ds.d_position = L_bi * s.velocity_body;

        auto L_ib = dcm::inertial_to_body(s.euler);
        Vec3d gravity_body = L_ib * Vec3d{0, 0, g};
        Vec3d coriolis     = s.omega_body.cross(s.velocity_body);
        ds.d_velocity_body = gravity_body - coriolis;

        auto H = dcm::euler_kinematic_matrix(s.euler);
        ds.d_euler = H * s.omega_body;
        ds.d_omega_body = Vec3d::zero();
        ds.d_mass = 0.0;
        return ds;
    };

    auto integrator = make_integrator<State, StateDerivative>("rk4");

    double t = 0.0;
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "  Time(s)   North(m)   Alt(m)   Speed(m/s)   Pitch(°)\n"
              << "  ─────────────────────────────────────────────────────\n";

    while (true) {
        double altitude = -state.position.z();

        if (std::fmod(t, 5.0) < dt * 0.5) {
            std::cout << "  " << std::setw(7) << t
                      << "  " << std::setw(9) << state.position.x()
                      << "  " << std::setw(7) << altitude
                      << "  " << std::setw(11) << state.speed()
                      << "  " << std::setw(9) << state.euler.theta * 180/M_PI
                      << "\n";
        }

        if (t > 0.1 && altitude <= 0.0) {
            std::cout << "\n  *** Impact at t = " << t << " s ***\n";
            break;
        }
        if (t > 120.0) {
            std::cout << "\n  *** Timeout ***\n";
            break;
        }

        state = integrator->step(compute_derivatives, t, state, dt);
        t += dt;
    }

    // Analytic comparison
    double theta = 45.0 * M_PI / 180.0;
    double range_analytic = V0*V0 * std::sin(2.0*theta) / g;
    double time_analytic  = 2.0*V0 * std::sin(theta) / g;

    std::cout << "\n  Analytic range:    " << range_analytic << " m"
              << "\n  Simulated range:   " << state.position.x() << " m"
              << "\n  Analytic time:     " << time_analytic << " s"
              << "\n  Simulated time:    " << t << " s"
              << "\n  Range error:       "
              << std::abs(state.position.x() - range_analytic) << " m\n"
              << std::endl;

    return 0;
}