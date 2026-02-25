#pragma once

#include "math/Vector3.hpp"
#include "math/Matrix3.hpp"
#include "math/Rotations.hpp"
#include <iostream>

namespace sim::core
{
    using namespace sim::math;
    /**
     * @brief Full 6-DOF simulation state (Euler angle formulation).
     * 
     * 13 states: position(3, NED inertial), velocity(3, body), 
     * Euler angles(3), angular rates(3, body), mass(1)
     */
    struct State {
        Vec3d position;       // [n, e, d] NED (m)
        Vec3d velocity_body;  // [u, v, w] body frame (m/s)
        EulerAnglesd euler;   // [phi, theta, psi] (rad)
        Vec3d omega_body;     // [p, q, r] body frame (rad/s)
        double mass{0.0};     // current vehicle mass (kg)

        [[nodiscard]] Vec3d velocity_intertial() const {
            return dcm::body_to_inertial(euler) * velocity_body;
        }

        [[nodiscard]] double speed() const noexcept {
            return velocity_body.magnitude();
        }

        [[nodiscard]] double alpha() const noexcept {
            return std::atan2(velocity_body.z(), velocity_body.x());
        }

        [[nodiscard]] double beta() const {
            double V = speed();
            if (V < 1e-6) return 0.0;
            return std::asin(velocity_body.y() / V);
        }
    };

    /// Time-derivative of State. Separate type to prevent misuse
    struct StateDerivative {
        Vec3d d_position;
        Vec3d d_velocity_body;
        Vec3d d_euler;
        Vec3d d_omega_body;
        double d_mass{0.0};
    };

    // State arithmetic for integrator (RK4)
    [[nodiscard]] inline State operator+(const State& s, const StateDerivative& ds) {
        State result;
        result.position       = s.position      + ds.d_position;
        result.velocity_body  = s.velocity_body + ds.d_velocity_body;
        result.euler.phi      = s.euler.phi     + ds.d_euler.x();
        result.euler.theta    = s.euler.theta   + ds.d_euler.y();
        result.euler.psi      = s.euler.psi     + ds.d_euler.z();
        result.omega_body     = s.omega_body    + ds.d_omega_body;
        result.mass           = s.mass          + ds.d_mass;
        return result;
    }

    [[nodiscard]] inline StateDerivative operator*(double dt, const StateDerivative& ds) {
        return {ds.d_position*dt, ds.d_velocity_body*dt, ds.d_euler*dt,
                ds.d_omega_body*dt, ds.d_mass*dt};
    }

    [[nodiscard]] inline StateDerivative operator*(const StateDerivative& ds, double dt) {
        return dt * ds;
    }

    [[nodiscard]] inline StateDerivative operator+(
    const StateDerivative& a, const StateDerivative& b)
    {
        return {a.d_position + b.d_position, a.d_velocity_body + b.d_velocity_body,
                a.d_euler + b.d_euler, a.d_omega_body + b.d_omega_body,
                a.d_mass + b.d_mass};
    }

    inline std::ostream& operator<<(std::ostream& os, const State& s) {
        os << "State {\n"
           << "  pos_NED:    " << s.position << "\n"
           << "  vel_body:   " << s.velocity_body << "\n"
           << "  euler(deg): [" << s.euler.phi*180/M_PI << ", "
                                   << s.euler.theta*180/M_PI << ", "
                                   << s.euler.psi*180/M_PI << "]\n"
           << "  omega_body: " << s.omega_body << "\n"
           << "  mass:       " << s.mass << " kg\n"
           << "  speed:      " << s.speed() << " m/s\n"
           << "  alpha:      " << s.alpha()*180/M_PI << " deg\n"
           << "  beta:       " << s.beta()*180/M_PI << " deg\n"
           << "}";
           return os;
    }
} // namespace sim::core