#include "core/EOM.hpp"
#include "math/Rotations.hpp"
#include <cmath>
#include <algorithm>

namespace sim::core
{
    using namespace sim::math;

    StateDerivative EOM::compute(
        double t,
        const State& state,
        const Vehicle& vehicle)
    {
        StateDerivative ds;

        // DCM (compute once, reuse)
        Mat3d L_bi = dcm::body_to_inertial(state.euler);
        Mat3d L_ib = L_bi.transposed();

        // 1. Atmosphere
        // Altitude = -Z in NED (positive Z is down)
        double altitude = -state.position.z();
        auto atmo = vehicle.atmosphere().compute(altitude);

        // 2. Airspeed and flight condition
        // TODO: subtract wind when IWind is implemented.
        // airspeed_body = velocity_body - L_ib * wind_ned
        Vec3d airspeed_body = state.velocity_body;

        double V = airspeed_body.magnitude();

        double alpha = 0.0;
        double beta  = 0.0;
        double mach  = 0.0;
        double qbar  = 0.0;

        if (V > 1e-6) {
            alpha = std::atan2(airspeed_body.z(), airspeed_body.x());
            beta  = std::asin(std::clamp(airspeed_body.y() / V, -1.0, 1.0));
            mach  = V / atmo.speed_of_sound;
            qbar  = 0.5 * atmo.density * V * V;
        }

        // 3. Gravity
        // The gravity model returns the acceleration (m/s^2) in NED.
        // Rotate to the body frame for use in the body-frame EOM. 
        Vec3d gravity_ned  = vehicle.gravity().compute(state.position);
        Vec3d gravity_body = L_ib * gravity_ned;

        // 4. Propulsion
        auto prop = vehicle.propulsion().compute_thrust(t);
        auto mass_props = vehicle.propulsion().compute_mass_properties(t);

        double mass   = mass_props.mass;
        Mat3d inertia = mass_props.inertia;
        
        // 5. Fin deflections
        // TODO: query IActuator when implemented. 
        Vec3d fin_deflections = Vec3d::zero();

        // 6. Aerodynamics
        auto aero = vehicle.aerodynamics().compute(
            mach, alpha, beta,
            state.omega_body, fin_deflections,
            qbar, vehicle.ref_area, vehicle.ref_length);
        
        // 7. Translational kinematics
        // d(pos)/dt|NED = L_bi * V_body;
        ds.d_position = L_bi * state.velocity_body;

        // 8. Translational dynamics
        // d(V)/dt|BODY = g_body + (F_thrust + F_aero) / m - cross(omega, V);
        Vec3d coriolis = state.omega_body.cross(state.velocity_body);
        ds.d_velocity_body = gravity_body + (prop.force + aero.force) / mass - coriolis;

        // 9. Rotational Kinematics
        // [d(phi)/dt, d(theta)/dt, d(psi)/dt] = H * [p, q, r]
        Mat3d H = dcm::euler_kinematic_matrix(state.euler);
        ds.d_euler = H * state.omega_body;

        // 10. Rotational dynamics
        // d(omega)/dt = inv(I) * (M_aero + M_thrust - cross(omega, I*omega));
        Vec3d M_total    = aero.moment + prop.moment;
        Vec3d gyroscopic = state.omega_body.cross(inertia * state.omega_body);
        ds.d_omega_body  = inertia.inverse() * (M_total - gyroscopic);

        // 11. Mass depletion
        ds.d_mass = prop.mass_flow_rate;

        return ds;
    }
}