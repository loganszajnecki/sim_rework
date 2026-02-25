#pragma once

#include "Vector3.hpp"
#include "Matrix3.hpp"
#include "Quaternion.hpp"

namespace sim::math {
    /**
     * @brief Euler angles in the aerospace convention:
     * Yaw -> Pitch -> Roll
     *    phi   = roll (about body x-axis)
     *    theta = pitch (about body y-axis)
     *    psi   = yaw (about body z-axis)
     * 
     * All angles in RADIANS. Gimbal lock at theta = +/- pi/2.
     */

    template <std::floating_point T = double>
    struct EulerAngles {
        T phi{};    // roll
        T theta{};  // pitch
        T psi{};    // yaw

        constexpr EulerAngles() noexcept = default;
        constexpr EulerAngles(T phi_, T theta_, T psi_) noexcept 
            : phi(phi_), theta(theta_), psi(psi_) {}
        
        /// Check proximity to gimbal lock
        [[nodiscard]] bool near_gimbal_lock(T tolerance = static_cast<T>(1e-4)) const noexcept {
            constexpr T half_pi = static_cast<T>(M_PI / 2.0);
            return std::abs(std::abs(theta) - half_pi) < tolerance;
        }
    };

    using EulerAnglesd = EulerAngles<double>;
    using EulerAnglesf = EulerAngles<float>;

    /*  Direction Cosine Matrix (DCM) builders
        Convention summary:
      
        Inertial-to-body:
            V_BF = R1(phi) * R2(theta) * R3(psi) * V_I
      
        Body-to-inertial (inverse / transpose, since R is orthogonal):
            V_I  = [R1(phi) * R2(theta) * R3(psi)]^T * V_BF
                 = R3(psi)^T * R2(theta)^T * R1(phi)^T * V_BF */

    namespace dcm 
    {      
        // ============= Elementary rotation matrices =============
        //
        //            [ 1    0       0   ]
        //  R1(φ) =  [ 0   cosφ   sinφ  ]
        //            [ 0  -sinφ   cosφ  ]
        //
        //            [ cosθ   0  -sinθ ]
        //  R2(θ) =  [  0     1    0   ]
        //            [ sinθ   0   cosθ ]
        //
        //            [ cosψ   sinψ   0 ]
        //  R3(ψ) =  [-sinψ   cosψ   0 ]
        //            [  0      0     1 ]
        // ========================================================
        
        /// R1(phi) — rotation about the X-axis (Roll)
        template <std::floating_point T>
        [[nodiscard]] constexpr Matrix3<T> rotation_x(T angle) noexcept {
            const T c = std::cos(angle);
            const T s = std::sin(angle);
            return {
                1,  0,  0,
                0,  c,  s,
                0, -s,  c
            };
        }

        /// R2(theta) — rotation about the Y-axis (Pitch)
        template <std::floating_point T>
        [[nodiscard]] constexpr Matrix3<T> rotation_y(T angle) noexcept {
            const T c = std::cos(angle);
            const T s = std::sin(angle);
            return {
                c, 0, -s,
                0, 1,  0,
                s, 0,  c
            };
        }

        /// R3(psi) — rotation about the Z-axis (Yaw)
        template <std::floating_point T>
        [[nodiscard]] constexpr Matrix3<T> rotation_z(T angle) noexcept {
            const T c = std::cos(angle);
            const T s = std::sin(angle);
            return {
                c, s, 0,
                -s, c, 0,
                0, 0, 1
            };
        }

        // Composite DCMs from Euler angles 

        // Yaw-Pitch-Roll
        template <std::floating_point T>
        [[nodiscard]] Matrix3<T> inertial_to_body(const EulerAngles<T>& e) noexcept {
            return rotation_x(e.phi) * rotation_y(e.theta) * rotation_z(e.psi);
        }

        // Roll-Pitch-Yaw
        template <std::floating_point T>
        [[nodiscard]] Matrix3<T> body_to_inertial(const EulerAngles<T>& e) noexcept {
            return inertial_to_body(e).transposed();
        }

        // DCM from quaternion
        /**
         * @brief Inertial-to-body DCM from a unit quaternion.
         *
         * Equivalent to L_ib = inertial_to_body(euler), but computed
         * directly from the quaternion without going through Euler angles.
         *
         * Given q = [w, x, y, z]:
         *
         *        [ 1-2(y²+z²)    2(xy+wz)    2(xz-wy) ]
         * L_ib = [ 2(xy-wz)    1-2(x²+z²)    2(yz+wx) ]
         *        [ 2(xz+wy)      2(yz-wx)  1-2(x²+y²) ]
         */
        template <std::floating_point T>
        [[nodiscard]] Matrix3<T> inertial_to_body(const Quaternion<T>& q) noexcept {
            const T xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
            const T xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
            const T wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;

            return {
                T{1} - T{2} * (yy + zz),  T{2} * (xy + wz),         T{2} * (xz - wy),
                T{2} * (xy - wz),         T{1} - T{2} * (xx + zz),   T{2} * (yz + wx),
                T{2} * (xz + wy),         T{2} * (yz - wx),          T{1} - T{2} * (xx + yy)
            };
        }
        /// Body-to-inertial DCM from unit quaternion
        template <std::floating_point T>
        [[nodiscard]] Matrix3<T> body_to_inertial(const Quaternion<T>& q) noexcept {
            return inertial_to_body(q).transposed();
        }

        /**
         * @brief Euler angle kinematic matrix.
         *
         * Relates body angular rates [p, q, r] to Euler angle rates [phid, thetad, psid]:
         *
         * SINGULAR at θ = +/- pi/2 (gimbal lock).
         */
        template <std::floating_point T>
        [[nodiscard]] Matrix3<T> euler_kinematic_matrix(const EulerAngles<T>& e) {
            const T sp = std::sin(e.phi);
            const T cp = std::cos(e.phi);
            const T tt = std::tan(e.theta);
            const T ct = std::cos(e.theta);

            if (std::abs(ct) < std::numeric_limits<T>::epsilon()) {
                throw std::runtime_error(
                    "Euler kinematic matrix singular: pitch angle at +/- 90 deg");
            }

            const T sect = T{1} / ct;

            return {
                T{1},  sp * tt,    cp * tt,
                T{0},  cp,        -sp,
                T{0},  sp * sect,  cp * sect
            };
        }
    } // namespace dcm

    // Conversions between Euler angles, quaternions, and DCMs
    namespace convert 
    {
        /**
         * @brief Euler angles → quaternion.
         *
         * Constructs the quaternion equivalent of the 3-2-1 rotation
         * sequence R1(phi) * R2(theta) * R3(psi), representing the
         * inertial-to-body transformation.
         */
        template <std::floating_point T>
        [[nodiscard]] Quaternion<T> euler_to_quaternion(const EulerAngles<T>& e) noexcept {
            const T cp = std::cos(e.phi   / T{2});
            const T sp = std::sin(e.phi   / T{2});
            const T ct = std::cos(e.theta / T{2});
            const T st = std::sin(e.theta / T{2});
            const T cs = std::cos(e.psi   / T{2});
            const T ss = std::sin(e.psi   / T{2});

            return {
                cp * ct * cs + sp * st * ss,   // w
                sp * ct * cs - cp * st * ss,   // x
                cp * st * cs + sp * ct * ss,   // y
                cp * ct * ss - sp * st * cs    // z
            };
        }

        /**
         * @brief Quaternion → Euler angles (3-2-1 / ZYX).
         *
         * Extracts φ, θ, ψ from a unit quaternion. The result matches
         * the angles that would produce the same rotation via
         * R1(φ) * R2(θ) * R3(ψ).
         *
         * WARNING: still produces gimbal lock at θ = +/- pi/2, because
         * Euler angles are inherently singular there. The quaternion
         * itself is fine — only the *extraction* to Euler is problematic.
         */
        template <std::floating_point T>
        [[nodiscard]] EulerAngles<T> quaternion_to_euler(const Quaternion<T>& q) noexcept {
            // Roll
            T sinr_cosp = T{2} * (q.w * q.x + q.y * q.z);
            T cosr_cosp = T{1} - T{2} * (q.x * q.x + q.y * q.y);
            T phi = std::atan2(sinr_cosp, cosr_cosp);

            // Pitch — clamped to avoid NaN from asin
            T sinp = T{2} * (q.w * q.y - q.z * q.x);
            T theta;
            if (std::abs(sinp) >= T{1}) {
                theta = std::copysign(static_cast<T>(M_PI / 2.0), sinp);
            } else {
                theta = std::asin(sinp);
            }

            // Yaw 
            T siny_cosp = T{2} * (q.w * q.z + q.x * q.y);
            T cosy_cosp = T{1} - T{2} * (q.y * q.y + q.z * q.z);
            T psi = std::atan2(siny_cosp, cosy_cosp);

            return {phi, theta, psi};
        }

        /**
         * @brief DCM → quaternion (Shepperd's method).
         *
         * Extracts a unit quaternion from a rotation matrix using the
         * numerically stable Shepperd method, which avoids division by
         * near-zero values by selecting the largest diagonal element.
         */
        template <std::floating_point T>
        [[nodiscard]] Quaternion<T> dcm_to_quaternion(const Matrix3<T>& m) noexcept {
            T trace = m.trace();
            Quaternion<T> q;

            if (trace > T{0}) {
                T s = T{2} * std::sqrt(trace + T{1});
                q.w = s / T{4};
                q.x = (m(1, 2) - m(2, 1)) / s;
                q.y = (m(2, 0) - m(0, 2)) / s;
                q.z = (m(0, 1) - m(1, 0)) / s;
            } else if (m(0, 0) > m(1, 1) && m(0, 0) > m(2, 2)) {
                T s = T{2} * std::sqrt(T{1} + m(0, 0) - m(1, 1) - m(2, 2));
                q.w = (m(1, 2) - m(2, 1)) / s;
                q.x = s / T{4};
                q.y = (m(0, 1) + m(1, 0)) / s;
                q.z = (m(0, 2) + m(2, 0)) / s;
            } else if (m(1, 1) > m(2, 2)) {
                T s = T{2} * std::sqrt(T{1} + m(1, 1) - m(0, 0) - m(2, 2));
                q.w = (m(2, 0) - m(0, 2)) / s;
                q.x = (m(0, 1) + m(1, 0)) / s;
                q.y = s / T{4};
                q.z = (m(1, 2) + m(2, 1)) / s;
            } else {
                T s = T{2} * std::sqrt(T{1} + m(2, 2) - m(0, 0) - m(1, 1));
                q.w = (m(0, 1) - m(1, 0)) / s;
                q.x = (m(0, 2) + m(2, 0)) / s;
                q.y = (m(1, 2) + m(2, 1)) / s;
                q.z = s / T{4};
            }

            // Ensure w > 0 for a canonical representation
            if (q.w < T{0}) {
                q = -q;
            }

            return q;
        }

        /**
         * @brief Quaternion kinematic derivative.
         *
         * Computes dot from the current quaternion and body angular rates [p, q, r]:
         *
         * This is the quaternion alternative to the Euler kinematic matrix.
         * Unlike the Euler form, it has NO singularities.
         */
        template <std::floating_point T>
        [[nodiscard]] Quaternion<T> quaternion_derivative(
            const Quaternion<T>& q, const Vector3<T>& omega) noexcept
        {
            const T p = omega.x(), qr = omega.y(), r = omega.z();

            return T{0.5} * Quaternion<T>{
                -p * q.x - qr * q.y - r * q.z,
                p * q.w + r  * q.y - qr * q.z,
                qr * q.w - r  * q.x + p * q.z,
                r * q.w + qr * q.x - p * q.y
            };
        }
    } // namespace convert

    namespace frames
    {
        /// Earth-fixed NED inertial frame
        /// +X = North, +Y = East, +Z = down (gravity acts in +Z)
        struct NED {
            static constexpr const char* name = "NED";
        };

        /// Body-fixed frame, origin at CG
        /// +X = forward (nose), +Y = starboard (right), +Z = ventral (belly)
        struct Body {
            static constexpr const char* name = "Body";
        };

        /// Launch-pad frame, aligned with NED at launch, origin at launch point
        struct Launch {
            static constexpr const char* name = "Launch";
        };
    } // namespace frames
} // namespace sim::math
