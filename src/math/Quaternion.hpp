#pragma once

#include "Vector3.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <concepts>
#include <iostream>
#include <stdexcept>

namespace sim::math {
    /**
     * @brief Unit quaternion for 3D rotation representation.
     * 
     * Quaternion layout: [w, x, y, z] where w is the scalar part
     * and (x, y, z) is the vector part.
     * 
     * For a rotation of angle theta about unit axis n_hat:
     * q = [cos(theta/2), sin(theta/2)*n_hat]
     * 
     * Advantages over Euler angles:
     *   - No gimbal lock (singularity-free)
     *   - Smooth interpolation via slerp
     *   - Compact (4 values vs 9 for a DCM)
     *   - Numerically stable for integration
     * 
     * Convention: Hamilton product (q1 * q2 applies q2 first, then q1)
     */

    template <std::floating_point T = double>
    class Quaternion 
    {
    public:
        // Data
        // Stored as [w, x, y, z] - scalar first
        T w{1};  // scalar part
        T x{0};  // vector part i
        T y{0};  // vector part j
        T z{0};  // vector part k

        // Constructors
        /// Default: identity quaternion (no rotation)
        constexpr Quaternion() noexcept = default;

        /// Construct from components
        constexpr Quaternion(T w_, T x_, T y_, T z_) noexcept
            : w(w_), x(x_), y(y_), z(z_) {}
        
        /// Construct from scalar and vector parts
        constexpr Quaternion(T w_, const Vector3<T>& v) noexcept 
            : w(w_), x(v.x()), y(v.y()), z(v.z()) {}

        // Accessors
        
        /// Extract vector (imaginary) part as a Vector3
        [[nodiscard]] constexpr Vector3<T> vector_part() const noexcept {
            return {x,y,z};
        }

        /// Extract the scalar (real) part
        [[nodiscard]] constexpr T scalar_part() const noexcept {
            return w;
        }

        // Quaternion arithmetic

        /**
         * @brief Hamilton product (quaternion multiplication).
         *
         * For rotations: (q1 * q2) represents applying q2 first, then q1.
         *
         * Formula:
         *   w' = w1*w2 - x1*x2 - y1*y2 - z1*z2
         *   x' = w1*x2 + x1*w2 + y1*z2 - z1*y2
         *   y' = w1*y2 - x1*z2 + y1*w2 + z1*x2
         *   z' = w1*z2 + x1*y2 - y1*x2 + z1*w2
         */
        [[nodiscard]] constexpr Quaternion operator*(const Quaternion& rhs) const noexcept {
            return {
                w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z,
                w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y,
                w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x,
                w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w
            };
        }

        /// Quaternion addition (used in integration, not rotation composition)
        [[nodiscard]] constexpr Quaternion operator+(const Quaternion& rhs) const noexcept {
            return {w + rhs.w, x + rhs.x, y + rhs.y, z + rhs.z};
        }

        /// Quaternion subtraction
        [[nodiscard]] constexpr Quaternion operator-(const Quaternion& rhs) const noexcept {
            return {w - rhs.w, x - rhs.x, y - rhs.y, z - rhs.z};
        }

        /// Scalar multiplication (used in integration for scaling derivatives)
        [[nodiscard]] constexpr Quaternion operator*(T s) const noexcept {
            return {w * s, x * s, y * s, z * s};
        }

        /// Scalar division
        [[nodiscard]] constexpr Quaternion operator/(T s) const {
            return {w / s, x / s, y / s, z / s};
        }

        /// Unary negation — represents the same rotation (q and -q are equivalent)
        [[nodiscard]] constexpr Quaternion operator-() const noexcept {
            return {-w, -x, -y, -z};
        }

        /**
         * @brief Conjugate: q* = [w, -x, -y, -z]
         *
         * For unit quaternions, the conjugate equals the inverse.
         * Useful for rotating vectors: v' = q * v * q*
         */
        [[nodiscard]] constexpr Quaternion conjugate() const noexcept {
            return {w, -x, -y, -z};
        }

        /// Squared norm
        [[nodiscard]] constexpr T norm_squared() const noexcept {
            return w * w + x * x + y * y + z * z;
        }

        /// Norm: |q|
        [[nodiscard]] T norm() const noexcept {
            return std::sqrt(norm_squared());
        }

        /**
         * @brief Normalize to unit quaternion.
         *
         * Should be called periodically during integration to correct
         * for numerical drift. A unit quaternion satisfies |q| = 1.
         */
        [[nodiscard]] Quaternion normalized() const {
            T n = norm();
            if (n < std::numeric_limits<T>::epsilon()) {
                throw std::runtime_error("Cannot normalize a near-zero quaternion");
            }
            return *this / n;
        }

        /**
         * @brief Inverse: q^-1 = q* / |q|^2
         *
         * For unit quaternions (|q| = 1), the inverse equals the conjugate.
         */
        [[nodiscard]] Quaternion inverse() const {
            T ns = norm_squared();
            if (ns < std::numeric_limits<T>::epsilon()) {
                throw std::runtime_error("Cannot invert a near-zero quaternion");
            }
            return conjugate() / ns;
        }

        /**
         * @brief Rotate a vector by this quaternion (active / body-to-inertial).
         *
         * v' = q * [0, v] * q*
         *
         * This performs an ACTIVE rotation, equivalent to multiplying by the
         * body-to-inertial DCM: v' = L_bi * v.
         *
         * To apply the inertial-to-body direction, use q.conjugate().rotate(v),
         * which is equivalent to v' = L_ib * v.
         */
        [[nodiscard]] Vector3<T> rotate(const Vector3<T>& v) const noexcept {
            // Optimized form (avoids constructing pure quaternions):
            // v' = v + 2w(u × v) + 2(u × (u × v))   where u = [x, y, z]
            Vector3<T> u = vector_part();
            Vector3<T> uv = u.cross(v);
            Vector3<T> uuv = u.cross(uv);
            return v + (T{2} * w) * uv + T{2} * uuv;
        }

        // Comparison
        /// Approximate equality (note: q and -q represent the same rotation)
        [[nodiscard]] constexpr bool approx_equal(
            const Quaternion& other,
            T epsilon = static_cast<T>(1e-12)) const noexcept
        {
            return (std::abs(w - other.w) < epsilon &&
                    std::abs(x - other.x) < epsilon &&
                    std::abs(y - other.y) < epsilon &&
                    std::abs(z - other.z) < epsilon);
        }

        /// Check if this represents the same rotation as other (accounts for q ≡ -q)
        [[nodiscard]] constexpr bool same_rotation(
            const Quaternion& other,
            T epsilon = static_cast<T>(1e-12)) const noexcept
        {
            return approx_equal(other, epsilon) || approx_equal(-other, epsilon);
        }

        // Static factories

        /// Identity quaternion (no rotation)
        [[nodiscard]] static constexpr Quaternion identity() noexcept {
            return {1, 0, 0, 0};
        }

        /**
         * @brief Construct from axis-angle representation.
         * @param axis  Rotation axis (will be normalized)
         * @param angle Rotation angle in radians
         *
         * q = [cos(theta/2), sin(theta/2) * nn_hat]
         */
        [[nodiscard]] static Quaternion from_axis_angle(
            const Vector3<T>& axis, T angle)
        {
            Vector3<T> n = axis.normalized();
            T half = angle / T{2};
            T s = std::sin(half);
            return {std::cos(half), n.x() * s, n.y() * s, n.z() * s};
        }

        /**
         * @brief Extract the axis and angle from this unit quaternion.
         * @return pair of (axis, angle) where angle is in [0, 2pi]
         *
         * If the rotation angle is near zero, returns (unit_x, 0) as
         * the axis is undefined for the identity rotation.
         */
        [[nodiscard]] std::pair<Vector3<T>, T> to_axis_angle() const {
            T half_angle = std::acos(std::clamp(w, T{-1}, T{1}));
            T angle = T{2} * half_angle;
            T s = std::sin(half_angle);

            if (std::abs(s) < std::numeric_limits<T>::epsilon()) {
                return {Vector3<T>::unit_x(), T{0}};
            }
            return {Vector3<T>{x / s, y / s, z / s}, angle};
        }
    };

    // Free functions
    /// scalar * quaternion (commutative convenience)
    template <std::floating_point T>
    [[nodiscard]] constexpr Quaternion<T> operator*(T s, const Quaternion<T>& q) noexcept {
        return q * s;
    }

    /// Stream output
    template <std::floating_point T>
    std::ostream& operator<<(std::ostream& os, const Quaternion<T>& q) {
        return os << "[" << q.w << ", " << q.x << ", " << q.y << ", " << q.z << "]";
    }

    /**
     * @brief Spherical linear interpolation (slerp) between two quaternions.
     *
     * Produces the shortest-path rotation between q0 and q1 at parameter t = [0,1].
     * t=0 returns q0, t=1 returns q1.
     *
     * Useful for smooth animation and trajectory blending.
     */
    template <std::floating_point T>
    [[nodiscard]] Quaternion<T> slerp(
        const Quaternion<T>& q0, const Quaternion<T>& q1, T t) noexcept
    {
        // Compute cosine of angle between quaternions
        T dot = q0.w * q1.w + q0.x * q1.x + q0.y * q1.y + q0.z * q1.z;

        // If dot < 0, negate one to take the shortest path
        Quaternion<T> q1_adj = q1;
        if (dot < T{0}) {
            q1_adj = -q1;
            dot = -dot;
        }

        // If quaternions are very close, fall back to linear interpolation
        // to avoid division by sin(theta) ~ 0
        if (dot > T{0.9995}) {
            Quaternion<T> result = q0 + t * (q1_adj - q0);
            return result.normalized();
        }

        T theta = std::acos(dot);
        T sin_theta = std::sin(theta);
        T w0 = std::sin((T{1} - t) * theta) / sin_theta;
        T w1 = std::sin(t * theta) / sin_theta;

        return w0 * q0 + w1 * q1_adj;
    }

    // Common aliases
    using Quatd = Quaternion<double>;
    using Quatf = Quaternion<float>;

} // namespace sim::math