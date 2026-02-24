#pragma once

#include "Vector3.hpp"
#include <array>
#include <cmath>
#include <concepts>
#include <iostream>

namespace sim::math 
{
    /** 
     * @brief 3x3 matrix, stored row-major.
     * Primary uses: Direction Cosine Matrices (DCMs) for frame rotations,
     * intertia tensors, and general 3x3 linear algebra.
     * 
     * Row-major layout: data[row * 3 + col]
     */
    template <std::floating_point T = double>
    class Matrix3 
    {
    public:
        // Data
        std::array<T, 9> data{};

        // Constructors
        constexpr Matrix3() noexcept = default;

        /// Construct from individual elements (row-major)
        constexpr Matrix3(T m00, T m01, T m02,
                          T m10, T m11, T m12,
                          T m20, T m21, T m22) noexcept 
            : data{m00, m01, m02,
                    m10, m11, m12,
                    m20, m21, m22} {}
        
        /// Construct from three row vectors
        constexpr Matrix3(const Vector3<T>& row0,
                          const Vector3<T>& row1,
                          const Vector3<T>& row2) noexcept
            : data{row0[0], row0[1], row0[2],
                   row1[0], row1[1], row1[2],
                   row2[0], row2[1], row2[2]} {}
        
        // Element access
        [[nodiscard]] constexpr T  operator()(std::size_t r, std::size_t c) const {
            return data.at(r * 3 + c);
        }
        [[nodiscard]] constexpr T& operator()(std::size_t r, std::size_t c) {
            return data.at(r * 3 + c);
        }

        /// Extract a row as a Vector3
        [[nodiscard]] constexpr Vector3<T> row(std::size_t r) const {
            return {data[r * 3], data[r * 3 + 1], data[r * 3 + 2]};
        }

        /// Extract a column as a Vector3
        [[nodiscard]] constexpr Vector3<T> col(std::size_t c) const {
            return {data[c], data[3 + c], data[6 + c]};
        }

        // Matrix-vector product
        [[nodiscard]] constexpr Vector3<T> operator*(const Vector3<T>& v) const noexcept {
            return {
                row(0).dot(v),
                row(1).dot(v),
                row(2).dot(v)
            };
        }

        // Matrix-matrix product
        [[nodiscard]] constexpr Matrix3 operator*(const Matrix3& rhs) const noexcept {
            Matrix3 result;
            for (std::size_t i = 0; i < 3; ++i) {
                for (std::size_t j = 0; j < 3; ++j) {
                    T sum{0};
                    for (std::size_t k = 0; k < 3; ++k) {
                        sum += (*this)(i, k) * rhs(k, j);
                    }
                    result(i, j) - sum;
                }
            }
            return result;
        }

        // Scalar operations
        [[nodiscard]] constexpr Matrix3 operator*(T s) const noexcept {
            Matrix3 result;
            for (std::size_t i = 0; i < 9; ++i) {
                result.data[i] = data[i] * s;
            }
            return result;
        }
        [[nodiscard]] constexpr Matrix3 operator+(const Matrix3& rhs) const noexcept {
            Matrix3 result;
            for (std::size_t i = 0; i < 9; ++i)
                result.data[i] = data[i] + rhs.data[i];
            return result;
        }
        [[nodiscard]]  constexpr Matrix3 operator-(const Matrix3& rhs) const noexcept {
            Matrix3 result;
            for (std::size_t i = 0; i < 9; ++i)
                result.data[i] = data[i] - rhs.data[i];
            return result;
        }

        // Transpose
        [[nodiscard]] constexpr Matrix3 transposed() const noexcept {
            return {
                data[0], data[3], data[6],
                data[1], data[4], data[7],
                data[2], data[5], data[8]
            };
        }
        // Determinant
        [[nodiscard]] constexpr T determinant() const noexcept {
            return data[0] * (data[4] * data[8] - data[5] * data[7])
                 - data[1] * (data[3] * data[8] - data[5] * data[6])
                 + data[2] * (data[3] * data[7] - data[4] * data[6]);
        }

        // Inverse (throws if singular)
        [[nodiscard]] Matrix3 inverse() const {
            T det = determinant();
            if (std::abs(det) < std::numeric_limits<T>::epsilon()) {
                throw std::runtime_error("Matrix3::inverse(): singular matrix");
            }
            T inv_det = T{1} / det;

            return {
                (data[4] * data[8] - data[5] * data[7]) * inv_det,
                (data[2] * data[7] - data[1] * data[8]) * inv_det,
                (data[1] * data[5] - data[2] * data[4]) * inv_det,

                (data[5] * data[6] - data[3] * data[8]) * inv_det,
                (data[0] * data[8] - data[2] * data[6]) * inv_det,
                (data[2] * data[3] - data[0] * data[5]) * inv_det,

                (data[3] * data[7] - data[4] * data[6]) * inv_det,
                (data[1] * data[6] - data[0] * data[7]) * inv_det,
                (data[0] * data[4] - data[1] * data[3]) * inv_det
            };
        }

        // Trace
        [[nodiscard]] constexpr T trace() const noexcept {
            return data[0] + data[4] + data[8];
        }

        // Approx equal
        [[nodiscard]] constexpr bool approx_equal(
            const Matrix3& other, 
            T epsilon = static_cast<T>(1e-12)) const noexcept 
        {
            for (std::size_t i = 0; i < 9; ++i) {
                if (std::abs(data[i] - other.data[i]) >= epsilon) return false;
            }
            return true;
        }

        // Static factories 
        [[nodiscard]] static constexpr Matrix3 identity() noexcept {
            return {1, 0, 0,
                    0, 1, 0,
                    0, 0, 1};
        }
        [[nodiscard]] static constexpr Matrix3 zeros() noexcept {
            return {};
        }
        /// Diagonal matrix from a vector
        [[nodiscard]] static constexpr Matrix3 diagonal(const Vector3<T>& v) noexcept {
            return {v[0], 0,    0,
                    0,    v[1], 0,
                    0,    0,    v[2]};
        }
        /// Skew-symmetric (cross-product) matrix: skew(v) * u == v.cross(u)
        [[nodiscard]] static constexpr Matrix3 skew(const Vector3<T>& v) noexcept {
            return { 0,    -v[2],  v[1],
                    v[2],  0,    -v[0],
                    -v[1],  v[0],  0};
        }
    };

    // Free functions
    template <std::floating_point T>
    constexpr Matrix3<T> operator*(T s, const Matrix3<T>& m) noexcept {
        return m * s;
    }
    template <std::floating_point T>
    std::ostream& operator<<(std::ostream& os, const Matrix3<T>& m) {
        for (std::size_t r = 0; r < 3; ++r) {
            os << "| " << m(r, 0) << "  " << m(r, 1) << "  " << m(r, 2) << " |\n";
        }
        return os;
    }
    // Aliases 
    using Mat3d = Matrix3<double>;
    using Mat3f = Matrix3<float>;

} // namespace sim::math