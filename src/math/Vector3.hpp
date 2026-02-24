#pragma once

// =============================================================================
// Standard Library Includes
// =============================================================================
// <array>       : std::array — fixed-size container with value semantics.
//                 Unlike raw C arrays (T arr[3]), std::array knows its size,
//                 supports iterators, and can be passed by value. It also
//                 enables "structured bindings" in C++17:
//                     auto [x, y, z] = vec.data;
//
// <cmath>       : std::sqrt, std::abs - the C++ wrappers around the C math
//                 library. We use <cmath> instead of <math.h> because the C++
//                 version places functions in namespace std and provides
//                 overloads for float/double/long double automatically.
//
// <concepts>    : C++20 feature. Provides std::floating_point, std::same_as,
//                 and other "concepts" - named constraints on template
//                 parameters.
//
// <iostream>    : For the operator<< overload (stream output / printing).
//
// <stdexcept>   : std::runtime_error - used in normalized() when the vector
//                 is near-zero and normalization would produce garbage.
//
// <type_traits> : std::numeric_limits — gives us machine epsilon and other
//                 properties of floating-point types.
// =============================================================================
#include <array>
#include <cmath>
#include <concepts>
#include <iostream>
#include <stdexcept>
#include <type_traits>

namespace sim::math
{
    /**
     * @brief 3D Vector class templated on scalar type.
     *
     * Supports arithmetic operations, dot/cross products, and
     * coordinate access by index or named component (x,y,z).
     *
     * Design notes:
     *   - Stored as std::array for contiguous memory and structured bindings.
     *   - constexpr throughout so vectors can be used in compile-time contexts.
     *   - operator[] is bounds-checked in debug builds via assert().
     */
    template <std::floating_point T = double>
    class Vector3
    {
    public:
        // Data
        std::array<T, 3> data{};

        // Constructors (default tells the compiler: "generate the default constructor for me")
        constexpr Vector3() noexcept = default;

        /* T is guaranteed by std::floating_point to be a primitive,
         thus taking parameters by const reference is not needed
         primitives are cheap to copy, so just pass by value*/
        constexpr Vector3(T x, T y, T z) noexcept : data{x, y, z} {}

        // Converting constructor allows us to construct a Vector3<double> from a Vector3<float>,
        // or vice-versa, but ONLY explicitly
        template <std::floating_point U>
            requires(!std::same_as<T, U>)
        constexpr explicit Vector3(const Vector3<U> &other) noexcept : data{static_cast<T>(other[0]),
                                                                            static_cast<T>(other[1]),
                                                                            static_cast<T>(other[2])} {}

        /*======================================================================== 
        Named acccessors x(), y(), z():
        These provide readable access: vec.x() instead of vec.data[0].
    
        --- CONST OVERLOADING ---
        We provide TWO versions of each accessor:
    
        T  x() const   — returns a COPY of the value. Called on const vectors.
        T& x()         — returns a REFERENCE to the value. Called on non-const
                            vectors. Allows modification: vec.x() = 5.0;
    
        The compiler chooses the correct overload based on whether the Vector3
        object is const or not:
    
            const Vec3d a{1, 2, 3};
            double val = a.x();       // calls the const version, returns copy
    
            Vec3d b{1, 2, 3};
            b.x() = 10.0;            // calls the non-const version, returns ref
        ========================================================================*/
        [[nodiscard]] constexpr T  x() const noexcept { return data[0]; }
        [[nodiscard]] constexpr T  y() const noexcept { return data[1]; }
        [[nodiscard]] constexpr T  z() const noexcept { return data[2]; }
        [[nodiscard]] constexpr T& x()       noexcept { return data[0]; }
        [[nodiscard]] constexpr T& y()       noexcept { return data[1]; }
        [[nodiscard]] constexpr T& z()       noexcept { return data[2]; }  

        /*======================================================================== 
        Index Access: operator[]
        Allows array-style access: vec[0], vec[1], vec[2].
        --- std::size_t ---
        This is an unsigned integer type guaranteed to be large enough to
        represent the size of any object in memory. It's the standard type for
        array indices and sizes. Using int instead would trigger a
        -Wsign-conversion warning (signed/unsigned mismatch).
        --- data.at(i) vs data[i] ---
        We use .at(i) which performs BOUNDS CHECKING — it throws
        std::out_of_range if i >= 3. The alternative data[i] does NO bounds
        checking and causes undefined behavior on out-of-range access.
                The tradeoff: .at() has a tiny runtime cost (one comparison + branch).
        For a simulation that does millions of vector operations, you might
        switch to data[i] in release builds for performance. But during
        development, the bounds checking catches real bugs.
        ========================================================================*/
        [[nodiscard]] constexpr T  operator[](std::size_t i) const { return data.at(i); }
        [[nodiscard]] constexpr T& operator[](std::size_t i)       { return data.at(i); }

        // Arithmetic Operators (vector-vector)
        [[nodiscard]] constexpr Vector3 operator+(const Vector3& rhs) const noexcept {
            return {data[0] + rhs[0], data[1] + rhs[1], data[2] + rhs[2]};
        }
        [[nodiscard]] constexpr Vector3 operator-(const Vector3& rhs) const noexcept {
            return {data[0] - rhs[0], data[1] - rhs[1], data[2] - rhs[2]};
        }
        [[nodiscard]] constexpr Vector3& operator+=(const Vector3& rhs) noexcept {
            data[0] += rhs[0]; data[1] += rhs[1]; data[2] += rhs[2];
            return *this;
        }
        [[nodiscard]] constexpr Vector3& operator-=(const Vector3& rhs) noexcept {
            data[0] -= rhs[0]; data[1] -= rhs[1]; data[2] -= rhs[2];
            return *this;
        }

        // Arithmetic Operators (scalar)
        // Note: these only handle vector * scalar because the lhs vector is *this*
        // For scalar * vector, we need a free function outside of the class, see below.
        [[nodiscard]] constexpr Vector3 operator*(T s) const noexcept {
            return {data[0] * s, data[1] * s, data[2] * s};
        }
        [[nodiscard]] constexpr Vector3 operator/(T s) const {
            return {data[0] / s, data[1] / s, data[2] / s};
        }
        [[nodiscard]] constexpr Vector3& operator*=(T s) noexcept {
            data[0] *= s; data[1] *= s; data[2] *= s;
            return *this;
        }
        [[nodiscard]] constexpr Vector3& operator/=(T s) {
            data[0] /= s; data[1] /= s; data[2] /= s;
            return *this;
        }

        // Returns a new vector with all components negated: -vec.
        //
        // This is a UNARY operator (one operand), distinct from binary subtraction
        // (two operands). The compiler distinguishes them by the number of
        // parameters: operator-() with no params = unary, operator-(rhs) = binary.
        // ─────────────────────────────────────────────────────────────────────────
        [[nodiscard]] constexpr Vector3 operator-() const noexcept {
            return {-data[0], -data[1], -data[2]};
        }

        // Dot product and cross product
        [[nodiscard]] constexpr T dot(const Vector3& rhs) const noexcept {
            return data[0] * rhs[0] + data[1] * rhs[1] + data[2] * rhs[2];
        }
        [[nodiscard]] constexpr Vector3 cross(const Vector3& rhs) const noexcept {
            return {
                data[1] * rhs[2] - data[2] * rhs[1],   // x = ay*bz - az*by
                data[2] * rhs[0] - data[0] * rhs[2],   // y = az*bx - ax*bz
                data[0] * rhs[1] - data[1] * rhs[0]    // z = ax*by - ay*bx
            };
        }

        // Magnitude
        [[nodiscard]] constexpr T magnitude_squared() const noexcept {
            return dot(*this);
        }

        [[nodiscard]] T magnitude() const noexcept {
            return std::sqrt(magnitude_squared());
        }

        // Normalization
        [[nodiscard]] Vector3 normalized() const {
            T mag = magnitude();
            if (mag < std::numeric_limits<T>::epsilon()) {
                throw std::runtime_error("Cannot normalize a near-zero vector");
            }
            return *this / mag;
        }

        // Approximate equality
        // Floating-point numbers should almost NEVER be compared with ==.
        // Due to rounding errors, 0.1 + 0.2 != 0.3 in IEEE 754 floating point.
        // Instead, we check if the difference is smaller than some epsilon.
        [[nodiscard]] constexpr bool aapprox_equal(
            const Vector3& other, 
            T epsilon = static_cast<T>(1e-12)) const noexcept 
        {
            return std::abs(data[0] - other[0]) < epsilon &&
               std::abs(data[1] - other[1]) < epsilon &&
               std::abs(data[2] - other[2]) < epsilon;
        }

        // Static Factory Methods
        // These are STATIC member functions — they belong to the class itself,
        // not to any particular instance. You call them on the type:
        //     Vec3d v = Vec3d::zero();
        //     Vec3d x = Vec3d::unit_x();
        [[nodiscard]] static constexpr Vector3 zero()   noexcept { return {0, 0, 0}; }
        [[nodiscard]] static constexpr Vector3 unit_x() noexcept { return {1, 0, 0}; }
        [[nodiscard]] static constexpr Vector3 unit_y() noexcept { return {0, 1, 0}; }
        [[nodiscard]] static constexpr Vector3 unit_z() noexcept { return {0, 0, 1}; }
    };

    /* =============================================================================
       Free Functions (outside the class)
       =============================================================================
      
       --- Why do we need a free function for scalar * vector? ---
      
       Member operators always have *this as the left operand:
           vec * 2.0   → vec.operator*(2.0)       (defined above as a member)
           2.0 * vec   → (2.0).operator*(vec)     (double has no such method!)
      
       To support "2.0 * vec" (scalar on the LEFT), we need a NON-MEMBER function
       where the scalar is the first parameter. This is a FREE FUNCTION (also
       called a "non-member operator overload").
      
       It simply delegates to the member version: v * s, which we already defined.
       This keeps scalar multiplication commutative: (s * v) == (v * s).
      
       --- Why template it separately? ---
       This free function is templated on T independently. Even though it's in the
       same namespace as Vector3, it's not a member of Vector3, so it needs its
       own template parameter list. The compiler deduces T from the arguments.
       =============================================================================*/
    template <std::floating_point T>
    constexpr Vector3<T> operator*(T s, const Vector3<T>& v) noexcept {
        return v * s;
    }

    // Stream Output Operator
    // Allows:  std::cout << vec << std::endl;
    // Output:  [1.5, -2.3, 0.7]
    template <std::floating_point T>
    std::ostream& operator<<(std::ostream& os, const Vector3<T>& v) {
        return os << "[" << v[0] << ", " << v[1] << ", " << v[2] << "]";
    }

    /* =============================================================================
    // Type Aliases
    // =============================================================================
    //
    // "using Vec3d = Vector3<double>;" creates a TYPE ALIAS — a new name for an
    // existing type. After this, Vec3d and Vector3<double> are interchangeable.
    //
    // This is the modern C++11 syntax. The older C++ way is:
    //     typedef Vector3<double> Vec3d;
    // Both do the same thing, but "using" is preferred because:
    //   1. It reads left-to-right (name = type), which is more natural.
    //   2. It works with templates (typedef cannot create template aliases).
    //
    // Aliases reduce verbosity throughout the codebase. Instead of writing
    // Vector3<double> everywhere, we write Vec3d — shorter and conventional.
    // The simulation will almost exclusively use Vec3d (double precision),
    // but Vec3f exists for cases where memory matters more than precision
    // (e.g., visualization data, GPU transfer).
       =============================================================================*/
    using Vec3d = Vector3<double>;
    using Vec3f = Vector3<float>;
} // namespace sim::math