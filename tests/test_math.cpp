#include <gtest/gtest.h>
#include <cmath>

#include "math/Vector3.hpp"
#include "math/Matrix3.hpp"
#include "math/Rotations.hpp"

using namespace sim::math;

// ═══════════════════════════════════════════════════════════════
//  Vector3
// ═══════════════════════════════════════════════════════════════

TEST(Vector3, DefaultConstructionIsZero) {
    Vec3d v;
    EXPECT_DOUBLE_EQ(v.x(), 0.0);
    EXPECT_DOUBLE_EQ(v.y(), 0.0);
    EXPECT_DOUBLE_EQ(v.z(), 0.0);
}

TEST(Vector3, Addition) {
    Vec3d a{1, 2, 3}, b{4, 5, 6};
    auto c = a + b;
    EXPECT_DOUBLE_EQ(c.x(), 5.0);
    EXPECT_DOUBLE_EQ(c.y(), 7.0);
    EXPECT_DOUBLE_EQ(c.z(), 9.0);
}

TEST(Vector3, ScalarMultiplyCommutative) {
    Vec3d a{1, 2, 3};
    auto c = a * 2.0;
    auto d = 2.0 * a;
    EXPECT_TRUE(c.approx_equal(d));
}

TEST(Vector3, DotProduct) {
    EXPECT_DOUBLE_EQ(Vec3d::unit_x().dot(Vec3d::unit_y()), 0.0);
    Vec3d c{1,2,3}, d{4,5,6};
    EXPECT_DOUBLE_EQ(c.dot(d), 32.0);
}

TEST(Vector3, CrossProduct) {
    auto z = Vec3d::unit_x().cross(Vec3d::unit_y());
    EXPECT_TRUE(z.approx_equal(Vec3d::unit_z()));
}

TEST(Vector3, Magnitude) {
    Vec3d v{3, 4, 0};
    EXPECT_DOUBLE_EQ(v.magnitude(), 5.0);
    auto n = v.normalized();
    EXPECT_NEAR(n.magnitude(), 1.0, 1e-14);
}

// ═══════════════════════════════════════════════════════════════
//  Matrix3
// ═══════════════════════════════════════════════════════════════

TEST(Matrix3, IdentityMultiply) {
    auto I = Mat3d::identity();
    Vec3d v{1, 2, 3};
    EXPECT_TRUE((I * v).approx_equal(v));
}

TEST(Matrix3, Transpose) {
    Mat3d m{1,2,3, 4,5,6, 7,8,9};
    auto mt = m.transposed();
    EXPECT_DOUBLE_EQ(mt(0,1), 4.0);
    EXPECT_DOUBLE_EQ(mt(1,0), 2.0);
}

TEST(Matrix3, Determinant) {
    EXPECT_DOUBLE_EQ(Mat3d::identity().determinant(), 1.0);
    Mat3d m{1,2,3, 0,1,4, 5,6,0};
    EXPECT_NEAR(m.determinant(), 1.0, 1e-12);
}

TEST(Matrix3, Inverse) {
    Mat3d m{1,2,3, 0,1,4, 5,6,0};
    auto product = m * m.inverse();
    EXPECT_TRUE(product.approx_equal(Mat3d::identity(), 1e-10));
}

TEST(Matrix3, SkewProducesCrossProduct) {
    Vec3d a{1,2,3}, b{4,5,6};
    auto result = Mat3d::skew(a) * b;
    EXPECT_TRUE(result.approx_equal(a.cross(b)));
}

// ═══════════════════════════════════════════════════════════════
//  Rotations / DCM
// ═══════════════════════════════════════════════════════════════

TEST(DCM, ZeroEulerIsIdentity) {
    auto L = dcm::body_to_inertial(EulerAnglesd{0,0,0});
    EXPECT_TRUE(L.approx_equal(Mat3d::identity(), 1e-14));
}

TEST(DCM, IsOrthogonal) {
    EulerAnglesd e{0.3, -0.5, 1.2};
    auto L = dcm::inertial_to_body(e);
    EXPECT_TRUE((L * L.transposed()).approx_equal(Mat3d::identity(), 1e-12));
}

TEST(DCM, DeterminantIsOne) {
    EulerAnglesd e{0.7, 0.2, -0.9};
    auto L = dcm::inertial_to_body(e);
    EXPECT_NEAR(L.determinant(), 1.0, 1e-12);
}

TEST(DCM, BodyToInertialIsTransposeOfInertialToBody) {
    EulerAnglesd e{0.3, 0.6, -1.1};
    auto L_bi = dcm::body_to_inertial(e);
    auto L_ib = dcm::inertial_to_body(e);
    EXPECT_TRUE(L_bi.approx_equal(L_ib.transposed(), 1e-12));
}

TEST(DCM, EulerKinematicMatrixAtZero) {
    auto H = dcm::euler_kinematic_matrix(EulerAnglesd{0,0,0});
    EXPECT_TRUE(H.approx_equal(Mat3d::identity(), 1e-14));
}