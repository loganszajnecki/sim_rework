#include <gtest/gtest.h>
#include <cmath>

#include "math/Vector3.hpp"
#include "math/Matrix3.hpp"
#include "math/Quaternion.hpp"
#include "math/Rotations.hpp"

using namespace sim::math;

// ═══════════════════════════════════════════════════════════════
//  Quaternion basics
// ═══════════════════════════════════════════════════════════════

TEST(Quaternion, DefaultIsIdentity) {
    Quatd q;
    EXPECT_DOUBLE_EQ(q.w, 1.0);
    EXPECT_DOUBLE_EQ(q.x, 0.0);
    EXPECT_DOUBLE_EQ(q.y, 0.0);
    EXPECT_DOUBLE_EQ(q.z, 0.0);
}

TEST(Quaternion, NormSquared) {
    Quatd q{1,2,3,4};
    EXPECT_DOUBLE_EQ(q.norm_squared(), 30.0);
}

TEST(Quaternion, Normalization) {
    auto qn = Quatd{1,2,3,4}.normalized();
    EXPECT_NEAR(qn.norm(), 1.0, 1e-14);
}

TEST(Quaternion, UnitInverseEqualsConjugate) {
    auto q = Quatd::from_axis_angle(Vec3d{1,1,1}, 0.7);
    EXPECT_TRUE(q.inverse().approx_equal(q.conjugate(), 1e-12));
}

TEST(Quaternion, TimesInverseIsIdentity) {
    auto q = Quatd::from_axis_angle(Vec3d{0,0,1}, 1.2);
    auto product = q * q.inverse();
    EXPECT_TRUE(product.approx_equal(Quatd::identity(), 1e-12));
}

TEST(Quaternion, QAndNegQSameRotation) {
    auto q = Quatd::from_axis_angle(Vec3d{1,0,0}, 0.8);
    EXPECT_TRUE(q.same_rotation(-q));
    Vec3d v{1,2,3};
    EXPECT_TRUE(q.rotate(v).approx_equal((-q).rotate(v), 1e-12));
}

// ═══════════════════════════════════════════════════════════════
//  Vector rotation
// ═══════════════════════════════════════════════════════════════

TEST(Quaternion, IdentityDoesNotRotate) {
    Vec3d v{1,2,3};
    EXPECT_TRUE(Quatd::identity().rotate(v).approx_equal(v));
}

TEST(Quaternion, NinetyDegreesAboutZMapsXtoY) {
    auto q = Quatd::from_axis_angle(Vec3d{0,0,1}, M_PI/2.0);
    EXPECT_TRUE(q.rotate(Vec3d{1,0,0}).approx_equal(Vec3d{0,1,0}, 1e-12));
}

// ═══════════════════════════════════════════════════════════════
//  Euler <-> Quaternion
// ═══════════════════════════════════════════════════════════════

TEST(Convert, ZeroEulerToIdentityQuat) {
    auto q = convert::euler_to_quaternion(EulerAnglesd{0,0,0});
    EXPECT_TRUE(q.approx_equal(Quatd::identity(), 1e-14));
}

TEST(Convert, EulerQuatRoundTrip) {
    auto test = [](double phi, double theta, double psi) {
        EulerAnglesd e{phi, theta, psi};
        auto q = convert::euler_to_quaternion(e);
        auto e2 = convert::quaternion_to_euler(q);
        EXPECT_NEAR(e2.phi,   e.phi,   1e-12);
        EXPECT_NEAR(e2.theta, e.theta, 1e-12);
        EXPECT_NEAR(e2.psi,   e.psi,   1e-12);
    };
    test(0.3, 0.5, 1.0);
    test(-0.7, 0.2, -0.4);
    test(0.1, -0.3, 2.0);
}

// ═══════════════════════════════════════════════════════════════
//  Quaternion DCM <-> Euler DCM
// ═══════════════════════════════════════════════════════════════

TEST(Convert, QuatDCMMatchesEulerDCM) {
    EulerAnglesd e{0.3, -0.5, 1.2};
    auto q = convert::euler_to_quaternion(e);
    EXPECT_TRUE(dcm::inertial_to_body(e).approx_equal(
                dcm::inertial_to_body(q), 1e-12));
}

TEST(Convert, QuatDCMIsOrthogonal) {
    auto q = Quatd::from_axis_angle(Vec3d{1,1,0}, 0.9);
    auto L = dcm::inertial_to_body(q);
    EXPECT_TRUE((L * L.transposed()).approx_equal(Mat3d::identity(), 1e-12));
    EXPECT_NEAR(L.determinant(), 1.0, 1e-12);
}

TEST(Convert, DCMQuatRoundTrip) {
    EulerAnglesd e{0.6, -0.3, 1.8};
    auto dcm_orig = dcm::inertial_to_body(e);
    auto q = convert::dcm_to_quaternion(dcm_orig);
    auto dcm_rebuilt = dcm::inertial_to_body(q);
    EXPECT_TRUE(dcm_orig.approx_equal(dcm_rebuilt, 1e-12));
}

// ═══════════════════════════════════════════════════════════════
//  q.rotate() vs DCM multiplication
// ═══════════════════════════════════════════════════════════════

TEST(Convert, RotateMatchesBodyToInertialDCM) {
    EulerAnglesd e{0.5, -0.3, 1.1};
    auto q = convert::euler_to_quaternion(e);
    Vec3d v{10, -5, 3};
    EXPECT_TRUE(q.rotate(v).approx_equal(dcm::body_to_inertial(q) * v, 1e-10));
}

TEST(Convert, ConjugateRotateMatchesInertialToBodyDCM) {
    EulerAnglesd e{0.5, -0.3, 1.1};
    auto q = convert::euler_to_quaternion(e);
    Vec3d v{10, -5, 3};
    EXPECT_TRUE(q.conjugate().rotate(v).approx_equal(
                dcm::inertial_to_body(q) * v, 1e-10));
}

// ═══════════════════════════════════════════════════════════════
//  Quaternion kinematic derivative
// ═══════════════════════════════════════════════════════════════

TEST(Convert, QdotZeroWhenOmegaZero) {
    auto q = Quatd::from_axis_angle(Vec3d{1,0,0}, 0.5);
    auto qdot = convert::quaternion_derivative(q, Vec3d::zero());
    EXPECT_NEAR(qdot.w, 0, 1e-14);
    EXPECT_NEAR(qdot.x, 0, 1e-14);
    EXPECT_NEAR(qdot.y, 0, 1e-14);
    EXPECT_NEAR(qdot.z, 0, 1e-14);
}

TEST(Convert, SmallStepPreservesUnitNorm) {
    auto q = Quatd::from_axis_angle(Vec3d{0,0,1}, 0.3);
    auto qdot = convert::quaternion_derivative(q, Vec3d{0.1, 0.2, 0.5});
    Quatd q_new = q + qdot * 0.001;
    EXPECT_NEAR(q_new.norm(), 1.0, 1e-5);
}

// ═══════════════════════════════════════════════════════════════
//  Slerp
// ═══════════════════════════════════════════════════════════════

TEST(Quaternion, SlerpEndpoints) {
    auto q0 = Quatd::identity();
    auto q1 = Quatd::from_axis_angle(Vec3d{0,0,1}, M_PI/2.0);
    EXPECT_TRUE(slerp(q0, q1, 0.0).same_rotation(q0, 1e-12));
    EXPECT_TRUE(slerp(q0, q1, 1.0).same_rotation(q1, 1e-12));
}

TEST(Quaternion, SlerpMidpoint) {
    auto q0 = Quatd::identity();
    auto q1 = Quatd::from_axis_angle(Vec3d{0,0,1}, M_PI/2.0);
    auto expected = Quatd::from_axis_angle(Vec3d{0,0,1}, M_PI/4.0);
    EXPECT_TRUE(slerp(q0, q1, 0.5).same_rotation(expected, 1e-10));
}