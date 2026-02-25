#include <gtest/gtest.h>
#include <memory>

#include "models/FlatEarthGravity.hpp"

using namespace sim::models;
using namespace sim::math;

TEST(FlatEarthGravity, DefaultIsStandardG) {
    FlatEarthGravity grav;
    auto g = grav.compute(Vec3d::zero());
    EXPECT_DOUBLE_EQ(g.x(), 0.0);
    EXPECT_DOUBLE_EQ(g.y(), 0.0);
    EXPECT_DOUBLE_EQ(g.z(), 9.80665);
}

TEST(FlatEarthGravity, CustomMagnitude) {
    FlatEarthGravity grav(3.711);  // Mars surface gravity
    auto g = grav.compute(Vec3d{1000, 2000, -500});
    EXPECT_DOUBLE_EQ(g.x(), 0.0);
    EXPECT_DOUBLE_EQ(g.y(), 0.0);
    EXPECT_DOUBLE_EQ(g.z(), 3.711);
}

TEST(FlatEarthGravity, IndependentOfPosition) {
    FlatEarthGravity grav;
    auto g1 = grav.compute(Vec3d{0, 0, 0});
    auto g2 = grav.compute(Vec3d{50000, 30000, -10000});
    EXPECT_TRUE(g1.approx_equal(g2));
}

TEST(FlatEarthGravity, WorksThroughInterface) {
    std::unique_ptr<IGravity> iface = std::make_unique<FlatEarthGravity>();
    auto g = iface->compute(Vec3d::zero());
    EXPECT_DOUBLE_EQ(g.z(), 9.80665);
}