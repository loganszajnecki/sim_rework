#include <gtest/gtest.h>
#include <cmath>
#include <memory>

#include "models/USStandard1976.hpp"

using namespace sim::models;

// ==============================================================
//  US Standard Atmosphere 1976 tests.
//
//  Reference values from the published NOAA/NASA/USAF tables.
//
//  Note on tolerances: the published tables are indexed by
//  geopotential altitude, but our model takes geometric altitude
//  as input (what the sim actually has). The conversion between
//  the two grows with altitude (~0.3% at 20 km, ~0.5% at 30 km),
//  so we use percentage-based tolerances for pressure and density.
// ==============================================================

class AtmosphereTest : public ::testing::Test {
protected:
    USStandard1976 atmo;

    // Helper: check within a relative tolerance (percentage/100)
    void expect_rel(double actual, double expected, double rel_tol,
                    const char* name, double alt_m)
    {
        double tol = std::abs(expected) * rel_tol;
        EXPECT_NEAR(actual, expected, tol)
            << name << " at " << alt_m << " m";
    }
};

// Sea level (exact match expected)

TEST_F(AtmosphereTest, SeaLevel) {
    auto s = atmo.compute(0.0);
    EXPECT_DOUBLE_EQ(s.temperature, 288.15);
    EXPECT_DOUBLE_EQ(s.pressure, 101325.0);
    EXPECT_NEAR(s.density,        1.2250, 0.0001);
    EXPECT_NEAR(s.speed_of_sound, 340.29, 0.1);
}

// Troposphere (0 - 11 km)

TEST_F(AtmosphereTest, Tropo_1km) {
    auto s = atmo.compute(1000.0);
    EXPECT_NEAR(s.temperature, 281.65,   0.1);
    expect_rel(s.pressure, 89874.6,   0.002, "P", 1000);
    expect_rel(s.density,  1.11164,   0.002, "rho", 1000);
}

TEST_F(AtmosphereTest, Tropo_5km) {
    auto s = atmo.compute(5000.0);
    EXPECT_NEAR(s.temperature, 255.65,   0.2);
    expect_rel(s.pressure, 54019.9,   0.005, "P", 5000);
    expect_rel(s.density,  0.736116,  0.005, "rho", 5000);
}

TEST_F(AtmosphereTest, Tropo_10km) {
    auto s = atmo.compute(10000.0);
    EXPECT_NEAR(s.temperature, 223.15,   0.5);
    expect_rel(s.pressure, 26436.3,   0.005, "P", 10000);
    expect_rel(s.density,  0.412707,  0.005, "rho", 10000);
}

// Tropopause (11 - 20 km, isothermal)

TEST_F(AtmosphereTest, Tropopause_12km) {
    auto s = atmo.compute(12000.0);
    EXPECT_NEAR(s.temperature, 216.65,   0.5);
    expect_rel(s.pressure, 19330.4,   0.005, "P", 12000);
    expect_rel(s.density,  0.310828,  0.005, "rho", 12000);
}

TEST_F(AtmosphereTest, Tropopause_20km) {
    auto s = atmo.compute(20000.0);
    EXPECT_NEAR(s.temperature, 216.65,   0.5);
    expect_rel(s.pressure, 5474.89,   0.015, "P", 20000);
    expect_rel(s.density,  0.0880349, 0.015, "rho", 20000);
}

// Stratosphere (20 - 32 km)

TEST_F(AtmosphereTest, Strato_25km) {
    auto s = atmo.compute(25000.0);
    EXPECT_NEAR(s.temperature, 221.65,   0.5);
    expect_rel(s.pressure, 2511.02,   0.02, "P", 25000);
    expect_rel(s.density,  0.0394658, 0.02, "rho", 25000);
}

TEST_F(AtmosphereTest, Strato_30km) {
    auto s = atmo.compute(30000.0);
    EXPECT_NEAR(s.temperature, 226.65,   0.5);
    expect_rel(s.pressure, 1171.87,   0.025, "P", 30000);
    expect_rel(s.density,  0.0180119, 0.025, "rho", 30000);
}

// Speed of sound

TEST_F(AtmosphereTest, SpeedOfSound_SeaLevel) {
    auto s = atmo.compute(0.0);
    EXPECT_NEAR(s.speed_of_sound, 340.29, 0.1);
}

TEST_F(AtmosphereTest, SpeedOfSound_11km) {
    auto s = atmo.compute(11000.0);
    EXPECT_NEAR(s.speed_of_sound, 295.07, 0.5);
}

// Interface polymorphism

TEST_F(AtmosphereTest, WorksThroughSmartPointer) {
    std::unique_ptr<IAtmosphere> iface = std::make_unique<USStandard1976>();
    auto s = iface->compute(0.0);
    EXPECT_NEAR(s.temperature, 288.15, 0.01);
    EXPECT_NEAR(s.pressure, 101325.0, 1.0);
}

// Edge cases

TEST_F(AtmosphereTest, BelowSeaLevel) {
    auto s = atmo.compute(-1000.0);
    EXPECT_NEAR(s.temperature, 294.65,   0.2);
    expect_rel(s.pressure, 113929.0,  0.005, "P", -1000);
    expect_rel(s.density,  1.347,     0.005, "rho", -1000);
}

TEST_F(AtmosphereTest, ClampsAboveMaxAltitude) {
    auto s = atmo.compute(100000.0);
    EXPECT_GT(s.temperature, 0.0);
    EXPECT_GT(s.pressure, 0.0);
    EXPECT_GT(s.density, 0.0);
    EXPECT_GT(s.speed_of_sound, 0.0);
}

// Monotonicity

TEST_F(AtmosphereTest, PressureDecreasesWithAltitude) {
    double prev = atmo.compute(0.0).pressure;
    for (double h = 1000.0; h <= 80000.0; h += 1000.0) {
        double p = atmo.compute(h).pressure;
        EXPECT_LT(p, prev) << "Pressure must decrease at h=" << h;
        prev = p;
    }
}

TEST_F(AtmosphereTest, DensityDecreasesWithAltitude) {
    double prev = atmo.compute(0.0).density;
    for (double h = 1000.0; h <= 80000.0; h += 1000.0) {
        double rho = atmo.compute(h).density;
        EXPECT_LT(rho, prev) << "Density must decrease at h=" << h;
        prev = rho;
    }
}