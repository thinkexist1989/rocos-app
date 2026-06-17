#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include <test/doctest.h>
#include <rocos_app/motion/cartesian_geometry.h>

#include <kdl/frames.hpp>
#include <cmath>

using namespace rocos::motion::cartesian;

TEST_CASE("computePathMetrics: pure translation") {
    KDL::Frame start(KDL::Rotation::Identity(), KDL::Vector(1, 0, 0));
    KDL::Frame goal(KDL::Rotation::Identity(), KDL::Vector(4, 0, 0));
    auto metrics = computePathMetrics(start, goal);
    CHECK(metrics.path_length == doctest::Approx(3.0).epsilon(1e-6));
    CHECK(metrics.translation_distance == doctest::Approx(3.0).epsilon(1e-6));
}

TEST_CASE("computePathMetrics: pure rotation") {
    KDL::Frame start;
    KDL::Frame goal;
    goal.M = KDL::Rotation::RotZ(M_PI / 2);
    auto metrics = computePathMetrics(start, goal);
    CHECK(metrics.translation_distance == doctest::Approx(0.0).epsilon(1e-6));
    CHECK(metrics.rotation_angle == doctest::Approx(M_PI / 2).epsilon(1e-6));
    CHECK(metrics.path_length == doctest::Approx(M_PI / 2 * kEquivalentRadius).epsilon(1e-6));
}

TEST_CASE("computePathMetrics: zero path") {
    KDL::Frame start;
    auto metrics = computePathMetrics(start, start);
    CHECK(metrics.path_length == doctest::Approx(0.0).epsilon(1e-10));
}

TEST_CASE("interpolateLinear: s=0 returns start") {
    KDL::Frame start(KDL::Rotation::Identity(), KDL::Vector(1, 2, 3));
    KDL::Frame goal(KDL::Rotation::Identity(), KDL::Vector(4, 5, 6));
    auto result = interpolateLinear(start, goal, 0.0);
    CHECK(result.p.x() == doctest::Approx(1.0));
    CHECK(result.p.y() == doctest::Approx(2.0));
    CHECK(result.p.z() == doctest::Approx(3.0));
}

TEST_CASE("interpolateLinear: s=1 returns goal") {
    KDL::Frame start(KDL::Rotation::Identity(), KDL::Vector(1, 2, 3));
    KDL::Frame goal(KDL::Rotation::Identity(), KDL::Vector(4, 5, 6));
    auto result = interpolateLinear(start, goal, 1.0);
    CHECK(result.p.x() == doctest::Approx(4.0));
    CHECK(result.p.y() == doctest::Approx(5.0));
    CHECK(result.p.z() == doctest::Approx(6.0));
}

TEST_CASE("interpolateLinear: s=0.5 midpoint") {
    KDL::Frame start(KDL::Rotation::Identity(), KDL::Vector(0, 0, 0));
    KDL::Frame goal(KDL::Rotation::Identity(), KDL::Vector(2, 4, 6));
    auto result = interpolateLinear(start, goal, 0.5);
    CHECK(result.p.x() == doctest::Approx(1.0));
    CHECK(result.p.y() == doctest::Approx(2.0));
    CHECK(result.p.z() == doctest::Approx(3.0));
}

TEST_CASE("computeNormalizedLimits: path_length=0") {
    auto limits = computeNormalizedLimits(0.0, 1.0, 1.0);
    CHECK(limits.max_velocity == doctest::Approx(1.0));
    CHECK(limits.max_acceleration == doctest::Approx(1.0));
    CHECK(limits.max_jerk == doctest::Approx(1.0));
}

TEST_CASE("computeNormalizedLimits: path_length=2, max_vel=1") {
    auto limits = computeNormalizedLimits(2.0, 1.0, 1.0);
    CHECK(limits.max_velocity == doctest::Approx(0.5).epsilon(1e-6));
    CHECK(limits.max_acceleration == doctest::Approx(0.5).epsilon(1e-6));
}

TEST_CASE("computeCircleCenter: three coplanar points") {
    double r = 2.0;
    KDL::Frame p1(KDL::Rotation::Identity(), KDL::Vector(r, 0, 0));
    KDL::Frame p2(KDL::Rotation::Identity(), KDL::Vector(0, r, 0));
    KDL::Frame p3(KDL::Rotation::Identity(), KDL::Vector(-r, 0, 0));
    KDL::Frame center;
    REQUIRE(computeCircleCenter(center, p1, p2, p3));
    CHECK(center.p.x() == doctest::Approx(0.0).epsilon(1e-6));
    CHECK(center.p.y() == doctest::Approx(0.0).epsilon(1e-6));
    CHECK(center.p.z() == doctest::Approx(0.0).epsilon(1e-6));
}

TEST_CASE("rotationAngle: 90 degrees Z") {
    auto R1 = KDL::Rotation::Identity();
    auto R2 = KDL::Rotation::RotZ(M_PI / 2);
    CHECK(rotationAngle(R1, R2) == doctest::Approx(M_PI / 2).epsilon(1e-6));
}

TEST_CASE("rotationAngle: zero rotation") {
    auto R1 = KDL::Rotation::Identity();
    CHECK(rotationAngle(R1, R1) == doctest::Approx(0.0).epsilon(1e-10));
}
