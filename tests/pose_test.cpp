// Poses as Eigen isometries and the controller's units: the rotation convention
// measured on the EA66 (R = Rz Ry Rx), the round trips, and the T_ab notation.

#include "ec/pose.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <numbers>

#include "ec/units.hpp"

namespace {

constexpr double kPi = std::numbers::pi;

Eigen::Matrix3d rx(double a) { return Eigen::AngleAxisd(a, Eigen::Vector3d::UnitX()).toRotationMatrix(); }
Eigen::Matrix3d ry(double a) { return Eigen::AngleAxisd(a, Eigen::Vector3d::UnitY()).toRotationMatrix(); }
Eigen::Matrix3d rz(double a) { return Eigen::AngleAxisd(a, Eigen::Vector3d::UnitZ()).toRotationMatrix(); }

TEST(Pose, TheControllersAnglesComposeAsRzRyRx) {
    EXPECT_TRUE(ec::rotation_from_rxyz(0.3, -0.2, 1.1).isApprox(rz(1.1) * ry(-0.2) * rx(0.3), 1e-15));
    // The tool pointing down, turned a quarter: rx = pi, rz = pi/2.
    Eigen::Matrix3d down;
    down << 0, 1, 0, 1, 0, 0, 0, 0, -1;
    EXPECT_TRUE(ec::rotation_from_rxyz(kPi, 0.0, kPi / 2.0).isApprox(down, 1e-15));
}

TEST(Pose, AnglesComeBackFromTheirRotation) {
    for (double a : {-3.0, -1.2, 0.0, 0.4, 2.9})
        for (double b : {-1.5, -0.7, 0.0, 0.3, 1.5})
            for (double c : {-3.1, -0.5, 0.0, 1.0, 3.1}) {
                const Eigen::Vector3d r = ec::rxyz_from_rotation(ec::rotation_from_rxyz(a, b, c));
                EXPECT_NEAR(r.x(), a, 1e-9);
                EXPECT_NEAR(r.y(), b, 1e-9);
                EXPECT_NEAR(r.z(), c, 1e-9);
            }
    // Gimbal lock: the angles are not unique, the rotation is.
    const Eigen::Matrix3d locked = ec::rotation_from_rxyz(0.4, kPi / 2.0, 1.0);
    const Eigen::Vector3d r = ec::rxyz_from_rotation(locked);
    EXPECT_EQ(r.x(), 0.0);
    EXPECT_TRUE(ec::rotation_from_rxyz(r.x(), r.y(), r.z()).isApprox(locked, 1e-9));
}

TEST(Pose, TheControllersUnitsAreMillimetresAndRadians) {
    const std::array<double, 6> wire{400.0, -50.0, 300.0, 3.0, 0.1, -0.2};
    const Eigen::Isometry3d T_base_tcp = ec::from_mm_rad(wire);
    EXPECT_TRUE(T_base_tcp.translation().isApprox(Eigen::Vector3d(0.4, -0.05, 0.3), 1e-15));
    EXPECT_TRUE(T_base_tcp.linear().isApprox(ec::rotation_from_rxyz(3.0, 0.1, -0.2), 1e-15));
    const auto back = ec::to_mm_rad(T_base_tcp);
    for (std::size_t i = 0; i < 6; ++i) EXPECT_NEAR(back[i], wire[i], 1e-9) << i;
    const Eigen::Isometry3d same = ec::isometry({0.4, -0.05, 0.3}, {3.0, 0.1, -0.2});
    EXPECT_TRUE(same.isApprox(T_base_tcp, 1e-15));
}

TEST(Pose, TheNotationMapsCoordinatesFromBToA) {
    // T_ab: frame b at (1, 2, 3) in a, turned a quarter about a's z.
    Eigen::Isometry3d T_ab = Eigen::Isometry3d::Identity();
    T_ab.translation() = Eigen::Vector3d(1, 2, 3);
    T_ab.linear() = rz(kPi / 2.0);
    const Eigen::Vector3d p_b(1, 0, 0);                                   // one metre along b's x
    EXPECT_TRUE((T_ab * p_b).isApprox(Eigen::Vector3d(1, 3, 3), 1e-15));  // p_a: along a's y from b's origin
    const Eigen::Quaterniond q_ab(T_ab.linear());
    EXPECT_TRUE((q_ab * p_b).isApprox(Eigen::Vector3d(0, 1, 0), 1e-15));  // v_a = q_ab * v_b
    // Chaining, as the controller's poseMul(p1, p2) = T1 * T2 (ADR 0012).
    const Eigen::Isometry3d T_bc = ec::from_mm_rad({0.0, 0.0, 100.0, 0.0, 0.0, 0.0});
    EXPECT_TRUE((T_ab * T_bc).translation().isApprox(Eigen::Vector3d(1, 2, 3.1), 1e-15));
    EXPECT_TRUE((T_ab.inverse() * T_ab).isApprox(Eigen::Isometry3d::Identity(), 1e-15));
}

}  // namespace
