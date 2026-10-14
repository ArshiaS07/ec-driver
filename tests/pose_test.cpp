// Poses as Eigen isometries and the controller's units: the rotation convention
// measured on the EA66 (R = Rz Ry Rx), the round trips, and the T_ab notation;
// and stamped poses, a PoseTrajectory.

#include "ec/pose.hpp"

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cmath>
#include <numbers>
#include <string>

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

TEST(Pose, AngleBetweenOrientationsIsExactForSmallAngles) {
    const Eigen::Matrix3d a = ec::rotation_from_rxyz(0.1, 0.2, 0.3);
    EXPECT_NEAR(ec::angle_between(a, a * rx(1e-7)), 1e-7, 1e-15);
    EXPECT_NEAR(ec::angle_between(a, a * ry(0.5)), 0.5, 1e-12);
    EXPECT_NEAR(ec::angle_between(a, a * rz(kPi)), kPi, 1e-9);
}

// ------------------------------------------------------------ PoseTrajectory

Eigen::Isometry3d pose(double x, double y, double z, const Eigen::Matrix3d& r = Eigen::Matrix3d::Identity()) {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() = Eigen::Vector3d(x, y, z);
    T.linear() = r;
    return T;
}

TEST(PoseTrajectory, InterpolatesInAStraightLineTurningEvenly) {
    const Eigen::Isometry3d a = pose(0.1, 0.2, 0.3, rx(0.2));
    const Eigen::Isometry3d b = pose(0.3, 0.2, 0.1, rx(0.2) * rz(0.6));
    EXPECT_TRUE(ec::interpolate(a, b, 0.0).isApprox(a, 1e-15));
    EXPECT_TRUE(ec::interpolate(a, b, 1.0).isApprox(b, 1e-15));
    const Eigen::Isometry3d q = ec::interpolate(a, b, 0.25);
    EXPECT_TRUE(q.translation().isApprox(Eigen::Vector3d(0.15, 0.2, 0.25), 1e-15));
    EXPECT_TRUE(q.linear().isApprox(rx(0.2) * rz(0.15), 1e-12));  // a quarter of the turn, about the same axis
    // The shorter way round: from +170 to -170 deg about z is 20 deg, through 180.
    const Eigen::Isometry3d c = pose(0, 0, 0, rz(ec::deg_to_rad(170.0)));
    const Eigen::Isometry3d d = pose(0, 0, 0, rz(ec::deg_to_rad(-170.0)));
    EXPECT_TRUE(ec::interpolate(c, d, 0.5).linear().isApprox(rz(kPi), 1e-12));
}

TEST(PoseTrajectory, PoseAtIsInterpolatedBetweenItsStampsAndHeldOutside) {
    using namespace std::chrono_literals;
    const ec::PoseTrajectory t{{0s, pose(0, 0, 0)}, {2s, pose(0.02, 0, 0)}, {3s, pose(0.02, 0.01, 0, rz(0.4))}};
    EXPECT_TRUE(validate(t));
    EXPECT_TRUE(ec::pose_at(t, -1s).isApprox(t.front().value));
    EXPECT_TRUE(ec::pose_at(t, 500ms).translation().isApprox(Eigen::Vector3d(0.005, 0, 0), 1e-15));
    EXPECT_TRUE(ec::pose_at(t, 2s).isApprox(t[1].value));
    const Eigen::Isometry3d later = ec::pose_at(t, 2750ms);
    EXPECT_TRUE(later.translation().isApprox(Eigen::Vector3d(0.02, 0.0075, 0), 1e-15));
    EXPECT_TRUE(later.linear().isApprox(rz(0.3), 1e-12));
    EXPECT_TRUE(ec::pose_at(t, 1h).isApprox(t.back().value));
    EXPECT_TRUE(ec::pose_at(ec::PoseTrajectory{}, 1s).isApprox(Eigen::Isometry3d::Identity()));
}

TEST(PoseTrajectory, ValidateSaysWhatIsWrong) {
    using namespace std::chrono_literals;
    const auto why = [](const ec::PoseTrajectory& t) {
        const auto ok = validate(t);
        EXPECT_FALSE(ok);
        return ok ? std::string{} : ok.error();
    };
    EXPECT_EQ(why({{0s, pose(0, 0, 0)}}), "1 poses; a trajectory needs at least 2");
    EXPECT_EQ(why({{1s, pose(0, 0, 0)}, {2s, pose(0, 0, 0)}}), "it starts at 1 s, not 0");
    EXPECT_EQ(why({{0s, pose(0, 0, 0)}, {0s, pose(0, 0, 0)}}), "pose 1: its stamp, 0 s, does not increase");
    Eigen::Isometry3d scaled = pose(0, 0, 0);
    scaled.linear() *= 1.01;
    EXPECT_EQ(why({{0s, pose(0, 0, 0)}, {1s, scaled}}),
              "pose 1: not a rigid transform (a finite position and a rotation)");
    EXPECT_FALSE(ec::validate({{0s, pose(0, 0, 0, rx(0.1) * -1.0)}, {1s, pose(0, 0, 0)}}));  // a reflection
    EXPECT_FALSE(ec::validate({{0s, pose(0, std::nan(""), 0)}, {1s, pose(0, 0, 0)}}));
}

}  // namespace
