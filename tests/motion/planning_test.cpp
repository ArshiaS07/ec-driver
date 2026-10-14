// Planned motion in joint space: the joint spline, the checks of a point and a
// step, and plan(), which solves a PoseTrajectory by inverse kinematics into a
// checked joint trajectory. The inverse kinematics here is a stand-in whose
// joints are the pose's own numbers, offset away from the singularities, so that
// the joint path can be checked against the poses.

#include "ec/motion/planning.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <numbers>
#include <string>
#include <vector>

#include "ec/pose.hpp"

namespace {

using namespace std::chrono_literals;
using ec::Joints;
using ec::PoseTrajectory;
using ec::motion::Error;
using ec::motion::JointSpline;

constexpr double kPi = std::numbers::pi;

// The stand-in arm: J1..J3 = x, y, z + 0.5 [m as rad], J4..J6 = rx, ry + 1, rz:
// J3 and J5 stay well away from 0 and pi for the poses used here.
Joints joints_of(const Eigen::Isometry3d& T) {
    const Eigen::Vector3d p = T.translation();
    const Eigen::Vector3d r = ec::rxyz_from_rotation(T.linear());
    return {p.x(), p.y(), p.z() + 0.5, r.x(), r.y() + 1.0, r.z()};
}
Eigen::Vector3d position_of(const Joints& q) { return {q[0], q[1], q[2] - 0.5}; }

// The stand-in inverse kinematics; records the references it was given.
struct StandInIk {
    std::vector<Joints>* references = nullptr;
    std::expected<Joints, Error> operator()(const Eigen::Isometry3d& T, const Joints& near) const {
        if (references != nullptr) references->push_back(near);
        return joints_of(T);
    }
};

// A pose at (x, y, z) [m], not turned.
ec::Stamped<Eigen::Isometry3d, ec::Clock::duration> at(ec::Clock::duration t, double x, double y, double z) {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() = Eigen::Vector3d(x, y, z);
    return {t, T};
}

Eigen::Vector3d center() { return {0.5, 0.0, 0.3}; }
constexpr double kRadius = 0.1;

// A circle of radius 0.1 m about (0.5, 0, 0.3), level, from +x counter-clockwise
// seen from above, once round in 15 s from rest to rest (the angle a
// smootherstep of the time), a pose every 20 ms, the tool's orientation held.
PoseTrajectory circle() {
    const Eigen::Matrix3d R = Eigen::AngleAxisd(2.0, Eigen::Vector3d::UnitX()).toRotationMatrix();
    PoseTrajectory motion;
    for (int k = 0; k <= 750; ++k) {
        const double s = k / 750.0;
        const double angle = 2.0 * kPi * s * s * s * (10.0 - 15.0 * s + 6.0 * s * s);
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.linear() = R;
        T.translation() = center() + kRadius * Eigen::Vector3d(std::cos(angle), std::sin(angle), 0.0);
        motion.push_back({k * 20ms, T});
    }
    return motion;
}

// ---------------------------------------------------------------- JointSpline

TEST(JointSpline, RefusesBadKnots) {
    EXPECT_FALSE(JointSpline::make({0.0}, {Joints{}}));
    EXPECT_FALSE(JointSpline::make({0.0, 1.0}, {Joints{}}));
    EXPECT_FALSE(JointSpline::make({0.0, 0.0}, {Joints{}, Joints{}}));
    EXPECT_FALSE(JointSpline::make({0.0, std::nan("")}, {Joints{}, Joints{}}));
}

TEST(JointSpline, RestEndsHaveZeroSlope) {
    const std::vector<double> x{0.0, 0.15, 0.3, 0.45};
    const std::vector<Joints> q{Joints{0.0}, Joints{0.21}, Joints{1.49}, Joints{3.0}};  // a ramp from rest
    const auto parabola = JointSpline::make(x, q);
    const auto rest = JointSpline::make(x, q, JointSpline::Ends::Rest);
    ASSERT_TRUE(parabola);
    ASSERT_TRUE(rest);
    EXPECT_LT(parabola->slope(1e-9)[0], 0.0);  // Bessel's end slope here points backwards
    EXPECT_NEAR(rest->slope(1e-9)[0], 0.0, 1e-6);
    EXPECT_NEAR(rest->slope(0.45 - 1e-9)[0], 0.0, 1e-6);
    for (int k = 1; k <= 45; ++k)  // forwards only: no step backwards from rest, none past the end
        EXPECT_GE(rest->position(0.01 * k)[0], rest->position(0.01 * (k - 1))[0]) << k;
}

TEST(JointSpline, ReproducesQuadraticsExactly) {
    const std::vector<double> x{0.0, 0.3, 0.45, 1.0, 1.2, 2.0};
    const auto f = [](double v) {
        Joints q{};
        for (std::size_t j = 0; j < q.size(); ++j) {
            const auto k = static_cast<double>(j);
            q[j] = (k + 1.0) * v * v - k * v + 0.5;
        }
        return q;
    };
    std::vector<Joints> q;
    q.reserve(x.size());
    for (double v : x) q.push_back(f(v));
    const auto spline = JointSpline::make(x, q);
    ASSERT_TRUE(spline);
    for (int k = 0; k < 146; ++k) {
        const double v = 0.001 + 0.0137 * k;
        for (std::size_t j = 0; j < 6; ++j) EXPECT_NEAR(spline->position(v)[j], f(v)[j], 1e-12) << v;
    }
}

TEST(JointSpline, APauseRestsWithoutOvershooting) {
    // Up to 1, held there from x = 2 to 3 (the same positions twice), then on to 3.
    const std::vector<double> x{0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
    const std::vector<Joints> q{Joints{0.0, 0.5}, Joints{0.5, 0.5}, Joints{1.0, 0.5},
                                Joints{1.0, 0.5}, Joints{2.0, 0.5}, Joints{3.0, 0.5}};
    const auto spline = JointSpline::make(x, q, JointSpline::Ends::Rest);
    ASSERT_TRUE(spline);
    EXPECT_EQ(spline->slope(2.0)[0], 0.0);
    for (int k = 0; k <= 100; ++k) {
        const double v = 0.05 * k;
        const double p = spline->position(v)[0];
        if (v >= 2.0 && v <= 3.0) {
            EXPECT_NEAR(p, 1.0, 1e-15) << v;  // held still
        } else if (v < 2.0) {
            EXPECT_LE(p, 1.0 + 1e-15) << v;  // not past it on the way
        } else {
            EXPECT_GE(p, 1.0 - 1e-15) << v;  // nor back after
        }
        EXPECT_NEAR(spline->position(v)[1], 0.5, 1e-15);
    }
    // Without the pause the joint passes x = 2 moving.
    const auto through =
        JointSpline::make({0.0, 1.0, 2.0, 4.0, 5.0}, {q[0], q[1], q[2], q[4], q[5]}, JointSpline::Ends::Rest);
    ASSERT_TRUE(through);
    EXPECT_GT(through->slope(2.0)[0], 0.3);
}

// ---------------------------------------------------------------------- checks

TEST(Checks, AStepMayNotJumpNorAPointComeCloseToASingularity) {
    const ec::motion::PlanOptions o;
    const Joints good{0.1, -1.5, 1.5, -1.5, -1.5, 0.0};
    EXPECT_TRUE(ec::motion::check_point(good, o));
    Joints next = good;
    next[3] += ec::deg_to_rad(9.0);
    EXPECT_TRUE(ec::motion::check_step(good, next, o));
    next[3] += ec::deg_to_rad(2.0);  // 11 deg in one step
    EXPECT_EQ(ec::motion::check_step(good, next, o).error(),
              "joint 4 turns 11.0 deg in one step: the arm would change configuration, or pass close to a singularity");
    Joints wrist = good;
    wrist[4] = ec::deg_to_rad(-178.0);
    EXPECT_NE(ec::motion::check_point(wrist, o).error().find("J5 at -178.0 deg: within 5.0 deg of the wrist"),
              std::string::npos);
    Joints elbow = good;
    elbow[2] = ec::deg_to_rad(3.0);
    EXPECT_NE(ec::motion::check_step(good, elbow, {.max_joint_step = 2.0}).error().find("elbow singularity"),
              std::string::npos);
    Joints beyond = good;
    beyond[0] = ec::deg_to_rad(361.0);
    EXPECT_EQ(ec::motion::check_point(beyond, o).error(), "joint 1 at 361.0 deg: beyond the joint limit");
}

// ---------------------------------------------------------------------- plan

TEST(Plan, SolvesEveryPointNearTheOneBeforeAndSamplesAtThePeriod) {
    const PoseTrajectory poses = circle();
    std::vector<Joints> references;
    const Joints start{0.1, 0.2, 0.3, 0.4, 1.5, 0.6};
    const auto t = ec::motion::plan(poses, start, StandInIk{&references});
    ASSERT_TRUE(t) << describe(t.error());
    ASSERT_EQ(references.size(), poses.size());  // poses at most 3 mm apart: none filled in
    EXPECT_EQ(references.front(), start);
    EXPECT_EQ(references[1], joints_of(poses.front().value));

    EXPECT_EQ(t->time.front(), 0.0);
    EXPECT_NEAR(t->time.back(), 15.0, 1e-12);
    for (std::size_t k = 1; k + 1 < t->size(); ++k) EXPECT_NEAR(t->time[k] - t->time[k - 1], 0.02, 1e-12);
    EXPECT_TRUE(validate(*t));
    for (const Joints& q : t->position)  // between the solved points the joints stay on the circle
        EXPECT_NEAR((position_of(q) - center()).norm(), kRadius, 2e-6);
}

TEST(Plan, StartsAndEndsAtRestWithoutStepsBackwards) {
    const auto t = ec::motion::plan(circle(), Joints{}, StandInIk{});
    ASSERT_TRUE(t) << describe(t.error());
    // The circle starts along +y (counter-clockwise from +x): y must not dip first.
    for (std::size_t k = 1; k < t->size() && t->time[k] < 1.0; ++k)
        EXPECT_GE(t->position[k][1], t->position[k - 1][1] - 1e-12) << "t = " << t->time[k];
    const std::size_t n = t->size();
    const double first = ec::max_abs_diff(t->position[1], t->position[0]) / (t->time[1] - t->time[0]);
    const double last = ec::max_abs_diff(t->position[n - 1], t->position[n - 2]) / (t->time[n - 1] - t->time[n - 2]);
    EXPECT_LT(first, 1e-3);  // rad/s: barely moving in the first and last 20 ms
    EXPECT_LT(last, 1e-3);
}

TEST(Plan, FillsInSparsePointsAlongStraightLinesAndEvenTurns) {
    std::vector<Joints> references;
    auto b = at(3s, 0.5, 0.03, 0.3);  // 30 mm along y
    b.value.linear() = Eigen::AngleAxisd(ec::deg_to_rad(4.0), Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const PoseTrajectory motion{at(0s, 0.5, 0.0, 0.3), b};
    const auto t = ec::motion::plan(motion, Joints{}, StandInIk{&references}, {.max_step = 0.003});
    ASSERT_TRUE(t) << describe(t.error());
    EXPECT_EQ(references.size(), 11u);  // 10 pieces of 3 mm (and of 0.4 deg)
    for (const Joints& q : t->position) {
        EXPECT_NEAR(q[0], 0.5, 1e-9);  // on the line
        EXPECT_NEAR(q[2], 0.8, 1e-9);
    }
}

TEST(Plan, PassesThroughEveryPoseAndRestsOnlyAtAPause) {
    // 20 mm along y in 2 s, then 20 more: the arm passes the middle pose moving.
    std::vector<Joints> references;
    const PoseTrajectory through{at(0s, 0.5, 0.0, 0.3), at(2s, 0.5, 0.02, 0.3), at(4s, 0.5, 0.04, 0.3)};
    const auto t = ec::motion::plan(through, Joints{}, StandInIk{});
    ASSERT_TRUE(t) << describe(t.error());
    EXPECT_NEAR(ec::motion::position_at(*t, 2.0)[1], 0.02, 1e-6);
    EXPECT_GT(ec::motion::position_at(*t, 2.02)[1] - ec::motion::position_at(*t, 1.98)[1], 0.0003);  // 10 mm/s

    // The middle pose stamped twice: the arm stops there for a second, without overshooting.
    const PoseTrajectory paused{at(0s, 0.5, 0.0, 0.3), at(2s, 0.5, 0.02, 0.3), at(3s, 0.5, 0.02, 0.3),
                                at(5s, 0.5, 0.04, 0.3)};
    const auto p = ec::motion::plan(paused, Joints{}, StandInIk{&references});
    ASSERT_TRUE(p) << describe(p.error());
    for (std::size_t k = 0; k < p->size(); ++k) {
        const double y = p->position[k][1];
        if (p->time[k] < 2.0) {
            EXPECT_LE(y, 0.02 + 1e-12) << p->time[k];  // not past the pause on the way
        } else if (p->time[k] <= 3.0) {
            EXPECT_NEAR(y, 0.02, 1e-15) << p->time[k];  // held still
        }
    }
    EXPECT_EQ(references.size(), 15u);  // the first, 7 more to each pose 20 mm on; the pause's not solved again
}

TEST(Plan, RefusesMotionsItCannotTrust) {
    const auto refused = [](const PoseTrajectory& motion, const ec::motion::InverseKinematics& ik,
                            const ec::motion::PlanOptions& o = {}) {
        const auto t = ec::motion::plan(motion, Joints{}, ik, o);
        EXPECT_FALSE(t);
        return t ? std::string{} : t.error().message;
    };
    const StandInIk ik;
    EXPECT_EQ(refused({}, ik), "not a motion: 0 poses; a trajectory needs at least 2");
    EXPECT_EQ(refused({at(1s, 0.5, 0, 0.3), at(2s, 0.5, 0, 0.3)}, ik), "not a motion: it starts at 1 s, not 0");
    EXPECT_NE(refused({at(0s, 0.5, 0, 0.3), at(0s, 0.5, 0, 0.3)}, ik).find("does not increase"), std::string::npos);
    auto skewed = at(1s, 0.5, 0, 0.3);
    skewed.value.linear() *= 2.0;
    EXPECT_NE(refused({at(0s, 0.5, 0, 0.3), skewed}, ik).find("not a rigid transform"), std::string::npos);
    // Too fast for the joints: 0.5 m in 0.1 s is 5 rad/s on the stand-in's J2.
    EXPECT_NE(refused({at(0s, 0.5, 0, 0.3), at(100ms, 0.5, 0.5, 0.3)}, ik).find("would move at"), std::string::npos);
    // Too many points.
    EXPECT_NE(refused({at(0s, 0.5, 0, 0.3), at(10s, 0.5, 50.0, 0.3)}, ik).find("more than"), std::string::npos);
}

TEST(Plan, NamesThePoseWhoseInverseKinematicsFailedAndKeepsTheKind) {
    const auto unreachable = [](const Eigen::Isometry3d&, const Joints&) -> std::expected<Joints, Error> {
        return std::unexpected(Error{Error::Kind::Command, "inverseKinematic: rejected: no solution", {}});
    };
    const auto t = ec::motion::plan(circle(), Joints{}, unreachable);
    ASSERT_FALSE(t);
    EXPECT_EQ(t.error().kind, Error::Kind::Command);  // the controller's answer, not a judgement of the motion
    EXPECT_EQ(t.error().message,
              "inverse kinematics failed for the pose at 0.000 s (600.0, 0.0, 300.0 mm): inverseKinematic: rejected: "
              "no solution");
    const auto lost = [](const Eigen::Isometry3d&, const Joints&) -> std::expected<Joints, Error> {
        return std::unexpected(Error{Error::Kind::Disconnected, "the command connection was lost", {}});
    };
    EXPECT_EQ(ec::motion::plan(circle(), Joints{}, lost).error().kind, Error::Kind::Disconnected);
}

TEST(Plan, RefusesAJumpAndTheWristAndElbowSingularities) {
    // Half way round, the stand-in changes configuration: J4 turns a quarter turn.
    const auto flipping = [](const Eigen::Isometry3d& T, const Joints&) -> std::expected<Joints, Error> {
        Joints q = joints_of(T);
        if (T.translation().y() < 0.0) q[3] += kPi / 2.0;
        return q;
    };
    const auto jump = ec::motion::plan(circle(), Joints{}, flipping);
    ASSERT_FALSE(jump);
    EXPECT_NE(jump.error().message.find("joint 4 turns 90.0 deg in one step"), std::string::npos)
        << jump.error().message;
    EXPECT_EQ(jump.error().message.rfind("the pose at 7.", 0), 0u) << jump.error().message;  // half way round

    const auto wrist = [](const Eigen::Isometry3d& T, const Joints&) -> std::expected<Joints, Error> {
        Joints q = joints_of(T);
        q[4] = ec::deg_to_rad(183.0);  // 3 deg from 180
        return q;
    };
    const auto w = ec::motion::plan(circle(), Joints{}, wrist);
    ASSERT_FALSE(w);
    EXPECT_NE(w.error().message.find("wrist singularity"), std::string::npos) << w.error().message;
    EXPECT_TRUE(ec::motion::plan(circle(), Joints{}, wrist, {.singularity_margin = ec::deg_to_rad(2.0)}));

    const auto elbow = [](const Eigen::Isometry3d& T, const Joints&) -> std::expected<Joints, Error> {
        Joints q = joints_of(T);
        q[2] = ec::deg_to_rad(-1.0);
        return q;
    };
    const auto e = ec::motion::plan(circle(), Joints{}, elbow);
    ASSERT_FALSE(e);
    EXPECT_NE(e.error().message.find("elbow singularity"), std::string::npos) << e.error().message;
}

TEST(Plan, TakesSolutionsGivenWithinOneTurnToTheNearestTurn) {
    // J6 turns with the circle, from -90 deg, but the solver reports it within (-3/2 pi, pi/2].
    const auto wrapping = [](const Eigen::Isometry3d& T, const Joints&) -> std::expected<Joints, Error> {
        Joints q = joints_of(T);
        q[5] = std::atan2(T.translation().y(), T.translation().x() - 0.5) - kPi / 2.0;
        return q;
    };
    const auto t = ec::motion::plan(circle(), Joints{}, wrapping);
    ASSERT_TRUE(t) << describe(t.error());
    EXPECT_NEAR(t->position.back()[5], 1.5 * kPi, 1e-9);  // all the way round, not back to -90 deg
}

TEST(Plan, SingularityMarginsReportTheClosestApproach) {
    const std::vector<Joints> q{Joints{0, 0, 1.0, 0, 2.0, 0}, Joints{0, 0, 3.0, 0, -0.2, 0}};
    const auto m = ec::motion::singularity_margins(q);
    EXPECT_NEAR(m.wrist, 0.2, 1e-12);        // J5 at -0.2 rad: 0.2 from 0
    EXPECT_NEAR(m.elbow, kPi - 3.0, 1e-12);  // J3 at 3 rad: 0.14 from pi
}

TEST(Trajectory, PeakRatesAndInterpolation) {
    ec::motion::JointTrajectory t;
    t.time = {0.0, 1.0, 2.0, 4.0};
    t.position = {Joints{0.0}, Joints{1.0}, Joints{1.0}, Joints{-1.0}};
    EXPECT_NEAR(ec::motion::peak_velocity(t)[0], 1.0, 1e-15);
    EXPECT_NEAR(ec::motion::peak_acceleration(t)[0], 1.0, 1e-15);
    EXPECT_NEAR(ec::motion::position_at(t, 0.5)[0], 0.5, 1e-15);
    EXPECT_NEAR(ec::motion::position_at(t, 3.0)[0], 0.0, 1e-15);
}

}  // namespace
