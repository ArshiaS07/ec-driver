// Planned paths in joint space: the circle's geometry, the speed profile, the
// joint spline, sampling a path, and plan(), which solves timed poses by
// inverse kinematics into a checked joint trajectory. The inverse kinematics
// here is a stand-in whose joints are the pose's own numbers, offset away from
// the singularities, so that the joint path can be checked against the poses.

#include "ec/motion/planning.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>
#include <string>
#include <vector>

#include "ec/path.hpp"
#include "ec/pose.hpp"

namespace {

using ec::Circle;
using ec::Joints;
using ec::motion::Error;
using ec::motion::JointSpline;
using ec::motion::SpeedProfile;
using ec::motion::TimedPose;

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

Circle circle() {
    Circle c;
    c.center = Eigen::Vector3d(0.5, 0.0, 0.3);
    c.radius = 0.1;
    c.start = Eigen::Vector3d::UnitX();
    c.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(2.0, Eigen::Vector3d::UnitX()));
    return c;
}

TimedPose at(double t, double x, double y, double z) {
    TimedPose p{.time = t, .pose = Eigen::Isometry3d::Identity()};
    p.pose.translation() = Eigen::Vector3d(x, y, z);
    return p;
}

// ---------------------------------------------------------------------- Circle

TEST(Circle, RefusesWhatIsNotACircle) {
    EXPECT_TRUE(validate(circle()));
    auto c = circle();
    c.radius = 0.0;
    EXPECT_FALSE(validate(c));
    c = circle();
    c.normal = Eigen::Vector3d::Zero();
    EXPECT_FALSE(validate(c));
    c = circle();
    c.turns = 0.0;
    EXPECT_FALSE(validate(c));
    c = circle();
    c.center.y() = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(validate(c));
    c = circle();
    c.orientation.coeffs() *= 2.0;
    EXPECT_FALSE(validate(c));
}

TEST(Circle, StartsTowardItsStartAndTurnsCounterClockwiseAboutTheNormal) {
    const Circle c = circle();
    const double l = length(c);
    EXPECT_NEAR(l, 2.0 * kPi * 0.1, 1e-12);
    EXPECT_TRUE(pose_at(c, 0.0).translation().isApprox(Eigen::Vector3d(0.6, 0.0, 0.3), 1e-12));
    EXPECT_TRUE(pose_at(c, l / 4.0).translation().isApprox(Eigen::Vector3d(0.5, 0.1, 0.3), 1e-12));
    EXPECT_TRUE(pose_at(c, l / 2.0).translation().isApprox(Eigen::Vector3d(0.4, 0.0, 0.3), 1e-12));
    for (int k = 0; k <= 100; ++k) {
        const Eigen::Isometry3d p = pose_at(c, l * k / 100.0);
        EXPECT_NEAR((p.translation() - c.center).norm(), 0.1, 1e-12);
        EXPECT_NEAR(p.translation().z(), 0.3, 1e-12);
        EXPECT_TRUE(p.linear().isApprox(c.orientation.toRotationMatrix(), 1e-15));  // the tool keeps its orientation
    }
    auto clockwise = c;
    clockwise.normal = Eigen::Vector3d(0.0, 0.0, -2.0);  // any length
    EXPECT_TRUE(pose_at(clockwise, l / 4.0).translation().isApprox(Eigen::Vector3d(0.5, -0.1, 0.3), 1e-12));
}

TEST(Circle, ChoosesAStartDirectionInItsPlane) {
    auto c = circle();
    c.start = Eigen::Vector3d::Zero();
    EXPECT_TRUE(ec::axes(c).u.isApprox(Eigen::Vector3d::UnitX()));  // base x
    c.start = Eigen::Vector3d(3.0, 0.0, 5.0);                       // only the part in the plane counts
    EXPECT_TRUE(ec::axes(c).u.isApprox(Eigen::Vector3d::UnitX()));
    c.normal = Eigen::Vector3d::UnitX();  // x is the normal: base y
    c.start = Eigen::Vector3d::Zero();
    EXPECT_TRUE(ec::axes(c).u.isApprox(Eigen::Vector3d::UnitY()));
    EXPECT_TRUE(ec::axes(c).v.isApprox(Eigen::Vector3d::UnitZ()));  // n x u
}

// --------------------------------------------------------------- SpeedProfile

TEST(SpeedProfile, RefusesImpossibleNumbers) {
    EXPECT_FALSE(SpeedProfile::make(-1.0, 0.1, 0.5));
    EXPECT_FALSE(SpeedProfile::make(1.0, 0.0, 0.5));
    EXPECT_FALSE(SpeedProfile::make(1.0, 0.1, -0.5));
    EXPECT_FALSE(SpeedProfile::make(std::nan(""), 0.1, 0.5));
    EXPECT_TRUE(SpeedProfile::make(0.0, 0.1, 0.5));
}

TEST(SpeedProfile, RampsCruisesAndStopsAtTheEnd) {
    const auto p = SpeedProfile::make(0.2, 0.05, 0.5);
    ASSERT_TRUE(p);
    EXPECT_NEAR(p->duration(), 4.5, 1e-12);  // ramp + distance / speed
    EXPECT_EQ(p->distance(0.0), 0.0);
    EXPECT_EQ(p->distance(4.5), 0.2);
    EXPECT_NEAR(p->speed(2.0), 0.05, 1e-15);
    EXPECT_EQ(p->speed(0.0), 0.0);
    EXPECT_NEAR(p->peak_acceleration(), 0.05 * kPi / 1.0, 1e-12);
    constexpr double dt = 1e-4;
    double last = 0.0;
    for (int k = 1; k * dt < p->duration(); ++k) {
        const double t = k * dt;
        const double s = p->distance(t);
        EXPECT_GE(s, last - 1e-15);
        EXPECT_LE(p->speed(t), 0.05 + 1e-12);
        EXPECT_NEAR((s - last) / dt, p->speed(t - dt / 2.0), 1e-6) << "t = " << t;  // speed is ds/dt
        last = s;
    }
}

TEST(SpeedProfile, TimeAtIsTheInverseOfDistance) {
    const auto p = SpeedProfile::make(0.2, 0.05, 0.5);
    ASSERT_TRUE(p);
    for (double t : {0.1, 0.4, 1.0, 2.5, 4.2, 4.45}) EXPECT_NEAR(p->time_at(p->distance(t)), t, 1e-9) << t;
    EXPECT_EQ(p->time_at(-1.0), 0.0);
    EXPECT_EQ(p->time_at(1.0), p->duration());
}

TEST(SpeedProfile, AShortDistanceLowersTheSpeed) {
    const auto p = SpeedProfile::make(0.01, 0.05, 0.5);
    ASSERT_TRUE(p);
    EXPECT_NEAR(p->cruise_speed(), 0.02, 1e-15);
    EXPECT_NEAR(p->duration(), 1.0, 1e-12);
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

// -------------------------------------------------------------------- sample

TEST(Sample, AStepThatIsNotPositiveGivesNothing) {
    const auto profile = SpeedProfile::make(length(circle()), 0.05, 0.5);
    ASSERT_TRUE(profile);
    EXPECT_TRUE(ec::motion::sample(circle(), *profile, 0.0).empty());
    EXPECT_TRUE(ec::motion::sample(circle(), *profile, -1.0).empty());
    EXPECT_FALSE(ec::motion::plan(std::vector<TimedPose>{}, Joints{}, StandInIk{}));
}

TEST(Sample, PosesEveryStepAlongThePathAndEveryStepsTimeInTheRamps) {
    const Circle c = circle();
    const auto profile = SpeedProfile::make(length(c), 0.05, 0.5);
    ASSERT_TRUE(profile);
    const auto poses = ec::motion::sample(c, *profile, 0.003);
    // 2 pi 0.1 m / 3 mm = 209.4 steps, and some more in the two 0.5 s ramps.
    EXPECT_GT(poses.size(), 211u);
    EXPECT_LT(poses.size(), 230u);
    EXPECT_EQ(poses.front().time, 0.0);
    EXPECT_EQ(poses.back().time, profile->duration());
    EXPECT_TRUE(poses.back().pose.isApprox(pose_at(c, length(c)), 1e-12));
    for (std::size_t k = 1; k < poses.size(); ++k) {
        EXPECT_GT(poses[k].time, poses[k - 1].time);
        EXPECT_LE(poses[k].time - poses[k - 1].time, 0.003 / 0.05 + 1e-9);  // a step's time at cruise speed
        EXPECT_LE((poses[k].pose.translation() - poses[k - 1].pose.translation()).norm(), 0.003 + 1e-12);
    }
}

// ---------------------------------------------------------------------- plan

std::vector<TimedPose> timed_circle() {
    const Circle c = circle();
    const auto profile = SpeedProfile::make(length(c), 0.05, 0.5);
    return ec::motion::sample(c, *profile, 0.003);
}

TEST(Plan, SolvesEveryPointNearTheOneBeforeAndSamplesAtThePeriod) {
    const auto poses = timed_circle();
    std::vector<Joints> references;
    const Joints start{0.1, 0.2, 0.3, 0.4, 1.5, 0.6};
    const auto t = ec::motion::plan(poses, start, StandInIk{&references});
    ASSERT_TRUE(t) << describe(t.error());
    ASSERT_EQ(references.size(), poses.size());  // points at most 3 mm apart: none filled in
    EXPECT_EQ(references.front(), start);
    EXPECT_EQ(references[1], joints_of(poses.front().pose));

    EXPECT_EQ(t->time.front(), 0.0);
    EXPECT_NEAR(t->time.back(), poses.back().time, 1e-12);
    for (std::size_t k = 1; k + 1 < t->size(); ++k) EXPECT_NEAR(t->time[k] - t->time[k - 1], 0.02, 1e-12);
    EXPECT_TRUE(validate(*t));
    for (const Joints& q : t->position)  // between the solved points the joints stay on the circle
        EXPECT_NEAR((position_of(q) - circle().center).norm(), 0.1, 2e-6);
}

TEST(Plan, StartsAndEndsAtRestWithoutStepsBackwards) {
    const auto t = ec::motion::plan(timed_circle(), Joints{}, StandInIk{});
    ASSERT_TRUE(t) << describe(t.error());
    // The circle starts along +y (counter-clockwise from +x): y must not dip first.
    for (std::size_t k = 1; k < t->size() && t->time[k] < 0.5; ++k)
        EXPECT_GE(t->position[k][1], t->position[k - 1][1] - 1e-9) << "t = " << t->time[k];
    const std::size_t n = t->size();
    const double first = ec::max_abs_diff(t->position[1], t->position[0]) / (t->time[1] - t->time[0]);
    const double last = ec::max_abs_diff(t->position[n - 1], t->position[n - 2]) / (t->time[n - 1] - t->time[n - 2]);
    EXPECT_LT(first, 1e-3);  // rad/s: barely moving in the first and last 20 ms
    EXPECT_LT(last, 1e-3);
}

TEST(Plan, FillsInSparsePointsAlongStraightLinesAndEvenTurns) {
    std::vector<Joints> references;
    TimedPose a = at(0.0, 0.5, 0.0, 0.3);
    TimedPose b = at(3.0, 0.5, 0.03, 0.3);  // 30 mm along y
    b.pose.linear() = Eigen::AngleAxisd(ec::deg_to_rad(4.0), Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const std::vector path{a, b};
    const auto t = ec::motion::plan(path, Joints{}, StandInIk{&references}, {.max_step = 0.003});
    ASSERT_TRUE(t) << describe(t.error());
    EXPECT_EQ(references.size(), 11u);  // 10 pieces of 3 mm (and of 0.4 deg)
    for (const Joints& q : t->position) {
        EXPECT_NEAR(q[0], 0.5, 1e-9);  // on the line
        EXPECT_NEAR(q[2], 0.8, 1e-9);
    }
}

TEST(Plan, RefusesPathsItCannotTrust) {
    const auto refused = [](std::vector<TimedPose> path, const ec::motion::InverseKinematics& ik,
                            const ec::motion::PlanOptions& o = {}) {
        const auto t = ec::motion::plan(path, Joints{}, ik, o);
        EXPECT_FALSE(t);
        return t ? std::string{} : t.error().message;
    };
    const StandInIk ik;
    EXPECT_NE(refused({at(1.0, 0.5, 0, 0.3), at(2.0, 0.5, 0, 0.3)}, ik).find("starts at 1 s"), std::string::npos);
    EXPECT_NE(refused({at(0.0, 0.5, 0, 0.3), at(0.0, 0.5, 0, 0.3)}, ik).find("does not increase"), std::string::npos);
    auto skewed = at(1.0, 0.5, 0, 0.3);
    skewed.pose.linear() *= 2.0;
    EXPECT_NE(refused({at(0.0, 0.5, 0, 0.3), skewed}, ik).find("not a pose"), std::string::npos);
    // Too fast for the joints: 0.5 m in 0.1 s is 5 rad/s on the stand-in's J2.
    EXPECT_NE(refused({at(0.0, 0.5, 0, 0.3), at(0.1, 0.5, 0.5, 0.3)}, ik).find("would move at"), std::string::npos);
    // Too many points.
    EXPECT_NE(refused({at(0.0, 0.5, 0, 0.3), at(10.0, 0.5, 50.0, 0.3)}, ik).find("more than"), std::string::npos);
}

TEST(Plan, NamesThePoseWhoseInverseKinematicsFailedAndKeepsTheKind) {
    const auto unreachable = [](const Eigen::Isometry3d&, const Joints&) -> std::expected<Joints, Error> {
        return std::unexpected(Error{Error::Kind::Command, "inverseKinematic: rejected: no solution", {}});
    };
    const auto t = ec::motion::plan(timed_circle(), Joints{}, unreachable);
    ASSERT_FALSE(t);
    EXPECT_EQ(t.error().kind, Error::Kind::Command);  // the controller's answer, not a judgement of the path
    EXPECT_NE(t.error().message.find("inverse kinematics failed for the pose at 0.000 s (600.0, 0.0, 300.0 mm): "
                                     "inverseKinematic: rejected: no solution"),
              std::string::npos)
        << t.error().message;
    const auto lost = [](const Eigen::Isometry3d&, const Joints&) -> std::expected<Joints, Error> {
        return std::unexpected(Error{Error::Kind::Disconnected, "the command connection was lost", {}});
    };
    EXPECT_EQ(ec::motion::plan(timed_circle(), Joints{}, lost).error().kind, Error::Kind::Disconnected);
}

TEST(Plan, RefusesAJumpAndTheWristAndElbowSingularities) {
    // Half way round, the stand-in changes configuration: J4 turns half a turn.
    const auto flipping = [](const Eigen::Isometry3d& T, const Joints&) -> std::expected<Joints, Error> {
        Joints q = joints_of(T);
        if (T.translation().y() < 0.0) q[3] += kPi / 2.0;
        return q;
    };
    const auto jump = ec::motion::plan(timed_circle(), Joints{}, flipping);
    ASSERT_FALSE(jump);
    EXPECT_NE(jump.error().message.find("joint 4 turns 90.0 deg"), std::string::npos) << jump.error().message;

    const auto wrist = [](const Eigen::Isometry3d& T, const Joints&) -> std::expected<Joints, Error> {
        Joints q = joints_of(T);
        q[4] = ec::deg_to_rad(183.0);  // 3 deg from 180
        return q;
    };
    const auto w = ec::motion::plan(timed_circle(), Joints{}, wrist);
    ASSERT_FALSE(w);
    EXPECT_NE(w.error().message.find("wrist singularity"), std::string::npos) << w.error().message;
    EXPECT_TRUE(ec::motion::plan(timed_circle(), Joints{}, wrist, {.singularity_margin = ec::deg_to_rad(2.0)}));

    const auto elbow = [](const Eigen::Isometry3d& T, const Joints&) -> std::expected<Joints, Error> {
        Joints q = joints_of(T);
        q[2] = ec::deg_to_rad(-1.0);
        return q;
    };
    const auto e = ec::motion::plan(timed_circle(), Joints{}, elbow);
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
    const auto t = ec::motion::plan(timed_circle(), Joints{}, wrapping);
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
