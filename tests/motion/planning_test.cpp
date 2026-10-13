// Planning a Cartesian path into a joint trajectory: the circle's geometry, the
// speed profile, the joint spline, solving by inverse kinematics, and timing.
// The inverse kinematics here is a stand-in: joint j is the pose's j-th number,
// so the joint path is the pose path and the geometry can be checked exactly.

#include "ec/motion/planning.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>
#include <vector>

#include "ec/path.hpp"

namespace {

using ec::Circle;
using ec::Joints;
using ec::Pose;
using ec::Vec3;
using ec::motion::Error;
using ec::motion::JointSpline;
using ec::motion::SpeedProfile;
using ec::motion::TimedPath;

constexpr double kPi = std::numbers::pi;

void expect_near(const Vec3& a, const Vec3& b, double tolerance) {
    for (std::size_t i = 0; i < 3; ++i) EXPECT_NEAR(a[i], b[i], tolerance) << "component " << i;
}

// Joint j = pose number j (x, y, z [m], rx, ry, rz [rad]); records the references it was given.
struct IdentityIk {
    std::vector<Joints>* references = nullptr;
    std::expected<Joints, Error> operator()(const Pose& p, const Joints& reference) const {
        if (references != nullptr) references->push_back(reference);
        return Joints{p.position[0], p.position[1], p.position[2], p.rotation[0], p.rotation[1], p.rotation[2]};
    }
};

Vec3 position_of(const Joints& q) { return {q[0], q[1], q[2]}; }

const Circle kCircle{.center = {0.5, 0.0, 0.3},
                     .radius = 0.1,
                     .normal = {0.0, 0.0, 1.0},
                     .start = {1.0, 0.0, 0.0},
                     .rotation = {kPi, 0.0, 0.0},
                     .turns = 1.0};

// ---------------------------------------------------------------------- Circle

TEST(Circle, RefusesWhatIsNotACircle) {
    EXPECT_TRUE(validate(kCircle));
    auto c = kCircle;
    c.radius = 0.0;
    EXPECT_FALSE(validate(c));
    c.radius = std::nan("");
    EXPECT_FALSE(validate(c));
    c = kCircle;
    c.normal = {0.0, 0.0, 0.0};
    EXPECT_FALSE(validate(c));
    c = kCircle;
    c.turns = 0.0;
    EXPECT_FALSE(validate(c));
    c = kCircle;
    c.center[1] = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(validate(c));
}

TEST(Circle, StartsTowardItsStartAndTurnsCounterClockwiseAboutTheNormal) {
    const double l = length(kCircle);
    EXPECT_NEAR(l, 2.0 * kPi * 0.1, 1e-12);
    expect_near(pose_at(kCircle, 0.0).position, {0.6, 0.0, 0.3}, 1e-12);
    expect_near(pose_at(kCircle, l / 4.0).position, {0.5, 0.1, 0.3}, 1e-12);  // from +x toward +y
    expect_near(pose_at(kCircle, l / 2.0).position, {0.4, 0.0, 0.3}, 1e-12);
    expect_near(pose_at(kCircle, l).position, {0.6, 0.0, 0.3}, 1e-12);
    for (int k = 0; k <= 100; ++k) {
        const Pose p = pose_at(kCircle, l * k / 100.0);
        EXPECT_NEAR(ec::distance(p.position, kCircle.center), 0.1, 1e-12);
        EXPECT_NEAR(p.position[2], 0.3, 1e-12);
        EXPECT_EQ(p.rotation, kCircle.rotation);  // the tool keeps its orientation
    }

    auto clockwise = kCircle;
    clockwise.normal = {0.0, 0.0, -2.0};  // any length
    expect_near(pose_at(clockwise, l / 4.0).position, {0.5, -0.1, 0.3}, 1e-12);
}

TEST(Circle, ChoosesAStartDirectionInItsPlane) {
    auto c = kCircle;
    c.start = {};
    expect_near(ec::axes(c).u, {1.0, 0.0, 0.0}, 1e-12);  // base x
    c.start = {3.0, 0.0, 5.0};                           // only the part in the plane counts
    expect_near(ec::axes(c).u, {1.0, 0.0, 0.0}, 1e-12);
    c.start = {0.0, 0.0, 1.0};  // along the normal: as if none were given
    expect_near(ec::axes(c).u, {1.0, 0.0, 0.0}, 1e-12);

    c.normal = {1.0, 0.0, 0.0};  // x is the normal: base y
    c.start = {};
    expect_near(ec::axes(c).u, {0.0, 1.0, 0.0}, 1e-12);
    expect_near(ec::axes(c).v, {0.0, 0.0, 1.0}, 1e-12);  // n x u

    c.normal = {0.0, 1.0, 1.0};  // a tilted plane that contains x
    const auto a = ec::axes(c);
    expect_near(a.u, {1.0, 0.0, 0.0}, 1e-12);
    EXPECT_NEAR(ec::dot(a.v, a.n), 0.0, 1e-12);
    EXPECT_NEAR(ec::norm(a.v), 1.0, 1e-12);
}

TEST(Circle, PartialTurnsAndDistancesOutsideTheCircle) {
    auto half = kCircle;
    half.turns = 0.5;
    EXPECT_NEAR(length(half), kPi * 0.1, 1e-12);
    expect_near(pose_at(half, 10.0).position, {0.4, 0.0, 0.3}, 1e-12);  // clamped to the end
    expect_near(pose_at(half, -1.0).position, {0.6, 0.0, 0.3}, 1e-12);  // and to the start
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
    EXPECT_EQ(p->distance(-1.0), 0.0);
    EXPECT_EQ(p->distance(4.5), 0.2);
    EXPECT_EQ(p->distance(10.0), 0.2);
    EXPECT_NEAR(p->speed(2.0), 0.05, 1e-15);
    EXPECT_EQ(p->speed(0.0), 0.0);
    EXPECT_EQ(p->speed(4.5), 0.0);
    EXPECT_NEAR(p->peak_acceleration(), 0.05 * kPi / 1.0, 1e-12);

    constexpr double dt = 1e-4;
    double last = 0.0;
    for (int k = 1; k * dt < p->duration(); ++k) {
        const double t = k * dt;
        const double s = p->distance(t);
        EXPECT_GE(s, last - 1e-15);
        EXPECT_LE(p->speed(t), 0.05 + 1e-12);
        EXPECT_NEAR((s - last) / dt, p->speed(t - dt / 2.0), 1e-6) << "t = " << t;  // speed is ds/dt
        const double a = (p->speed(t) - p->speed(t - dt)) / dt;
        EXPECT_LE(std::abs(a), p->peak_acceleration() + 1e-6);
        last = s;
    }
}

TEST(SpeedProfile, AShortDistanceLowersTheSpeed) {
    const auto p = SpeedProfile::make(0.01, 0.05, 0.5);
    ASSERT_TRUE(p);
    EXPECT_NEAR(p->cruise_speed(), 0.02, 1e-15);
    EXPECT_NEAR(p->duration(), 1.0, 1e-12);
    EXPECT_NEAR(p->distance(0.5), 0.005, 1e-12);  // half way at half time
}

TEST(SpeedProfile, WithoutARampTheSpeedJumps) {
    const auto p = SpeedProfile::make(0.1, 0.05, 0.0);
    ASSERT_TRUE(p);
    EXPECT_NEAR(p->duration(), 2.0, 1e-12);
    EXPECT_NEAR(p->distance(1.0), 0.05, 1e-12);
    EXPECT_EQ(p->speed(1e-9), 0.05);
    EXPECT_TRUE(std::isinf(p->peak_acceleration()));
}

// ---------------------------------------------------------------- JointSpline

TEST(JointSpline, RefusesBadKnots) {
    EXPECT_FALSE(JointSpline::make({0.0}, {Joints{}}));
    EXPECT_FALSE(JointSpline::make({0.0, 1.0}, {Joints{}}));
    EXPECT_FALSE(JointSpline::make({0.0, 0.0}, {Joints{}, Joints{}}));
    EXPECT_FALSE(JointSpline::make({0.0, std::nan("")}, {Joints{}, Joints{}}));
    EXPECT_FALSE(JointSpline::make({0.0, 1.0}, {Joints{}, Joints{std::nan("")}}));
}

TEST(JointSpline, ReproducesQuadraticsExactly) {
    // Unevenly spaced knots; joint j follows (j + 1) x^2 - j x + 0.5.
    const std::vector<double> x{0.0, 0.3, 0.45, 1.0, 1.2, 2.0};
    const auto f = [](double v) {
        Joints q{};
        for (std::size_t j = 0; j < q.size(); ++j) {
            const auto k = static_cast<double>(j);
            q[j] = (k + 1.0) * v * v - k * v + 0.5;
        }
        return q;
    };
    const auto df = [](double v) {
        Joints q{};
        for (std::size_t j = 0; j < q.size(); ++j) {
            const auto k = static_cast<double>(j);
            q[j] = 2.0 * (k + 1.0) * v - k;
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
        const Joints got = spline->position(v);
        const Joints slope = spline->slope(v);
        for (std::size_t j = 0; j < got.size(); ++j) {
            EXPECT_NEAR(got[j], f(v)[j], 1e-12) << "x = " << v << ", joint " << j;
            EXPECT_NEAR(slope[j], df(v)[j], 1e-10) << "x = " << v << ", joint " << j;
        }
    }
}

TEST(JointSpline, PassesThroughItsKnotsAndHoldsOutsideThem) {
    const std::vector<double> x{0.0, 1.0, 3.0};
    const std::vector<Joints> q{Joints{0.0}, Joints{1.0}, Joints{-2.0}};
    const auto spline = JointSpline::make(x, q);
    ASSERT_TRUE(spline);
    for (std::size_t k = 0; k < x.size(); ++k) EXPECT_EQ(spline->position(x[k])[0], q[k][0]);
    EXPECT_EQ(spline->position(-5.0)[0], 0.0);
    EXPECT_EQ(spline->position(5.0)[0], -2.0);
    EXPECT_EQ(spline->slope(-5.0)[0], 0.0);
    EXPECT_EQ(spline->slope(5.0)[0], 0.0);
    // Slopes are continuous across a knot.
    EXPECT_NEAR(spline->slope(1.0 - 1e-9)[0], spline->slope(1.0 + 1e-9)[0], 1e-6);

    const auto line = JointSpline::make({0.0, 2.0}, {Joints{1.0}, Joints{3.0}});
    ASSERT_TRUE(line);
    EXPECT_NEAR(line->position(0.5)[0], 1.5, 1e-15);
    EXPECT_NEAR(line->slope(0.5)[0], 1.0, 1e-15);
}

// ---------------------------------------------------------------------- solve

TEST(Solve, SolvesTheCircleEveryFewMillimetresFromTheStart) {
    std::vector<Joints> references;
    const Joints start{0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
    const auto path = ec::motion::solve(kCircle, start, IdentityIk{&references}, {.spacing = 0.003});
    ASSERT_TRUE(path) << describe(path.error());
    // 2 pi 0.1 m / 3 mm = 209.4: 210 intervals, 211 points.
    ASSERT_EQ(path->poses.size(), 211u);
    EXPECT_EQ(path->joints.knots().front(), 0.0);
    EXPECT_EQ(path->joints.knots().back(), length(kCircle));
    EXPECT_EQ(path->length(), length(kCircle));
    ASSERT_EQ(references.size(), 211u);
    EXPECT_EQ(references.front(), start);                        // the first near the start
    EXPECT_EQ(references[1], path->joints.positions().front());  // then each near the one before

    // Between the solved points, the interpolated joints stay on the circle.
    for (int k = 0; k * 0.0007 <= path->length(); ++k) {
        const double s = k * 0.0007;
        const Vec3 p = position_of(path->joints.position(s));
        EXPECT_NEAR(ec::distance(p, kCircle.center), 0.1, 2e-6) << "s = " << s;
        EXPECT_NEAR(p[2], 0.3, 1e-12);
    }
}

TEST(Solve, TakesSolutionsGivenWithinOneTurnToTheNearestTurn) {
    // Joint 6 turns with the circle, but the solver reports it within (-pi, pi].
    const auto wrapping = [](const Pose& p, const Joints&) -> std::expected<Joints, Error> {
        const double angle = std::atan2(p.position[1] - kCircle.center[1], p.position[0] - kCircle.center[0]);
        return Joints{0.0, 0.0, 0.0, 0.0, 0.0, angle};
    };
    const auto path = ec::motion::solve(kCircle, Joints{}, wrapping);
    ASSERT_TRUE(path) << describe(path.error());
    EXPECT_NEAR(path->joints.positions().back()[5], 2.0 * kPi, 1e-9);  // all the way round, not back to 0

    EXPECT_EQ(ec::nearest_turn(Joints{0, 0, 0, 0, 0, -3.0}, Joints{0, 0, 0, 0, 0, 3.0}, 2.0 * kPi)[5],
              -3.0 + 2.0 * kPi);
    EXPECT_EQ(ec::nearest_turn(Joints{0, 0, 0, 0, 0, -3.0}, Joints{0, 0, 0, 0, 0, 3.0}, 3.2)[5], -3.0);  // limit
}

TEST(Solve, RefusesAJumpBetweenNeighbours) {
    // The "arm" changes configuration half way: joint 4 turns by half a turn.
    const auto flipping = [](const Pose& p, const Joints&) -> std::expected<Joints, Error> {
        Joints q{p.position[0], p.position[1], p.position[2], p.rotation[0], p.rotation[1], p.rotation[2]};
        if (p.position[1] < 0.0) q[3] += kPi;
        return q;
    };
    const auto path = ec::motion::solve(kCircle, Joints{}, flipping);
    ASSERT_FALSE(path);
    EXPECT_EQ(path.error().kind, Error::Kind::Invalid);
    EXPECT_NE(path.error().message.find("joint 4 turns 180.0 deg"), std::string::npos) << path.error().message;
}

TEST(Solve, ReportsAPoseWithoutSolution) {
    const auto unreachable = [](const Pose& p, const Joints&) -> std::expected<Joints, Error> {
        if (p.position[0] < 0.45) return std::unexpected(Error{Error::Kind::Command, "inverseKinematic: failed", {}});
        return Joints{};
    };
    const auto path = ec::motion::solve(kCircle, Joints{}, unreachable);
    ASSERT_FALSE(path);
    EXPECT_EQ(path.error().kind, Error::Kind::Invalid);
    EXPECT_NE(path.error().message.find("no joint positions for the pose at"), std::string::npos);
    EXPECT_NE(path.error().message.find("inverseKinematic: failed"), std::string::npos);

    const auto lost = [](const Pose&, const Joints&) -> std::expected<Joints, Error> {
        return std::unexpected(Error{Error::Kind::Disconnected, "the command connection was lost", {}});
    };
    EXPECT_EQ(ec::motion::solve(kCircle, Joints{}, lost).error().kind, Error::Kind::Disconnected);
}

TEST(Solve, RefusesTooManyPoints) {
    const auto path = ec::motion::solve(kCircle, Joints{}, IdentityIk{}, {.spacing = 1e-5, .max_points = 5'000});
    ASSERT_FALSE(path);
    EXPECT_EQ(path.error().kind, Error::Kind::Invalid);
}

// ------------------------------------------------------------------- TimedPath

TEST(TimedPath, SamplesFromStartToEndAtThePeriod) {
    auto path = ec::motion::solve(kCircle, Joints{}, IdentityIk{});
    ASSERT_TRUE(path);
    const auto timed = TimedPath::make(std::move(*path), 0.05, 0.5);
    ASSERT_TRUE(timed) << describe(timed.error());
    EXPECT_NEAR(timed->duration(), 0.5 + length(kCircle) / 0.05, 1e-12);

    const auto t = timed->sample(0.008);
    ASSERT_GT(t.size(), 3u);
    EXPECT_EQ(t.time.front(), 0.0);
    EXPECT_EQ(t.time.back(), timed->duration());
    const double last = t.time[t.size() - 1] - t.time[t.size() - 2];
    EXPECT_GE(last, 0.004);
    EXPECT_LE(last, 0.012);
    EXPECT_TRUE(validate(t)) << validate(t).error();
    expect_near(position_of(t.position.front()), {0.6, 0.0, 0.3}, 1e-12);
    expect_near(position_of(t.position.back()), {0.6, 0.0, 0.3}, 1e-9);

    // The velocity is the derivative of the position; the speed along the circle is the profile's.
    for (int k = 0; 0.01 + 0.37 * k < timed->duration(); ++k) {
        const double s = 0.01 + 0.37 * k;
        const Joints v = timed->velocity(s);
        const Joints a = timed->position(s - 1e-6);
        const Joints b = timed->position(s + 1e-6);
        for (std::size_t j = 0; j < 3; ++j) EXPECT_NEAR(v[j], (b[j] - a[j]) / 2e-6, 1e-5) << "t = " << s;
        EXPECT_NEAR(ec::norm(position_of(v)), timed->profile().speed(s), 1e-4) << "t = " << s;
    }
}

TEST(TimedPath, PeakRatesAndInterpolation) {
    ec::motion::JointTrajectory t;
    t.time = {0.0, 1.0, 2.0, 4.0};
    t.position = {Joints{0.0}, Joints{1.0}, Joints{1.0}, Joints{-1.0}};
    EXPECT_NEAR(ec::motion::peak_velocity(t)[0], 1.0, 1e-15);
    EXPECT_NEAR(ec::motion::peak_acceleration(t)[0], 1.0, 1e-15);  // 1 rad/s to 0 within (1 + 1) / 2 s
    EXPECT_NEAR(ec::motion::position_at(t, 0.5)[0], 0.5, 1e-15);
    EXPECT_NEAR(ec::motion::position_at(t, 3.0)[0], 0.0, 1e-15);
    EXPECT_EQ(ec::motion::position_at(t, -1.0)[0], 0.0);
    EXPECT_EQ(ec::motion::position_at(t, 9.0)[0], -1.0);
}

}  // namespace
