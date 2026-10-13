// The streams built for Cartesian paths, on the bench: following a timed joint
// trajectory through transparent transmission, and streaming TCP poses solved
// by the controller's inverse kinematics.

#include "ec/motion/follow.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <numbers>
#include <string>
#include <vector>

#include "ec/motion/stream.hpp"
#include "support/bench.hpp"

namespace {

using namespace std::chrono_literals;
using ec::Joints;
using ec::Pose;
using ec::command::Json;
using ec::motion::Channel;
using ec::motion::End;
using ec::motion::Error;
using ec::motion::FollowTrajectory;
using ec::motion::JointTrajectory;
using ec::motion::Phase;
using ec::test::Bench;

std::vector<double> put_joint(const Bench& b, std::size_t j) {
    std::vector<double> out;
    for (const auto& sent : b.sent)
        if (sent.method == "tt_put_servo_joint_to_buf") out.push_back(sent.params["targetPos"][j].get<double>());
    return out;
}

// Joint 1 from 0 to 0.05 rad in 0.6 s along a half cosine, from rest to rest, a point every 50 ms.
JointTrajectory smooth_move() {
    JointTrajectory t;
    for (int k = 0; k <= 12; ++k) {
        const double s = 0.05 * k;
        t.time.push_back(s);
        t.position.push_back(Joints{0.025 * (1.0 - std::cos(std::numbers::pi * s / 0.6)), 0, 0, 0, 0, 0});
    }
    return t;
}

// ----------------------------------------------------------- FollowTrajectory

TEST(FollowTrajectory, SendsTheTrajectoryOnePointPerCycleThenWaitsForTheArm) {
    Bench b;
    const JointTrajectory path = smooth_move();
    auto a = make_activity(FollowTrajectory{.path = path});
    ASSERT_EQ(b.step(a), Phase::Running);
    ASSERT_EQ(b.methods(), std::vector<std::string>{"transparent_transmission_init"});

    int cycles = 0;
    std::expected<Phase, Error> r = Phase::Running;
    while (r && *r == Phase::Running && cycles < 200) {
        if (const auto q = put_joint(b, 0); !q.empty()) b.state.joint_position[0] = ec::deg_to_rad(q.back());
        r = b.step(a);
        ++cycles;
    }
    ASSERT_TRUE(r) << describe(r.error());
    EXPECT_EQ(*r, Phase::Done);

    // The n-th point is the trajectory at n periods, interpolated between its points.
    const auto q = put_joint(b, 0);
    ASSERT_GE(q.size(), 76u);  // 0.6 s at 8 ms, then the last point until the arm rests there
    for (std::size_t n = 0; n < 76; ++n)
        EXPECT_NEAR(q[n], ec::rad_to_deg(ec::motion::position_at(path, 0.008 * static_cast<double>(n))[0]), 1e-9)
            << "point " << n;
    for (std::size_t n = 76; n < q.size(); ++n) EXPECT_NEAR(q[n], ec::rad_to_deg(0.05), 1e-9);

    // Done: the queue drains while the arm rests, then it is cleared.
    while (!b.close(a, End::Done)) {
    }
    EXPECT_EQ(b.methods().back(), "tt_clear_servo_joint_buf");
    EXPECT_EQ(b.count("stop"), 0u);
}

TEST(FollowTrajectory, RefusesATrajectoryTheStreamCannotFollow) {
    const auto refused = [](const FollowTrajectory& spec) {
        Bench b;
        auto a = make_activity(spec);
        const auto r = b.step(a);
        EXPECT_TRUE(b.sent.empty());
        return r ? std::string{} : r.error().message;
    };
    // Too fast for the stream's speed limit (10 % of the datasheet's: 15 deg/s on J1).
    JointTrajectory fast = smooth_move();
    for (auto& q : fast.position) q[0] *= 20.0;
    EXPECT_NE(refused({.path = fast}).find("would move at"), std::string::npos);

    // From rest to full speed at once: too abrupt.
    JointTrajectory abrupt;
    abrupt.time = {0.0, 0.05, 0.1, 0.15};
    abrupt.position = {Joints{0.0}, Joints{0.01}, Joints{0.02}, Joints{0.03}};
    EXPECT_NE(refused({.path = abrupt}).find("would accelerate at"), std::string::npos);

    // Too few points.
    JointTrajectory two;
    two.time = {0.0, 1.0};
    two.position = {Joints{}, Joints{}};
    EXPECT_NE(refused({.path = two}).find("at least 3"), std::string::npos);
}

TEST(FollowTrajectory, RefusesToStartAwayFromTheFirstPoint) {
    Bench b;
    b.state.joint_position[2] = ec::deg_to_rad(3.0);
    auto a = make_activity(FollowTrajectory{.path = smooth_move()});
    const auto r = b.step(a);
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().kind, Error::Kind::Invalid);
    EXPECT_NE(r.error().message.find("3.00 deg from the first point"), std::string::npos) << r.error().message;
    EXPECT_TRUE(b.close(a, End::Cancelled));  // nothing was sent: nothing to close
}

TEST(FollowTrajectory, AStopClosesTheStreamAtOnce) {
    Bench b;
    auto a = make_activity(FollowTrajectory{.path = smooth_move()});
    for (int i = 0; i < 20; ++i) ASSERT_TRUE(b.step(a));
    ASSERT_FALSE(b.close(a, End::Stopped));
    const auto sent = b.methods();
    ASSERT_GE(sent.size(), 2u);
    EXPECT_EQ(sent[sent.size() - 2], "stop");  // no deceleration along the trajectory: stop, clear
    EXPECT_EQ(sent.back(), "tt_clear_servo_joint_buf");
    while (!b.close(a, End::Stopped)) {
    }
    EXPECT_EQ(b.count("tt_clear_servo_joint_buf"), 1u);
}

TEST(FollowTrajectory, AnArmThatDoesNotArriveIsATrackingError) {
    Bench b;
    auto a = make_activity(FollowTrajectory{.path = smooth_move(), .settle = 100ms});
    std::expected<Phase, Error> r = Phase::Running;
    for (int i = 0; i < 200 && r && *r == Phase::Running; ++i) r = b.step(a);  // the arm never moves
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().kind, Error::Kind::Tracking);
}

// ------------------------------------------------------------ PoseServoStream

// The bench's inverse kinematics: joint j [deg] is a tenth of the pose's x, y, z [mm] for j = 0..2.
Json solve(const ec::command::Request& r) {
    const auto& p = r.params["targetPose"];
    return Json::array({p[0].get<double>() / 10.0, p[1].get<double>() / 10.0, p[2].get<double>() / 10.0, 0, 0, 0});
}

Joints joints_of(const Pose& p) {
    return {ec::deg_to_rad(p.position[0] * 100.0),
            ec::deg_to_rad(p.position[1] * 100.0),
            ec::deg_to_rad(p.position[2] * 100.0),
            0,
            0,
            0};
}

struct PoseStream {
    std::shared_ptr<Channel<Pose>> channel = std::make_shared<Channel<Pose>>();
    ec::motion::PoseServoStream stream;
    explicit PoseStream(const ec::motion::PoseServoOptions& o = {}) : stream(o, channel) {}
};

const Pose kHere{{0.5, 0.0, 0.3}, {std::numbers::pi, 0.0, 0.0}};

// A bench whose arm is at kHere and whose controller solves poses as above.
Bench pose_bench() {
    Bench b;
    b.state.joint_position = joints_of(kHere);
    b.answers["inverseKinematic"] = [](const ec::command::Request& r) -> std::expected<Json, ec::command::Error> {
        return solve(r);
    };
    return b;
}

TEST(PoseServoStream, SolvesEachNewPoseNearThePreviousSolutionAndStreamsTheJoints) {
    Bench b = pose_bench();
    PoseStream s;
    ASSERT_EQ(b.step(s.stream), Phase::Running);  // init
    ASSERT_EQ(b.step(s.stream), Phase::Running);  // accepted: holds where the arm is
    EXPECT_TRUE(s.channel->is_open());

    Pose target = kHere;
    target.position[0] += 0.0001;  // 0.1 mm
    s.channel->push(target, b.now);
    ASSERT_EQ(b.step(s.stream), Phase::Running);
    const auto& ask = b.sent[b.sent.size() - 2];
    ASSERT_EQ(ask.method, "inverseKinematic");
    EXPECT_NEAR(ask.params["targetPose"][0].get<double>(), 500.1, 1e-9);              // mm
    EXPECT_NEAR(ask.params["targetPose"][3].get<double>(), std::numbers::pi, 1e-12);  // rad
    EXPECT_EQ(ask.params["unit_type"], 1);
    EXPECT_NEAR(ask.params["referencePos"][0].get<double>(), 50.0, 1e-9);  // the measured joints, in degrees
    EXPECT_EQ(b.sent.back().method, "tt_put_servo_joint_to_buf");

    // The solution arrives in the next cycle and becomes the target; an unchanged pose is not asked again.
    ASSERT_EQ(b.step(s.stream), Phase::Running);
    EXPECT_EQ(b.count("inverseKinematic"), 1u);
    for (int i = 0; i < 20; ++i) {
        s.channel->push(target, b.now);  // the same pose, pushed again: asked again, near the solution
        ASSERT_EQ(b.step(s.stream), Phase::Running);
    }
    EXPECT_NEAR(b.sent[b.sent.size() - 2].params["referencePos"][0].get<double>(), 50.01, 1e-9);
    EXPECT_NEAR(put_joint(b, 0).back(), 50.01, 1e-9);  // shaped onto the solution
}

TEST(PoseServoStream, APoseWithoutSolutionEndsTheStream) {
    Bench b = pose_bench();
    b.fail("inverseKinematic");
    PoseStream s;
    ASSERT_TRUE(b.step(s.stream));
    ASSERT_TRUE(b.step(s.stream));
    s.channel->push(kHere, b.now);
    ASSERT_TRUE(b.step(s.stream));
    const auto r = b.step(s.stream);
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().kind, Error::Kind::Invalid);  // the target's fault: closes gracefully
    EXPECT_NE(r.error().message.find("no joint positions for the target (500.0, 0.0, 300.0 mm)"), std::string::npos)
        << r.error().message;
}

TEST(PoseServoStream, ASolutionThatJumpsEndsTheStream) {
    Bench b = pose_bench();
    PoseStream s(ec::motion::PoseServoOptions{.max_step = ec::deg_to_rad(5.0)});
    ASSERT_TRUE(b.step(s.stream));
    ASSERT_TRUE(b.step(s.stream));
    Pose far = kHere;
    far.position[1] += 0.06;  // joint 2 would turn 6 degrees at once
    s.channel->push(far, b.now);
    ASSERT_TRUE(b.step(s.stream));
    const auto r = b.step(s.stream);
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().kind, Error::Kind::Invalid);
    EXPECT_NE(r.error().message.find("joint 2 would turn 6.0 deg"), std::string::npos) << r.error().message;
}

TEST(PoseServoStream, StalePosesHoldThenStarve) {
    Bench b = pose_bench();
    PoseStream s;
    ASSERT_TRUE(b.step(s.stream));
    ASSERT_TRUE(b.step(s.stream));
    s.channel->push(kHere, b.now);
    std::expected<Phase, Error> r = Phase::Running;
    int cycles = 0;
    while (r && cycles < 200) {
        r = b.step(s.stream);
        ++cycles;
    }
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().kind, Error::Kind::Starved);
    EXPECT_NEAR(cycles * 8.0, 600.0, 24.0);  // 100 ms to hold, 500 ms holding
}

TEST(PoseServoStream, ACloseRequestEndsItGracefully) {
    Bench b = pose_bench();
    PoseStream s;
    ASSERT_TRUE(b.step(s.stream));
    ASSERT_TRUE(b.step(s.stream));
    s.channel->request_close();
    EXPECT_EQ(b.step(s.stream), Phase::Done);
    int cycles = 0;
    while (!b.close(s.stream, End::Done) && cycles < 100) ++cycles;
    EXPECT_EQ(b.methods().back(), "tt_clear_servo_joint_buf");
    EXPECT_EQ(b.count("stop"), 0u);
}

}  // namespace
