// Streamed motion on the bench: the shaper's limits and landing, the position
// stream's bumpless start, watchdog, tracking check and both kinds of close,
// and the velocity and jog streams.

#include "ec/motion/stream.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "ec/motion/shaper.hpp"
#include "support/bench.hpp"

namespace {

using namespace std::chrono_literals;
using ec::Joints;
using ec::command::Json;
using ec::motion::Channel;
using ec::motion::End;
using ec::motion::Error;
using ec::motion::Phase;
using ec::motion::RateLimiter;
using ec::test::Bench;

constexpr double kDt = 0.008;
const RateLimiter::Limits kLimits{Joints{1, 1, 1, 1, 1, 1}, Joints{4, 4, 4, 4, 4, 4}};  // rad/s, rad/s^2

TEST(RateLimiter, ReachesAStepTargetWithinItsLimitsAndLandsOnIt) {
    RateLimiter r(kLimits, kDt);
    r.reset(Joints{});
    const Joints target{0.5, -0.2, 0, 0, 0, 0};
    double previous_v = 0.0;
    double peak = 0.0;
    int steps = 0;
    while (r.position() != target && steps < 2000) {
        const double before = r.position()[0];
        r.next(target);
        const double v = r.velocity()[0];
        EXPECT_LE(std::abs(v - previous_v), 4 * kDt + 1e-9);  // acceleration limit
        EXPECT_LE(std::abs(v), 1.0 + 1e-9);                   // velocity limit
        EXPECT_LE(r.position()[0], 0.5 + 1e-12);              // never past the target
        EXPECT_GE(r.position()[0], before);                   // never back
        peak = std::max(peak, v);
        previous_v = v;
        ++steps;
    }
    EXPECT_EQ(r.position(), target);
    EXPECT_LE(std::abs(r.velocity()[0]), 4 * kDt);  // the landing period's speed
    r.next(target);                                 // the next period: at rest on the target
    EXPECT_EQ(r.position(), target);
    EXPECT_TRUE(r.at_rest());
    EXPECT_NEAR(peak, 1.0, 1e-9);  // far enough to reach the velocity limit
    EXPECT_LT(steps, 120);         // about 0.75 s: 0.25 s up, 0.25 s cruise, 0.25 s down
}

TEST(RateLimiter, FollowsAMovingTargetAndReversesWithoutJumps) {
    RateLimiter r(kLimits, kDt);
    r.reset(Joints{});
    double previous_v = 0.0;
    for (int k = 0; k < 400; ++k) {
        const double t = k * kDt;
        r.next(Joints{0.3 * std::sin(2.0 * t), 0, 0, 0, 0, 0});
        EXPECT_LE(std::abs(r.velocity()[0] - previous_v), 4 * kDt + 1e-9);
        previous_v = r.velocity()[0];
    }
    EXPECT_NEAR(r.position()[0], 0.3 * std::sin(2.0 * 399 * kDt), 0.05);  // tracks, with a small lag
}

TEST(RateLimiter, BrakeStopsAtTheDecelerationLimit) {
    RateLimiter r(kLimits, kDt);
    r.reset(Joints{});
    for (int k = 0; k < 100; ++k) r.next(Joints{10, 0, 0, 0, 0, 0});  // cruising at 1 rad/s
    ASSERT_NEAR(r.velocity()[0], 1.0, 1e-9);
    int steps = 0;
    while (!r.at_rest() && steps < 1000) {
        const double v = r.velocity()[0];
        r.brake();
        EXPECT_NEAR(v - r.velocity()[0], std::min(v, 4 * kDt), 1e-9);
        ++steps;
    }
    EXPECT_EQ(steps, 32);  // 1 rad/s at 4 rad/s^2: 0.25 s
}

struct Servo {
    std::shared_ptr<Channel<Joints>> channel = std::make_shared<Channel<Joints>>();
    ec::motion::ServoStream stream;
    explicit Servo(const ec::motion::ServoOptions& o = {}) : stream(o, channel) {}
};

// Steps until the init is accepted and the first point is sent.
void open(Bench& b, Servo& s) {
    ASSERT_EQ(b.step(s.stream), Phase::Running);
    ASSERT_EQ(b.step(s.stream), Phase::Running);
}

std::vector<double> put_joint(const Bench& b, std::size_t j) {
    std::vector<double> out;
    for (const auto& sent : b.sent)
        if (sent.method == "tt_put_servo_joint_to_buf") out.push_back(sent.params["targetPos"][j].get<double>());
    return out;
}

TEST(ServoStream, OpensWithTheControllersParametersAndStartsWhereTheArmIs) {
    Bench b;
    b.state.joint_position = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
    Servo s;
    open(b, s);
    ASSERT_EQ(b.sent[0].method, "transparent_transmission_init");
    EXPECT_EQ(b.sent[0].params, Json::parse(R"({"lookahead":40,"t":8,"smoothness":0.1})"));
    ASSERT_EQ(b.sent[1].method, "tt_put_servo_joint_to_buf");
    EXPECT_NEAR(b.sent[1].params["targetPos"][0].get<double>(), ec::rad_to_deg(0.1), 1e-9);  // bumpless
    for (int i = 0; i < 10; ++i) ASSERT_TRUE(b.step(s.stream));                              // no target yet: it stays
    EXPECT_EQ(b.count("tt_put_servo_joint_to_buf"), 11u);
    for (double q : put_joint(b, 5)) EXPECT_NEAR(q, ec::rad_to_deg(0.6), 1e-9);
}

TEST(ServoStream, ShapesATargetIntoOnePointPerCycleWithinTheLimits) {
    Bench b;
    Servo s(ec::motion::ServoOptions{.max_velocity = Joints{0.5, 0.5, 0.5, 0.5, 0.5, 0.5}});
    open(b, s);
    for (int i = 0; i < 100; ++i) {
        s.channel->push(Joints{1.0, 0, 0, 0, 0, 0}, b.now);
        b.state.joint_position = {ec::deg_to_rad(put_joint(b, 0).back()), 0, 0, 0, 0, 0};  // the arm follows
        ASSERT_EQ(b.step(s.stream), Phase::Running);
    }
    const auto q = put_joint(b, 0);
    for (std::size_t k = 1; k < q.size(); ++k) EXPECT_LE(q[k] - q[k - 1], ec::rad_to_deg(0.5 * 0.008) + 1e-9);
    EXPECT_GT(q.back(), 15.0);  // well on its way to 57 deg
}

TEST(ServoStream, AStaleTargetHoldsThenStarves) {
    Bench b;
    Servo s;
    open(b, s);
    s.channel->push(Joints{0.2, 0, 0, 0, 0, 0}, b.now);
    std::expected<Phase, Error> r = Phase::Running;
    for (int i = 0; i < 200 && r; ++i) {
        b.state.joint_position = {ec::deg_to_rad(put_joint(b, 0).back()), 0, 0, 0, 0, 0};
        r = b.step(s.stream);
    }
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().kind, Error::Kind::Starved);
    // Held from 100 ms (13 cycles), starved after 600 ms: the last points stand still.
    const auto q = put_joint(b, 0);
    EXPECT_NEAR(q[q.size() - 1], q[q.size() - 2], 1e-12);
    EXPECT_LT(q.back(), ec::rad_to_deg(0.2));  // braked before reaching the target
}

TEST(ServoStream, ATargetBeyondTheJointLimitEndsTheStream) {
    Bench b;
    Servo s;
    open(b, s);
    s.channel->push(Joints{7.0, 0, 0, 0, 0, 0}, b.now);
    const auto r = b.step(s.stream);
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().kind, Error::Kind::Invalid);
}

TEST(ServoStream, AnArmFarFromItsSetpointsIsATrackingError) {
    Bench b;
    Servo s;
    open(b, s);
    b.state.joint_position = {ec::deg_to_rad(6.0), 0, 0, 0, 0, 0};
    const auto r = b.step(s.stream);
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().kind, Error::Kind::Tracking);
}

TEST(ServoStream, ARefusedPointIsACommandError) {
    Bench b;
    b.fail("tt_put_servo_joint_to_buf");
    Servo s;
    open(b, s);
    const auto r = b.step(s.stream);
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().kind, Error::Kind::Command);
}

TEST(ServoStream, ImpossibleOptionsAreInvalid) {
    {
        Bench b;
        b.period = 8500us;
        Servo s;
        EXPECT_EQ(b.step(s.stream).error().kind, Error::Kind::Invalid);
    }
    {
        Bench b;
        Servo s(ec::motion::ServoOptions{.lookahead = 5ms});
        EXPECT_EQ(b.step(s.stream).error().kind, Error::Kind::Invalid);
        EXPECT_TRUE(b.close(s.stream, End::Cancelled));
        EXPECT_TRUE(b.sent.empty());
    }
}

TEST(ServoStream, ClosingGracefullyDeceleratesDrainsAndClears) {
    Bench b;
    Servo s;
    open(b, s);
    for (int i = 0; i < 30; ++i) {
        s.channel->push(Joints{1.0, 0, 0, 0, 0, 0}, b.now);
        b.state.joint_position = {ec::deg_to_rad(put_joint(b, 0).back()), 0, 0, 0, 0, 0};
        ASSERT_TRUE(b.step(s.stream));
    }
    s.channel->request_close();
    EXPECT_EQ(b.step(s.stream), Phase::Done);
    int cycles = 0;
    bool closed = false;
    while (!closed && cycles < 300) {
        b.state.joint_position = {ec::deg_to_rad(put_joint(b, 0).back()), 0, 0, 0, 0, 0};
        b.state.joint_velocity = {};
        closed = b.close(s.stream, End::Done);
        ++cycles;
    }
    EXPECT_TRUE(closed);
    EXPECT_EQ(b.count("stop"), 0u);
    EXPECT_EQ(b.count("tt_clear_servo_joint_buf"), 1u);
    const auto q = put_joint(b, 0);
    EXPECT_NEAR(q[q.size() - 1], q[q.size() - 2], 1e-12);  // ended at rest
}

TEST(ServoStream, AStopEndsItAtOnce) {
    Bench b;
    Servo s;
    open(b, s);
    EXPECT_FALSE(b.close(s.stream, End::Stopped));
    EXPECT_EQ(b.count("stop"), 1u);
    EXPECT_EQ(b.count("tt_clear_servo_joint_buf"), 1u);
    EXPECT_TRUE(b.close(s.stream, End::Stopped));
}

TEST(ServoStream, WithoutRepliesTheInitSaysSoAndPointsExpectNone) {
    Bench b;
    Servo s(ec::motion::ServoOptions{.replies = false});
    open(b, s);
    EXPECT_EQ(b.sent[0].params["response_enable"], 0);
    b.fail("tt_put_servo_joint_to_buf");  // would fail if a reply were expected
    for (int i = 0; i < 5; ++i) EXPECT_EQ(b.step(s.stream), Phase::Running);
}

struct Velocity {
    std::shared_ptr<Channel<Joints>> channel = std::make_shared<Channel<Joints>>();
    ec::motion::VelocityStream stream;
    explicit Velocity(const ec::motion::VelocityOptions& o = {}) : stream(o, channel) {}
};

TEST(VelocityStream, SendsTheNewestVelocityEveryCycleScaledToTheLimits) {
    Bench b;
    Velocity v(ec::motion::VelocityOptions{.acceleration = 1.0, .max_velocity = Joints{0.1, 0.1, 0.1, 0.1, 0.1, 0.1}});
    EXPECT_EQ(b.step(v.stream), Phase::Running);
    EXPECT_TRUE(b.sent.empty());  // nothing pushed yet
    v.channel->push(Joints{0.2, 0.1, 0, 0, 0, 0}, b.now);
    ASSERT_TRUE(b.step(v.stream));
    ASSERT_EQ(b.methods(), std::vector<std::string>{"moveBySpeedj"});
    const Json& p = b.sent[0].params;
    EXPECT_NEAR(p["vj"][0].get<double>(), ec::rad_to_deg(0.1), 1e-9);  // halved, as is every joint
    EXPECT_NEAR(p["vj"][1].get<double>(), ec::rad_to_deg(0.05), 1e-9);
    EXPECT_EQ(p["acc"], 57);
    EXPECT_DOUBLE_EQ(p["t"].get<double>(), 0.04);
}

TEST(VelocityStream, AStaleTargetBecomesZeroThenStarves) {
    Bench b;
    Velocity v;
    v.channel->push(Joints{0.05, 0, 0, 0, 0, 0}, b.now);
    std::expected<Phase, Error> r = Phase::Running;
    for (int i = 0; i < 200 && r; ++i) r = b.step(v.stream);
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().kind, Error::Kind::Starved);
    EXPECT_EQ(b.sent.back().params["vj"][0].get<double>(), 0.0);
}

TEST(VelocityStream, CloseSendsStopjAndWaitsForRest) {
    Bench b;
    Velocity v;
    v.channel->push(Joints{0.05, 0, 0, 0, 0, 0}, b.now);
    ASSERT_TRUE(b.step(v.stream));
    EXPECT_FALSE(b.close(v.stream, End::Faulted));
    EXPECT_EQ(b.count("stopj"), 1u);
    EXPECT_EQ(b.count("stop"), 1u);  // urgent
    EXPECT_TRUE(b.close(v.stream, End::Faulted));
}

}  // namespace
