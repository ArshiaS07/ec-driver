// ec::Robot against a minimal controller on 127.0.0.1: a command port that
// answers every call, and a monitor port that sends a frame every 8 ms. Checks
// the wiring: connecting, state, typed calls, kinematics, a planned move, a
// servo session and a pose session, stop from another thread, and stopping on
// destruction.

#include "ec/robot.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "ec/command/movement.hpp"
#include "ec/command/param.hpp"
#include "ec/motion/point_to_point.hpp"
#include "ec/pose.hpp"
#include "support/frame_builder.hpp"
#include "support/loopback_server.hpp"

namespace {

using namespace std::chrono_literals;
using ec::command::Json;
using ec::motion::Error;

// Ports 8055 and 8056 of a robot that accepts everything and stands still.
class MiniController {
public:
    MiniController() {
        commands_thread_ = std::jthread([this](const std::stop_token& stop) { serve_commands(stop); });
        frames_thread_ = std::jthread([this](const std::stop_token& stop) { send_frames(stop); });
    }
    ~MiniController() {
        stop_.store(true);
        // Both threads end once the robot's connections close, or at once if none came.
    }
    MiniController(const MiniController&) = delete;
    MiniController& operator=(const MiniController&) = delete;

    // These tests check the wiring, not the timing: a frame up to 500 ms old
    // still counts as fresh, so a busy machine cannot fail them.
    ec::RobotOptions options() const {
        ec::RobotOptions o{.host = "127.0.0.1",
                           .command_port = commands_.port(),
                           .monitor_port = frames_.port(),
                           .first_state_within = 2s};
        o.control.thresholds.fresh_within = 500ms;
        return o;
    }
    std::vector<std::string> seen() const {
        const std::lock_guard lock(mutex_);
        return seen_;
    }
    std::size_t count(const std::string& method) const {
        const auto s = seen();
        return static_cast<std::size_t>(std::count(s.begin(), s.end(), method));
    }
    // The parameters of the last call of `method`.
    Json last(const std::string& method) const {
        const std::lock_guard lock(mutex_);
        const auto it = last_.find(method);
        return it == last_.end() ? Json{} : it->second;
    }

private:
    void serve_commands(const std::stop_token& stop) {
        while (auto connection = accept_or_quit(commands_, stop)) {
            ec::test::Peer& peer = *connection;
            for (;;) {
                const std::string line = peer.receive_line();
                if (line.empty()) break;
                const Json request = Json::parse(line);
                const std::string method = request["method"].get<std::string>();
                const Json params = request.contains("params") ? request["params"] : Json{};
                {
                    const std::lock_guard lock(mutex_);
                    seen_.push_back(method);
                    last_[method] = params;
                }
                std::string result = "true";
                if (method == "getRobotMode") result = "2";
                if (method == "getSpeed") result = "100";
                if (method == "inverseKinematic")  // the reference itself: the arm stands still
                    result = params.contains("referencePos") ? params["referencePos"].dump() : "[0,0,0,0,0,0]";
                if (method == "positiveKinematic") result = "[400.0,-50.0,300.0,3.0,0.1,-0.2]";
                if (method == "check_trajectory") result = "0";
                peer.send(Json{{"jsonrpc", "2.0"}, {"result", result}, {"id", request["id"]}}.dump() + "\n");
            }
        }
    }

    void send_frames(const std::stop_token& stop) {
        while (auto connection = accept_or_quit(frames_, stop)) {
            ec::test::Peer& peer = *connection;
            const auto frame = ec::test::make_frame(ec::test::FrameFields{});
            const std::string bytes(reinterpret_cast<const char*>(frame.data()), frame.size());
            try {
                while (!stop.stop_requested()) {
                    peer.send(bytes);
                    std::this_thread::sleep_for(8ms);
                }
            } catch (const std::runtime_error&) {
                peer.close();  // the robot closed the monitor connection: wait for the next one
            }
        }
    }

    // Waits for the robot to connect, in slices so that a stop is seen; nullopt once stopped.
    std::optional<ec::test::Peer> accept_or_quit(ec::test::LoopbackServer& server, const std::stop_token& stop) {
        while (!stop.stop_requested() && !stop_.load()) {
            try {
                return server.accept(100ms);
            } catch (const std::runtime_error&) {
                continue;  // nothing yet: look again
            }
        }
        return std::nullopt;
    }

    ec::test::LoopbackServer commands_;
    ec::test::LoopbackServer frames_;
    mutable std::mutex mutex_;
    std::vector<std::string> seen_;
    std::map<std::string, Json> last_;
    std::atomic<bool> stop_{false};
    std::jthread commands_thread_;
    std::jthread frames_thread_;
};

TEST(Robot, ConnectFailsWithoutTheCommandPort) {
    std::uint16_t closed = 0;
    {
        const ec::test::LoopbackServer gone;
        closed = gone.port();
    }
    const auto robot =
        ec::Robot::connect(ec::RobotOptions{.host = "127.0.0.1", .command_port = closed, .connect_timeout = 500ms});
    ASSERT_FALSE(robot);
    EXPECT_EQ(robot.error().kind, Error::Kind::Disconnected);
    EXPECT_NE(robot.error().message.find("command port"), std::string::npos);
}

TEST(Robot, ConnectFailsWithoutMonitorFrames) {
    ec::test::LoopbackServer commands;
    ec::test::LoopbackServer silent;  // accepts, never sends
    const auto robot = ec::Robot::connect(ec::RobotOptions{.host = "127.0.0.1",
                                                           .command_port = commands.port(),
                                                           .monitor_port = silent.port(),
                                                           .first_state_within = 200ms});
    ASSERT_FALSE(robot);
    EXPECT_NE(robot.error().message.find("no frame"), std::string::npos);
}

TEST(Robot, ConnectsAndReportsTheState) {
    MiniController controller;
    auto robot = ec::Robot::connect(controller.options());
    ASSERT_TRUE(robot) << describe(robot.error());
    const auto s = robot->state();
    ASSERT_TRUE(s);
    EXPECT_GT(s->seq, 0u);
    EXPECT_EQ(s->state.robot_mode, ec::RobotMode::Remote);
    EXPECT_LT(s->age(), 1s);
    const auto next = robot->next_state(s->seq, 1s);
    ASSERT_TRUE(next);
    EXPECT_GT(next->seq, s->seq);
    for (int i = 0; i < 100 && robot->status().cycles == 0; ++i) std::this_thread::sleep_for(5ms);
    const auto status = robot->status();
    EXPECT_EQ(status.mode, ec::motion::Mode::Idle);
    EXPECT_TRUE(status.connected);
    EXPECT_TRUE(status.conditions.contains(ec::motion::kCanStartMotion));
    EXPECT_TRUE(robot->feedback_status().connected);
}

TEST(Robot, TypedCallsGoThroughTheControlLoop) {
    MiniController controller;
    auto robot = ec::Robot::connect(controller.options());
    ASSERT_TRUE(robot);
    EXPECT_EQ(robot->call(ec::command::GetRobotMode{}), ec::RobotMode::Remote);
    const auto refused = robot->call(ec::command::MoveByJoint{});
    ASSERT_FALSE(refused);
    EXPECT_EQ(refused.error().kind, Error::Kind::Refused);
    EXPECT_EQ(controller.count("moveByJoint"), 0u);
}

TEST(Robot, KinematicsAreTheControllersInSiUnits) {
    MiniController controller;
    auto robot = ec::Robot::connect(controller.options());
    ASSERT_TRUE(robot);
    const Eigen::Isometry3d T_base_tcp = ec::from_mm_rad({400.0, -50.0, 300.0, 3.0, 0.1, -0.2});
    const ec::Joints near{0.1, -1.5, 1.5, -1.5, -1.5, 0.0};
    const auto q = robot->ik(T_base_tcp, near);
    ASSERT_TRUE(q) << describe(q.error());
    const Json ik = controller.last("inverseKinematic");
    for (std::size_t i = 0; i < 6; ++i)  // mm and radians
        EXPECT_NEAR(ik["targetPose"][i].get<double>(), (std::array{400.0, -50.0, 300.0, 3.0, 0.1, -0.2})[i], 1e-9);
    EXPECT_NEAR(ik["referencePos"][1].get<double>(), -85.9437, 1e-4);  // degrees
    EXPECT_EQ(ik["unit_type"], 1);
    for (std::size_t j = 0; j < q->size(); ++j) EXPECT_NEAR((*q)[j], near[j], 1e-12);  // back in radians

    const auto T = robot->fk(near);
    ASSERT_TRUE(T) << describe(T.error());
    EXPECT_NEAR(controller.last("positiveKinematic")["targetPos"][2].get<double>(), 85.9437, 1e-4);
    EXPECT_TRUE(T->translation().isApprox(Eigen::Vector3d(0.4, -0.05, 0.3), 1e-12));  // metres
    EXPECT_TRUE(T->linear().isApprox(ec::rotation_from_rxyz(3.0, 0.1, -0.2), 1e-12));
}

TEST(Robot, PlansWithTheControllersKinematicsAndPlaysThroughTheTimestampService) {
    MiniController controller;
    auto robot = ec::Robot::connect(controller.options());
    ASSERT_TRUE(robot);
    // The mini controller answers inverseKinematic with the reference itself:
    // the arm stays where it is (all joints 0 deg: J3 and J5 sit on singularities).
    const ec::motion::TimedPose here{.time = 0.0, .pose = Eigen::Isometry3d::Identity()};
    const ec::motion::TimedPose later{.time = 1.0, .pose = Eigen::Isometry3d::Identity()};
    const std::vector path{here, later};
    const auto refused = robot->plan(path);
    ASSERT_FALSE(refused);
    EXPECT_NE(refused.error().message.find("singularity"), std::string::npos) << refused.error().message;

    const auto relaxed = robot->plan(path, {.singularity_margin = 0.0});
    ASSERT_TRUE(relaxed) << describe(relaxed.error());
    EXPECT_EQ(controller.count("inverseKinematic"), 1u + 2u);  // the refusal came at the first point
    const auto played = robot->play(*relaxed).wait();
    EXPECT_TRUE(played) << describe(played.error());
    EXPECT_EQ(controller.count("start_trajectory"), 1u);
}

TEST(Robot, RunsAPlannedMove) {
    MiniController controller;
    auto robot = ec::Robot::connect(controller.options());
    ASSERT_TRUE(robot);
    const auto here = robot->state()->state.joint_position;
    const auto result = robot->run(ec::motion::MoveJoint{.target = here, .speed = 0.1}).wait();
    EXPECT_TRUE(result) << describe(result.error());
    EXPECT_EQ(controller.count("moveByJoint"), 1u);
}

TEST(Robot, AServoSessionOpensStreamsAndClosesGracefully) {
    MiniController controller;
    auto robot = ec::Robot::connect(controller.options());
    ASSERT_TRUE(robot);
    auto session = robot->servo(ec::motion::ServoOptions{.lookahead = 24ms});
    ASSERT_TRUE(session) << describe(session.error());
    EXPECT_TRUE(session->running());
    auto target = robot->state()->state.joint_position;
    target[5] += ec::deg_to_rad(1.0);
    for (int i = 0; i < 20; ++i) {
        session->target(target);
        std::this_thread::sleep_for(8ms);
    }
    const auto closed = session->close();
    EXPECT_TRUE(closed) << describe(closed.error());
    EXPECT_EQ(controller.count("transparent_transmission_init"), 1u);
    EXPECT_GT(controller.count("tt_put_servo_joint_to_buf"), 10u);
    EXPECT_EQ(controller.count("tt_clear_servo_joint_buf"), 1u);
    EXPECT_EQ(controller.count("stop"), 0u);
}

TEST(Robot, StopFromAnotherThreadEndsAStreamAtOnce) {
    MiniController controller;
    auto robot = ec::Robot::connect(controller.options());
    ASSERT_TRUE(robot);
    auto session = robot->servo();
    ASSERT_TRUE(session);
    std::jthread([&] { robot->stop(); }).join();
    const auto ended = session->job().wait_for(2s);
    ASSERT_TRUE(ended);
    ASSERT_FALSE(*ended);
    EXPECT_EQ(ended->error().kind, Error::Kind::Stopped);
    EXPECT_EQ(controller.count("stop"), 1u);
    EXPECT_EQ(controller.count("tt_clear_servo_joint_buf"), 1u);
    EXPECT_FALSE(session->running());
}

TEST(Robot, DestroyingTheRobotStopsWhatRuns) {
    MiniController controller;
    ec::motion::Job job;
    {
        auto robot = ec::Robot::connect(controller.options());
        ASSERT_TRUE(robot);
        auto session = robot->servo();
        ASSERT_TRUE(session);
        job = session->job();
        session->close_wait = 0ms;  // leave it to the robot
        // the session is destroyed first, then the robot
    }
    ASSERT_TRUE(job.done());
}

}  // namespace
