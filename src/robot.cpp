#include "ec/robot.hpp"

#include <format>
#include <thread>

#include "detail/connection.hpp"
#include "ec/feedback/frame.hpp"
#include "ec/feedback/receiver.hpp"
#include "ec/pose.hpp"
#include "ec/sdk/client.hpp"
#include "ec/sdk/kinematics.hpp"
#include "ec/sdk/movement.hpp"
#include "ec/sdk/param.hpp"
#include "ec/sdk/servo.hpp"
#include "ec/status.hpp"

namespace ec {

std::expected<Robot, Error> Robot::connect(RobotOptions o) {
    auto client = sdk::Client::connect(
        o.host, {.port = o.command_port, .connect_timeout = o.connect_timeout, .reply_timeout = o.reply_timeout});
    if (!client)
        return std::unexpected(Error{Error::Kind::Disconnected, std::format("command port {}:{}: {}", o.host,
                                                                            o.command_port, client.error().message)});
    auto c = std::make_shared<detail::Connection>(std::move(*client));
    if (o.monitor) {
        feedback::Receiver::Options receiver_options;
        receiver_options.port = o.monitor_port;
        // Each frame decoded and kept; the frames counted here are Latest's sequence numbers.
        c->receiver = std::make_unique<feedback::Receiver>(
            o.host, receiver_options,
            [latest = c->latest, on_feedback = std::move(o.on_feedback),
             seq = std::uint64_t{0}](const feedback::Sample& s) mutable {
                const auto frame = feedback::decode(s.raw);
                if (!frame) return;
                const Feedback f{.seq = ++seq, .arrival = s.arrival, .state = feedback::to_si(*frame), .sample = s};
                latest->publish(f);
                if (on_feedback) on_feedback(f);
            });
        if (!c->latest->wait_newer(0, o.first_feedback_within)) {
            const auto status = c->receiver->status();
            return std::unexpected(
                Error{Error::Kind::Disconnected,
                      std::format("monitor port {}:{}: no frame within {} ms{}", o.host, o.monitor_port,
                                  o.first_feedback_within.count(),
                                  status.last_error ? " (" + status.last_error->message() + ")" : std::string{})});
        }
    }
    return Robot(std::move(c));
}

Robot::Robot(Robot&&) noexcept = default;
Robot& Robot::operator=(Robot&&) noexcept = default;
Robot::~Robot() = default;

std::expected<sdk::Json, Error> Robot::call_json(const sdk::Request& request, sdk::Kind kind) {
    if (!sdk::is_read_only(kind) && c_->motion_open())
        return std::unexpected(Error{
            Error::Kind::Busy, std::format("{}: a motion is open: only queries and computations", request.method)});
    return c_->call(request);
}

std::optional<Feedback> Robot::feedback() const {
    if (const auto e = c_->latest->get()) return e->value;
    return std::nullopt;
}

std::optional<Feedback> Robot::next_feedback(std::uint64_t after, std::chrono::nanoseconds timeout) const {
    if (const auto e = c_->latest->wait_newer(after, timeout)) return e->value;
    return std::nullopt;
}

std::expected<Joints, Error> Robot::ik(const Eigen::Isometry3d& T_base_tcp, const Joints& near) {
    auto q = c_->send(sdk::InverseKinematic{.target = to_mm_rad(T_base_tcp), .reference = rad_to_deg(near)});
    if (!q) return std::unexpected(std::move(q.error()));
    return nearest_turn(deg_to_rad(*q), near, kEc66JointLimit);  // in case it comes within one turn
}

std::expected<Eigen::Isometry3d, Error> Robot::fk(const Joints& joints) {
    auto pose = c_->send(sdk::PositiveKinematic{.joints = rad_to_deg(joints)});
    if (!pose) return std::unexpected(std::move(pose.error()));
    return from_mm_rad(*pose);
}

std::expected<Joints, Error> Robot::joints() {
    auto q = c_->send(sdk::GetJointPos{});
    if (!q) return std::unexpected(std::move(q.error()));
    return deg_to_rad(*q);
}

std::expected<void, Error> Robot::bring_up() {
    using namespace std::chrono_literals;
    if (c_->motion_open()) return std::unexpected(Error{Error::Kind::Busy, "bring-up: a motion is open"});
    // The manual's examples (2.2.7.6, 2.2.9.10), their pauses included.
    const auto state = c_->send(sdk::GetRobotState{});
    if (!state) return std::unexpected(state.error());
    if (*state == RobotState::Alarm) {
        if (auto ok = c_->send(sdk::ClearAlarm{}); !ok) return std::unexpected(ok.error());
        std::this_thread::sleep_for(500ms);
    }
    const auto synced = c_->send(sdk::GetMotorStatus{});
    if (!synced) return std::unexpected(synced.error());
    if (!*synced) {
        if (auto ok = c_->send(sdk::SyncMotorStatus{}); !ok) return std::unexpected(ok.error());
        std::this_thread::sleep_for(500ms);
    }
    std::this_thread::sleep_for(500ms);
    const auto servo = c_->send(sdk::GetServoStatus{});
    if (!servo) return std::unexpected(servo.error());
    if (!*servo) {
        if (auto ok = c_->send(sdk::SetServoStatus{.on = true}); !ok) return std::unexpected(ok.error());
        // The manual's 2.2.7.6 waits a second after it; here until the servos report on, 3 s at most.
        for (int i = 0; i < 30; ++i) {
            std::this_thread::sleep_for(100ms);
            const auto on = c_->send(sdk::GetServoStatus{});
            if (!on) return std::unexpected(on.error());
            if (*on) return {};
        }
        return std::unexpected(Error{Error::Kind::Timeout, "the servos did not come on within 3 s"});
    }
    return {};
}

void Robot::stop() noexcept {
    try {
        c_->stop_requested.store(true);
        std::function<void()> stop_open;
        {
            const std::lock_guard lock(c_->motions);
            stop_open = c_->stop_open;
        }
        if (stop_open) stop_open();
        const auto stopped = c_->send(sdk::Stop{});
        static_cast<void>(stopped);  // nothing more to do if it fails
    } catch (...) {                  // NOLINT(bugprone-empty-catch): noexcept; a failed stop has no one to tell
    }
}

}  // namespace ec
