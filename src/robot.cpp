#include "ec/robot.hpp"

#include <format>
#include <thread>

#include "detail/connection.hpp"
#include "ec/sdk/client.hpp"
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
    auto c = std::make_shared<detail::Connection>(std::move(*client), o.host);
    c->log.port = o.log_port;
    if (o.monitor) {
        c->monitor = std::make_unique<Monitor>(o.host, Monitor::Options{.receiver = {.port = o.monitor_port},
                                                                        .on_sample = std::move(o.on_sample),
                                                                        .on_state = std::move(o.on_state)});
        if (!c->monitor->next(0, o.first_state_within)) {
            const auto status = c->monitor->status().receiver;
            return std::unexpected(
                Error{Error::Kind::Disconnected,
                      std::format("monitor port {}:{}: no frame within {} ms{}", o.host, o.monitor_port,
                                  o.first_state_within.count(),
                                  status.last_error ? " (" + status.last_error->message() + ")" : std::string{})});
        }
    }
    return Robot(std::move(c));
}

Robot::Robot(Robot&&) noexcept = default;

Robot& Robot::operator=(Robot&& other) noexcept {
    if (this != &other) {
        end();
        c_ = std::move(other.c_);
    }
    return *this;
}

Robot::~Robot() { end(); }

void Robot::end() noexcept {
    // A motion should not outlive the code that can supervise it.
    if (c_ && c_->motion_open()) c_->stop();
}

std::expected<sdk::Json, Error> Robot::call_json(const sdk::Request& request, sdk::Kind kind) {
    return c_->call_checked(request, kind);
}

const Monitor& Robot::monitor() const noexcept { return *c_->monitor; }

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

}  // namespace ec
