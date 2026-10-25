#include "ec/robot.hpp"

#include <format>

#include "detail/connection.hpp"
#include "ec/sdk/client.hpp"

namespace ec {

std::expected<Robot, Error> Robot::connect(RobotOptions o) {
    auto client = sdk::Client::connect(
        o.host, {.port = o.command_port, .connect_timeout = o.connect_timeout, .reply_timeout = o.reply_timeout});
    if (!client)
        return std::unexpected(Error{Error::Kind::Disconnected, std::format("command port {}:{}: {}", o.host,
                                                                            o.command_port, client.error().message)});
    auto c = std::make_shared<detail::Connection>(std::move(*client), o.host);
    c->log.port = o.log_port;
    c->on_call = std::move(o.on_call);
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

}  // namespace ec
