#pragma once
/// @file
/// ec::System: the controller as a whole: its mode, version, alarms, log and
/// clock (SDK manual 2.2.2, 2.2.8, chapter 4; docs/ec66.md §5.1, §5.9). Each
/// operation is one call on the command connection (8055), except log(),
/// which reads the log port (8058) on a connection of its own.
///
///     auto system = robot.system();
///     for (const auto& line : system.log({.level = ec::log::Level::Error, .lines = 20}).value()) ...

#include <chrono>
#include <cstdint>
#include <expected>
#include <memory>
#include <string>
#include <vector>

#include "ec/error.hpp"
#include "ec/log/reader.hpp"
#include "ec/status.hpp"

namespace ec {

namespace detail {
struct Connection;
}

/// A handle onto a robot's controller: cheap to copy, usable from any thread
/// (calls wait for each other). Once its Robot is gone, every operation fails
/// with Disconnected. Only set_clock() is Busy while a motion is open.
class System {
public:
    /// getRobotMode [S 2.2.2.2]: the key switch, the operator's (also on the
    /// monitoring path: robot.monitor().latest()->system.mode).
    [[nodiscard]] std::expected<RobotMode, Error> mode() const;
    /// getSoftVersion [S 2.2.8.1]: the controller's software, e.g. "V3.15.3".
    [[nodiscard]] std::expected<std::string, Error> version() const;
    /// getAlarmNum [S 2.2.2.56]: the numbers of the last five alarms, as the
    /// controller sends them (shape and meaning unverified on the EA66).
    [[nodiscard]] std::expected<std::vector<std::int64_t>, Error> alarms() const;
    /// The controller's log (port 8058, ec::log::read on the robot's host):
    /// its lines, in its format. Disconnected or Timeout if the port fails.
    [[nodiscard]] std::expected<std::vector<std::string>, Error> log(const ec::log::Request& request = {}) const;
    /// set_system_time [S 2.2.2.74]: the controller's clock, sent as the UTC
    /// calendar fields of `t` (1970..2037, else Invalid; firmware 3.10.2 or
    /// later). Whether the controller takes them as UTC is unverified: compare
    /// the monitoring path's controller_time with this PC's clock afterwards.
    [[nodiscard]] std::expected<void, Error> set_clock(std::chrono::sys_seconds t) const;

private:
    friend class Robot;
    explicit System(std::weak_ptr<detail::Connection> session) noexcept : session_(std::move(session)) {}
    std::weak_ptr<detail::Connection> session_;
};

}  // namespace ec
