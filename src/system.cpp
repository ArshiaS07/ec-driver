#include "ec/system.hpp"

#include <format>

#include "detail/connection.hpp"
#include "ec/sdk/param.hpp"
#include "ec/sdk/system.hpp"

namespace ec {

std::expected<RobotMode, Error> System::mode() const { return detail::send(session_, sdk::GetRobotMode{}); }

std::expected<std::string, Error> System::version() const { return detail::send(session_, sdk::GetSoftVersion{}); }

std::expected<std::vector<std::int64_t>, Error> System::alarms() const {
    return detail::send(session_, sdk::GetAlarmNum{});
}

std::expected<std::vector<std::string>, Error> System::log(const ec::log::Request& request) const {
    const auto c = session_.lock();
    if (!c) return std::unexpected(Error{Error::Kind::Disconnected, "log: the robot is gone"});
    auto lines = ec::log::read(c->host, request, c->log);
    if (!lines) {
        const auto kind =
            lines.error().kind == net::NetError::Kind::Timeout ? Error::Kind::Timeout : Error::Kind::Disconnected;
        return std::unexpected(
            Error{kind, std::format("log port {}:{}: {}", c->host, c->log.port, lines.error().message())});
    }
    return std::move(*lines);
}

std::expected<void, Error> System::set_clock(std::chrono::sys_seconds t) const {
    using namespace std::chrono;
    const auto day = floor<days>(t);
    const year_month_day date{day};
    const hh_mm_ss time{t - day};
    if (date.year() < year{1970} || date.year() > year{2037})
        return std::unexpected(Error{Error::Kind::Invalid, "set_system_time: 1970..2037"});
    const sdk::SetSystemTime call{
        .time = {static_cast<int>(date.year()), static_cast<int>(unsigned{date.month()}),
                 static_cast<int>(unsigned{date.day()}), static_cast<int>(time.hours().count()),
                 static_cast<int>(time.minutes().count()), static_cast<int>(time.seconds().count())}};
    const auto ok = detail::send(session_, call);
    if (!ok) return std::unexpected(ok.error());
    return {};
}

}  // namespace ec
