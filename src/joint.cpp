#include "ec/joint.hpp"

#include <cstddef>
#include <format>

#include "detail/connection.hpp"
#include "detail/part.hpp"
#include "ec/sdk/backdrive.hpp"
#include "ec/sdk/param.hpp"
#include "ec/sdk/system.hpp"
#include "ec/units.hpp"

namespace ec {
namespace {

constexpr double kPerMille = 1e-3;

// This joint's entry of a six-joint reply, scaled.
template <sdk::Command C>
std::expected<double, Error> pick(const std::weak_ptr<detail::Connection>& session, int number, double scale) {
    if (auto ok = detail::in_range("joint", number, 1, 6); !ok) return std::unexpected(ok.error());
    const auto all = detail::send(session, C{});
    if (!all) return std::unexpected(all.error());
    return (*all)[static_cast<std::size_t>(number - 1)] * scale;
}

}  // namespace

std::expected<double, Error> Joint::position() const {
    return pick<sdk::GetJointPos>(session_, number_, deg_to_rad(1.0));
}

std::expected<double, Error> Joint::velocity() const {
    return pick<sdk::GetJointSpeed>(session_, number_, deg_to_rad(1.0));
}

std::expected<double, Error> Joint::torque() const { return pick<sdk::GetJointTorques>(session_, number_, 1.0); }

std::expected<double, Error> Joint::torque_of_rated() const {
    return pick<sdk::GetMotorTorque>(session_, number_, kPerMille);
}

std::expected<double, Error> Joint::temperature() const { return pick<sdk::GetJointTemp>(session_, number_, 1.0); }

std::expected<bool, Error> Joint::brake_released() const {
    if (auto ok = detail::in_range("joint", number_, 1, 6); !ok) return std::unexpected(ok.error());
    const auto all = detail::send(session_, sdk::GetServoBrakeOffStatus{});
    if (!all) return std::unexpected(all.error());
    const auto i = static_cast<std::size_t>(number_ - 1);
    if (all->size() <= i)
        return std::unexpected(Error{
            Error::Kind::Command, std::format("get_servo_brake_off_status: {} entries, no J{}", all->size(), number_)});
    return (*all)[i] != 0;
}

std::expected<std::string, Error> Joint::firmware() const {
    if (auto ok = detail::in_range("joint", number_, 1, 6); !ok) return std::unexpected(ok.error());
    return detail::send(session_, sdk::GetJointVersion{.axis = number_ - 1});
}

}  // namespace ec
