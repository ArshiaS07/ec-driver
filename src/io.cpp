#include "ec/io.hpp"

#include <format>
#include <string_view>

#include "detail/connection.hpp"
#include "detail/part.hpp"
#include "ec/sdk/io.hpp"

namespace ec {
namespace {

using detail::done;
using detail::on;

// The box's digital channels: 0..19, and 48..`last` on the flange [S 2.2.5.1-3].
std::expected<void, Error> digital(std::string_view what, int address, int last) {
    if ((address >= 0 && address <= 19) || (address >= 48 && address <= last)) return {};
    return std::unexpected(Error{Error::Kind::Invalid, std::format("{} {}: 0..19 or 48..{}", what, address, last)});
}

}  // namespace

std::expected<bool, Error> Io::input(int address) const {
    if (auto ok = digital("getInput address", address, 51); !ok) return std::unexpected(ok.error());
    return on(detail::send(session_, sdk::GetInput{.address = address}));
}

std::expected<bool, Error> Io::output(int address) const {
    if (auto ok = digital("getOutput address", address, 51); !ok) return std::unexpected(ok.error());
    return on(detail::send(session_, sdk::GetOutput{.address = address}));
}

std::expected<void, Error> Io::set_output(int address, bool value) const {
    if (auto ok = digital("setOutput address", address, 49); !ok) return std::unexpected(ok.error());
    return done(detail::send(session_, sdk::SetOutput{.address = address, .on = value}));
}

std::expected<double, Error> Io::analog_input(int address) const {
    if (auto ok = detail::in_range("getAnalogInput address", address, 0, 2); !ok) return std::unexpected(ok.error());
    return detail::send(session_, sdk::GetAnalogInput{.address = address});
}

std::expected<double, Error> Io::analog_output(int address) const {
    if (auto ok = detail::in_range("get_analog_output address", address, 0, 4); !ok) return std::unexpected(ok.error());
    return detail::send(session_, sdk::GetAnalogOutput{.address = address});
}

std::expected<void, Error> Io::set_analog_output(int address, double volts) const {
    if (auto ok = detail::in_range("setAnalogOutput address", address, 0, 4); !ok) return std::unexpected(ok.error());
    const double low = address == 4 ? 0.0 : -10.0;
    if (!(volts >= low && volts <= 10.0))
        return std::unexpected(Error{Error::Kind::Invalid,
                                     std::format("setAnalogOutput {}: {} V, not within {}..10", address, volts, low)});
    return done(detail::send(session_, sdk::SetAnalogOutput{.address = address, .value_v = volts}));
}

std::expected<bool, Error> Io::virtual_input(int address) const {
    if (auto ok = detail::in_range("getVirtualInput address", address, 0, 399); !ok) return std::unexpected(ok.error());
    return on(detail::send(session_, sdk::GetVirtualInput{.address = address}));
}

std::expected<bool, Error> Io::virtual_output(int address) const {
    if (auto ok = detail::in_range("getVirtualOutput address", address, 400, 1535); !ok)
        return std::unexpected(ok.error());
    return on(detail::send(session_, sdk::GetVirtualOutput{.address = address}));
}

std::expected<void, Error> Io::set_virtual_output(int address, bool value) const {
    if (auto ok = detail::in_range("setVirtualOutput address", address, 528, 799); !ok)
        return std::unexpected(ok.error());
    return done(detail::send(session_, sdk::SetVirtualOutput{.address = address, .on = value}));
}

std::expected<sdk::FlangeButtonFunction, Error> Io::flange_button(sdk::FlangeButton button) const {
    return detail::send(session_, sdk::CheckFlangeButton{.button = button});
}

}  // namespace ec
