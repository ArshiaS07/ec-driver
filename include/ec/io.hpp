#pragma once
/// @file
/// ec::Io: the control box's I/O and the controller's virtual bits (SDK manual
/// 2.2.5, 2.2.2.52; docs/ec66.md §4.3). Each operation is one call on the
/// command connection (8055). Digital and analog values are also on the
/// monitoring path: robot.monitor().latest()->io. Fieldbus registers
/// (Profinet, EtherNet/IP, getRegisters) stay typed calls (robot.call).
///
///     robot.io().set_output(3, true);

#include <expected>
#include <memory>

#include "ec/error.hpp"
#include "ec/sdk/param.hpp"

namespace ec {

namespace detail {
struct Connection;
}

/// A handle onto a robot's I/O: cheap to copy, usable from any thread. Once
/// its Robot is gone, every operation fails with Disconnected. Addresses
/// outside the manual's are Invalid with nothing sent; the settings need
/// REMOTE mode and are Busy while a motion is open.
class Io {
public:
    /// getInput [S 2.2.5.1]: digital input 0..19 or 48..51.
    [[nodiscard]] std::expected<bool, Error> input(int address) const;
    /// getOutput [S 2.2.5.2]: digital output 0..19 or 48..51.
    [[nodiscard]] std::expected<bool, Error> output(int address) const;
    /// setOutput [S 2.2.5.3]: digital output 0..19 or 48..49.
    [[nodiscard]] std::expected<void, Error> set_output(int address, bool on) const;

    /// getAnalogInput [S 2.2.5.8]: input 0..2 [V], -10..10.
    [[nodiscard]] std::expected<double, Error> analog_input(int address) const;
    /// get_analog_output [S 2.2.5.9]: output 0..4 [V].
    [[nodiscard]] std::expected<double, Error> analog_output(int address) const;
    /// setAnalogOutput [S 2.2.5.10]: -10..10 V on outputs 0..3, 0..10 V on 4.
    [[nodiscard]] std::expected<void, Error> set_analog_output(int address, double volts) const;

    /// getVirtualInput [S 2.2.5.4]: M0..M399.
    [[nodiscard]] std::expected<bool, Error> virtual_input(int address) const;
    /// getVirtualOutput [S 2.2.5.5]: M400..M1535; the controller's own flags
    /// among them (M472 precise, M473 initialised, M476 ready: docs/ec66.md §4.3).
    [[nodiscard]] std::expected<bool, Error> virtual_output(int address) const;
    /// setVirtualOutput [S 2.2.5.6]: M528..M799.
    [[nodiscard]] std::expected<void, Error> set_virtual_output(int address, bool on) const;

    /// checkFlangeButton [S 2.2.2.52]: what an end-effector button does.
    [[nodiscard]] std::expected<sdk::FlangeButtonFunction, Error> flange_button(sdk::FlangeButton button) const;

private:
    friend class Robot;
    explicit Io(std::weak_ptr<detail::Connection> session) noexcept : session_(std::move(session)) {}
    std::weak_ptr<detail::Connection> session_;
};

}  // namespace ec
