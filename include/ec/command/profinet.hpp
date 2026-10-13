#pragma once
/// @file
/// Profinet service (ProfinetService), SDK manual section 2.2.10: the
/// controller's Profinet registers. The manual's request template for the
/// float setter names the int setter (a copy-paste error); its example uses
/// set_profinet_float_output_registers, as here.

#include <string_view>
#include <vector>

#include "ec/command/json.hpp"
#include "ec/command/reply.hpp"
#include "ec/command/request.hpp"

namespace ec::command {

/// §2.2.10.1 The int input registers: `length` from `address`.
struct GetProfinetIntInputRegisters {
    static constexpr std::string_view method = "get_profinet_int_input_registers";
    static constexpr Kind kind = Kind::Query;
    using Reply = std::vector<int>;
    int address = 0;  ///< addr, 0..31
    int length = 1;   ///< length, 1..32; address + length <= 32

    [[nodiscard]] Json params() const { return Json{{"addr", address}, {"length", length}}; }
};

/// §2.2.10.2 The int output registers: `length` from `address`.
struct GetProfinetIntOutputRegisters {
    static constexpr std::string_view method = "get_profinet_int_output_registers";
    static constexpr Kind kind = Kind::Query;
    using Reply = std::vector<int>;
    int address = 0;  ///< addr, 0..31
    int length = 1;   ///< length, 1..32; address + length <= 32

    [[nodiscard]] Json params() const { return Json{{"addr", address}, {"length", length}}; }
};

/// §2.2.10.3 The float input registers: `length` from `address`.
struct GetProfinetFloatInputRegisters {
    static constexpr std::string_view method = "get_profinet_float_input_registers";
    static constexpr Kind kind = Kind::Query;
    using Reply = std::vector<double>;
    int address = 0;  ///< addr, 0..31
    int length = 1;   ///< length, 1..32; address + length <= 32

    [[nodiscard]] Json params() const { return Json{{"addr", address}, {"length", length}}; }
};

/// §2.2.10.4 The float output registers: `length` from `address`.
struct GetProfinetFloatOutputRegisters {
    static constexpr std::string_view method = "get_profinet_float_output_registers";
    static constexpr Kind kind = Kind::Query;
    using Reply = std::vector<double>;
    int address = 0;  ///< addr, 0..31
    int length = 1;   ///< length, 1..32; address + length <= 32

    [[nodiscard]] Json params() const { return Json{{"addr", address}, {"length", length}}; }
};

/// §2.2.10.5 Sets int output registers from `address`, one per value. Remote mode only.
struct SetProfinetIntOutputRegisters {
    static constexpr std::string_view method = "set_profinet_int_output_registers";
    static constexpr Kind kind = Kind::Setting;
    using Reply = Ok;
    int address = 0;            ///< addr, 0..31; address + the number of values <= 32
    std::vector<int> values{};  ///< value; its size is sent as length

    [[nodiscard]] Json params() const { return Json{{"addr", address}, {"length", values.size()}, {"value", values}}; }
};

/// §2.2.10.6 Sets float output registers from `address`, one per value. Remote mode only.
struct SetProfinetFloatOutputRegisters {
    static constexpr std::string_view method = "set_profinet_float_output_registers";
    static constexpr Kind kind = Kind::Setting;
    using Reply = Ok;
    int address = 0;               ///< addr, 0..31; address + the number of values <= 32
    std::vector<double> values{};  ///< value; its size is sent as length

    [[nodiscard]] Json params() const { return Json{{"addr", address}, {"length", values.size()}, {"value", values}}; }
};

static_assert(Command<GetProfinetIntInputRegisters>);
static_assert(Command<GetProfinetIntOutputRegisters>);
static_assert(Command<GetProfinetFloatInputRegisters>);
static_assert(Command<GetProfinetFloatOutputRegisters>);
static_assert(Command<SetProfinetIntOutputRegisters>);
static_assert(Command<SetProfinetFloatOutputRegisters>);

}  // namespace ec::command
