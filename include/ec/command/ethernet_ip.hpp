#pragma once
/// @file
/// EtherNet/IP registers, SDK manual section 2.2.12 (firmware 3.5.2 or later).
/// The manual's template for the int setter leaves `value` out; its example
/// sends it, as here.

#include <string_view>
#include <vector>

#include "ec/command/json.hpp"
#include "ec/command/reply.hpp"
#include "ec/command/request.hpp"

namespace ec::command {

/// §2.2.12.1 The int input registers: `length` from `address`.
struct GetEipIntInputRegisters {
    static constexpr std::string_view method = "get_eip_int_input_registers";
    static constexpr Kind kind = Kind::Query;
    using Reply = std::vector<int>;
    int address = 0;  ///< addr, 0..31
    int length = 1;   ///< length, 1..32; address + length <= 32

    [[nodiscard]] Json params() const { return Json{{"addr", address}, {"length", length}}; }
};

/// §2.2.12.2 The int output registers: `length` from `address`.
struct GetEipIntOutputRegisters {
    static constexpr std::string_view method = "get_eip_int_output_registers";
    static constexpr Kind kind = Kind::Query;
    using Reply = std::vector<int>;
    int address = 0;  ///< addr, 0..31
    int length = 1;   ///< length, 1..32; address + length <= 32

    [[nodiscard]] Json params() const { return Json{{"addr", address}, {"length", length}}; }
};

/// §2.2.12.3 The float input registers: `length` from `address`.
struct GetEipFloatInputRegisters {
    static constexpr std::string_view method = "get_eip_float_input_registers";
    static constexpr Kind kind = Kind::Query;
    using Reply = std::vector<double>;
    int address = 0;  ///< addr, 0..31
    int length = 1;   ///< length, 1..32; address + length <= 32

    [[nodiscard]] Json params() const { return Json{{"addr", address}, {"length", length}}; }
};

/// §2.2.12.4 The float output registers: `length` from `address`.
struct GetEipFloatOutputRegisters {
    static constexpr std::string_view method = "get_eip_float_output_registers";
    static constexpr Kind kind = Kind::Query;
    using Reply = std::vector<double>;
    int address = 0;  ///< addr, 0..31
    int length = 1;   ///< length, 1..32; address + length <= 32

    [[nodiscard]] Json params() const { return Json{{"addr", address}, {"length", length}}; }
};

/// §2.2.12.5 Sets int output registers from `address`, one per value. Remote mode only.
struct SetEipIntOutputRegisters {
    static constexpr std::string_view method = "set_eip_int_output_registers";
    static constexpr Kind kind = Kind::Setting;
    using Reply = Ok;
    int address = 0;            ///< addr, 0..31; address + the number of values <= 32
    std::vector<int> values{};  ///< value; its size is sent as length

    [[nodiscard]] Json params() const { return Json{{"addr", address}, {"length", values.size()}, {"value", values}}; }
};

/// §2.2.12.6 Sets float output registers from `address`, one per value. Remote mode only.
struct SetEipFloatOutputRegisters {
    static constexpr std::string_view method = "set_eip_float_output_registers";
    static constexpr Kind kind = Kind::Setting;
    using Reply = Ok;
    int address = 0;               ///< addr, 0..31; address + the number of values <= 32
    std::vector<double> values{};  ///< value; its size is sent as length

    [[nodiscard]] Json params() const { return Json{{"addr", address}, {"length", values.size()}, {"value", values}}; }
};

static_assert(Command<GetEipIntInputRegisters>);
static_assert(Command<GetEipIntOutputRegisters>);
static_assert(Command<GetEipFloatInputRegisters>);
static_assert(Command<GetEipFloatOutputRegisters>);
static_assert(Command<SetEipIntOutputRegisters>);
static_assert(Command<SetEipFloatOutputRegisters>);

}  // namespace ec::command
