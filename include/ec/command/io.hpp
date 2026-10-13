#pragma once
/// @file
/// IO service (IOService), SDK manual section 2.2.5. The 8056 stream carries
/// the digital and analog IO too (digital_ioInput, analog_ioInput, ...).

#include <cstdint>
#include <string_view>
#include <vector>

#include "ec/command/json.hpp"
#include "ec/command/reply.hpp"
#include "ec/command/request.hpp"

namespace ec::command {

/// §2.2.5.1 A digital input: addresses 0..19 and 48..51.
struct GetInput {
    static constexpr std::string_view method = "getInput";
    static constexpr Kind kind = Kind::Query;
    using Reply = Flag;
    int address = 0;  ///< addr

    [[nodiscard]] Json params() const { return Json{{"addr", address}}; }
};

/// §2.2.5.2 A digital output: addresses 0..19 and 48..51.
struct GetOutput {
    static constexpr std::string_view method = "getOutput";
    static constexpr Kind kind = Kind::Query;
    using Reply = Flag;
    int address = 0;  ///< addr

    [[nodiscard]] Json params() const { return Json{{"addr", address}}; }
};

/// §2.2.5.3 Sets a digital output: addresses 0..19 and 48..49. Remote mode only.
struct SetOutput {
    static constexpr std::string_view method = "setOutput";
    static constexpr Kind kind = Kind::Setting;
    using Reply = Ok;
    int address = 0;  ///< addr
    bool on = false;  ///< status

    [[nodiscard]] Json params() const { return Json{{"addr", address}, {"status", on ? 1 : 0}}; }
};

/// §2.2.5.4 A virtual input M: addresses 0..399.
struct GetVirtualInput {
    static constexpr std::string_view method = "getVirtualInput";
    static constexpr Kind kind = Kind::Query;
    using Reply = Flag;
    int address = 0;  ///< addr

    [[nodiscard]] Json params() const { return Json{{"addr", address}}; }
};

/// §2.2.5.5 A virtual output M: addresses 400..1535.
struct GetVirtualOutput {
    static constexpr std::string_view method = "getVirtualOutput";
    static constexpr Kind kind = Kind::Query;
    using Reply = Flag;
    int address = 400;  ///< addr

    [[nodiscard]] Json params() const { return Json{{"addr", address}}; }
};

/// §2.2.5.6 Sets a virtual output M: addresses 528..799. Remote mode only.
struct SetVirtualOutput {
    static constexpr std::string_view method = "setVirtualOutput";
    static constexpr Kind kind = Kind::Setting;
    using Reply = Ok;
    int address = 528;  ///< addr
    bool on = false;    ///< status

    [[nodiscard]] Json params() const { return Json{{"addr", address}, {"status", on ? 1 : 0}}; }
};

/// §2.2.5.7 16 * `length` virtual IOs from `address`, 16 per number.
struct GetRegisters {
    static constexpr std::string_view method = "getRegisters";
    static constexpr Kind kind = Kind::Query;
    using Reply = std::vector<std::int64_t>;
    int address = 0;  ///< addr, 0..1535
    int length = 1;   ///< len, 1..96; address + 16 * length <= 1536

    [[nodiscard]] Json params() const { return Json{{"addr", address}, {"len", length}}; }
};

/// §2.2.5.8 An analog input [V], -10..10: addresses 0..2.
struct GetAnalogInput {
    static constexpr std::string_view method = "getAnalogInput";
    static constexpr Kind kind = Kind::Query;
    using Reply = double;
    int address = 0;  ///< addr

    [[nodiscard]] Json params() const { return Json{{"addr", address}}; }
};

/// §2.2.5.9 An analog output [V]: addresses 0..4.
struct GetAnalogOutput {
    static constexpr std::string_view method = "get_analog_output";
    static constexpr Kind kind = Kind::Query;
    using Reply = double;
    int address = 0;  ///< addr

    [[nodiscard]] Json params() const { return Json{{"addr", address}}; }
};

/// §2.2.5.10 Sets an analog output [V]: -10..10 on 0..3, 0..10 on 4. Remote mode only.
struct SetAnalogOutput {
    static constexpr std::string_view method = "setAnalogOutput";
    static constexpr Kind kind = Kind::Setting;
    using Reply = Ok;
    int address = 0;       ///< addr
    double value_v = 0.0;  ///< value

    [[nodiscard]] Json params() const { return Json{{"addr", address}, {"value", value_v}}; }
};

static_assert(Command<GetInput>);
static_assert(Command<GetOutput>);
static_assert(Command<SetOutput>);
static_assert(Command<GetVirtualInput>);
static_assert(Command<GetVirtualOutput>);
static_assert(Command<SetVirtualOutput>);
static_assert(Command<GetRegisters>);
static_assert(Command<GetAnalogInput>);
static_assert(Command<GetAnalogOutput>);
static_assert(Command<SetAnalogOutput>);

}  // namespace ec::command
