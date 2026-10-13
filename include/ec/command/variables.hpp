#pragma once
/// @file
/// Variable service (VarService), SDK manual section 2.2.6: the controller's
/// system variables B (int), I (short), D (double), P (joint positions) and V
/// (poses), addresses 0..255.

#include <array>
#include <cstdint>
#include <string_view>

#include "ec/command/json.hpp"
#include "ec/command/reply.hpp"
#include "ec/command/request.hpp"

namespace ec::command {

/// §2.2.6.1 B variable, 0..2147483647.
struct GetSysVarB {
    static constexpr std::string_view method = "getSysVarB";
    static constexpr Kind kind = Kind::Query;
    using Reply = std::int64_t;
    int address = 0;  ///< addr

    [[nodiscard]] Json params() const { return Json{{"addr", address}}; }
};

/// §2.2.6.2 Sets a B variable. Remote mode only.
struct SetSysVarB {
    static constexpr std::string_view method = "setSysVarB";
    static constexpr Kind kind = Kind::Setting;
    using Reply = Ok;
    int address = 0;         ///< addr
    std::int64_t value = 0;  ///< value, 0..2147483647

    [[nodiscard]] Json params() const { return Json{{"addr", address}, {"value", value}}; }
};

/// §2.2.6.3 I variable, -32767..32767.
struct GetSysVarI {
    static constexpr std::string_view method = "getSysVarI";
    static constexpr Kind kind = Kind::Query;
    using Reply = int;
    int address = 0;  ///< addr

    [[nodiscard]] Json params() const { return Json{{"addr", address}}; }
};

/// §2.2.6.4 Sets an I variable. Remote mode only.
struct SetSysVarI {
    static constexpr std::string_view method = "setSysVarI";
    static constexpr Kind kind = Kind::Setting;
    using Reply = Ok;
    int address = 0;  ///< addr
    int value = 0;    ///< value, -32767..32767

    [[nodiscard]] Json params() const { return Json{{"addr", address}, {"value", value}}; }
};

/// §2.2.6.5 D variable, -1e9..1e9.
struct GetSysVarD {
    static constexpr std::string_view method = "getSysVarD";
    static constexpr Kind kind = Kind::Query;
    using Reply = double;
    int address = 0;  ///< addr

    [[nodiscard]] Json params() const { return Json{{"addr", address}}; }
};

/// §2.2.6.6 Sets a D variable. Remote mode only.
struct SetSysVarD {
    static constexpr std::string_view method = "setSysVarD";
    static constexpr Kind kind = Kind::Setting;
    using Reply = Ok;
    int address = 0;     ///< addr
    double value = 0.0;  ///< value

    [[nodiscard]] Json params() const { return Json{{"addr", address}, {"value", value}}; }
};

/// §2.2.6.7 Whether a P variable is enabled.
struct GetSysVarPState {
    static constexpr std::string_view method = "getSysVarPState";
    static constexpr Kind kind = Kind::Query;
    using Reply = Flag;
    int address = 0;  ///< addr

    [[nodiscard]] Json params() const { return Json{{"addr", address}}; }
};

/// §2.2.6.8 P variable: joint positions [deg] of all eight axes.
struct GetSysVarP {
    static constexpr std::string_view method = "getSysVarP";
    static constexpr Kind kind = Kind::Query;
    using Reply = std::array<double, 8>;
    int address = 0;  ///< addr

    [[nodiscard]] Json params() const { return Json{{"addr", address}}; }
};

/// §2.2.6.9 Sets a P variable. Remote mode only. Firmware 2.15.2 or later.
struct SetSysVarP {
    static constexpr std::string_view method = "setSysVarP";
    static constexpr Kind kind = Kind::Setting;
    using Reply = Ok;
    int address = 0;     ///< addr
    JointsDeg joints{};  ///< pos (the vendor's example; the template's "pos:" is a typo)

    [[nodiscard]] Json params() const { return Json{{"addr", address}, {"pos", joints}}; }
};

/// §2.2.6.10 V variable: a pose, x, y, z [mm], rx, ry, rz [rad].
struct GetSysVarV {
    static constexpr std::string_view method = "getSysVarV";
    static constexpr Kind kind = Kind::Query;
    using Reply = PoseMmRad;
    int address = 0;  ///< addr

    [[nodiscard]] Json params() const { return Json{{"addr", address}}; }
};

/// §2.2.6.11 Sets a V variable. Remote mode only. The vendor's examples use
/// both "pose" and "pos" for the value; "pose" is sent (unverified).
struct SetSysVarV {
    static constexpr std::string_view method = "setSysVarV";
    static constexpr Kind kind = Kind::Setting;
    using Reply = Ok;
    int address = 0;   ///< addr
    PoseMmRad pose{};  ///< pose

    [[nodiscard]] Json params() const { return Json{{"addr", address}, {"pose", pose}}; }
};

/// §2.2.6.12 Saves the variables. Remote mode only. Firmware 2.15.2 or later.
struct SaveVarData {
    static constexpr std::string_view method = "save_var_data";
    static constexpr Kind kind = Kind::Action;
    using Reply = Ok;
};

static_assert(Command<GetSysVarB>);
static_assert(Command<SetSysVarB>);
static_assert(Command<GetSysVarI>);
static_assert(Command<SetSysVarI>);
static_assert(Command<GetSysVarD>);
static_assert(Command<SetSysVarD>);
static_assert(Command<GetSysVarPState>);
static_assert(Command<GetSysVarP>);
static_assert(Command<SetSysVarP>);
static_assert(Command<GetSysVarV>);
static_assert(Command<SetSysVarV>);
static_assert(Command<SaveVarData>);

}  // namespace ec::command
