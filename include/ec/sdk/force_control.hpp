#pragma once
/// @file
/// Force control service, SDK manual section 2.2.16. Force control modifies how
/// the other motion families move (docs/sdk_calls.md); not a family of its own.

#include <array>
#include <optional>
#include <string_view>

#include "ec/sdk/json.hpp"
#include "ec/sdk/reply.hpp"
#include "ec/sdk/request.hpp"

namespace ec::sdk {

/// §2.2.16.1 Enables force control. Each parameter left out keeps the
/// controller's default.
struct StartForceMode {
    static constexpr std::string_view method = "start_force_mode";
    static constexpr Kind kind = Kind::Motion;
    using Reply = Ok;
    /// mode: 0 fixed, 1 point, 2 motion, 3 TCP, 4 orientation; default 0.
    std::optional<int> mode{};
    /// arr_frame: the force-control frame, mm and rad; default the base frame.
    std::optional<PoseMmRad> frame{};
    /// arr_optional, per axis: 0 motion control, 1 force tracking, 2 fixed,
    /// 3 floating, 4 spring, 5 floating and motion, 6 spring and motion.
    std::optional<std::array<int, 6>> compliance{};
    /// arr_torque: target forces [N] (EA66: -60..60), then torques [N m] (-1.5..1.5).
    std::optional<std::array<double, 6>> target_wrench{};
    /// arr_speed: speed limits [mm/s] (0..200), then [deg/s] (0..11); default 100, 5.73.
    std::optional<std::array<double, 6>> speed_limits{};

    [[nodiscard]] Json params() const {
        Json p = Json::object();
        if (mode) p["mode"] = *mode;
        if (frame) p["arr_frame"] = *frame;
        if (compliance) p["arr_optional"] = *compliance;
        if (target_wrench) p["arr_torque"] = *target_wrench;
        if (speed_limits) p["arr_speed"] = *speed_limits;
        return p;
    }
};

/// §2.2.16.2 Disables force control. Firmware 3.6.2 or later.
struct EndForceMode {
    static constexpr std::string_view method = "end_force_mode";
    static constexpr Kind kind = Kind::Motion;
    using Reply = Ok;
};

/// §2.2.16.3 Whether force control is on. Firmware 3.6.2 or later.
struct GetForceModeState {
    static constexpr std::string_view method = "get_force_mode_state";
    static constexpr Kind kind = Kind::Query;
    using Reply = bool;
};

/// §2.2.16.4 Zeroes the force sensor. Firmware 3.6.2 or later.
struct ZeroForceSensor {
    static constexpr std::string_view method = "zero_force_sensor";
    static constexpr Kind kind = Kind::Action;
    using Reply = Ok;
};

/// §2.2.16.5 Starts force-seeking motion. Remote mode only.
struct FcAct {
    static constexpr std::string_view method = "fc_act";
    static constexpr Kind kind = Kind::Motion;
    using Reply = Ok;
};

/// §2.2.16.6 Ends force-seeking motion. Remote mode only. Firmware 3.9.2 or later.
struct FcDeact {
    static constexpr std::string_view method = "fc_deact";
    static constexpr Kind kind = Kind::Motion;
    using Reply = Ok;
};

static_assert(Command<StartForceMode>);
static_assert(Command<EndForceMode>);
static_assert(Command<GetForceModeState>);
static_assert(Command<ZeroForceSensor>);
static_assert(Command<FcAct>);
static_assert(Command<FcDeact>);

}  // namespace ec::sdk
