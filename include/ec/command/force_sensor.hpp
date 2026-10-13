#pragma once
/// @file
/// External force sensor service, SDK manual section 2.2.13: the PC feeds the
/// controller a force sensor's readings (StartPushForce, PushExternalForce per
/// reading, StopPushForce).

#include <array>
#include <string_view>

#include "ec/command/json.hpp"
#include "ec/command/reply.hpp"
#include "ec/command/request.hpp"

namespace ec::command {

/// §2.2.13.1 Starts a transmission of force readings.
struct StartPushForce {
    static constexpr std::string_view method = "start_push_force";
    static constexpr Kind kind = Kind::Session;
    using Reply = Ok;
    bool replies = true;  ///< return_flag: whether PushExternalForce replies

    [[nodiscard]] Json params() const { return Json{{"return_flag", replies}}; }
};

/// §2.2.13.2 One reading, in the sensor's output frame. Note the manual's
/// units: forces in kg and torques in kg m (kgf and kgf m), not N and N m.
struct PushExternalForce {
    static constexpr std::string_view method = "push_external_force";
    static constexpr Kind kind = Kind::Session;
    using Reply = Ok;
    int index = 0;                          ///< sequence number, 0..65535
    std::array<double, 6> wrench_kg_kgm{};  ///< torque_arry (sic): fx, fy, fz [kg], tx, ty, tz [kg m]

    [[nodiscard]] Json params() const { return Json{{"index", index}, {"torque_arry", wrench_kg_kgm}}; }
};

/// §2.2.13.3 Ends the transmission.
struct StopPushForce {
    static constexpr std::string_view method = "stop_push_force";
    static constexpr Kind kind = Kind::Session;
    using Reply = Ok;
};

/// §2.2.13.4 Where the force data comes from: 0 and 1 internal, 2 the SDK,
/// 3 Lua, 4 the end effector.
struct GetForceCtrlMode {
    static constexpr std::string_view method = "get_force_ctrl_mode";
    static constexpr Kind kind = Kind::Query;
    using Reply = int;
};

/// §2.2.13.5 Force [N] and torque [N m] at the TCP from the external sensor;
/// only while one is connected. Firmware 3.9.2 or later.
struct GetTcpForceBySensor {
    static constexpr std::string_view method = "get_tcp_force_by_sensor";
    static constexpr Kind kind = Kind::Query;
    using Reply = std::array<double, 6>;
};

static_assert(Command<StartPushForce>);
static_assert(Command<PushExternalForce>);
static_assert(Command<StopPushForce>);
static_assert(Command<GetForceCtrlMode>);
static_assert(Command<GetTcpForceBySensor>);

}  // namespace ec::command
