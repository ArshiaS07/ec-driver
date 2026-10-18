#pragma once
/// @file
/// Movement service (MovementService), SDK manual section 2.2.3. These types
/// describe the calls; nothing here sends anything. Every motion command is
/// remote mode only, and point-to-point and path commands start only from
/// standstill: the manual says to send stop and wait for the arm to stop first.

#include <array>
#include <cstdint>
#include <expected>
#include <format>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

#include "ec/sdk/error.hpp"
#include "ec/sdk/json.hpp"
#include "ec/sdk/reply.hpp"
#include "ec/sdk/request.hpp"

namespace ec::sdk {

/// Ends a motion early, and moves on, once a condition holds: the cond_*
/// parameters of the motion commands.
struct StopCondition {
    enum class Source : std::uint8_t {
        DigitalInput = 0,   ///< an input X reaches `value`
        DigitalOutput = 1,  ///< an output Y reaches `value`
        Expression = 2,     ///< `expression` holds (the syntax of the JBI UNTIL condition)
    };
    Source source = Source::DigitalInput;  ///< cond_type
    int address = 0;                       ///< cond_num, 0..63; for inputs and outputs
    bool value = true;                     ///< cond_value; for inputs and outputs
    std::string expression{};              ///< cond_judgment; for Expression

    /// Adds the cond_* members to a command's parameters.
    void add_to(Json& params) const {
        params["cond_type"] = static_cast<int>(source);
        if (source == Source::Expression) {
            params["cond_judgment"] = expression;
        } else {
            params["cond_num"] = address;
            params["cond_value"] = value ? 1 : 0;
        }
    }
};

/// How a Cartesian speed is given (speed_type).
enum class SpeedType : std::uint8_t {
    Linear = 0,              ///< V: TCP speed [mm/s], 1..3000
    Rotational = 1,          ///< VR: rotational speed [deg/s], 1..300
    AbsoluteLinear = 2,      ///< AV: TCP speed [mm/s] within the controller's linear speed limits
    AbsoluteRotational = 3,  ///< AVR: rotational speed [deg/s] within its rotational speed limits
};

/// The interpolation of a waypoint (moveType, §2.2.3.5).
enum class PathMoveType : std::uint8_t { Joint = 0, Line = 1, RotateAboutTool = 2, Arc = 3 };

/// A program's run state (§2.2.3.14).
enum class JbiRunState : std::uint8_t { Stopped = 0, Paused = 1, EmergencyStop = 2, Running = 3, Error = 4 };
[[nodiscard]] constexpr bool is_valid(JbiRunState s) noexcept { return static_cast<int>(s) <= 4; }

/// The program file being run and its state (§2.2.3.14).
struct JbiState {
    std::string name{};  ///< jbiName
    JbiRunState state = JbiRunState::Stopped;
    bool operator==(const JbiState&) const = default;
};

namespace detail {
/// The members the point-to-point commands share, in the manual's order.
inline void add_profile(Json& p, std::optional<int> acc_pct, std::optional<int> dec_pct,
                        const std::optional<StopCondition>& until) {
    if (acc_pct) p["acc"] = *acc_pct;
    if (dec_pct) p["dec"] = *dec_pct;
    if (until) until->add_to(p);
}
}  // namespace detail

/// §2.2.3.1 Joint motion to a target position. The reply means started; the
/// motion has ended when the robot state is back to stop.
struct MoveByJoint {
    static constexpr std::string_view method = "moveByJoint";
    static constexpr Kind kind = Kind::Motion;
    using Reply = Ok;
    JointsDeg target{};                    ///< targetPos, each in [-360, 360]
    double speed_pct = 0.0;                ///< speed, [0.01, 100] % of the joints' maximum speed
    std::optional<int> acc_pct{};          ///< acc, [1, 100] %; controller default 80
    std::optional<int> dec_pct{};          ///< dec, [1, 100] %; controller default: acc
    std::optional<StopCondition> until{};  ///< cond_*: end early when this holds

    [[nodiscard]] Json params() const {
        Json p;  // members in the manual's order
        p["targetPos"] = target;
        p["speed"] = speed_pct;
        detail::add_profile(p, acc_pct, dec_pct, until);
        return p;
    }
};

/// §2.2.3.2 Straight-line TCP motion to the pose of a joint position. The
/// reply means started; ended when the robot state is back to stop.
struct MoveByLine {
    static constexpr std::string_view method = "moveByLine";
    static constexpr Kind kind = Kind::Motion;
    using Reply = Ok;
    JointsDeg target{};                        ///< targetPos, each in [-360, 360]
    SpeedType speed_type = SpeedType::Linear;  ///< speed_type, always sent
    double speed = 0.0;                        ///< speed, in the unit speed_type gives
    std::optional<int> acc_pct{};              ///< acc, [1, 100] %; controller default 80
    std::optional<int> dec_pct{};              ///< dec, [1, 100] %; controller default: acc
    std::optional<StopCondition> until{};      ///< cond_*

    [[nodiscard]] Json params() const {
        Json p;
        p["targetPos"] = target;
        p["speed_type"] = static_cast<int>(speed_type);
        p["speed"] = speed;
        detail::add_profile(p, acc_pct, dec_pct, until);
        return p;
    }
};

/// §2.2.3.3 Circular-arc TCP motion through the pose of `middle` to that of `target`.
struct MoveByArc {
    static constexpr std::string_view method = "moveByArc";
    static constexpr Kind kind = Kind::Motion;
    using Reply = Ok;
    JointsDeg middle{};                        ///< midPos
    JointsDeg target{};                        ///< targetPos
    SpeedType speed_type = SpeedType::Linear;  ///< speed_type, always sent
    double speed = 0.0;                        ///< speed, in the unit speed_type gives
    std::optional<int> acc_pct{};              ///< acc, [1, 100] %
    std::optional<int> dec_pct{};              ///< dec, [1, 100] %
    std::optional<StopCondition> until{};      ///< cond_*

    [[nodiscard]] Json params() const {
        Json p;
        p["midPos"] = middle;
        p["targetPos"] = target;
        p["speed_type"] = static_cast<int>(speed_type);
        p["speed"] = speed;
        detail::add_profile(p, acc_pct, dec_pct, until);
        return p;
    }
};

/// §2.2.3.4 Rotation about the TCP to the orientation of a joint position.
/// Deprecated by the manual; its stop condition has no expression form.
struct MoveByRotate {
    static constexpr std::string_view method = "moveByRotate";
    static constexpr Kind kind = Kind::Motion;
    using Reply = Ok;
    JointsDeg target{};                                    ///< targetPos
    SpeedType speed_type = SpeedType::AbsoluteRotational;  ///< speed_type, always sent
    double speed = 0.0;                                    ///< speed, in the unit speed_type gives
    std::optional<int> acc_pct{};                          ///< acc, [1, 100] %
    std::optional<int> dec_pct{};                          ///< dec, [1, 100] %
    std::optional<StopCondition> until{};                  ///< cond_type 0 or 1 only

    [[nodiscard]] Json params() const {
        Json p;
        p["targetPos"] = target;
        p["speed_type"] = static_cast<int>(speed_type);
        p["speed"] = speed;
        detail::add_profile(p, acc_pct, dec_pct, until);
        return p;
    }
};

/// §2.2.3.5 Appends a waypoint to the path that MoveByPath runs.
struct AddPathPoint {
    static constexpr std::string_view method = "addPathPoint";
    static constexpr Kind kind = Kind::Motion;
    using Reply = Ok;
    JointsDeg point{};                             ///< wayPoint
    PathMoveType move_type = PathMoveType::Joint;  ///< moveType
    /// speed: for Joint, [1, 100] % of the joints' maximum; otherwise in the unit
    /// speed_type gives.
    double speed = 0.0;
    SpeedType speed_type = SpeedType::Linear;  ///< speed_type; sent for Cartesian move types only
    std::optional<int> acc_pct{};              ///< acc, [1, 100] %
    std::optional<int> dec_pct{};              ///< dec, [1, 100] %
    double blend_radius_mm = 0.0;              ///< circular_radius, [0, 2608]; 0 for the last point
    std::optional<StopCondition> until{};      ///< cond_*

    [[nodiscard]] Json params() const {
        Json p;
        p["wayPoint"] = point;
        p["moveType"] = static_cast<int>(move_type);
        if (move_type != PathMoveType::Joint) p["speed_type"] = static_cast<int>(speed_type);
        p["speed"] = speed;
        if (acc_pct) p["acc"] = *acc_pct;
        if (dec_pct) p["dec"] = *dec_pct;
        p["circular_radius"] = blend_radius_mm;
        if (until) until->add_to(p);
        return p;
    }
};

/// §2.2.3.6 Clears the path.
struct ClearPathPoint {
    static constexpr std::string_view method = "clearPathPoint";
    static constexpr Kind kind = Kind::Motion;
    using Reply = Ok;
};

/// §2.2.3.7 Runs the path; replies with its number of waypoints. The controller
/// answers -1 on failure, which decodes as Kind::Refused.
struct MoveByPath {
    static constexpr std::string_view method = "moveByPath";
    static constexpr Kind kind = Kind::Motion;
    using Reply = int;

    static std::expected<int, Error> decode(const Json& r) {
        const auto n = ReplyDecoder<int>::decode(r);
        if (n && *n < 0) return std::unexpected(Error{Error::Kind::Refused, 0, "controller returned -1"});
        return n;
    }
};

/// §2.2.3.8 Jogs one axis, or along one direction of the selected coordinate
/// system, for as long as the command is repeated: the arm stops 1 s after the
/// last one, or at once on Stop.
struct Jog {
    static constexpr std::string_view method = "jog";
    static constexpr Kind kind = Kind::Motion;
    using Reply = Ok;
    int index = 0;                      ///< 0..11: axis or direction, two per axis (minus, then plus)
    std::optional<double> speed_pct{};  ///< speed, [0.05, 100] % of the manual speed

    [[nodiscard]] Json params() const {
        Json p{{"index", index}};
        if (speed_pct) p["speed"] = *speed_pct;
        return p;
    }
};

/// §2.2.3.9 Stops the arm, whatever moves it.
struct Stop {
    static constexpr std::string_view method = "stop";
    static constexpr Kind kind = Kind::Motion;
    using Reply = Ok;
};

/// §2.2.3.10 Runs (or resumes) the program.
struct Run {
    static constexpr std::string_view method = "run";
    static constexpr Kind kind = Kind::Motion;
    using Reply = Ok;
};

/// §2.2.3.11 Pauses the program.
struct Pause {
    static constexpr std::string_view method = "pause";
    static constexpr Kind kind = Kind::Motion;
    using Reply = Ok;
};

/// §2.2.3.12 Whether a program file exists.
struct CheckJbiExist {
    static constexpr std::string_view method = "checkJbiExist";
    static constexpr Kind kind = Kind::Query;
    using Reply = Flag;
    std::string filename{};  ///< filename

    [[nodiscard]] Json params() const { return Json{{"filename", filename}}; }
};

/// §2.2.3.13 Runs a program file; only from standstill.
struct RunJbi {
    static constexpr std::string_view method = "runJbi";
    static constexpr Kind kind = Kind::Motion;
    using Reply = Ok;
    std::string filename{};  ///< filename

    [[nodiscard]] Json params() const { return Json{{"filename", filename}}; }
};

/// §2.2.3.14 The program being run and its state, from {"jbiName", "runState"}.
struct GetJbiState {
    static constexpr std::string_view method = "getJbiState";
    static constexpr Kind kind = Kind::Query;
    using Reply = JbiState;

    static std::expected<JbiState, Error> decode(const Json& r) {
        if (!r.is_object() || !r.contains("jbiName") || !r.contains("runState"))
            return bad_reply(std::format("expected jbiName and runState, got {}", r.dump()));
        auto name = ReplyDecoder<std::string>::decode(r["jbiName"]);
        if (!name) return std::unexpected(name.error());
        const auto state = ReplyDecoder<JbiRunState>::decode(r["runState"]);
        if (!state) return std::unexpected(state.error());
        return JbiState{std::move(*name), *state};
    }
};

/// §2.2.3.15 Sets the play/remote speed [%], which scales point-to-point and path motion.
struct SetSpeed {
    static constexpr std::string_view method = "setSpeed";
    static constexpr Kind kind = Kind::Setting;
    using Reply = Ok;
    double value_pct = 100.0;  ///< value, [0.05, 100]

    [[nodiscard]] Json params() const { return Json{{"value", value_pct}}; }
};

/// §2.2.3.16 Joint velocity command, valid for `duration_s`: the newest replaces
/// the previous without slowing down; when it expires the arm stops.
struct MoveBySpeedj {
    static constexpr std::string_view method = "moveBySpeedj";
    static constexpr Kind kind = Kind::Motion;
    using Reply = Ok;
    std::array<double, 6> velocity_deg_s{};  ///< vj
    int acc_deg_s2 = 0;                      ///< acc, > 0
    double duration_s = 0.0;                 ///< t, > 0

    [[nodiscard]] Json params() const { return Json{{"vj", velocity_deg_s}, {"acc", acc_deg_s2}, {"t", duration_s}}; }
};

/// §2.2.3.17 Ends joint velocity motion, decelerating at `acc_deg_s2`.
struct Stopj {
    static constexpr std::string_view method = "stopj";
    static constexpr Kind kind = Kind::Motion;
    using Reply = Ok;
    int acc_deg_s2 = 0;  ///< acc, > 0

    [[nodiscard]] Json params() const { return Json{{"acc", acc_deg_s2}}; }
};

/// §2.2.3.18 TCP velocity command, valid for `duration_s`, like MoveBySpeedj.
struct MoveBySpeedl {
    static constexpr std::string_view method = "moveBySpeedl";
    static constexpr Kind kind = Kind::Motion;
    using Reply = Ok;
    std::array<double, 6> velocity{};     ///< v: x, y, z [mm/s], then rx, ry, rz [deg/s]
    int acc_mm_s2 = 0;                    ///< acc, > 0
    std::optional<int> acc_rot_deg_s2{};  ///< arot, > 0
    double duration_s = 0.0;              ///< t, > 0

    [[nodiscard]] Json params() const {
        Json p;
        p["v"] = velocity;
        p["acc"] = acc_mm_s2;
        if (acc_rot_deg_s2) p["arot"] = *acc_rot_deg_s2;
        p["t"] = duration_s;
        return p;
    }
};

/// §2.2.3.19 Ends TCP velocity motion.
struct Stopl {
    static constexpr std::string_view method = "stopl";
    static constexpr Kind kind = Kind::Motion;
    using Reply = Ok;
    int acc_mm_s2 = 0;                    ///< acc, > 0
    std::optional<int> acc_rot_deg_s2{};  ///< arot, > 0

    [[nodiscard]] Json params() const {
        Json p{{"acc", acc_mm_s2}};
        if (acc_rot_deg_s2) p["arot"] = *acc_rot_deg_s2;
        return p;
    }
};

/// §2.2.3.20 Straight-line motion to a pose given in a user frame. Firmware 2.16.2 or later.
struct MoveByLineCoord {
    static constexpr std::string_view method = "moveByLineCoord";
    static constexpr Kind kind = Kind::Motion;
    using Reply = Ok;
    PoseMmRad target{};                                ///< targetUserPose, in the user frame
    SpeedType speed_type = SpeedType::AbsoluteLinear;  ///< speed_type, always sent
    double speed = 0.0;                                ///< speed, in the unit speed_type gives
    std::optional<int> acc_pct{};                      ///< acc, [1, 100] %
    std::optional<int> dec_pct{};                      ///< dec, [1, 100] %
    std::optional<PoseMmRad> user_frame{};             ///< user_coord; none: the current frame
    std::optional<StopCondition> until{};              ///< cond_*

    [[nodiscard]] Json params() const {
        Json p;
        p["targetUserPose"] = target;
        p["speed_type"] = static_cast<int>(speed_type);
        p["speed"] = speed;
        if (acc_pct) p["acc"] = *acc_pct;
        if (dec_pct) p["dec"] = *dec_pct;
        if (user_frame) p["user_coord"] = *user_frame;
        if (until) until->add_to(p);
        p["unit_type"] = 1;
        return p;
    }
};

/// §2.2.3.21 Calibrates the encoders' zero positions. Changes the robot's
/// calibration: never send it casually.
struct CalibrateEncoderZeroPosition {
    static constexpr std::string_view method = "calibrate_encoder_zero_position";
    static constexpr Kind kind = Kind::Action;
    using Reply = Ok;
};

static_assert(Command<MoveByJoint>);
static_assert(Command<MoveByLine>);
static_assert(Command<MoveByArc>);
static_assert(Command<MoveByRotate>);
static_assert(Command<AddPathPoint>);
static_assert(Command<ClearPathPoint>);
static_assert(Command<MoveByPath>);
static_assert(Command<Jog>);
static_assert(Command<Stop>);
static_assert(Command<Run>);
static_assert(Command<Pause>);
static_assert(Command<CheckJbiExist>);
static_assert(Command<RunJbi>);
static_assert(Command<GetJbiState>);
static_assert(Command<SetSpeed>);
static_assert(Command<MoveBySpeedj>);
static_assert(Command<Stopj>);
static_assert(Command<MoveBySpeedl>);
static_assert(Command<Stopl>);
static_assert(Command<MoveByLineCoord>);
static_assert(Command<CalibrateEncoderZeroPosition>);

}  // namespace ec::sdk
