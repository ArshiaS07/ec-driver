#pragma once
/// @file
/// Transparent transmission service, SDK manual section 2.2.7: position
/// streaming. The controller keeps a queue of joint targets; after the
/// lookahead time it takes one target per period and smooths the path through
/// them (JBI manual 2.4.11; docs/motion.md 2.4). Remote mode only.

#include <optional>
#include <string_view>

#include "ec/sdk/json.hpp"
#include "ec/sdk/reply.hpp"
#include "ec/sdk/request.hpp"

namespace ec::sdk {

/// §2.2.7.1 Starts streaming: an empty queue, consumed every `period_ms` once
/// `lookahead_ms` has passed after the first target.
struct TransparentTransmissionInit {
    static constexpr std::string_view method = "transparent_transmission_init";
    static constexpr Kind kind = Kind::Motion;
    using Reply = Ok;
    int lookahead_ms = 40;          ///< lookahead, [10, 1000]; longer than the period
    int period_ms = 8;              ///< t: sampling time, [2, 100]
    double smoothness = 0.1;        ///< smoothness: the smoothing gain, [0, 1]
    std::optional<bool> replies{};  ///< response_enable: whether TtPutServoJointToBuf replies; default yes

    [[nodiscard]] Json params() const {
        Json p{{"lookahead", lookahead_ms}, {"t", period_ms}, {"smoothness", smoothness}};
        if (replies) p["response_enable"] = *replies ? 1 : 0;
        return p;
    }
};

/// §2.2.7.2 Sets the current target directly. Deprecated: use TtPutServoJointToBuf.
struct TtSetCurrentServoJoint {
    static constexpr std::string_view method = "tt_set_current_servo_joint";
    static constexpr Kind kind = Kind::Motion;
    using Reply = Ok;
    JointsDeg target{};  ///< targetPos

    [[nodiscard]] Json params() const { return Json{{"targetPos", target}}; }
};

/// §2.2.7.3 Whether the controller is streaming.
struct GetTransparentTransmissionState {
    static constexpr std::string_view method = "get_transparent_transmission_state";
    static constexpr Kind kind = Kind::Query;
    using Reply = Flag;
};

/// §2.2.7.4 Appends one joint target to the queue: one per period. Replies only
/// if the stream was started with replies (the default). The manual also
/// accepts a pose (targetPose); untested on firmware V3.15.3, so not typed here.
struct TtPutServoJointToBuf {
    static constexpr std::string_view method = "tt_put_servo_joint_to_buf";
    static constexpr Kind kind = Kind::Motion;
    using Reply = Ok;
    JointsDeg target{};  ///< targetPos, each in [-360, 360]

    [[nodiscard]] Json params() const { return Json{{"targetPos", target}}; }
};

/// §2.2.7.5 Empties the queue and ends streaming.
struct TtClearServoJointBuf {
    static constexpr std::string_view method = "tt_clear_servo_joint_buf";
    static constexpr Kind kind = Kind::Motion;
    using Reply = Ok;
};

static_assert(Command<TransparentTransmissionInit>);
static_assert(Command<TtSetCurrentServoJoint>);
static_assert(Command<GetTransparentTransmissionState>);
static_assert(Command<TtPutServoJointToBuf>);
static_assert(Command<TtClearServoJointBuf>);

}  // namespace ec::sdk
