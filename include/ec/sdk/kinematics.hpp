#pragma once
/// @file
/// Kinematics service (KinematicsService), SDK manual section 2.2.4: pure
/// computations on the controller, safe at any time. Every call sends
/// unit_type 1, so angles in poses are radians.

#include <optional>
#include <string_view>

#include "ec/sdk/json.hpp"
#include "ec/sdk/request.hpp"

namespace ec::sdk {

/// §2.2.4.1 Joint positions for a pose (one solution, near `reference`).
struct InverseKinematic {
    static constexpr std::string_view method = "inverseKinematic";
    static constexpr Kind kind = Kind::Compute;
    using Reply = JointsDeg;
    PoseMmRad target{};                    ///< targetPose
    std::optional<JointsDeg> reference{};  ///< referencePos: the solution is chosen near it

    [[nodiscard]] Json params() const {
        Json p;
        p["targetPose"] = target;
        if (reference) p["referencePos"] = *reference;
        p["unit_type"] = 1;
        return p;
    }
};

/// §2.2.4.2 The TCP pose of joint positions.
struct PositiveKinematic {
    static constexpr std::string_view method = "positiveKinematic";
    static constexpr Kind kind = Kind::Compute;
    using Reply = PoseMmRad;
    JointsDeg joints{};  ///< targetPos

    [[nodiscard]] Json params() const { return Json{{"targetPos", joints}, {"unit_type", 1}}; }
};

/// §2.2.4.3 A base-frame pose expressed in a user frame.
struct ConvertPoseFromCartToUser {
    static constexpr std::string_view method = "convertPoseFromCartToUser";
    static constexpr Kind kind = Kind::Compute;
    using Reply = PoseMmRad;
    PoseMmRad pose{};    ///< targetPose, in the base frame
    int user_frame = 0;  ///< userNo, 0..7

    [[nodiscard]] Json params() const { return Json{{"targetPose", pose}, {"userNo", user_frame}, {"unit_type", 1}}; }
};

/// §2.2.4.4 A user-frame pose expressed in the base frame.
struct ConvertPoseFromUserToCart {
    static constexpr std::string_view method = "convertPoseFromUserToCart";
    static constexpr Kind kind = Kind::Compute;
    using Reply = PoseMmRad;
    PoseMmRad pose{};    ///< targetPose, in the user frame
    int user_frame = 0;  ///< userNo, 0..7

    [[nodiscard]] Json params() const { return Json{{"targetPose", pose}, {"userNo", user_frame}, {"unit_type", 1}}; }
};

/// §2.2.4.5 The product T1 * T2 of two poses (measured on the EA66, ADR 0012).
struct PoseMul {
    static constexpr std::string_view method = "poseMul";
    static constexpr Kind kind = Kind::Compute;
    using Reply = PoseMmRad;
    PoseMmRad first{};   ///< pose1
    PoseMmRad second{};  ///< pose2

    [[nodiscard]] Json params() const { return Json{{"pose1", first}, {"pose2", second}, {"unit_type", 1}}; }
};

/// §2.2.4.6 The inverse of a pose.
struct PoseInv {
    static constexpr std::string_view method = "poseInv";
    static constexpr Kind kind = Kind::Compute;
    using Reply = PoseMmRad;
    PoseMmRad pose{};  ///< pose

    [[nodiscard]] Json params() const { return Json{{"pose", pose}, {"unit_type", 1}}; }
};

static_assert(Command<InverseKinematic>);
static_assert(Command<PositiveKinematic>);
static_assert(Command<ConvertPoseFromCartToUser>);
static_assert(Command<ConvertPoseFromUserToCart>);
static_assert(Command<PoseMul>);
static_assert(Command<PoseInv>);

}  // namespace ec::sdk
