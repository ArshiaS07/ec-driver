#pragma once
/// @file
/// The Command concept every SDK call satisfies, and Request, the type-erased
/// form of a call that the codec encodes.

#include <array>
#include <concepts>
#include <expected>
#include <string_view>

#include "ec/sdk/error.hpp"
#include "ec/sdk/json.hpp"
#include "ec/sdk/reply.hpp"

namespace ec::sdk {

/// Joint positions as the controller sends and expects them: J1..J6 [deg].
using JointsDeg = std::array<double, 6>;

/// A pose as the commands here send and receive it: x, y, z [mm], then rx, ry,
/// rz [rad], composed as R = Rz(rz) * Ry(ry) * Rx(rx) (measured on the EA66). Every command
/// with a `unit_type` parameter sends 1 (radians), so this is also what replies hold.
using PoseMmRad = std::array<double, 6>;

/// What a call does to the controller; decides what may be sent when. From
/// docs/sdk_calls.md, except that a motion family's queries (state, progress,
/// validity checks) count as queries here.
enum class Kind {
    Query,    ///< reads; changes nothing
    Compute,  ///< a computation on the controller (kinematics); changes nothing
    Setting,  ///< changes stored or selected configuration (tool, frames, payload, speed, IO, variables)
    Action,   ///< a one-shot change of controller state (servo on, clear alarm, backdrive, ...)
    Motion,   ///< part of a motion family's protocol; moves the arm or prepares to
    Session,  ///< part of a non-motion open/data/close protocol (serial ports, force data)
};

/// Queries and computations: safe to send at any time.
[[nodiscard]] constexpr bool is_read_only(Kind k) noexcept { return k == Kind::Query || k == Kind::Compute; }

/// Lower-case name, e.g. "query".
[[nodiscard]] constexpr std::string_view to_string(Kind k) noexcept {
    switch (k) {
        case Kind::Query: return "query";
        case Kind::Compute: return "compute";
        case Kind::Setting: return "setting";
        case Kind::Action: return "action";
        case Kind::Motion: return "motion";
        case Kind::Session: return "session";
    }
    return "?";
}

/// The command has a params() member.
template <typename C>
concept HasParams = requires(const C& c) { c.params(); };

/// The command has its own decode(), with the required signature.
template <typename C>
concept HasDecode = requires(const Json& result) {
    { C::decode(result) } -> std::same_as<std::expected<typename C::Reply, Error>>;
};

/// One SDK call, described as a type. Required:
///   - `static constexpr std::string_view method`: the JSON-RPC method name;
///   - `static constexpr Kind kind`: what the call does;
///   - `using Reply = ...`: what the call returns.
/// Optional:
///   - `Json params() const`: the request's "params"; without it, none are sent;
///   - `static std::expected<Reply, Error> decode(const Json&)`: only when
///     ReplyDecoder<Reply> does not fit the reply's shape.
/// A command performs no I/O: it only describes the call.
template <typename C>
concept Command =
    requires {
        { C::method } -> std::convertible_to<std::string_view>;
        { C::kind } -> std::convertible_to<Kind>;
        typename C::Reply;
    } && (!HasParams<C> || requires(const C& c) {
        { c.params() } -> std::same_as<Json>;
    }) && (!requires { &C::decode; } || HasDecode<C>) && (HasDecode<C> || DecodableReply<typename C::Reply>);

/// A call with its type erased: what the codec encodes.
struct Request {
    std::string_view method;  ///< usually a command's static name; must outlive the Request
    Json params;              ///< null: the request has no "params" member
};

/// The request for `command`.
template <Command C>
[[nodiscard]] Request to_request(const C& command) {
    if constexpr (HasParams<C>)
        return Request{C::method, command.params()};
    else
        return Request{C::method, nullptr};
}

/// Decodes the (unwrapped) result of a `C` call into C::Reply, with C::decode if
/// the command has one and ReplyDecoder<C::Reply> otherwise.
template <Command C>
[[nodiscard]] std::expected<typename C::Reply, Error> decode_reply(const Json& result) {
    if constexpr (HasDecode<C>)
        return C::decode(result);
    else
        return ReplyDecoder<typename C::Reply>::decode(result);
}

}  // namespace ec::sdk
