#pragma once
/// @file
/// The Command concept every SDK call satisfies, and Request, the type-erased
/// form of a call that the codec encodes.

#include <array>
#include <concepts>
#include <expected>
#include <string_view>

#include "ec/command/error.hpp"
#include "ec/command/json.hpp"
#include "ec/command/reply.hpp"

namespace ec::command {

/// Joint positions as the controller sends and expects them: J1..J6 [deg].
using JointsDeg = std::array<double, 6>;

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

}  // namespace ec::command
