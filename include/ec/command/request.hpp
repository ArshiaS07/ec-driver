#pragma once
// Building blocks of the port-8055 command protocol: the error type, the
// Command concept every SDK call satisfies, and Request, the type-erased form
// that gets encoded and sent.
//
// A command is a description of one SDK call (method name, parameters, how to
// read the reply). It performs no I/O; sending is the client's job. That split
// is what makes every command testable against the manual without a robot.

#include <array>
#include <concepts>
#include <expected>
#include <string>
#include <string_view>

#include <nlohmann/json.hpp>

namespace ec::command {

// ordered_json keeps object keys in insertion order, so requests go out with
// their fields in the manual's order. JSON does not require any order, but the
// controller's parser is undocumented; matching the manual removes one unknown.
using Json = nlohmann::ordered_json;

// Wire value types. Units are the controller's, as in the manual.
using JointsDeg = std::array<double, 6>;  // J1..J6 [deg]

struct Error {
    enum class Kind {
        MethodNotFound,  // -32601: unknown method (typo, or missing in this firmware)
        Rejected,        // -32693: parameter missing or out of range, or a condition
                         //         not met (e.g. not in remote mode)
        Refused,         // the call returned false (the manual's "failure: false")
        Remote,          // any other JSON-RPC error code
        BadReply,        // reply not understood: malformed JSON, wrong shape or value
    };
    Kind kind;
    int code = 0;         // JSON-RPC error code; 0 for Refused and BadReply
    std::string message;  // the controller's message, or what was wrong with the reply
};
[[nodiscard]] std::string_view to_string(Error::Kind k) noexcept;

// Reply type of calls that return only success or failure.
struct Ok {
    bool operator==(const Ok&) const = default;
};

// Every SDK call is a type satisfying this concept.
template <typename C>
concept Command = requires(const C& c, const Json& result) {
    { C::method } -> std::convertible_to<std::string_view>;
    typename C::Reply;
    { c.params() } -> std::same_as<Json>;  // null: the request has no "params" member
    { C::decode(result) } -> std::same_as<std::expected<typename C::Reply, Error>>;
};

// A command with its type erased: what the codec encodes. Motion families
// return sequences of these for the supervisor to send.
struct Request {
    std::string_view method;  // always one of the commands' static method names
    Json params;              // null: no "params" member
};

template <Command C>
[[nodiscard]] Request to_request(const C& c) {
    return Request{C::method, c.params()};
}

}  // namespace ec::command
