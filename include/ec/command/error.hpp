#pragma once
/// @file
/// Errors of the port-8055 command layer.

#include <expected>
#include <string>
#include <string_view>
#include <utility>

namespace ec::command {

/// Why a call failed: reported by the controller, or found by the client.
struct Error {
    enum class Kind {
        // Reported by the controller:
        MethodNotFound,  ///< JSON-RPC -32601: unknown method (a typo, or not in this firmware)
        Rejected,        ///< -32693, the manual's JRPC_INTERNAL_ERROR: a parameter missing or out of
                         ///< range, or a condition not met (e.g. not in remote mode)
        Refused,         ///< a call documented as "success: true; failure: false" returned false
        Remote,          ///< any other error the controller reported; see `code`
        // Found by the client:
        BadReply,      ///< not a JSON-RPC response, or a result of the wrong type or range
        Timeout,       ///< no reply within the timeout; the connection stays usable
        Disconnected,  ///< not connected, connecting failed, or the connection was lost and closed
    };
    Kind kind;
    int code = 0;         ///< JSON-RPC error code, or the OS error code for Disconnected; otherwise 0
    std::string message;  ///< the controller's message, or what went wrong
};

/// Short name of an error kind, e.g. "timeout".
[[nodiscard]] constexpr std::string_view to_string(Error::Kind k) noexcept {
    switch (k) {
        case Error::Kind::MethodNotFound: return "method not found";
        case Error::Kind::Rejected: return "rejected by controller";
        case Error::Kind::Refused: return "controller returned false";
        case Error::Kind::Remote: return "controller error";
        case Error::Kind::BadReply: return "bad reply";
        case Error::Kind::Timeout: return "timeout";
        case Error::Kind::Disconnected: return "disconnected";
    }
    return "unknown";
}

/// A Kind::BadReply error saying `what` was wrong, for reply decoders.
[[nodiscard]] inline std::unexpected<Error> bad_reply(std::string what) {
    return std::unexpected(Error{Error::Kind::BadReply, 0, std::move(what)});
}

}  // namespace ec::command
