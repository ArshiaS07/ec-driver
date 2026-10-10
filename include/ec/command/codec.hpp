#pragma once
// JSON-RPC 2.0 framing for port 8055: requests out, replies in. Pure: no
// sockets, no clocks. The client adds I/O and id bookkeeping on top.
//
// Observed controller behaviour (firmware V3.15.3) this layer absorbs:
//   * every result is wrapped in a JSON string:
//       {"jsonrpc":"2.0","result":"[2.08,-90.53,...]","id":2}
//       {"jsonrpc":"2.0","result":"\"V3.15.3\"","id":1}
//     parse_reply() unwraps it, so commands see the plain value.
//   * errors use the standard JSON-RPC error object:
//       {"jsonrpc":"2.0","error":{"code":-32601,"message":"Method not found."},"id":5}

#include <cstdint>
#include <expected>
#include <string>
#include <string_view>

#include "ec/command/request.hpp"

namespace ec::command {

// One request line, terminated by '\n' as the protocol requires.
[[nodiscard]] std::string encode(const Request& request, std::uint64_t id);

struct Reply {
    std::uint64_t id = 0;
    std::expected<Json, Error> result;  // the unwrapped result, or the controller's error
};

// Parses one reply line (with or without the trailing newline).
// Fails with Error::Kind::BadReply only if the line is not a JSON-RPC reply at
// all; a well-formed error reply succeeds, with the error inside Reply::result.
[[nodiscard]] std::expected<Reply, Error> parse_reply(std::string_view line);

// Interprets a reply as the given command's reply type.
template <Command C>
[[nodiscard]] std::expected<typename C::Reply, Error> decode(const Reply& reply) {
    if (!reply.result) return std::unexpected(reply.result.error());
    return C::decode(*reply.result);
}

}  // namespace ec::command
