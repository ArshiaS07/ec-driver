#pragma once
/// @file
/// JSON-RPC 2.0 lines for port 8055: requests out, responses in. Pure: no
/// sockets, no clocks; the client adds the I/O and the ids.

#include <cstdint>
#include <expected>
#include <optional>
#include <string>
#include <string_view>

#include "ec/sdk/error.hpp"
#include "ec/sdk/json.hpp"
#include "ec/sdk/request.hpp"

namespace ec::sdk {

/// One request line, '\n'-terminated, with its members in the manual's order:
/// {"jsonrpc":"2.0","method":...,"params":...,"id":...}. "params" is left out
/// when Request::params is null.
[[nodiscard]] std::string encode(const Request& request, std::uint64_t id);

/// One parsed response line.
struct Response {
    /// The id of the request it answers. nullopt for "id": null, which JSON-RPC
    /// uses for errors the server could not attribute to a request.
    std::optional<std::uint64_t> id;
    /// The result, or the error the controller reported. The controller sends
    /// every result as JSON text inside a JSON string ("result":"[2.08,...]");
    /// this holds the parsed inner value. A string that is not valid JSON text is
    /// kept as it is.
    std::expected<Json, Error> result;
};

/// Parses one response line, with or without its newline. Fails, with
/// Kind::BadReply, only if the line is not a JSON-RPC response; an error
/// response parses successfully, with the error in Response::result.
[[nodiscard]] std::expected<Response, Error> parse_response(std::string_view line);

}  // namespace ec::sdk
