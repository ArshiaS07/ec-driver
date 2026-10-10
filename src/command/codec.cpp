#include "ec/command/codec.hpp"

#include <utility>

namespace ec::command {
namespace {

// Error codes the manual names (section 2.1).
constexpr int kMethodNotFound = -32601;  // JRPC_METHOD_NOT_FOUND
constexpr int kInternalError = -32693;   // JRPC_INTERNAL_ERROR

Error from_error_object(const Json& e) {
    const auto code = e.find("code");
    const auto message = e.find("message");
    Error error{Error::Kind::Remote, 0, {}};
    if (code != e.end() && code->is_number_integer()) error.code = code->get<int>();
    if (message != e.end() && message->is_string()) error.message = message->get<std::string>();
    if (error.code == kMethodNotFound) error.kind = Error::Kind::MethodNotFound;
    if (error.code == kInternalError) error.kind = Error::Kind::Rejected;
    return error;
}

// The controller sends results as JSON text inside a JSON string. Parse that
// text; if it is not valid JSON, keep the plain string.
Json unwrap(const Json& result) {
    if (!result.is_string()) return result;
    Json inner = Json::parse(result.get_ref<const std::string&>(), nullptr, /*allow_exceptions=*/false);
    return inner.is_discarded() ? result : inner;
}

}  // namespace

std::string encode(const Request& request, std::uint64_t id) {
    Json j;
    j["jsonrpc"] = "2.0";
    j["method"] = request.method;
    if (!request.params.is_null()) j["params"] = request.params;
    j["id"] = id;
    std::string line = j.dump();
    line += '\n';
    return line;
}

std::expected<Response, Error> parse_response(std::string_view line) {
    const Json j = Json::parse(line, nullptr, /*allow_exceptions=*/false);
    if (j.is_discarded()) return bad_reply("not valid JSON");
    if (!j.is_object()) return bad_reply("not a JSON object");

    const auto id = j.find("id");
    if (id == j.end()) return bad_reply("no id");
    if (!id->is_null() && !id->is_number_unsigned()) return bad_reply("id is neither a non-negative integer nor null");

    Response response;
    if (!id->is_null()) response.id = id->get<std::uint64_t>();

    if (const auto error = j.find("error"); error != j.end()) {
        if (!error->is_object()) return bad_reply("error member is not an object");
        response.result = std::unexpected(from_error_object(*error));
    } else if (const auto result = j.find("result"); result != j.end()) {
        if (!response.id) return bad_reply("result with a null id");
        response.result = unwrap(*result);
    } else {
        return bad_reply("neither result nor error");
    }
    return response;
}

}  // namespace ec::command
