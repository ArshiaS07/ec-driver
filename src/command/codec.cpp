#include "ec/command/codec.hpp"

namespace ec::command {
namespace {

std::unexpected<Error> bad_reply(std::string what) {
    return std::unexpected(Error{Error::Kind::BadReply, 0, std::move(what)});
}

Error from_error_object(const Json& e) {
    const int code = e.contains("code") && e["code"].is_number_integer() ? e["code"].get<int>() : 0;
    std::string message = e.contains("message") && e["message"].is_string() ? e["message"].get<std::string>() : "";
    Error::Kind kind = Error::Kind::Remote;
    if (code == -32601) kind = Error::Kind::MethodNotFound;
    if (code == -32693) kind = Error::Kind::Rejected;
    return Error{kind, code, std::move(message)};
}

// The controller sends results as JSON text inside a JSON string. Parse that
// text; if it is not valid JSON, keep the plain string (robust to firmware that
// sends an unquoted text result).
Json unwrap(const Json& result) {
    if (!result.is_string()) return result;
    const auto& text = result.get_ref<const std::string&>();
    Json inner = Json::parse(text, nullptr, /*allow_exceptions=*/false);
    return inner.is_discarded() ? result : inner;
}

}  // namespace

std::string_view to_string(Error::Kind k) noexcept {
    switch (k) {
        case Error::Kind::MethodNotFound: return "method not found";
        case Error::Kind::Rejected: return "rejected by controller";
        case Error::Kind::Refused: return "controller returned false";
        case Error::Kind::Remote: return "controller error";
        case Error::Kind::BadReply: return "bad reply";
    }
    return "unknown";
}

std::string encode(const Request& request, std::uint64_t id) {
    Json j;  // field order as in the manual: jsonrpc, method, params, id
    j["jsonrpc"] = "2.0";
    j["method"] = request.method;
    if (!request.params.is_null()) j["params"] = request.params;
    j["id"] = id;
    std::string line = j.dump();
    line += '\n';
    return line;
}

std::expected<Reply, Error> parse_reply(std::string_view line) {
    const Json j = Json::parse(line, nullptr, /*allow_exceptions=*/false);
    if (j.is_discarded()) return bad_reply("not valid JSON");
    if (!j.is_object()) return bad_reply("not a JSON object");
    if (!j.contains("id") || !j["id"].is_number_unsigned()) return bad_reply("missing or invalid id");

    Reply reply;
    reply.id = j["id"].get<std::uint64_t>();
    if (j.contains("error")) {
        if (!j["error"].is_object()) return bad_reply("error member is not an object");
        reply.result = std::unexpected(from_error_object(j["error"]));
    } else if (j.contains("result")) {
        reply.result = unwrap(j["result"]);
    } else {
        return bad_reply("neither result nor error");
    }
    return reply;
}

}  // namespace ec::command
