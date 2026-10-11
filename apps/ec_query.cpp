// ec_query: send one SDK call to the controller's command port (8055) and print
// the reply.
//
//   ec_query [--host H] [--port N] [--timeout MS] [--write] <method> [params...]
//
// params: one JSON object ('{"index":0}') or key=value pairs (index=0
// targetPos=[0,-90,0,-90,90,0]); a value is parsed as JSON when it is valid
// JSON, otherwise taken as text. Examples:
//
//   ec_query getRobotMode
//   ec_query getDH index=0
//   ec_query get_tcp_pose coordinate_num=-1 tool_num=-1 unit_type=1
//
// Read-only methods (names starting with "get" or "check") are sent as they are.
// Anything else may move the robot or change its configuration, and this tool
// bypasses every check the driver will have: it needs --write.
//
// Prints the result as JSON on stdout. Exit codes: 0 success; 1 the call failed
// (the error goes to stderr); 2 bad command line.

#include <cctype>
#include <chrono>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "cli.hpp"
#include "ec/command/client.hpp"

namespace {

using ec::command::Json;

bool is_read_only(std::string_view method) {
    auto starts_with = [&](std::string_view prefix) {
        if (method.size() < prefix.size()) return false;
        for (std::size_t i = 0; i < prefix.size(); ++i)
            if (std::tolower(static_cast<unsigned char>(method[i])) != prefix[i]) return false;
        return true;
    };
    return starts_with("get") || starts_with("check");
}

// "key=value": params[key] = value, as JSON if it parses, otherwise as text.
bool add_pair(Json& params, std::string_view arg) {
    const auto eq = arg.find('=');
    if (eq == std::string_view::npos || eq == 0) return false;
    const std::string key(arg.substr(0, eq));
    const std::string text(arg.substr(eq + 1));
    Json value = Json::parse(text, nullptr, /*allow_exceptions=*/false);
    params[key] = value.is_discarded() ? Json(text) : value;
    return true;
}

int run(int argc, char** argv) {
    std::string host = "192.168.0.200";  // this tool's default; the library has none
    ec::command::Client::Options options;
    std::int64_t timeout_ms = options.reply_timeout.count();
    bool allow_write = false;
    std::string method;
    std::vector<std::string> param_args;

    CLI::App app{"Send one SDK call to the controller's command port (8055) and print the reply."};
    app.add_option("--host", host, "Controller address")->capture_default_str();
    app.add_option("--port", options.port, "Command port")->capture_default_str()->check(CLI::Range(1, 65535));
    app.add_option("--timeout", timeout_ms, "Reply timeout [ms]")->capture_default_str()->check(CLI::PositiveNumber);
    app.add_flag("--write", allow_write, "Allow methods that may move the robot or change its configuration");
    app.add_option("method", method, "SDK method, e.g. getRobotMode")->required();
    app.add_option("params", param_args, "One JSON object, or key=value pairs");
    if (const auto code = cli::parse(app, argc, argv)) return *code;
    options.reply_timeout = std::chrono::milliseconds(timeout_ms);

    Json params;  // stays null without parameters: the request then has no "params"
    for (const std::string& a : param_args) {
        if (a.starts_with('{')) {
            params = Json::parse(a, nullptr, /*allow_exceptions=*/false);
            if (params.is_discarded() || !params.is_object()) {
                cli::err("params is not a JSON object: {}\n", a);
                return cli::kUsage;
            }
        } else if (!add_pair(params, a)) {
            cli::err("expected key=value, got: {}\n", a);
            return cli::kUsage;
        }
    }
    if (!is_read_only(method) && !allow_write) {
        cli::err(
            "'{}' is not a read-only method; it may move the robot or change its configuration.\n"
            "Pass --write if you really mean to send it.\n",
            method);
        return cli::kUsage;
    }

    auto client = ec::command::Client::connect(host, options);
    if (!client) {
        cli::err("{}:{}: {}\n", host, options.port, client.error().message);
        return cli::kFailed;
    }
    const auto result = client->call(ec::command::Request{method, params});
    if (!result) {
        const auto& e = result.error();
        cli::err("{}{}{}\n", ec::command::to_string(e.kind), e.message.empty() ? "" : ": ", e.message);
        return cli::kFailed;
    }
    // Invalid UTF-8 from the controller is printed as U+FFFD instead of throwing.
    cli::out("{}\n", result->dump(-1, ' ', false, Json::error_handler_t::replace));
    return cli::kOk;
}

}  // namespace

int main(int argc, char** argv) {
    return cli::guarded("ec_query", [&] { return run(argc, argv); });
}
