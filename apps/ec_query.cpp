// ec_query: send one SDK call to port 8055 and print the reply.
//
//   usage: ec_query [--host H] [--port N] [--timeout MS] [--write] <method> [params]
//
//     --host H       controller address (default 192.168.0.200)
//     --port N       command port (default 8055)
//     --timeout MS   reply timeout (default 1000)
//     --write        allow methods that change the robot (see below)
//     params         either one JSON object:   '{"index":0}'
//                    or key=value pairs:        index=0  targetPos=[0,-90,0,-90,90,0]
//                    (values are parsed as JSON when possible, else taken as text)
//
// Examples:
//   ec_query getRobotMode
//   ec_query getDH index=0
//   ec_query get_tcp_pose coordinate_num=-1 tool_num=-1 unit_type=1
//
// Safety: only read-only methods (names starting with "get" or "check") are
// sent by default. Anything else can move the robot or change its
// configuration, and this tool bypasses every safety check the driver will
// have; it needs --write.
//
// Prints the unwrapped result as JSON on stdout. On error, prints the error on
// stderr and exits with code 1.

#include <cctype>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <format>
#include <string>
#include <string_view>

#include "ec/command/client.hpp"

namespace {

using ec::command::Json;

template <typename... Args>
void err(std::format_string<Args...> fmt, Args&&... args) {
    const std::string s = std::format(fmt, std::forward<Args>(args)...);
    std::fwrite(s.data(), 1, s.size(), stderr);
}

constexpr std::string_view kUsage =
    "usage: ec_query [--host H] [--port N] [--timeout MS] [--write] <method> [params]\n"
    "  params: one JSON object, or key=value pairs (e.g. index=0)\n";

bool is_read_only(std::string_view method) {
    auto starts_with = [&](std::string_view prefix) {
        if (method.size() < prefix.size()) return false;
        for (std::size_t i = 0; i < prefix.size(); ++i)
            if (std::tolower(static_cast<unsigned char>(method[i])) != prefix[i]) return false;
        return true;
    };
    return starts_with("get") || starts_with("check");
}

// "key=value" → params[key] = value (JSON if it parses, else a string).
bool add_pair(Json& params, std::string_view arg) {
    const auto eq = arg.find('=');
    if (eq == std::string_view::npos || eq == 0) return false;
    const std::string key(arg.substr(0, eq));
    const std::string text(arg.substr(eq + 1));
    Json value = Json::parse(text, nullptr, /*allow_exceptions=*/false);
    params[key] = value.is_discarded() ? Json(text) : value;
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    ec::command::Client::Options options;
    bool allow_write = false;
    std::string method;
    Json params;  // stays null if no parameters are given

    for (int i = 1; i < argc; ++i) {
        const std::string_view a = argv[i];
        if (a == "--host" && i + 1 < argc) {
            options.host = argv[++i];
        } else if (a == "--port" && i + 1 < argc) {
            options.port = static_cast<std::uint16_t>(std::atoi(argv[++i]));
        } else if (a == "--timeout" && i + 1 < argc) {
            options.reply_timeout = std::chrono::milliseconds(std::atoi(argv[++i]));
        } else if (a == "--write") {
            allow_write = true;
        } else if (!a.empty() && a[0] == '-') {
            err("{}", kUsage);
            return 2;
        } else if (method.empty()) {
            method = a;
        } else if (a.starts_with('{')) {
            params = Json::parse(a, nullptr, /*allow_exceptions=*/false);
            if (params.is_discarded() || !params.is_object()) {
                err("params is not a valid JSON object: {}\n", a);
                return 2;
            }
        } else if (!add_pair(params, a)) {
            err("expected key=value, got: {}\n{}", a, kUsage);
            return 2;
        }
    }
    if (method.empty()) {
        err("{}", kUsage);
        return 2;
    }
    if (!is_read_only(method) && !allow_write) {
        err("'{}' is not a read-only method; it may move the robot or change its configuration.\n"
            "Pass --write if you really mean to send it.\n",
            method);
        return 2;
    }

    auto client = ec::command::Client::connect(options);
    if (!client) {
        err("{}:{}: {}\n", options.host, options.port, client.error().message);
        return 1;
    }
    const auto result = client->call(ec::command::Request{method, params});
    if (!result) {
        const auto& e = result.error();
        err("{}{}{}\n", ec::command::to_string(e.kind), e.message.empty() ? "" : ": ", e.message);
        return 1;
    }
    const std::string out = result->dump() + "\n";
    std::fwrite(out.data(), 1, out.size(), stdout);
    return 0;
}
