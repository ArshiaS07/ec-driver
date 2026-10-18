// ec_query: send one SDK call to the controller's command port (8055) and print
// the reply; or list every call; or try every query.
//
//   ec_query [--host H] [--port N] [--timeout MS] [--write] [--untyped] <method> [params...]
//   ec_query --list
//   ec_query [--host H] --check-all
//
// params: one JSON object ('{"index":0}') or key=value pairs (index=0
// targetPos=[0,-90,0,-90,90,0]); a value is parsed as JSON when it is valid
// JSON, otherwise taken as text. Examples:
//
//   ec_query getRobotMode
//   ec_query getDH index=0
//   ec_query get_tcp_pose coordinate_num=-1 tool_num=-1 unit_type=1
//
// Every call of the SDK manual is known by name and kind (--list). Queries and
// computations are sent as they are. Anything else may move the robot or
// change its configuration, and this tool bypasses every check the driver
// has: it needs --write. A name that is not a known call is refused, unless
// --untyped (and --write) is given, for calls of newer firmware.
//
// The reply is printed as JSON on stdout, and checked against the call's
// reply type; a reply that does not fit is reported on stderr. --check-all
// sends every query with its default parameters and reports which replies fit
// their types: a read-only way to check the driver's types on a robot.
//
// Exit codes: 0 success; 1 the call failed (or, with --check-all, a reply did
// not fit its type); 2 bad command line.

#include <chrono>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "cli.hpp"
#include "ec/sdk/all.hpp"
#include "ec/sdk/client.hpp"

namespace {

using ec::sdk::Json;

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

std::string describe(const ec::sdk::Error& e) {
    return std::string(ec::sdk::to_string(e.kind)) + (e.message.empty() ? "" : ": " + e.message);
}

// Invalid UTF-8 from the controller is printed as U+FFFD instead of throwing.
std::string dump(const Json& j) { return j.dump(-1, ' ', false, Json::error_handler_t::replace); }

int list() {
    for (const auto& c : ec::sdk::all_commands()) cli::out("{}\t{}\n", c.method, ec::sdk::to_string(c.kind));
    return cli::kOk;
}

int check_all(ec::sdk::Client& client) {
    int ok = 0;
    int mismatches = 0;
    int errors = 0;
    for (const auto& c : ec::sdk::all_commands()) {
        if (c.kind != ec::sdk::Kind::Query) continue;
        const auto result = client.call(c.example());
        if (!result) {
            ++errors;
            cli::out("error     {}: {}\n", c.method, describe(result.error()));
            if (!client.is_connected()) break;
            continue;
        }
        if (const auto fits = c.check_reply(*result); !fits) {
            ++mismatches;
            cli::out("mismatch  {}: {} (reply {})\n", c.method, fits.error().message, dump(*result));
            continue;
        }
        ++ok;
        cli::out("ok        {}\n", c.method);
    }
    cli::out("{} fit their types, {} did not, {} failed\n", ok, mismatches, errors);
    return mismatches == 0 ? cli::kOk : cli::kFailed;
}

int run(int argc, char** argv) {
    std::string host = "192.168.0.200";  // this tool's default; the library has none
    ec::sdk::Client::Options options;
    std::int64_t timeout_ms = options.reply_timeout.count();
    bool allow_write = false;
    bool untyped = false;
    bool list_calls = false;
    bool check = false;
    std::string method;
    std::vector<std::string> param_args;

    CLI::App app{"Send one SDK call to the controller's command port (8055) and print the reply."};
    app.add_option("--host", host, "Controller address")->capture_default_str();
    app.add_option("--port", options.port, "Command port")->capture_default_str()->check(CLI::Range(1, 65535));
    app.add_option("--timeout", timeout_ms, "Reply timeout [ms]")->capture_default_str()->check(CLI::PositiveNumber);
    app.add_flag("--write", allow_write, "Allow calls that may move the robot or change its configuration");
    app.add_flag("--untyped", untyped, "Allow a method that is not a known call (needs --write too)");
    auto* list_flag = app.add_flag("--list", list_calls, "List every known call and its kind");
    auto* check_flag = app.add_flag("--check-all", check, "Send every query and check its reply's type");
    auto* method_arg = app.add_option("method", method, "SDK method, e.g. getRobotMode");
    app.add_option("params", param_args, "One JSON object, or key=value pairs");
    list_flag->excludes(method_arg)->excludes(check_flag);
    check_flag->excludes(method_arg);
    if (const auto code = cli::parse(app, argc, argv)) return *code;
    options.reply_timeout = std::chrono::milliseconds(timeout_ms);

    if (list_calls) return list();
    if (!check && method.empty()) {
        cli::err("a method is required (or --list, or --check-all)\n");
        return cli::kUsage;
    }

    const ec::sdk::CommandInfo* info = nullptr;
    Json params;  // stays null without parameters: the request then has no "params"
    if (!check) {
        info = ec::sdk::find_command(method);
        if (info == nullptr && !untyped) {
            cli::err(
                "'{}' is not a call of the SDK manual (ec_query --list shows them).\n"
                "Pass --untyped --write to send it anyway.\n",
                method);
            return cli::kUsage;
        }
        if ((info == nullptr || !ec::sdk::is_read_only(info->kind)) && !allow_write) {
            cli::err(
                "'{}' is not read-only ({}); it may move the robot or change its configuration.\n"
                "Pass --write if you really mean to send it.\n",
                method, info ? ec::sdk::to_string(info->kind) : "unknown kind");
            return cli::kUsage;
        }
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
    }

    auto client = ec::sdk::Client::connect(host, options);
    if (!client) {
        cli::err("{}:{}: {}\n", host, options.port, client.error().message);
        return cli::kFailed;
    }
    if (check) return check_all(*client);

    const auto result = client->call(ec::sdk::Request{method, params});
    if (!result) {
        cli::err("{}\n", describe(result.error()));
        return cli::kFailed;
    }
    cli::out("{}\n", dump(*result));
    if (info != nullptr) {
        if (const auto fits = info->check_reply(*result); !fits)
            cli::err("note: the reply does not fit the type of {}: {}\n", method, fits.error().message);
    }
    return cli::kOk;
}

}  // namespace

int main(int argc, char** argv) {
    return cli::guarded("ec_query", [&] { return run(argc, argv); });
}
