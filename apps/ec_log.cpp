// ec_log: print the controller's log (port 8058, SDK manual chapter 4), read
// only: the last lines, or all of them, of one level and the levels above it.
//
//   ec_log [host] [--port N] [--level error|warning|info] [--lines N | --all]
//
// One exchange, as the manual's example: the type, the count, the reply (it
// ends when the port has been quiet for half a second; the manual marks no
// end), then "exit". The lines are the controller's, as it sends them.
//
// Exit codes: 0 read; 1 the port could not be read; 2 bad command line.

#include <cstdint>
#include <map>
#include <string>

#include "cli.hpp"
#include "ec/log/reader.hpp"

namespace {

int run(int argc, char** argv) {
    std::string host = "192.168.0.200";  // this tool's default; the library has none
    std::uint16_t port = 8058;
    std::string level = "info";
    std::uint32_t lines = 50;
    bool all = false;

    CLI::App app{"Print the controller's log (port 8058), read only."};
    app.add_option("host", host, "Controller address")->capture_default_str();
    app.add_option("--port", port, "Log port")->capture_default_str()->check(CLI::Range(1, 65535));
    const std::map<std::string, ec::log::Level> levels{
        {"error", ec::log::Level::Error}, {"warning", ec::log::Level::Warning}, {"info", ec::log::Level::Info}};
    app.add_option("--level", level, "error, warning (and errors) or info (everything)")
        ->capture_default_str()
        ->transform(CLI::IsMember({"error", "warning", "info"}, CLI::ignore_case));
    auto* count = app.add_option("--lines", lines, "The last this many lines")->capture_default_str();
    app.add_flag("--all", all, "The whole log")->excludes(count);
    if (const auto code = cli::parse(app, argc, argv)) return *code;

    const auto read = ec::log::read(
        host, {.level = levels.at(level), .lines = all ? std::nullopt : std::optional(lines)}, {.port = port});
    if (!read) {
        cli::err("log port {}:{}: {}\n", host, port, read.error().message());
        return cli::kFailed;
    }
    for (const auto& line : *read) cli::out("{}\n", line);
    if (read->empty()) cli::err("(no lines)\n");
    return cli::kOk;
}

}  // namespace

int main(int argc, char** argv) {
    return cli::guarded("ec_log", [&] { return run(argc, argv); });
}
