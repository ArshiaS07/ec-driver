#pragma once
// Shared by the command-line tools: printing, argument parsing with uniform
// exit codes, and a guard that keeps exceptions from escaping main().

#include <CLI/CLI.hpp>
#include <cstdio>
#include <exception>
#include <format>
#include <optional>
#include <string>
#include <utility>

namespace cli {

// Exit codes of every tool.
inline constexpr int kOk = 0;
inline constexpr int kFailed = 1;  // the tool ran, and what it was asked to do failed
inline constexpr int kUsage = 2;   // bad command line

template <typename... Args>
void out(std::format_string<Args...> fmt, Args&&... args) {
    const std::string s = std::format(fmt, std::forward<Args>(args)...);
    std::fwrite(s.data(), 1, s.size(), stdout);
}

template <typename... Args>
void err(std::format_string<Args...> fmt, Args&&... args) {
    const std::string s = std::format(fmt, std::forward<Args>(args)...);
    std::fwrite(s.data(), 1, s.size(), stderr);
}

// Parses the command line: nullopt to carry on, or the code to exit with
// (kOk after --help, kUsage after an error; CLI11 prints either).
inline std::optional<int> parse(CLI::App& app, int argc, char** argv) {
    try {
        app.parse(argc, argv);
        return std::nullopt;
    } catch (const CLI::ParseError& e) {
        return app.exit(e) == 0 ? kOk : kUsage;
    }
}

// Runs `body`, the tool's real main. An exception that reaches here is printed
// and becomes kFailed; the printing itself cannot throw.
template <typename Body>
int guarded(const char* tool, Body&& body) noexcept {
    try {
        return std::forward<Body>(body)();
    } catch (const std::exception& e) {
        std::fputs(tool, stderr);
        std::fputs(": ", stderr);
        std::fputs(e.what(), stderr);
        std::fputc('\n', stderr);
    } catch (...) {
        std::fputs(tool, stderr);
        std::fputs(": unknown error\n", stderr);
    }
    return kFailed;
}

}  // namespace cli
