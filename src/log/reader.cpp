#include "ec/log/reader.hpp"

#include <array>
#include <cstddef>
#include <span>
#include <utility>

namespace ec::log {
namespace {

using Clock = std::chrono::steady_clock;

std::span<const std::uint8_t> as_bytes(std::string_view s) {
    return {reinterpret_cast<const std::uint8_t*>(s.data()), s.size()};
}

std::vector<std::string> split_lines(std::string_view text) {
    std::vector<std::string> lines;
    while (!text.empty()) {
        const std::size_t end = text.find('\n');
        std::string_view line = text.substr(0, end);
        if (!line.empty() && line.back() == '\r') line.remove_suffix(1);
        lines.emplace_back(line);
        if (end == std::string_view::npos) break;
        text.remove_prefix(end + 1);
    }
    return lines;
}

}  // namespace

std::string_view to_string(Level level) noexcept {
    switch (level) {
        case Level::Error: return "Error";
        case Level::Warning: return "Warning";
        case Level::Info: return "Info";
    }
    return "Info";
}

std::expected<std::vector<std::string>, net::NetError> read(const std::string& host, const Request& request,
                                                            const Options& o) {
    auto socket = net::TcpSocket::connect(host, o.port, o.connect_timeout);
    if (!socket) return std::unexpected(socket.error());

    // Two lines, sent one after the other, as the manual's example does.
    const std::string type = std::string("Type=") + std::string(to_string(request.level)) + "\n";
    const std::string count = (request.lines ? std::to_string(*request.lines) : std::string("all")) + "\n";
    for (const std::string& line : {type, count})
        if (auto sent = socket->send_all(as_bytes(line), o.connect_timeout); !sent)
            return std::unexpected(sent.error());

    std::string text;
    std::array<std::uint8_t, 4096> buffer{};
    const auto deadline = Clock::now() + o.limit;
    auto wait = o.reply_within;
    for (;;) {
        const auto n = socket->receive(buffer, wait);
        if (!n) {
            if (n.error().kind == net::NetError::Kind::Timeout || n.error().kind == net::NetError::Kind::Closed) break;
            return std::unexpected(n.error());
        }
        text.append(reinterpret_cast<const char*>(buffer.data()), *n);
        if (Clock::now() >= deadline) return std::unexpected(net::NetError{net::NetError::Kind::Timeout});
        wait = o.quiet;
    }
    const auto said_goodbye = socket->send_all(as_bytes("exit\n"), o.connect_timeout);
    static_cast<void>(said_goodbye);  // the reply is complete either way
    return split_lines(text);
}

}  // namespace ec::log
