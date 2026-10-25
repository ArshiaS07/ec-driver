#pragma once
/// @file
/// The controller's log interface, port 8058 (SDK manual chapter 4): the
/// controller's decoded log, as text lines. One exchange per connection, as
/// the manual's example: connect, send the type, then a line count or "all",
/// read, send "exit". The controller does the work (it keeps and filters the
/// log); this only asks and reads. Port 8059, the raw log (chapter 5), is not
/// read here.

#include <chrono>
#include <cstdint>
#include <expected>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "ec/net/tcp_socket.hpp"

namespace ec::log {

/// The manual's log types. Each includes the ones above it: Warning gives
/// errors and warnings, Info gives everything.
enum class Level : std::uint8_t { Error, Warning, Info };

/// The manual's word, as sent: "Error", "Warning", "Info".
[[nodiscard]] std::string_view to_string(Level level) noexcept;

/// What to ask for.
struct Request {
    Level level = Level::Info;
    /// The last this many lines; nullopt: the whole log ("all"), which can be long.
    std::optional<std::uint32_t> lines = 50;
};

struct Options {
    std::uint16_t port = 8058;
    std::chrono::milliseconds connect_timeout{2'000};  ///< also for sending the request
    /// For the first bytes of the reply (the manual's example waits 2 s).
    std::chrono::milliseconds reply_within{2'000};
    /// The manual marks no end of the reply: it is complete once nothing more
    /// has come for this long, or when the controller closes the connection.
    std::chrono::milliseconds quiet{500};
    /// A reply still arriving after this long is a Timeout.
    std::chrono::milliseconds limit{30'000};
};

/// The lines the controller sent, in its order, without their line ends. Their
/// format is the controller's (the manual shows none; unverified on the EA66).
/// Empty if nothing came within Options::reply_within. Errors: the connection
/// could not be made, the request not sent, the connection failed while
/// reading, or Options::limit passed (Timeout).
[[nodiscard]] std::expected<std::vector<std::string>, net::NetError> read(const std::string& host,
                                                                          const Request& request = {},
                                                                          const Options& options = {});

}  // namespace ec::log
