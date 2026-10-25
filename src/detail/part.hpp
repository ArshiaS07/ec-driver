#pragma once
// What the parts share: replies into the driver's types, and the manual's
// ranges, checked before anything is sent. Private to the library.

#include <expected>
#include <format>
#include <string_view>

#include "ec/error.hpp"
#include "ec/sdk/reply.hpp"

namespace ec::detail {

/// An Ok reply as nothing (the error as it is).
[[nodiscard]] inline std::expected<void, Error> done(const std::expected<sdk::Ok, Error>& ok) {
    if (!ok) return std::unexpected(ok.error());
    return {};
}

/// A 0/1 reply as a bool.
[[nodiscard]] inline std::expected<bool, Error> on(const std::expected<sdk::Flag, Error>& flag) {
    if (!flag) return std::unexpected(flag.error());
    return sdk::is_on(*flag);
}

/// Invalid unless low <= n <= high, naming what and the range.
[[nodiscard]] inline std::expected<void, Error> in_range(std::string_view what, int n, int low, int high) {
    if (n < low || n > high)
        return std::unexpected(Error{Error::Kind::Invalid, std::format("{} {}: {}..{}", what, n, low, high)});
    return {};
}

}  // namespace ec::detail
