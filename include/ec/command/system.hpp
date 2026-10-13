#pragma once
/// @file
/// System service (SystemService), SDK manual section 2.2.8.

#include <expected>
#include <format>
#include <string>
#include <string_view>

#include "ec/command/error.hpp"
#include "ec/command/json.hpp"
#include "ec/command/reply.hpp"
#include "ec/command/request.hpp"

namespace ec::command {

/// §2.2.8.1 Controller software version, e.g. "V3.15.3".
struct GetSoftVersion {
    static constexpr std::string_view method = "getSoftVersion";
    static constexpr Kind kind = Kind::Query;
    using Reply = std::string;
};

/// §2.2.8.2 A servo drive's version, for servo version 11 or later. The manual
/// does not say whether it is a number or a string; either decodes, as text.
struct GetJointVersion {
    static constexpr std::string_view method = "getJointVersion";
    static constexpr Kind kind = Kind::Query;
    using Reply = std::string;
    int axis = 0;  ///< 0..7 for axes 1..8

    [[nodiscard]] Json params() const { return Json{{"axis", axis}}; }

    static std::expected<std::string, Error> decode(const Json& r) {
        if (r.is_string()) return r.get<std::string>();
        if (r.is_number()) return r.dump();
        return bad_reply(std::format("expected a version, got {}", r.dump()));
    }
};

static_assert(Command<GetSoftVersion>);
static_assert(Command<GetJointVersion>);

}  // namespace ec::command
