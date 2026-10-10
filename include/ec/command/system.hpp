#pragma once
// System service (SystemService), SDK manual section 2.2.8.

#include <expected>
#include <string>
#include <string_view>

#include "ec/command/request.hpp"

namespace ec::command {

// §2.2.8.1 Get controller software version, e.g. "V3.15.3".
struct GetSoftVersion {
    static constexpr std::string_view method = "getSoftVersion";
    using Reply = std::string;
    [[nodiscard]] Json params() const { return nullptr; }
    [[nodiscard]] static std::expected<Reply, Error> decode(const Json& result);
};

static_assert(Command<GetSoftVersion>);

}  // namespace ec::command
