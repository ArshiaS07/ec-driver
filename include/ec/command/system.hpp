#pragma once
/// @file
/// System service (SystemService), SDK manual section 2.2.8.

#include <string>
#include <string_view>

#include "ec/command/request.hpp"

namespace ec::command {

/// §2.2.8.1 Controller software version, e.g. "V3.15.3".
struct GetSoftVersion {
    static constexpr std::string_view method = "getSoftVersion";
    using Reply = std::string;
};

static_assert(Command<GetSoftVersion>);

}  // namespace ec::command
