#pragma once
/// @file
/// ec::CallRecord: one exchange on the command connection (8055) as it
/// happened, for a call log beside a recording (RobotOptions::on_call,
/// record::CallWriter).

#include <expected>
#include <string>

#include "ec/error.hpp"
#include "ec/sdk/json.hpp"
#include "ec/stamped.hpp"

namespace ec {

struct CallRecord {
    std::string method{};
    sdk::Json params{};                       ///< null: the request had none
    std::expected<sdk::Json, Error> reply{};  ///< the result, unwrapped; or why there is none
    Clock::time_point sent{};                 ///< when the request was sent (this PC's steady clock)
    Clock::time_point answered{};             ///< when the reply came, or the call failed
};

}  // namespace ec
