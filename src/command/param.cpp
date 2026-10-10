#include "ec/command/param.hpp"

#include "decode_helpers.hpp"

namespace ec::command {

std::expected<GetRobotState::Reply, Error> GetRobotState::decode(const Json& result) {
    return detail::as_enum<RobotState>(result);
}

std::expected<GetRobotMode::Reply, Error> GetRobotMode::decode(const Json& result) {
    return detail::as_enum<RobotMode>(result);
}

std::expected<GetJointPos::Reply, Error> GetJointPos::decode(const Json& result) {
    return detail::as_doubles<6>(result);
}

std::expected<GetDH::Reply, Error> GetDH::decode(const Json& result) { return detail::as_double(result); }

}  // namespace ec::command
