#include "ec/command/system.hpp"

#include "decode_helpers.hpp"

namespace ec::command {

std::expected<GetSoftVersion::Reply, Error> GetSoftVersion::decode(const Json& result) {
    return detail::as_string(result);
}

}  // namespace ec::command
