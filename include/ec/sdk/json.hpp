#pragma once
/// @file
/// The JSON type of the command layer.

#include <nlohmann/json.hpp>

namespace ec::sdk {

/// JSON value whose objects keep their keys in insertion order, so requests go
/// out with their members in the order the code sets them: the manual's order.
using Json = nlohmann::ordered_json;

}  // namespace ec::sdk
