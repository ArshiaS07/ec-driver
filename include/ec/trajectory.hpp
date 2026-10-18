#pragma once
/// @file
/// A joint trajectory: joint positions, each stamped with its time since the
/// start, the shape the controller's timestamp service takes (SDK manual
/// 2.2.9.2: push_pos, a timestamp and a position). motion::Timestamped runs one.

#include <vector>

#include "ec/stamped.hpp"
#include "ec/units.hpp"

namespace ec {

/// Joint positions [rad] at times since the start: the first at 0, then
/// strictly increasing.
using JointTrajectory = std::vector<Stamped<Joints, Clock::duration>>;

}  // namespace ec
