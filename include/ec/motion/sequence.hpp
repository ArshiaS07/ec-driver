#pragma once
/// @file
/// Activities run back to back as one: each step starts as soon as the one
/// before it has closed (PLCopen's "buffered" mode).

#include <chrono>
#include <cstddef>
#include <expected>
#include <string_view>
#include <utility>
#include <vector>

#include "ec/motion/activity.hpp"

namespace ec::motion {

/// Runs its steps in order. A step that has finished is closed before the next
/// starts; the next waits up to `start_wait` for its start conditions (the
/// state can take a frame or two to show standstill after a move). Needs what
/// its current step needs. If a step fails, the sequence fails with that
/// error, and close() closes the failing step.
class Sequence {
public:
    Sequence() = default;

    /// From specs and activities alike, e.g. `Sequence(MoveJoint{...}, PlayTrajectory{...})`.
    template <typename... Steps>
        requires(sizeof...(Steps) > 0 && (std::constructible_from<AnyActivity, Steps &&> && ...))
    explicit Sequence(Steps&&... steps) {
        steps_.reserve(sizeof...(Steps));
        (steps_.emplace_back(std::forward<Steps>(steps)), ...);
    }
    explicit Sequence(std::vector<AnyActivity> steps) : steps_(std::move(steps)) {}

    /// Appends a step.
    Sequence& then(AnyActivity step) {
        steps_.push_back(std::move(step));
        return *this;
    }

    std::chrono::milliseconds start_wait{1'000};

    [[nodiscard]] Needs needs() const;
    [[nodiscard]] std::string_view name() const noexcept { return "sequence"; }
    [[nodiscard]] std::expected<Phase, Error> step(const Cycle& c, Outbox& out);
    [[nodiscard]] bool close(const Cycle& c, Outbox& out, End why);
    [[nodiscard]] std::size_t size() const noexcept { return steps_.size(); }

private:
    enum class Stage { Starting, Stepping, Closing, Finished };

    std::vector<AnyActivity> steps_;
    std::size_t index_ = 0;
    Stage stage_ = Stage::Stepping;  // the first step was started by the executor
    Clock::time_point waiting_since_{};
};

}  // namespace ec::motion
