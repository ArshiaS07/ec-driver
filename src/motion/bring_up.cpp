#include "ec/motion/bring_up.hpp"

#include <format>

#include "ec/command/param.hpp"
#include "ec/command/reply.hpp"
#include "ec/command/servo.hpp"

namespace ec::motion {
namespace {

constexpr Conditions kBase{Condition::Fresh, Condition::Remote, Condition::NoEmergencyStop};

// The condition a step brings about, and its command's method.
Condition effect(int step) noexcept {
    constexpr Condition effects[] = {Condition::NoAlarm, Condition::NoCollision, Condition::Synced, Condition::ServoOn};
    return effects[step];
}

std::string_view method(int step) noexcept {
    constexpr std::string_view methods[] = {command::ClearAlarm::method, command::ResetCollisionState::method,
                                            command::SyncMotorStatus::method, command::SetServoStatus::method};
    return methods[step];
}

}  // namespace

Needs BringUpActivity::needs() const noexcept { return {kBase, kBase}; }

std::expected<Phase, Error> BringUpActivity::step(const Cycle& c, Outbox& out) {
    if (current_) {
        const int i = static_cast<int>(*current_);
        for (const Reply& r : c.replies) {
            if (r.method != method(i)) continue;
            if (!r.result) return std::unexpected(command_failed(r.method, r.result.error()));
            if (const auto ok = command::ReplyDecoder<command::Ok>::decode(*r.result); !ok)
                return std::unexpected(command_failed(r.method, ok.error()));
            replied_ = true;
        }
        if (!replied_ || !c.conditions.has(effect(i))) {
            if (c.now - sent_at_ > spec_.step_timeout)
                return std::unexpected(Error{Error::Kind::Command,
                                             std::format("bring_up: {} sent, but no {} after {} ms", method(i),
                                                         to_string(effect(i)), spec_.step_timeout.count()),
                                             {}});
            return Phase::Running;
        }
        current_.reset();
    }

    // The first step the state still needs, in the vendor's order.
    for (int i = 0; i < 4; ++i) {
        if (c.conditions.has(effect(i))) continue;
        switch (static_cast<Step>(i)) {
            case Step::ClearAlarm: out.send(command::ClearAlarm{}); break;
            case Step::ResetCollision: out.send(command::ResetCollisionState{}); break;
            case Step::Sync: out.send(command::SyncMotorStatus{}); break;
            case Step::ServoOn: out.send(command::SetServoStatus{.on = true}); break;
        }
        current_ = static_cast<Step>(i);
        sent_at_ = c.now;
        replied_ = false;
        return Phase::Running;
    }
    return c.conditions.contains(kCanMove) ? Phase::Done : Phase::Running;
}

}  // namespace ec::motion
