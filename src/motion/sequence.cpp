#include "ec/motion/sequence.hpp"

#include <format>

namespace ec::motion {

Needs Sequence::needs() const {
    if (index_ >= steps_.size()) return {};
    return steps_[index_].needs();  // before the start: the first step's
}

std::expected<Phase, Error> Sequence::step(const Cycle& c, Outbox& out) {
    for (;;) {
        if (index_ >= steps_.size()) {
            stage_ = Stage::Finished;
            return Phase::Done;
        }
        AnyActivity& current = steps_[index_];
        switch (stage_) {
            case Stage::Starting: {
                const Conditions missing = current.needs().to_start.missing_from(c.conditions);
                if (!missing.empty()) {
                    if (c.now - waiting_since_ <= start_wait) return Phase::Running;
                    return std::unexpected(Error{Error::Kind::Refused,
                                                 std::format("step {} ({}) cannot start without: {}", index_ + 1,
                                                             current.name(), to_string(missing)),
                                                 missing});
                }
                stage_ = Stage::Stepping;
                [[fallthrough]];
            }
            case Stage::Stepping: {
                const auto r = current.step(c, out);
                if (!r) return std::unexpected(r.error());
                if (*r == Phase::Running) return Phase::Running;
                stage_ = Stage::Closing;
                [[fallthrough]];
            }
            case Stage::Closing:
                if (!current.close(c, out, End::Done)) return Phase::Running;
                ++index_;
                stage_ = Stage::Starting;
                waiting_since_ = c.now;
                break;  // the next step may start in this same cycle
            case Stage::Finished: return Phase::Done;
        }
    }
}

bool Sequence::close(const Cycle& c, Outbox& out, End why) {
    if (stage_ == Stage::Finished || index_ >= steps_.size()) return true;
    if (stage_ == Stage::Starting) return true;  // the previous step is closed, the next never started
    return steps_[index_].close(c, out, why);
}

}  // namespace ec::motion
