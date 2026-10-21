#include "ec/monitor.hpp"

#include <utility>

#include "ec/feedback/frame.hpp"

namespace ec {

Monitor::Monitor() noexcept = default;

Monitor::Monitor(std::string host, Options options) : options_(std::move(options)) {
    receiver_ = std::make_unique<feedback::Receiver>(std::move(host), options_.receiver,
                                                     [this](const feedback::Sample& s) { on_sample(s); });
}

Monitor::~Monitor() = default;

void Monitor::on_sample(const feedback::Sample& sample) {
    if (options_.on_sample) options_.on_sample(sample);
    const auto frame = feedback::decode(sample.raw);
    if (!frame) {
        undecodable_.fetch_add(1);
        return;
    }
    // One thread publishes: the seq given here is the one Latest counts.
    const State state = state_from(*frame, states_.fetch_add(1) + 1, sample.arrival);
    latest_.publish(state);
    if (options_.on_state) options_.on_state(state);
}

std::optional<State> Monitor::latest() const {
    if (auto e = latest_.get()) return std::move(e->value);
    return std::nullopt;
}

std::optional<State> Monitor::next(std::uint64_t after, std::chrono::nanoseconds timeout) const {
    if (auto e = latest_.wait_newer(after, timeout)) return std::move(e->value);
    return std::nullopt;
}

Monitor::Status Monitor::status() const {
    Status s;
    if (receiver_) s.receiver = receiver_->status();
    s.states = states_.load();
    s.undecodable = undecodable_.load();
    return s;
}

}  // namespace ec
