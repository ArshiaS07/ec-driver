#pragma once
/// @file
/// The newest value of something that keeps changing, shared between threads.

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <utility>

#include "ec/stamped.hpp"

namespace ec {

/// Holds the newest of a sequence of values: one thread publishes, any number
/// of threads read it or wait for the next one. A new value replaces the old
/// one; nothing is queued, so a slow reader skips values and can count them
/// from the sequence numbers.
///
///     ec::Latest<ec::feedback::Sample> latest;   // the receiver publishes into it
///     std::uint64_t seen = 0;
///     while (auto e = latest.wait_newer(seen, 50ms)) {
///         seen = e->seq;   // e->value: the newest sample
///     }                    // nullopt: nothing new for 50 ms
///
/// Thread-safe. Each call holds a mutex only while it copies one value in or
/// out; only wait_newer() waits, for at most its timeout. `T` must be copyable,
/// and small enough to copy under a lock (a 1 KB frame takes well under a
/// microsecond).
template <typename T>
class Latest {
public:
    /// A published value with its sequence number: 1 for the first value, one
    /// more for each publish() after it.
    struct Entry {
        std::uint64_t seq;
        T value;
    };

    /// Makes `value` the newest and wakes every waiting reader.
    void publish(T value) {
        {
            std::lock_guard lock(mutex_);
            value_ = std::move(value);
            ++seq_;
        }
        published_.notify_all();
    }

    /// The newest value, or nullopt if none was published yet.
    [[nodiscard]] std::optional<Entry> get() const {
        std::lock_guard lock(mutex_);
        return entry();
    }

    /// The newest value, as soon as its sequence number is greater than `seq`:
    /// at once if it already is. nullopt if `timeout` passes first. Pass 0 to
    /// wait for the first value, or the last Entry::seq seen to wait for the next.
    [[nodiscard]] std::optional<Entry> wait_newer(std::uint64_t seq, std::chrono::nanoseconds timeout) const {
        const auto now = Clock::now();
        const auto deadline = timeout >= Clock::time_point::max() - now ? Clock::time_point::max() : now + timeout;
        std::unique_lock lock(mutex_);
        if (!published_.wait_until(lock, deadline, [&] { return seq_ > seq; })) return std::nullopt;
        return entry();
    }

private:
    std::optional<Entry> entry() const {  // with the mutex held
        if (!value_) return std::nullopt;
        return Entry{seq_, *value_};
    }

    mutable std::mutex mutex_;
    mutable std::condition_variable published_;
    std::uint64_t seq_ = 0;
    std::optional<T> value_;
};

}  // namespace ec
