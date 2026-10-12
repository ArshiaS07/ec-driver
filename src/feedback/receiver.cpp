#include "ec/feedback/receiver.hpp"

#include <algorithm>
#include <exception>
#include <utility>

#include "ec/feedback/reader.hpp"

namespace ec::feedback {

namespace {

// How often a connection without data checks for stop() and the silence timeout.
constexpr Receiver::Timeout kSlice{100};

Framer::Stats operator+(const Framer::Stats& a, const Framer::Stats& b) noexcept {
    return {a.frames + b.frames, a.resyncs + b.resyncs, a.bytes_discarded + b.bytes_discarded};
}

}  // namespace

Receiver::Receiver(std::string host, Callback on_sample) : Receiver(std::move(host), Options{}, std::move(on_sample)) {}

Receiver::Receiver(std::string host, const Options& options, Callback on_sample)
    : host_(std::move(host)),
      options_(options),
      on_sample_(std::move(on_sample)),
      thread_([this](const std::stop_token& stop) { run(stop); }) {}

Receiver::~Receiver() { stop(); }

void Receiver::stop() noexcept {
    thread_.request_stop();
    if (thread_.joinable() && thread_.get_id() != std::this_thread::get_id()) thread_.join();
}

Receiver::Status Receiver::status() const {
    std::lock_guard lock(mutex_);
    return status_;
}

bool Receiver::pause(const std::stop_token& stop, Timeout delay) {
    std::unique_lock lock(mutex_);
    wake_.wait_for(lock, stop, delay, [] { return false; });
    return !stop.stop_requested();
}

void Receiver::run(const std::stop_token& stop) {
    Framer::Stats closed{};  // framing statistics of the connections already closed
    for (bool first = true; !stop.stop_requested(); first = false) {
        if (!first && !pause(stop, options_.retry_delay)) return;
        auto reader = Reader::connect(host_, options_.port, options_.connect_timeout);
        if (!reader) {
            std::lock_guard lock(mutex_);
            status_.last_error = reader.error();
            continue;
        }
        {
            std::lock_guard lock(mutex_);
            status_.connected = true;
            ++status_.connections;
        }

        auto last_frame = std::chrono::steady_clock::now();
        std::optional<net::NetError> lost;
        while (!lost && !stop.stop_requested()) {
            const auto sample = reader->next(std::min(kSlice, options_.silence_timeout));
            if (!sample) {
                const bool silent = std::chrono::steady_clock::now() - last_frame >= options_.silence_timeout;
                if (sample.error().kind != net::NetError::Kind::Timeout || silent) lost = sample.error();
                continue;
            }
            last_frame = sample->arrival;
            std::string failure;
            try {
                on_sample_(*sample);
            } catch (const std::exception& e) {
                failure = std::string("the callback threw: ") + e.what();
            } catch (...) {
                failure = "the callback threw";
            }
            std::lock_guard lock(mutex_);
            if (!failure.empty()) {
                status_.connected = false;
                status_.failure = std::move(failure);
                return;
            }
            ++status_.samples;
            status_.framing = closed + reader->stats();
        }

        closed = closed + reader->stats();
        std::lock_guard lock(mutex_);
        status_.connected = false;
        status_.framing = closed;
        if (lost) status_.last_error = lost;
    }
}

}  // namespace ec::feedback
