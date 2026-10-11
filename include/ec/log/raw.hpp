#pragma once
/// @file
/// Raw recordings of the monitoring stream: every frame exactly as received,
/// with its arrival time. The format is in docs/log-format.md.

#include <cstddef>
#include <cstdint>
#include <expected>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>

#include "ec/feedback/sample.hpp"
#include "ec/log/clock.hpp"
#include "ec/log/sink.hpp"

namespace ec::log {

inline constexpr std::size_t kRawHeaderSize = 24;                        ///< magic, then the ClockPair
inline constexpr std::size_t kRawRecordSize = 8 + feedback::kFrameSize;  ///< arrival ns, then the frame

/// Writes a new raw recording, one record per sample.
class RawWriter final : public Sink<feedback::Sample> {
public:
    /// Creates `path`, replacing any file there, and writes the header.
    [[nodiscard]] static std::expected<RawWriter, std::string> create(const std::filesystem::path& path);

    /// Appends one record. After a failure, writes nothing more; see error().
    void write(const feedback::Sample& sample) override;

    /// Writes what is buffered and closes the file; the first failure, if any.
    [[nodiscard]] std::expected<void, std::string> close();

    [[nodiscard]] const std::optional<std::string>& error() const noexcept { return error_; }
    [[nodiscard]] std::uint64_t written() const noexcept { return written_; }

private:
    RawWriter(std::ofstream out, std::filesystem::path path);

    std::ofstream out_;
    std::filesystem::path path_;
    std::optional<std::string> error_;
    std::uint64_t written_ = 0;
};

/// Reads a raw recording back, record by record.
class RawReader {
public:
    /// Opens `path` and checks its header.
    [[nodiscard]] static std::expected<RawReader, std::string> open(const std::filesystem::path& path);

    /// The next sample, or nullopt at the end. A last record cut short (a
    /// recording that stopped abruptly) also ends it; truncated() then says so.
    [[nodiscard]] std::optional<feedback::Sample> next();

    /// The clocks read when the recording started.
    [[nodiscard]] const ClockPair& clocks() const noexcept { return clocks_; }
    [[nodiscard]] bool truncated() const noexcept { return truncated_; }

private:
    RawReader(std::ifstream in, ClockPair clocks);

    std::ifstream in_;
    ClockPair clocks_;
    bool truncated_ = false;
};

}  // namespace ec::log
