#pragma once
/// @file
/// The call log: every exchange on the command connection, one JSON object per
/// line, on the same steady clock as the recordings so that both line up.
/// The format is in docs/recording-format.md.
///
///     auto log = ec::record::CallWriter::create("run.calls.jsonl").value();
///     options.on_call = [&log](const ec::CallRecord& c) { log.write(c); };

#include <cstdint>
#include <expected>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <string_view>

#include "ec/call_record.hpp"
#include "ec/record/sink.hpp"

namespace ec::record {

/// The format the files follow; the number changes when existing readers would break.
inline constexpr std::string_view kCallsFormat = "ec-driver.calls/1";

/// Writes a new call log. Use it from one thread at a time (a Robot's on_call
/// is: its calls are serialised).
class CallWriter final : public Sink<CallRecord> {
public:
    /// Creates `path`, replacing any file there, and writes the header line:
    /// the format, and the ClockPair read now.
    [[nodiscard]] static std::expected<CallWriter, std::string> create(const std::filesystem::path& path);

    /// Appends one line, flushed (a log survives a crash of this process).
    /// After a failure, writes nothing more; see error().
    void write(const CallRecord& call) override;

    [[nodiscard]] const std::optional<std::string>& error() const noexcept { return error_; }
    [[nodiscard]] std::uint64_t written() const noexcept { return written_; }

private:
    CallWriter(std::ofstream out, std::filesystem::path path);

    std::ofstream out_;
    std::filesystem::path path_;
    std::optional<std::string> error_;
    std::uint64_t written_ = 0;
};

}  // namespace ec::record
