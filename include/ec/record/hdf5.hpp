#pragma once
/// @file
/// HDF5 recordings: one file, one group per stream, one dataset per field and one row
/// per record, in SI units. The format is in docs/recording-format.md. Part of the
/// optional target ec::hdf5.

#include <cstdint>
#include <expected>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "ec/feedback/sample.hpp"
#include "ec/record/sink.hpp"
#include "ec/state.hpp"

namespace ec::record {

/// The format the files follow; the number changes when existing readers would break.
inline constexpr std::string_view kH5Schema = "ec-driver.recording/2";

/// An HDF5 recording file. It owns the file; writers add their groups to it. Use a
/// file and its writers from one thread. Move-only; closed on destruction.
class H5File {
public:
    /// Creates `path`, replacing any file there, and writes the root
    /// attributes: the schema, and the ClockPair read now.
    [[nodiscard]] static std::expected<H5File, std::string> create(const std::filesystem::path& path);

    H5File(H5File&& other) noexcept;
    H5File& operator=(H5File&& other) noexcept;
    H5File(const H5File&) = delete;
    H5File& operator=(const H5File&) = delete;
    ~H5File();

private:
    friend class H5StateWriter;
    struct Impl;
    explicit H5File(std::unique_ptr<Impl> impl) noexcept;
    std::unique_ptr<Impl> impl_;
};

/// Writes the monitoring stream into one group of an H5File as ec::State: each
/// frame decoded (state_from), one row per frame, one dataset per member, in
/// groups laid out as the struct is (`joints/position`, `tcp/pose`,
/// `drives/servo_on`, ...). Rows are buffered and written a chunk at a time,
/// on flush(), and on destruction. Frames that do not decode are skipped and
/// counted; a raw recording keeps them.
class H5StateWriter final : public Sink<feedback::Sample> {
public:
    /// A writer for the group `group` (e.g. "/state") of `file`; the caller
    /// chooses where it goes. The file must outlive the writer (moving the
    /// H5File handle is fine).
    [[nodiscard]] static std::expected<H5StateWriter, std::string> create(H5File& file, std::string group);

    /// Buffers one row. After a failure, writes nothing more; see error().
    void write(const feedback::Sample& sample) override;

    /// Writes the buffered rows; the first failure, if any.
    [[nodiscard]] std::expected<void, std::string> flush();

    [[nodiscard]] const std::optional<std::string>& error() const noexcept { return error_; }
    [[nodiscard]] std::uint64_t written() const noexcept { return written_; }  ///< rows in the file
    [[nodiscard]] std::uint64_t skipped() const noexcept { return skipped_; }  ///< frames that did not decode

    H5StateWriter(H5StateWriter&& other) noexcept;
    H5StateWriter& operator=(H5StateWriter&&) = delete;
    H5StateWriter(const H5StateWriter&) = delete;
    H5StateWriter& operator=(const H5StateWriter&) = delete;
    ~H5StateWriter() override;

private:
    H5StateWriter(H5File::Impl* file, std::string group) noexcept;
    void write_buffered() noexcept;  // the buffered rows to the file; failures go to error_
    void set_error(const char* what) noexcept;

    H5File::Impl* file_;  // the file's state, which stays put when the H5File handle moves
    std::string group_;
    std::vector<State> rows_;  // buffered
    std::optional<std::string> error_;
    std::uint64_t written_ = 0;
    std::uint64_t skipped_ = 0;
};

}  // namespace ec::record
