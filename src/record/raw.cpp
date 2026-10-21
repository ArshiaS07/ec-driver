#include "ec/record/raw.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <format>
#include <utility>

namespace ec::record {
namespace {

constexpr std::array<char, 8> kMagic{'E', 'C', 'R', 'A', 'W', '0', '0', '1'};

// Our own fields are little-endian, whatever the host.
std::array<char, 8> little_endian(std::int64_t v) {
    auto u = std::bit_cast<std::uint64_t>(v);
    if constexpr (std::endian::native == std::endian::big) u = std::byteswap(u);
    return std::bit_cast<std::array<char, 8>>(u);
}

std::int64_t from_little_endian(const char* bytes) {
    std::array<char, 8> b{};
    std::copy_n(bytes, b.size(), b.begin());
    auto u = std::bit_cast<std::uint64_t>(b);
    if constexpr (std::endian::native == std::endian::big) u = std::byteswap(u);
    return std::bit_cast<std::int64_t>(u);
}

}  // namespace

RawWriter::RawWriter(std::ofstream out, std::filesystem::path path) : out_(std::move(out)), path_(std::move(path)) {}

std::expected<RawWriter, std::string> RawWriter::create(const std::filesystem::path& path) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) return std::unexpected(std::format("{}: cannot create the file", path.string()));
    const ClockPair clocks = ClockPair::now();
    const auto steady = little_endian(clocks.steady_ns);
    const auto system = little_endian(clocks.system_ns);
    out.write(kMagic.data(), kMagic.size());
    out.write(steady.data(), steady.size());
    out.write(system.data(), system.size());
    if (!out) return std::unexpected(std::format("{}: cannot write the header", path.string()));
    return RawWriter(std::move(out), path);
}

void RawWriter::write(const feedback::Sample& sample) {
    if (error_) return;
    const auto arrival = little_endian(to_ns(sample.arrival));
    out_.write(arrival.data(), arrival.size());
    out_.write(reinterpret_cast<const char*>(sample.raw.data()), static_cast<std::streamsize>(sample.raw.size()));
    if (!out_) {
        error_ = std::format("{}: write failed after {} records", path_.string(), written_);
        return;
    }
    ++written_;
}

std::expected<void, std::string> RawWriter::close() {
    if (out_.is_open()) {
        out_.close();
        if (!out_ && !error_) error_ = std::format("{}: closing failed", path_.string());
    }
    if (error_) return std::unexpected(*error_);
    return {};
}

RawReader::RawReader(std::ifstream in, ClockPair clocks) : in_(std::move(in)), clocks_(clocks) {}

std::expected<RawReader, std::string> RawReader::open(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return std::unexpected(std::format("{}: cannot open the file", path.string()));
    std::array<char, kRawHeaderSize> header{};
    if (!in.read(header.data(), header.size()) || !std::equal(kMagic.begin(), kMagic.end(), header.begin()))
        return std::unexpected(std::format("{}: not a raw recording of this version", path.string()));
    const ClockPair clocks{from_little_endian(header.data() + 8), from_little_endian(header.data() + 16)};
    return RawReader(std::move(in), clocks);
}

std::optional<feedback::Sample> RawReader::next() {
    std::array<char, kRawRecordSize> record{};
    in_.read(record.data(), record.size());
    const auto got = in_.gcount();
    if (got == 0) return std::nullopt;
    if (got < static_cast<std::streamsize>(record.size())) {
        truncated_ = true;
        return std::nullopt;
    }
    feedback::Sample sample;
    sample.arrival = from_ns(from_little_endian(record.data()));
    std::copy_n(record.begin() + 8, feedback::kFrameSize, reinterpret_cast<char*>(sample.raw.data()));
    return sample;
}

}  // namespace ec::record
