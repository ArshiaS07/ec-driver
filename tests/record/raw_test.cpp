// Raw recordings: what goes in comes back out, byte for byte, and files that
// are cut short or are not recordings are told apart.

#include "ec/record/raw.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <fstream>
#include <vector>

#include "support/frame_builder.hpp"
#include "support/temp_file.hpp"

namespace {

using namespace std::chrono_literals;
using ec::feedback::Sample;
using ec::record::RawReader;
using ec::record::RawWriter;
using ec::test::TempFile;

std::vector<Sample> samples(int n) {
    std::vector<Sample> out;
    out.reserve(static_cast<std::size_t>(n));
    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < n; ++i)
        out.push_back(Sample{t0 + i * 8ms, ec::test::make_frame(1000u + static_cast<std::uint64_t>(i))});
    return out;
}

void record(const TempFile& file, const std::vector<Sample>& in) {
    auto writer = RawWriter::create(file.path());
    ASSERT_TRUE(writer) << writer.error();
    for (const Sample& s : in) writer->write(s);
    EXPECT_EQ(writer->written(), in.size());
    const auto closed = writer->close();
    ASSERT_TRUE(closed) << closed.error();
}

TEST(RawRecording, RoundTripIsExact) {
    const TempFile file("round_trip.ecraw");
    const auto in = samples(3);
    record(file, in);
    EXPECT_EQ(std::filesystem::file_size(file.path()), ec::record::kRawHeaderSize + 3 * ec::record::kRawRecordSize);

    auto reader = RawReader::open(file.path());
    ASSERT_TRUE(reader) << reader.error();
    EXPECT_GT(reader->clocks().system_ns, 1'600'000'000'000'000'000);  // after 2020: a real wall-clock time
    for (const Sample& expected : in) {
        const auto got = reader->next();
        ASSERT_TRUE(got.has_value());
        EXPECT_EQ(got->arrival, expected.arrival);
        EXPECT_EQ(got->raw, expected.raw);
    }
    EXPECT_FALSE(reader->next().has_value());
    EXPECT_FALSE(reader->truncated());
}

TEST(RawRecording, RecordCutShortEndsTheRecordingAndIsReported) {
    const TempFile file("cut_short.ecraw");
    record(file, samples(2));
    std::filesystem::resize_file(file.path(), std::filesystem::file_size(file.path()) - 100);

    auto reader = RawReader::open(file.path());
    ASSERT_TRUE(reader) << reader.error();
    EXPECT_TRUE(reader->next().has_value());
    EXPECT_FALSE(reader->next().has_value());
    EXPECT_TRUE(reader->truncated());
}

TEST(RawRecording, OtherFilesAreRefused) {
    const TempFile file("not_a_recording.ecraw");
    std::ofstream(file.path()) << "not a recording, but long enough to hold a header";
    EXPECT_FALSE(RawReader::open(file.path()));
    EXPECT_FALSE(RawReader::open(file.path().string() + ".missing"));
}

}  // namespace
