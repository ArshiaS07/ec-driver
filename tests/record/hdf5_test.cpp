// HDF5 recordings, read back with HighFive: the layout of ec::State, shapes, SI
// values, units, enum labels, the root attributes, and rows that span several
// chunks.

#include "ec/record/hdf5.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <highfive/highfive.hpp>
#include <numbers>
#include <string>
#include <vector>

#include "ec/pose.hpp"
#include "support/frame_builder.hpp"
#include "support/temp_file.hpp"

namespace {

using namespace std::chrono_literals;
using ec::feedback::Sample;
using ec::record::H5File;
using ec::record::H5StateWriter;
using ec::test::make_frame;
using ec::test::TempFile;

template <typename T>
std::vector<T> read_all(const HighFive::DataSet& set) {
    std::vector<T> out(set.getElementCount());
    set.read_raw(out.data(), HighFive::create_datatype<T>());
    return out;
}

std::vector<std::string> enum_labels(const HighFive::DataSet& set) {
    const HighFive::DataType datatype = set.getDataType();  // owns the id: keep it alive while using it
    const hid_t type = datatype.getId();
    std::vector<std::string> labels;
    for (int i = 0; i < H5Tget_nmembers(type); ++i) {
        char* name = H5Tget_member_name(type, static_cast<unsigned>(i));
        labels.emplace_back(name);
        H5free_memory(name);
    }
    return labels;
}

TEST(H5Recording, WritesTheStateMemberByMemberWithUnitsAndLabels) {
    const TempFile file("recording.h5");
    const auto t0 = std::chrono::steady_clock::now();
    {
        auto h5 = H5File::create(file.path());
        ASSERT_TRUE(h5) << h5.error();
        auto writer = H5StateWriter::create(*h5, "/robot/state");
        ASSERT_TRUE(writer) << writer.error();
        for (int i = 0; i < 3; ++i)
            writer->write(Sample{t0 + i * 8ms, make_frame(1000u + static_cast<std::uint64_t>(i))});
        auto broken = make_frame();
        broken[ec::feedback::kFrameSize - 1] ^= 0xFF;  // no matching word: does not decode
        writer->write(Sample{t0, broken});
        const auto flushed = writer->flush();
        ASSERT_TRUE(flushed) << flushed.error();
        EXPECT_EQ(writer->written(), 3u);
        EXPECT_EQ(writer->skipped(), 1u);
    }

    const HighFive::File f(file.path().string(), HighFive::File::ReadOnly);
    EXPECT_EQ(f.getAttribute("schema").read<std::string>(), "ec-driver.recording/2");
    EXPECT_GT(f.getAttribute("system_ns_at_start").read<std::int64_t>(), 1'600'000'000'000'000'000);
    EXPECT_TRUE(f.hasAttribute("steady_ns_at_start"));

    const auto arrival = read_all<std::int64_t>(f.getDataSet("/robot/state/arrival"));
    ASSERT_EQ(arrival.size(), 3u);
    EXPECT_EQ(arrival[1] - arrival[0], 8'000'000);  // ns
    EXPECT_EQ(f.getDataSet("/robot/state/arrival").getAttribute("unit").read<std::string>(), "ns");
    EXPECT_EQ(read_all<std::int64_t>(f.getDataSet("/robot/state/controller_time")),
              (std::vector<std::int64_t>{1000, 1001, 1002}));

    const auto position = f.getDataSet("/robot/state/joints/position");
    EXPECT_EQ(position.getDimensions(), (std::vector<std::size_t>{3, 6}));
    EXPECT_EQ(position.getAttribute("unit").read<std::string>(), "rad");
    EXPECT_DOUBLE_EQ(read_all<double>(position)[5], 15.0 * std::numbers::pi / 180.0);  // row 0, J6
    EXPECT_DOUBLE_EQ(read_all<double>(f.getDataSet("/robot/state/joints/torque_of_rated"))[0], 0.300);

    // A pose is its 4x4 matrix, row by row: rotation, translation [m] in the last column.
    const auto pose = f.getDataSet("/robot/state/tcp/pose");
    EXPECT_EQ(pose.getDimensions(), (std::vector<std::size_t>{3, 4, 4}));
    const auto m = read_all<double>(pose);
    const Eigen::Matrix4d expected = ec::from_mm_rad({100, 101, 102, 103, 104, 105}).matrix();
    for (Eigen::Index r = 0; r < 4; ++r)
        for (Eigen::Index c = 0; c < 4; ++c)
            EXPECT_DOUBLE_EQ(m[static_cast<std::size_t>(r * 4 + c)], expected(r, c)) << r << "," << c;
    EXPECT_DOUBLE_EQ(m[3], 0.100);  // x: mm on the wire, m in the recording
    EXPECT_EQ(f.getDataSet("/robot/state/tcp/force").getDimensions(), (std::vector<std::size_t>{3, 3}));
    EXPECT_DOUBLE_EQ(read_all<double>(f.getDataSet("/robot/state/tcp/speed"))[0], 0.0125);

    const auto state = f.getDataSet("/robot/state/motion/state");
    EXPECT_FALSE(state.hasAttribute("unit"));
    const auto labels = enum_labels(state);
    EXPECT_NE(std::find(labels.begin(), labels.end(), "running"), labels.end());
    EXPECT_EQ(read_all<std::int32_t>(state), (std::vector<std::int32_t>{3, 3, 3}));  // running
    EXPECT_EQ(read_all<std::int32_t>(f.getDataSet("/robot/state/system/mode")), (std::vector<std::int32_t>{2, 2, 2}));

    EXPECT_EQ(read_all<std::uint8_t>(f.getDataSet("/robot/state/drives/servo_on")),
              (std::vector<std::uint8_t>{1, 1, 1}));
    EXPECT_EQ(read_all<std::uint64_t>(f.getDataSet("/robot/state/io/digital_inputs"))[0], 0x8001u);
    EXPECT_EQ(f.getDataSet("/robot/state/io/analog_outputs").getDimensions(), (std::vector<std::size_t>{3, 5}));
}

TEST(H5Recording, RowsSpanningSeveralChunksAllArrive) {
    const TempFile file("chunks.h5");
    constexpr int kRows = 2500;  // more than two chunks, written on the way and at the end
    const auto t0 = std::chrono::steady_clock::now();
    {
        auto h5 = H5File::create(file.path());
        ASSERT_TRUE(h5) << h5.error();
        auto writer = H5StateWriter::create(*h5, "/state");
        ASSERT_TRUE(writer) << writer.error();
        for (int i = 0; i < kRows; ++i) writer->write(Sample{t0 + i * 8ms, make_frame(static_cast<std::uint64_t>(i))});
    }  // the rest is written on destruction

    const HighFive::File f(file.path().string(), HighFive::File::ReadOnly);
    const auto time = read_all<std::int64_t>(f.getDataSet("/state/controller_time"));
    ASSERT_EQ(time.size(), static_cast<std::size_t>(kRows));
    EXPECT_EQ(time.front(), 0);
    EXPECT_EQ(time.back(), kRows - 1);
}

TEST(H5Recording, UnwritablePathIsAnError) {
    const auto h5 = H5File::create(std::filesystem::current_path() / "no_such_directory" / "recording.h5");
    EXPECT_FALSE(h5);
}

}  // namespace
