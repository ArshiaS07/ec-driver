#include "ec/record/hdf5.hpp"

#include <array>
#include <concepts>
#include <cstddef>
#include <exception>
#include <format>
#include <highfive/highfive.hpp>
#include <tuple>
#include <type_traits>
#include <utility>

#include "ec/feedback/frame.hpp"
#include "ec/record/clock.hpp"
#include "ec/status.hpp"

namespace ec::record {
namespace {

using feedback::State;

constexpr std::size_t kChunkRows = 1024;  // rows per chunk, and per write
constexpr unsigned kDeflateLevel = 4;     // past it, files barely shrink while writing keeps slowing

// ---------------------------------------------------------------- storage
// How a member type is stored: the HDF5 element type, the shape of one row,
// and that row's elements.

template <typename T>
struct Storage;

template <>
struct Storage<double> {
    using Element = double;
    static std::vector<std::size_t> cell() { return {}; }
    static HighFive::DataType type() { return HighFive::create_datatype<double>(); }
    static void append(std::vector<Element>& out, double v) { out.push_back(v); }
};

template <std::size_t N>
struct Storage<std::array<double, N>> {
    using Element = double;
    static std::vector<std::size_t> cell() { return {N}; }
    static HighFive::DataType type() { return HighFive::create_datatype<double>(); }
    static void append(std::vector<Element>& out, const std::array<double, N>& v) {
        out.insert(out.end(), v.begin(), v.end());
    }
};

template <typename I>
    requires std::same_as<I, std::int64_t> || std::same_as<I, std::uint64_t>
struct Storage<I> {
    using Element = I;
    static std::vector<std::size_t> cell() { return {}; }
    static HighFive::DataType type() { return HighFive::create_datatype<I>(); }
    static void append(std::vector<Element>& out, I v) { out.push_back(v); }
};

// Booleans as h5py stores them: a one-byte HDF5 enum, FALSE and TRUE.
template <>
struct Storage<bool> {
    using Element = std::uint8_t;
    static std::vector<std::size_t> cell() { return {}; }
    static HighFive::DataType type() { return HighFive::create_datatype<bool>(); }
    static void append(std::vector<Element>& out, bool v) { out.push_back(v ? 1 : 0); }
};

// Status enums as HDF5 enums labelled with their names, so readers see
// "running". A value the manual does not define is stored as it is, unlabelled.
template <typename E>
    requires std::is_enum_v<E>
struct Storage<E> {
    using Element = E;
    static std::vector<std::size_t> cell() { return {}; }
    static HighFive::DataType type() {
        std::vector<typename HighFive::EnumType<E>::member_def> members;
        for (int v = 0; v <= 255; ++v) {
            const auto e = static_cast<E>(v);
            if (is_valid(e)) members.emplace_back(std::string(to_string(e)), e);
        }
        return HighFive::EnumType<E>(members);
    }
    static void append(std::vector<Element>& out, E v) { out.push_back(v); }
};

// ---------------------------------------------------------------- columns

// One dataset of a record type: the member it holds, and its unit (none for
// enums, booleans and bit sets, which are not quantities).
template <typename T>
struct Column {
    const char* name;
    const char* unit;
    T State::* member;
};

template <typename T>
constexpr Column<T> column(const char* name, const char* unit, T State::* member) {
    return {name, unit, member};
}

// Every field of feedback::State, in its order. Keep in step with the struct
// and with docs/recording-format.md.
constexpr auto kFeedbackColumns = std::make_tuple(
    column("controller_time", "ms", &State::controller_time_ms),
    column("joint_position", "rad", &State::joint_position), column("joint_velocity", "rad/s", &State::joint_velocity),
    column("joint_acceleration", "rad/s^2", &State::joint_acceleration),
    column("joint_torque", "N m", &State::joint_torque),
    column("joint_torque_external", "N m", &State::joint_torque_external),
    column("joint_torque_rated", "1", &State::joint_torque_rated), column("motor_speed", "rad/s", &State::motor_speed),
    column("joint_temperature", "degC", &State::joint_temperature),
    column("tcp_position_base", "m", &State::tcp_position_base),
    column("tcp_rotation_base", "rad", &State::tcp_rotation_base),
    column("tcp_position_user", "m", &State::tcp_position_user),
    column("tcp_rotation_user", "rad", &State::tcp_rotation_user),
    column("flange_position_base", "m", &State::flange_position_base),
    column("flange_rotation_base", "rad", &State::flange_rotation_base),
    column("flange_position_user", "m", &State::flange_position_user),
    column("flange_rotation_user", "rad", &State::flange_rotation_user), column("tcp_speed", "m/s", &State::tcp_speed),
    column("tcp_acceleration", "m/s^2", &State::tcp_acceleration), column("tcp_force", "N", &State::tcp_force),
    column("tcp_torque", "N m", &State::tcp_torque), column("robot_state", "", &State::robot_state),
    column("robot_mode", "", &State::robot_mode), column("cycle_mode", "", &State::cycle_mode),
    column("brakes_released", "", &State::brakes_released), column("motors_synced", "", &State::motors_synced),
    column("collision", "", &State::collision), column("emergency_stop", "", &State::emergency_stop),
    column("drag_enabled", "", &State::drag_enabled),
    column("force_sensor_connected", "", &State::force_sensor_connected),
    column("digital_input", "", &State::digital_input), column("digital_output", "", &State::digital_output),
    column("analog_input", "V", &State::analog_input), column("analog_output", "V", &State::analog_output));

// ---------------------------------------------------------------- writing

// The group at `path`, created with its parents if missing.
HighFive::Group require_group(HighFive::File& file, std::string_view path) {
    HighFive::Group group = file.getGroup("/");
    std::size_t begin = 0;
    while (begin < path.size()) {
        const std::size_t end = std::min(path.find('/', begin), path.size());
        if (end > begin) {
            const std::string name(path.substr(begin, end - begin));
            group = group.exist(name) ? group.getGroup(name) : group.createGroup(name);
        }
        begin = end + 1;
    }
    return group;
}

// Appends `rows` rows to the dataset `name` of `group`. The first call creates
// it: extendible along rows, chunked, byte-shuffled (which groups the bytes of
// similar values, so that deflate finds their runs) and deflated, with its unit.
template <typename Element>
void append(HighFive::Group& group, const std::string& name, std::string_view unit,
            const std::vector<std::size_t>& cell, const HighFive::DataType& type, const std::vector<Element>& values,
            std::size_t rows) {
    std::vector<std::size_t> count{rows};
    count.insert(count.end(), cell.begin(), cell.end());

    if (!group.exist(name)) {
        std::vector<std::size_t> initial = count;
        initial[0] = 0;
        std::vector<std::size_t> limit = count;
        limit[0] = HighFive::DataSpace::UNLIMITED;
        std::vector<std::size_t> chunk = count;
        chunk[0] = kChunkRows;
        HighFive::DataSetCreateProps properties;
        properties.add(HighFive::Chunking(chunk));
        properties.add(HighFive::Shuffle());
        properties.add(HighFive::Deflate(kDeflateLevel));
        HighFive::DataSet created = group.createDataSet(name, HighFive::DataSpace(initial, limit), type, properties);
        if (!unit.empty()) created.createAttribute("unit", std::string(unit));
    }

    HighFive::DataSet set = group.getDataSet(name);
    std::vector<std::size_t> extent = set.getDimensions();
    std::vector<std::size_t> offset(count.size(), 0);
    offset[0] = extent[0];
    extent[0] += rows;
    set.resize(extent);
    set.select(offset, count).write_raw(values.data(), type);
}

template <typename T>
void append_column(HighFive::Group& group, const Column<T>& column, const std::vector<State>& rows) {
    using S = Storage<T>;
    std::vector<typename S::Element> values;
    values.reserve(rows.size() * (S::cell().empty() ? 1 : S::cell()[0]));
    for (const State& row : rows) S::append(values, row.*column.member);
    append(group, column.name, column.unit, S::cell(), S::type(), values, rows.size());
}

}  // namespace

// ---------------------------------------------------------------- H5File

struct H5File::Impl {
    HighFive::File file;
};

H5File::H5File(std::unique_ptr<Impl> impl) noexcept : impl_(std::move(impl)) {}
H5File::H5File(H5File&& other) noexcept = default;
H5File& H5File::operator=(H5File&& other) noexcept = default;
H5File::~H5File() = default;

std::expected<H5File, std::string> H5File::create(const std::filesystem::path& path) {
    try {
        const HighFive::SilenceHDF5 silence;
        // The HDF5 1.10 file format: readable by every HDF5 since 1.10, so by
        // older MATLAB and h5py builds too.
        HighFive::FileAccessProps access;
        access.add(HighFive::FileVersionBounds(H5F_LIBVER_V110, H5F_LIBVER_V110));
        auto impl = std::make_unique<Impl>(Impl{HighFive::File(path.string(), HighFive::File::Truncate, access)});
        const ClockPair clocks = ClockPair::now();
        impl->file.createAttribute("schema", std::string(kH5Schema));
        impl->file.createAttribute("steady_ns_at_start", clocks.steady_ns);
        impl->file.createAttribute("system_ns_at_start", clocks.system_ns);
        return H5File(std::move(impl));
    } catch (const std::exception& e) {
        return std::unexpected(std::format("{}: {}", path.string(), e.what()));
    }
}

// ---------------------------------------------------------------- H5FeedbackWriter

H5FeedbackWriter::H5FeedbackWriter(H5File::Impl* file, std::string group) noexcept
    : file_(file), group_(std::move(group)) {}

// The Sink base has no state, so it is default-constructed rather than moved.
H5FeedbackWriter::H5FeedbackWriter(H5FeedbackWriter&& other) noexcept
    : file_(std::exchange(other.file_, nullptr)),
      group_(std::move(other.group_)),
      t_(std::move(other.t_)),
      rows_(std::move(other.rows_)),
      error_(std::move(other.error_)),
      written_(other.written_),
      skipped_(other.skipped_) {}

H5FeedbackWriter::~H5FeedbackWriter() { write_buffered(); }

std::expected<H5FeedbackWriter, std::string> H5FeedbackWriter::create(H5File& file, std::string group) {
    try {
        const HighFive::SilenceHDF5 silence;
        require_group(file.impl_->file, group);  // now, so that a bad path fails here
    } catch (const std::exception& e) {
        return std::unexpected(std::format("{}: {}", group, e.what()));
    }
    return H5FeedbackWriter(file.impl_.get(), std::move(group));
}

void H5FeedbackWriter::write(const feedback::Sample& sample) {
    if (error_) return;
    const auto frame = feedback::decode(sample.raw);
    if (!frame) {
        ++skipped_;
        return;
    }
    t_.push_back(to_ns(sample.arrival));
    rows_.push_back(feedback::to_si(*frame));
    if (rows_.size() >= kChunkRows) write_buffered();
}

std::expected<void, std::string> H5FeedbackWriter::flush() {
    write_buffered();
    if (error_) return std::unexpected(*error_);
    return {};
}

void H5FeedbackWriter::write_buffered() noexcept {
    if (error_ || rows_.empty() || file_ == nullptr) return;
    try {
        const HighFive::SilenceHDF5 silence;
        HighFive::Group group = require_group(file_->file, group_);
        append(group, "t", "ns", {}, HighFive::create_datatype<std::int64_t>(), t_, rows_.size());
        std::apply([&](const auto&... columns) { (append_column(group, columns, rows_), ...); }, kFeedbackColumns);
        file_->file.flush();  // what is written survives a crash of this process
        written_ += rows_.size();
        t_.clear();
        rows_.clear();
    } catch (const std::exception& e) {
        set_error(e.what());
    } catch (...) {
        set_error("unknown error");
    }
}

void H5FeedbackWriter::set_error(const char* what) noexcept {
    try {
        error_ = std::format("{}: {}", group_, what);
    } catch (...) {
        error_.emplace();  // no memory for the message; the writer must still stop
    }
}

}  // namespace ec::record
