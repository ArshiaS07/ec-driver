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
#include "ec/units.hpp"

namespace ec::record {
namespace {

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

template <>
struct Storage<Eigen::Vector3d> {
    using Element = double;
    static std::vector<std::size_t> cell() { return {3}; }
    static HighFive::DataType type() { return HighFive::create_datatype<double>(); }
    static void append(std::vector<Element>& out, const Eigen::Vector3d& v) {
        out.insert(out.end(), {v.x(), v.y(), v.z()});
    }
};

// A pose as its 4x4 homogeneous matrix, row by row (HDF5 is row-major; Eigen
// stores columns).
template <>
struct Storage<Eigen::Isometry3d> {
    using Element = double;
    static std::vector<std::size_t> cell() { return {4, 4}; }
    static HighFive::DataType type() { return HighFive::create_datatype<double>(); }
    static void append(std::vector<Element>& out, const Eigen::Isometry3d& pose) {
        const Eigen::Matrix4d& m = pose.matrix();
        for (Eigen::Index r = 0; r < 4; ++r)
            for (Eigen::Index c = 0; c < 4; ++c) out.push_back(m(r, c));
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

// One dataset: where it goes in the writer's group (`part`, empty for the
// group itself), its name, its unit (none for enums, booleans and bit sets,
// which are not quantities), and how a row's value is read from a State.
template <typename Get>
struct Column {
    const char* part;
    const char* name;
    const char* unit;
    Get get;
};

template <typename Get>
constexpr Column<Get> column(const char* part, const char* name, const char* unit, Get get) {
    return {part, name, unit, get};
}

// One member of every joint, J1..J6.
template <double JointState::* Member>
std::array<double, kJointCount> per_joint(const State& s) {
    std::array<double, kJointCount> out{};
    for (std::size_t i = 0; i < kJointCount; ++i) out[i] = s.joints[i].*Member;
    return out;
}

// Every member of ec::State, in its order and under its part's name. Keep in
// step with the struct and with docs/recording-format.md.
constexpr auto kStateColumns = std::make_tuple(
    column("", "arrival", "ns", [](const State& s) { return to_ns(s.arrival); }),
    column("", "controller_time", "ms", [](const State& s) { return s.controller_time.time_since_epoch().count(); }),
    column("joints", "position", "rad", per_joint<&JointState::position>),
    column("joints", "velocity", "rad/s", per_joint<&JointState::velocity>),
    column("joints", "acceleration", "rad/s^2", per_joint<&JointState::acceleration>),
    column("joints", "torque", "N m", per_joint<&JointState::torque>),
    column("joints", "external_torque", "N m", per_joint<&JointState::external_torque>),
    column("joints", "torque_of_rated", "1", per_joint<&JointState::torque_of_rated>),
    column("joints", "motor_speed", "rad/s", per_joint<&JointState::motor_speed>),
    column("joints", "temperature", "degC", per_joint<&JointState::temperature>),
    column("tcp", "pose", "m", [](const State& s) { return s.tcp.pose; }),
    column("tcp", "pose_in_user", "m", [](const State& s) { return s.tcp.pose_in_user; }),
    column("tcp", "flange_pose", "m", [](const State& s) { return s.tcp.flange_pose; }),
    column("tcp", "flange_pose_in_user", "m", [](const State& s) { return s.tcp.flange_pose_in_user; }),
    column("tcp", "speed", "m/s", [](const State& s) { return s.tcp.speed; }),
    column("tcp", "acceleration", "m/s^2", [](const State& s) { return s.tcp.acceleration; }),
    column("tcp", "force", "N", [](const State& s) { return s.tcp.force; }),
    column("tcp", "torque", "N m", [](const State& s) { return s.tcp.torque; }),
    column("drives", "servo_on", "", [](const State& s) { return s.drives.servo_on; }),
    column("drives", "synced", "", [](const State& s) { return s.drives.synced; }),
    column("motion", "state", "", [](const State& s) { return s.motion.state; }),
    column("motion", "hand_guiding", "", [](const State& s) { return s.motion.hand_guiding; }),
    column("safety", "emergency_stop", "", [](const State& s) { return s.safety.emergency_stop; }),
    column("safety", "collision", "", [](const State& s) { return s.safety.collision; }),
    column("io", "digital_inputs", "",
           [](const State& s) { return static_cast<std::uint64_t>(s.io.digital_inputs.to_ullong()); }),
    column("io", "digital_outputs", "",
           [](const State& s) { return static_cast<std::uint64_t>(s.io.digital_outputs.to_ullong()); }),
    column("io", "analog_inputs", "V", [](const State& s) { return s.io.analog_inputs; }),
    column("io", "analog_outputs", "V", [](const State& s) { return s.io.analog_outputs; }),
    column("programs", "cycle_mode", "", [](const State& s) { return s.programs.cycle_mode; }),
    column("system", "mode", "", [](const State& s) { return s.system.mode; }),
    column("system", "force_sensor_connected", "", [](const State& s) { return s.system.force_sensor_connected; }));

// ---------------------------------------------------------------- writing

// The group at `path` below `group` (from the root if `path` starts with a
// slash), created with its parents if missing.
HighFive::Group require_group(HighFive::Group group, std::string_view path) {
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

template <typename Get>
void append_column(HighFive::Group& group, const Column<Get>& column, const std::vector<State>& rows) {
    using T = std::remove_cvref_t<std::invoke_result_t<Get, const State&>>;
    using S = Storage<T>;
    std::size_t per_row = 1;
    for (const std::size_t n : S::cell()) per_row *= n;
    std::vector<typename S::Element> values;
    values.reserve(rows.size() * per_row);
    for (const State& row : rows) S::append(values, column.get(row));
    HighFive::Group part = require_group(group, column.part);
    append(part, column.name, column.unit, S::cell(), S::type(), values, rows.size());
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

// ---------------------------------------------------------------- H5StateWriter

H5StateWriter::H5StateWriter(H5File::Impl* file, std::string group) noexcept : file_(file), group_(std::move(group)) {}

// The Sink base has no state, so it is default-constructed rather than moved.
H5StateWriter::H5StateWriter(H5StateWriter&& other) noexcept
    : file_(std::exchange(other.file_, nullptr)),
      group_(std::move(other.group_)),
      rows_(std::move(other.rows_)),
      error_(std::move(other.error_)),
      written_(other.written_),
      skipped_(other.skipped_) {}

H5StateWriter::~H5StateWriter() { write_buffered(); }

std::expected<H5StateWriter, std::string> H5StateWriter::create(H5File& file, std::string group) {
    try {
        const HighFive::SilenceHDF5 silence;
        require_group(file.impl_->file.getGroup("/"), group);  // now, so that a bad path fails here
    } catch (const std::exception& e) {
        return std::unexpected(std::format("{}: {}", group, e.what()));
    }
    return H5StateWriter(file.impl_.get(), std::move(group));
}

void H5StateWriter::write(const feedback::Sample& sample) {
    if (error_) return;
    const auto frame = feedback::decode(sample.raw);
    if (!frame) {
        ++skipped_;
        return;
    }
    rows_.push_back(state_from(*frame, written_ + rows_.size() + 1, sample.arrival));
    if (rows_.size() >= kChunkRows) write_buffered();
}

std::expected<void, std::string> H5StateWriter::flush() {
    write_buffered();
    if (error_) return std::unexpected(*error_);
    return {};
}

void H5StateWriter::write_buffered() noexcept {
    if (error_ || rows_.empty() || file_ == nullptr) return;
    try {
        const HighFive::SilenceHDF5 silence;
        HighFive::Group group = require_group(file_->file.getGroup("/"), group_);
        std::apply([&](const auto&... columns) { (append_column(group, columns, rows_), ...); }, kStateColumns);
        file_->file.flush();  // what is written survives a crash of this process
        written_ += rows_.size();
        rows_.clear();
    } catch (const std::exception& e) {
        set_error(e.what());
    } catch (...) {
        set_error("unknown error");
    }
}

void H5StateWriter::set_error(const char* what) noexcept {
    try {
        error_ = std::format("{}: {}", group_, what);
    } catch (...) {
        error_.emplace();  // no memory for the message; the writer must still stop
    }
}

}  // namespace ec::record
