#include "ec/programs.hpp"

#include <cstddef>

#include "detail/connection.hpp"
#include "detail/part.hpp"
#include "ec/pose.hpp"
#include "ec/sdk/param.hpp"
#include "ec/sdk/variables.hpp"

namespace ec {
namespace {

using detail::done;
using detail::on;

std::expected<void, Error> address(int a) { return detail::in_range("variable address", a, 0, 255); }

std::expected<void, Error> invalid(const char* what) { return std::unexpected(Error{Error::Kind::Invalid, what}); }

}  // namespace

std::expected<bool, Error> Programs::exists(const std::string& name) const {
    return on(detail::send(session_, sdk::CheckJbiExist{.filename = name}));
}

std::expected<void, Error> Programs::run(const std::string& name) const {
    if (name.empty()) return invalid("runJbi: no program name");
    return done(detail::send(session_, sdk::RunJbi{.filename = name}));
}

std::expected<sdk::JbiState, Error> Programs::state() const { return detail::send(session_, sdk::GetJbiState{}); }

std::expected<int, Error> Programs::line() const { return detail::send(session_, sdk::GetCurrentJobLine{}); }

std::expected<void, Error> Programs::pause() const { return done(detail::send(session_, sdk::Pause{})); }

std::expected<void, Error> Programs::resume() const { return done(detail::send(session_, sdk::Run{})); }

std::expected<CycleMode, Error> Programs::cycle_mode() const { return detail::send(session_, sdk::GetCycleMode{}); }

std::expected<void, Error> Programs::set_cycle_mode(CycleMode mode) const {
    if (!is_valid(mode)) return invalid("setCycleMode: not a cycle mode");
    return done(detail::send(session_, sdk::SetCycleMode{.mode = mode}));
}

std::expected<std::int64_t, Error> Programs::b(int a) const {
    if (auto ok = address(a); !ok) return std::unexpected(ok.error());
    return detail::send(session_, sdk::GetSysVarB{.address = a});
}

std::expected<void, Error> Programs::set_b(int a, std::int64_t value) const {
    if (auto ok = address(a); !ok) return ok;
    if (value < 0 || value > 2147483647) return invalid("setSysVarB: 0..2147483647");
    return done(detail::send(session_, sdk::SetSysVarB{.address = a, .value = value}));
}

std::expected<int, Error> Programs::i(int a) const {
    if (auto ok = address(a); !ok) return std::unexpected(ok.error());
    return detail::send(session_, sdk::GetSysVarI{.address = a});
}

std::expected<void, Error> Programs::set_i(int a, int value) const {
    if (auto ok = address(a); !ok) return ok;
    if (value < -32767 || value > 32767) return invalid("setSysVarI: -32767..32767");
    return done(detail::send(session_, sdk::SetSysVarI{.address = a, .value = value}));
}

std::expected<double, Error> Programs::d(int a) const {
    if (auto ok = address(a); !ok) return std::unexpected(ok.error());
    return detail::send(session_, sdk::GetSysVarD{.address = a});
}

std::expected<void, Error> Programs::set_d(int a, double value) const {
    if (auto ok = address(a); !ok) return ok;
    if (!(value >= -1e9 && value <= 1e9)) return invalid("setSysVarD: -1e9..1e9");
    return done(detail::send(session_, sdk::SetSysVarD{.address = a, .value = value}));
}

std::expected<Joints, Error> Programs::p(int a) const {
    if (auto ok = address(a); !ok) return std::unexpected(ok.error());
    const auto all = detail::send(session_, sdk::GetSysVarP{.address = a});
    if (!all) return std::unexpected(all.error());
    Joints q{};
    for (std::size_t j = 0; j < q.size(); ++j) q[j] = deg_to_rad((*all)[j]);
    return q;
}

std::expected<void, Error> Programs::set_p(int a, const Joints& q) const {
    if (auto ok = address(a); !ok) return ok;
    return done(detail::send(session_, sdk::SetSysVarP{.address = a, .joints = rad_to_deg(q)}));
}

std::expected<bool, Error> Programs::p_enabled(int a) const {
    if (auto ok = address(a); !ok) return std::unexpected(ok.error());
    return on(detail::send(session_, sdk::GetSysVarPState{.address = a}));
}

std::expected<Eigen::Isometry3d, Error> Programs::v(int a) const {
    if (auto ok = address(a); !ok) return std::unexpected(ok.error());
    const auto pose = detail::send(session_, sdk::GetSysVarV{.address = a});
    if (!pose) return std::unexpected(pose.error());
    return from_mm_rad(*pose);
}

std::expected<void, Error> Programs::set_v(int a, const Eigen::Isometry3d& pose) const {
    if (auto ok = address(a); !ok) return ok;
    return done(detail::send(session_, sdk::SetSysVarV{.address = a, .pose = to_mm_rad(pose)}));
}

std::expected<void, Error> Programs::save_variables() const { return done(detail::send(session_, sdk::SaveVarData{})); }

}  // namespace ec
