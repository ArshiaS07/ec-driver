#include "ec/model.hpp"

#include <format>
#include <string_view>

#include "detail/connection.hpp"
#include "detail/part.hpp"
#include "ec/pose.hpp"
#include "ec/sdk/kinematics.hpp"
#include "ec/sdk/param.hpp"

namespace ec {
namespace {

constexpr double kMmPerMetre = 1000.0;

using detail::done;

// Tools and user frames: 0..7 [S 2.2.2.11, 2.2.2.13].
std::expected<void, Error> check_number(std::string_view what, int n) { return detail::in_range(what, n, 0, 7); }

std::expected<Eigen::Isometry3d, Error> pose(const std::expected<sdk::PoseMmRad, Error>& p) {
    if (!p) return std::unexpected(p.error());
    return from_mm_rad(*p);
}

}  // namespace

std::expected<Joints, Error> Model::ik(const Eigen::Isometry3d& T_base_tcp, const std::optional<Joints>& near) const {
    sdk::InverseKinematic call{.target = to_mm_rad(T_base_tcp)};
    if (near) call.reference = rad_to_deg(*near);
    const auto q = detail::send(session_, call);
    if (!q) return std::unexpected(q.error());
    return deg_to_rad(*q);
}

std::expected<Eigen::Isometry3d, Error> Model::fk(const Joints& q) const {
    return pose(detail::send(session_, sdk::PositiveKinematic{.joints = rad_to_deg(q)}));
}

std::expected<Eigen::Isometry3d, Error> Model::to_user(const Eigen::Isometry3d& T_base_x, int user) const {
    if (auto ok = check_number("user frame", user); !ok) return std::unexpected(ok.error());
    return pose(
        detail::send(session_, sdk::ConvertPoseFromCartToUser{.pose = to_mm_rad(T_base_x), .user_frame = user}));
}

std::expected<Eigen::Isometry3d, Error> Model::from_user(const Eigen::Isometry3d& T_user_x, int user) const {
    if (auto ok = check_number("user frame", user); !ok) return std::unexpected(ok.error());
    return pose(
        detail::send(session_, sdk::ConvertPoseFromUserToCart{.pose = to_mm_rad(T_user_x), .user_frame = user}));
}

std::expected<double, Error> Model::dh(int index) const {
    if (auto ok = detail::in_range("getDH index", index, 0, 11); !ok) return std::unexpected(ok.error());
    return detail::send(session_, sdk::GetDH{.index = index});
}

std::expected<int, Error> Model::robot_type() const { return detail::send(session_, sdk::GetRobotType{}); }

std::expected<int, Error> Model::robot_subtype() const { return detail::send(session_, sdk::GetRobotSubtype{}); }

std::expected<int, Error> Model::active_tool() const { return detail::send(session_, sdk::GetAutoRunToolNumber{}); }

std::expected<void, Error> Model::select_tool(int tool) const {
    if (auto ok = check_number("tool", tool); !ok) return std::unexpected(ok.error());
    return done(detail::send(session_, sdk::SetAutoRunToolNumber{.tool = tool}));
}

std::expected<Eigen::Isometry3d, Error> Model::tool(int tool) const {
    if (auto ok = check_number("tool", tool); !ok) return std::unexpected(ok.error());
    return pose(detail::send(session_, sdk::GetTcpPos{.tool = tool}));
}

std::expected<void, Error> Model::set_tool(int tool, const Eigen::Isometry3d& T_flange_tcp) const {
    if (auto ok = check_number("tool", tool); !ok) return std::unexpected(ok.error());
    if (T_flange_tcp.translation().cwiseAbs().maxCoeff() > 0.5)  // [-500, 500] mm [S 2.2.2.20]
        return std::unexpected(Error{Error::Kind::Invalid, "cmd_set_tcp: x, y, z within -0.5..0.5 m"});
    return done(detail::send(session_, sdk::CmdSetTcp{.tool = tool, .point = to_mm_rad(T_flange_tcp)}));
}

std::expected<Payload, Error> Model::payload(int tool) const {
    if (auto ok = check_number("tool", tool); !ok) return std::unexpected(ok.error());
    const auto p = detail::send(session_, sdk::GetToolPayload{.tool = tool});
    if (!p) return std::unexpected(p.error());
    return Payload{.mass = p->mass_kg,
                   .centre_of_mass = Eigen::Vector3d(p->cog_mm[0], p->cog_mm[1], p->cog_mm[2]) / kMmPerMetre};
}

std::expected<void, Error> Model::set_payload(int tool, const Payload& p) const {
    if (auto ok = check_number("tool", tool); !ok) return std::unexpected(ok.error());
    // EA66: m in [0, 7.2] kg; cog x, y, z in [-5000, 5000] mm [S 2.2.2.19].
    if (!(p.mass >= 0.0 && p.mass <= 7.2))
        return std::unexpected(Error{Error::Kind::Invalid, "cmd_set_payload: mass within 0..7.2 kg (EA66)"});
    if (!(p.centre_of_mass.cwiseAbs().maxCoeff() <= 5.0))
        return std::unexpected(Error{Error::Kind::Invalid, "cmd_set_payload: centre of mass within -5..5 m"});
    const Eigen::Vector3d cog = p.centre_of_mass * kMmPerMetre;
    return done(detail::send(
        session_, sdk::CmdSetPayload{.tool = tool, .mass_kg = p.mass, .cog_mm = {cog.x(), cog.y(), cog.z()}}));
}

std::expected<int, Error> Model::active_user_frame() const { return detail::send(session_, sdk::GetUserNumber{}); }

std::expected<void, Error> Model::select_user_frame(int user) const {
    if (auto ok = check_number("user frame", user); !ok) return std::unexpected(ok.error());
    return done(detail::send(session_, sdk::SetUserNumber{.user_frame = user}));
}

std::expected<Eigen::Isometry3d, Error> Model::user_frame(int user) const {
    if (auto ok = check_number("user frame", user); !ok) return std::unexpected(ok.error());
    return pose(detail::send(session_, sdk::GetUserFrame{.user_frame = user}));
}

std::expected<void, Error> Model::set_user_frame(int user, const Eigen::Isometry3d& T_base_user) const {
    if (auto ok = check_number("user frame", user); !ok) return std::unexpected(ok.error());
    return done(detail::send(session_, sdk::SetUserFrame{.user_frame = user, .frame = to_mm_rad(T_base_user)}));
}

}  // namespace ec
