#include "ec/pose.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <format>

namespace ec {
namespace {

double seconds(Clock::duration d) { return std::chrono::duration<double>(d).count(); }

bool is_rotation(const Eigen::Matrix3d& r) {
    return r.allFinite() && (r.transpose() * r - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() < 1e-6 &&
           r.determinant() > 0.0;
}

}  // namespace

Eigen::Matrix3d rotation_from_rxyz(double rx, double ry, double rz) noexcept {
    return (Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
}

Eigen::Vector3d rxyz_from_rotation(const Eigen::Matrix3d& r) noexcept {
    // R = Rz Ry Rx: r(2,0) = -sin ry; r(1,0)/r(0,0) = tan rz; r(2,1)/r(2,2) = tan rx.
    const double cy = std::hypot(r(0, 0), r(1, 0));
    const double ry = std::atan2(-r(2, 0), cy);
    if (cy < 1e-9) return {0.0, ry, std::atan2(-r(0, 1), r(1, 1))};  // gimbal lock: rx = 0 carries none
    return {std::atan2(r(2, 1), r(2, 2)), ry, std::atan2(r(1, 0), r(0, 0))};
}

Eigen::Isometry3d isometry(const std::array<double, 3>& position, const std::array<double, 3>& rxyz) noexcept {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(position[0], position[1], position[2]);
    pose.linear() = rotation_from_rxyz(rxyz[0], rxyz[1], rxyz[2]);
    return pose;
}

Eigen::Isometry3d from_mm_rad(const std::array<double, 6>& p) noexcept {
    return isometry({p[0] / 1000.0, p[1] / 1000.0, p[2] / 1000.0}, {p[3], p[4], p[5]});
}

std::array<double, 6> to_mm_rad(const Eigen::Isometry3d& pose) noexcept {
    const Eigen::Vector3d t = pose.translation() * 1000.0;
    const Eigen::Vector3d r = rxyz_from_rotation(pose.linear());
    return {t.x(), t.y(), t.z(), r.x(), r.y(), r.z()};
}

double angle_between(const Eigen::Matrix3d& a, const Eigen::Matrix3d& b) noexcept {
    const Eigen::Quaterniond q(a.transpose() * b);  // from the quaternion: exact for small angles too
    return 2.0 * std::atan2(q.vec().norm(), std::abs(q.w()));
}

Eigen::Isometry3d interpolate(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b, double u) noexcept {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = a.translation() + u * (b.translation() - a.translation());
    pose.linear() = Eigen::Quaterniond(a.linear()).slerp(u, Eigen::Quaterniond(b.linear())).toRotationMatrix();
    return pose;
}

std::expected<void, std::string> validate(const PoseTrajectory& trajectory) {
    if (trajectory.size() < 2)
        return std::unexpected(std::format("{} poses; a trajectory needs at least 2", trajectory.size()));
    if (trajectory.front().stamp != Clock::duration::zero())
        return std::unexpected(std::format("it starts at {} s, not 0", seconds(trajectory.front().stamp)));
    for (std::size_t k = 0; k < trajectory.size(); ++k) {
        const auto& p = trajectory[k];
        if (k > 0 && !(p.stamp > trajectory[k - 1].stamp))
            return std::unexpected(std::format("pose {}: its stamp, {} s, does not increase", k, seconds(p.stamp)));
        if (!p.value.translation().allFinite() || !is_rotation(p.value.linear()))
            return std::unexpected(std::format("pose {}: not a rigid transform (a finite position and a rotation)", k));
    }
    return {};
}

Eigen::Isometry3d pose_at(const PoseTrajectory& trajectory, Clock::duration t) noexcept {
    if (trajectory.empty()) return Eigen::Isometry3d::Identity();
    if (t <= trajectory.front().stamp) return trajectory.front().value;
    if (t >= trajectory.back().stamp) return trajectory.back().value;
    const auto next = std::upper_bound(trajectory.begin(), trajectory.end(), t,
                                       [](Clock::duration time, const auto& p) { return time < p.stamp; });
    const auto& a = *(next - 1);
    const auto& b = *next;
    return interpolate(a.value, b.value, seconds(t - a.stamp) / seconds(b.stamp - a.stamp));
}

}  // namespace ec
