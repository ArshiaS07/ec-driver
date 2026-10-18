#include "ec/pose.hpp"

#include <cmath>

namespace ec {

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

}  // namespace ec
