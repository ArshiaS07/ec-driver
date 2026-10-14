#pragma once
/// @file
/// Planned motion in joint space (docs/motion.md 5.11, ADR 0017): a motion of
/// the TCP, a PoseTrajectory (ec/pose.hpp), is solved by inverse kinematics
/// into joint positions, checked, and sampled into a JointTrajectory, which the
/// controller's timestamp service plays (PlayTrajectory). Pure, apart from the
/// inverse kinematics the caller supplies (the controller's, through Robot::ik).

#include <Eigen/Geometry>
#include <cstddef>
#include <expected>
#include <functional>
#include <numbers>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "ec/motion/error.hpp"
#include "ec/motion/trajectory.hpp"
#include "ec/pose.hpp"
#include "ec/units.hpp"

namespace ec::motion {

/// Joint positions interpolated over an increasing parameter by cubic Hermite
/// segments, the slope at each inner knot taken from the parabola through it and
/// its neighbours (Bessel's method): positions and slopes are continuous, and
/// quadratics are reproduced exactly. Where all the positions at two neighbouring
/// knots are the same (a pause), the motion rests at both, instead of passing
/// through; next to a knot at rest the slopes are kept within three times their
/// segment's secant, so that no segment overshoots into or out of rest (Fritsch
/// and Carlson). Outside the knots the end positions hold.
class JointSpline {
public:
    /// The slopes at the first and last knot.
    enum class Ends {
        Parabola,  ///< from the parabola through the three end knots: exact for quadratics
        Rest,      ///< zero: the motion starts and ends at rest
    };

    /// The spline, or why not: at least 2 knots, strictly increasing and
    /// finite, with one finite position per knot.
    [[nodiscard]] static std::expected<JointSpline, std::string> make(std::vector<double> knots,
                                                                      std::vector<Joints> positions,
                                                                      Ends ends = Ends::Parabola);

    [[nodiscard]] Joints position(double x) const noexcept;
    /// dq/dx; zero outside the knots.
    [[nodiscard]] Joints slope(double x) const noexcept;
    [[nodiscard]] const std::vector<double>& knots() const noexcept { return x_; }
    [[nodiscard]] const std::vector<Joints>& positions() const noexcept { return q_; }

private:
    JointSpline(std::vector<double> x, std::vector<Joints> q, std::vector<Joints> m)
        : x_(std::move(x)), q_(std::move(q)), m_(std::move(m)) {}
    [[nodiscard]] std::size_t segment(double x) const noexcept;  // k with x_k <= x < x_k+1, for x inside

    std::vector<double> x_;
    std::vector<Joints> q_;
    std::vector<Joints> m_;  // the slope at each knot
};

/// The inverse kinematics plan() calls: the joint positions [rad] that put the
/// TCP at T_base_tcp, chosen near `near`. Robot::ik is one.
using InverseKinematics =
    std::function<std::expected<Joints, Error>(const Eigen::Isometry3d& T_base_tcp, const Joints& near)>;

/// How plan() solves and checks a motion.
struct PlanOptions {
    /// [m]: neighbouring poses further apart are joined by a straight line,
    /// solved every max_step along it.
    double max_step = 0.003;
    double max_turn = deg_to_rad(1.0);  ///< [rad]: the same for the orientation, turned evenly (slerp)
    /// [rad]: neighbouring solutions further apart than this on a joint are
    /// refused: the arm would change configuration, or pass close to a singularity.
    double max_joint_step = deg_to_rad(10.0);
    /// [rad]: how far J5 (the wrist) and J3 (the elbow) must stay from 0 and 180
    /// deg, where the EC series' arm is singular (its layout is Universal Robots').
    double singularity_margin = deg_to_rad(5.0);
    double period = 0.02;             ///< [s] between the trajectory's points
    TrajectoryLimits limits{};        ///< joint positions, speeds, accelerations
    std::size_t max_points = 10'000;  ///< solved points at most
};

/// Why the arm should not be at joint positions `q` [rad] on a planned or
/// streamed motion, or nothing: a joint beyond limits.joint_limit, or J5 or J3
/// within singularity_margin of 0 or 180 deg (the wrist and elbow singularities).
[[nodiscard]] std::expected<void, std::string> check_point(const Joints& q, const PlanOptions& options);

/// Why the arm should not go from joint positions `from` to `to` [rad] as one
/// step of a motion, or nothing: a joint turning more than max_joint_step (a
/// change of configuration, or a pass close to a singularity), or what
/// check_point(to) finds. plan() checks every point so; a stream of TCP poses
/// should check each target's solution so too.
[[nodiscard]] std::expected<void, std::string> check_step(const Joints& from, const Joints& to,
                                                          const PlanOptions& options);

/// Turns a motion of the TCP into a joint trajectory. The motion must be valid
/// (ec::validate) and start where the arm is: its first pose is solved near
/// `start`. The trajectory runs from rest to rest and passes through every pose
/// at its time without stopping, except where a pose is stamped twice (a pause:
/// the arm rests there). Steps: poses further apart than max_step or max_turn are
/// joined by straight lines and filled in; every point is solved by `ik`, each
/// near the one before (a solution given within one turn taken to the nearest
/// turn) and checked (check_point, check_step); the solutions are interpolated
/// over time (JointSpline) and sampled every `period`, ending exactly at the last
/// time. Fails with Kind::Invalid, naming the pose, for an invalid motion, a
/// failed check, or a trajectory beyond the limits (too fast for the joints); a
/// failure of `ik` keeps its kind (the controller found no solution, or did not
/// answer, ...) and gains the pose in its message. Every point but a pause costs
/// one call of `ik`.
[[nodiscard]] std::expected<JointTrajectory, Error> plan(const PoseTrajectory& motion, const Joints& start,
                                                         const InverseKinematics& ik, const PlanOptions& options = {});

/// How close joint positions come to the EC series' wrist and elbow
/// singularities: the least distance of J5, and of J3, from 0 or 180 deg [rad].
struct SingularityMargins {
    double wrist = std::numbers::pi / 2.0;
    double elbow = std::numbers::pi / 2.0;
};
[[nodiscard]] SingularityMargins singularity_margins(std::span<const Joints> positions) noexcept;

}  // namespace ec::motion
