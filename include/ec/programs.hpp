#pragma once
/// @file
/// ec::Programs: the controller's program engine: JBI programs, the cycle
/// mode, the system variables (SDK manual 2.2.3.10-14, 2.2.6; docs/ec66.md
/// §4.2). The controller runs the programs; each operation is one call on the
/// command connection (8055). A running program is the motion controller's
/// work like any motion: robot.motion().stop() ends it too.
///
///     auto programs = robot.programs();
///     if (programs.exists("pick").value()) programs.run("pick");

#include <Eigen/Geometry>
#include <cstdint>
#include <expected>
#include <memory>
#include <string>

#include "ec/error.hpp"
#include "ec/sdk/movement.hpp"
#include "ec/status.hpp"
#include "ec/units.hpp"

namespace ec {

namespace detail {
struct Connection;
}

/// A handle onto a robot's program engine: cheap to copy, usable from any
/// thread. Once its Robot is gone, every operation fails with Disconnected.
/// Variable addresses are 0..255 and values within the manual's ranges, else
/// Invalid with nothing sent; everything but the readings needs REMOTE mode
/// and is Busy while a motion is open.
class Programs {
public:
    /// checkJbiExist [S 2.2.3.12].
    [[nodiscard]] std::expected<bool, Error> exists(const std::string& name) const;
    /// runJbi [S 2.2.3.13]: from standstill only.
    [[nodiscard]] std::expected<void, Error> run(const std::string& name) const;
    /// getJbiState [S 2.2.3.14]: the program and its run state.
    [[nodiscard]] std::expected<sdk::JbiState, Error> state() const;
    /// getCurrentJobLine [S 2.2.2.8]: it counts the lines of point data too,
    /// unlike the pendant.
    [[nodiscard]] std::expected<int, Error> line() const;
    /// pause [S 2.2.3.11].
    [[nodiscard]] std::expected<void, Error> pause() const;
    /// run [S 2.2.3.10]: resumes a paused program.
    [[nodiscard]] std::expected<void, Error> resume() const;

    /// getCycleMode [S 2.2.2.7] (also on the monitoring path: programs.cycle_mode).
    [[nodiscard]] std::expected<CycleMode, Error> cycle_mode() const;
    /// setCycleMode [S 2.2.2.23].
    [[nodiscard]] std::expected<void, Error> set_cycle_mode(CycleMode mode) const;

    /// The system variables, as the pendant names them [S 2.2.6].
    [[nodiscard]] std::expected<std::int64_t, Error> b(int address) const;  ///< getSysVarB: 0..2147483647
    [[nodiscard]] std::expected<void, Error> set_b(int address, std::int64_t value) const;  ///< setSysVarB
    [[nodiscard]] std::expected<int, Error> i(int address) const;                     ///< getSysVarI: -32767..32767
    [[nodiscard]] std::expected<void, Error> set_i(int address, int value) const;     ///< setSysVarI
    [[nodiscard]] std::expected<double, Error> d(int address) const;                  ///< getSysVarD: -1e9..1e9
    [[nodiscard]] std::expected<void, Error> set_d(int address, double value) const;  ///< setSysVarD
    /// getSysVarP [S 2.2.6.8]: joint positions [rad] (the arm's six of the eight axes).
    [[nodiscard]] std::expected<Joints, Error> p(int address) const;
    [[nodiscard]] std::expected<void, Error> set_p(int address, const Joints& q) const;  ///< setSysVarP
    [[nodiscard]] std::expected<bool, Error> p_enabled(int address) const;               ///< getSysVarPState
    /// getSysVarV [S 2.2.6.10]: a pose, T_base_x.
    [[nodiscard]] std::expected<Eigen::Isometry3d, Error> v(int address) const;
    /// setSysVarV [S 2.2.6.11]; the vendor's examples disagree on the field
    /// name, "pose" is sent (unverified).
    [[nodiscard]] std::expected<void, Error> set_v(int address, const Eigen::Isometry3d& pose) const;
    /// save_var_data [S 2.2.6.12]: keeps the variables over a restart.
    [[nodiscard]] std::expected<void, Error> save_variables() const;

private:
    friend class Robot;
    explicit Programs(std::weak_ptr<detail::Connection> session) noexcept : session_(std::move(session)) {}
    std::weak_ptr<detail::Connection> session_;
};

}  // namespace ec
