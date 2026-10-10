#include "ec/feedback/frame.hpp"

#include <cassert>

#include "detail/big_endian.hpp"

namespace ec::feedback {

bool is_matching_word(std::span<const std::uint8_t, 4> b) noexcept {
    return b[0] == 0xEC && b[1] == 0x80 && b[2] == 0x56 && b[3] == 0xEC;
}

std::expected<Frame, DecodeError> decode(std::span<const std::uint8_t, kFrameSize> bytes) noexcept {
    if (!is_matching_word(bytes.subspan<kMatchOffset, 4>())) return std::unexpected(DecodeError::BadMatchingWord);

    detail::BigEndianReader r(bytes);
    Frame f;
    // Strictly in wire order; the trailing number is the field's byte offset.
    // clang-format off
    f.message_size           = r.read<std::uint32_t>();                          //   0
    f.timestamp_ms           = r.read<std::uint64_t>();                          //   4
    f.cycle_mode             = static_cast<CycleMode>(r.read<std::uint8_t>());   //  12
    f.joint_pos_deg          = r.read_array<double, kWireAxes>();                //  13
    f.tcp_pose_base          = r.read_array<double, 6>();                        //  77
    f.tcp_pose_user          = r.read_array<double, 6>();                        // 125
    f.joint_torque_permille  = r.read_array<double, kWireAxes>();                // 173
    f.robot_state            = static_cast<RobotState>(r.read<std::int32_t>());  // 237
    f.servo_ready            = r.read<std::int32_t>() != 0;                      // 241
    f.motors_synced          = r.read<std::int32_t>() != 0;                      // 245
    f.motor_speed_rpm        = r.read_array<std::int32_t, kWireAxes>();          // 249
    f.robot_mode             = static_cast<RobotMode>(r.read<std::int32_t>());   // 281
    f.analog_in_v            = r.read_array<double, 3>();                        // 285
    f.analog_out_v           = r.read_array<double, 5>();                        // 309
    f.digital_in             = r.read<std::uint64_t>();                          // 349
    f.digital_out            = r.read<std::uint64_t>();                          // 357
    f.collision              = r.read<std::uint8_t>() != 0;                      // 365
    f.flange_pose_base       = r.read_array<double, 6>();                        // 366
    f.flange_pose_user       = r.read_array<double, 6>();                        // 414
    f.emergency_stop         = r.read<std::uint8_t>() != 0;                      // 462
    f.tcp_speed_mm_s         = r.read<double>();                                 // 463
    f.joint_vel_deg_s        = r.read_array<double, kWireAxes>();                // 471
    f.tcp_acc_mm_s2          = r.read<double>();                                 // 535
    f.joint_acc_deg_s2       = r.read_array<double, kWireAxes>();                // 543
    f.joint_temp_c           = r.read_array<double, kArmAxes>();                 // 607
    f.joint_torque_nm        = r.read_array<double, kArmAxes>();                 // 655
    f.joint_torque_ext_nm    = r.read_array<double, kArmAxes>();                 // 703
    f.tcp_wrench_ext         = r.read_array<double, 6>();                        // 751
    f.drag_enabled           = r.read<std::uint8_t>() != 0;                      // 799
    f.force_sensor_connected = r.read<std::uint8_t>() != 0;                      // 800
    // clang-format on

    // A wrong field type above shifts everything after it; this catches it.
    assert(r.position() == kValidBytes);

    if (f.message_size != kFrameSize) return std::unexpected(DecodeError::BadMessageSize);
    return f;
}

std::string_view to_string(DecodeError e) noexcept {
    switch (e) {
        case DecodeError::BadMatchingWord: return "bad matching word";
        case DecodeError::BadMessageSize: return "message size field is not 1024";
    }
    return "unknown";
}

}  // namespace ec::feedback
