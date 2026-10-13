#pragma once
// Test helper: builds wire-format 8056 frames independently of the decoder.
// Every field gets a distinct value, so a shifted offset in the decoder makes
// a later field fail its check.

#include <algorithm>
#include <array>
#include <bit>
#include <cstdint>
#include <vector>

#include "ec/feedback/frame.hpp"

namespace ec::test {

// Appends values in big-endian (wire) byte order.
class WireWriter {
public:
    template <typename T>
    void put(T v) {
        auto raw = std::bit_cast<std::array<std::uint8_t, sizeof(T)>>(v);
        if constexpr (std::endian::native == std::endian::little)
            for (std::size_t i = sizeof(T); i-- > 0;) bytes.push_back(raw[i]);
        else
            bytes.insert(bytes.end(), raw.begin(), raw.end());
    }
    template <typename T>
    void put_seq(T first, int n) {  // first, first+1, ..., first+n-1
        for (int i = 0; i < n; ++i) put<T>(static_cast<T>(first + i));
    }
    std::vector<std::uint8_t> bytes;
};

// A valid frame with a distinct value in every field.
inline feedback::RawFrame make_frame(std::uint64_t timestamp_ms = 1791200688449ull) {
    WireWriter w;
    // clang-format off
    w.put<std::uint32_t>(1024);              // message size
    w.put<std::uint64_t>(timestamp_ms);      // timestamp
    w.put<std::uint8_t>(2);                  // cycle mode
    w.put_seq<double>(10.0, 8);              // machinePos
    w.put_seq<double>(100.0, 6);             // machinePose
    w.put_seq<double>(200.0, 6);             // machineUserPose
    w.put_seq<double>(300.0, 8);             // torque per-mille
    w.put<std::int32_t>(3);                  // robotState: running
    w.put<std::int32_t>(1);                  // servoReady
    w.put<std::int32_t>(1);                  // can_motor_run
    w.put_seq<std::int32_t>(-4, 8);          // motor_speed
    w.put<std::int32_t>(2);                  // robotMode: remote
    w.put_seq<double>(1.0, 3);               // analog in
    w.put_seq<double>(5.0, 5);               // analog out
    w.put<std::uint64_t>(0x8001ull);         // DI
    w.put<std::uint64_t>(0x00F0ull);         // DO
    w.put<std::uint8_t>(1);                  // collision
    w.put_seq<double>(400.0, 6);             // flange base
    w.put_seq<double>(500.0, 6);             // flange user
    w.put<std::uint8_t>(1);                  // e-stop
    w.put<double>(12.5);                     // tcp speed
    w.put_seq<double>(600.0, 8);             // joint speed
    w.put<double>(-3.25);                    // tcp acc
    w.put_seq<double>(700.0, 8);             // joint acc
    w.put_seq<double>(30.0, 6);              // temperature
    w.put_seq<double>(-1.5, 6);              // joint torque
    w.put_seq<double>(0.25, 6);              // ext joint torque
    w.put_seq<double>(800.0, 6);             // ext tcp wrench
    w.put<std::uint8_t>(1);                  // drag
    w.put<std::uint8_t>(0);                  // force sensor connected
    w.bytes.resize(feedback::kMatchOffset, 0);         // reserved
    w.put<std::uint32_t>(feedback::kMatchingWord);
    // clang-format on

    feedback::RawFrame out{};
    std::copy(w.bytes.begin(), w.bytes.end(), out.begin());
    return out;
}

// The fields the motion layer's conditions read; the rest as make_frame().
struct FrameFields {
    std::int32_t robot_state = 0;  // stop
    bool servo = true;
    bool synced = true;
    std::int32_t robot_mode = 2;  // remote
    bool collision = false;
    bool estop = false;
    std::array<double, 6> joints_deg{0, -90, 0, -90, 90, 0};
    std::array<double, 6> joint_speed_deg_s{};
};

// A valid frame of a robot as `f` describes: by default remote, servos on,
// synchronised, stopped, at rest.
inline feedback::RawFrame make_frame(const FrameFields& f, std::uint64_t timestamp_ms = 1791200688449ull) {
    WireWriter w;
    // clang-format off
    w.put<std::uint32_t>(1024);
    w.put<std::uint64_t>(timestamp_ms);
    w.put<std::uint8_t>(2);
    for (double q : f.joints_deg) w.put<double>(q);
    w.put<double>(0.0); w.put<double>(0.0);   // external axes
    w.put_seq<double>(100.0, 6);
    w.put_seq<double>(200.0, 6);
    w.put_seq<double>(300.0, 8);
    w.put<std::int32_t>(f.robot_state);
    w.put<std::int32_t>(f.servo ? 1 : 0);
    w.put<std::int32_t>(f.synced ? 1 : 0);
    w.put_seq<std::int32_t>(0, 8);
    w.put<std::int32_t>(f.robot_mode);
    w.put_seq<double>(1.0, 3);
    w.put_seq<double>(5.0, 5);
    w.put<std::uint64_t>(0);
    w.put<std::uint64_t>(0);
    w.put<std::uint8_t>(f.collision ? 1 : 0);
    w.put_seq<double>(400.0, 6);
    w.put_seq<double>(500.0, 6);
    w.put<std::uint8_t>(f.estop ? 1 : 0);
    w.put<double>(0.0);
    for (double v : f.joint_speed_deg_s) w.put<double>(v);
    w.put<double>(0.0); w.put<double>(0.0);
    w.put<double>(0.0);
    w.put_seq<double>(0.0, 8);
    w.put_seq<double>(30.0, 6);
    w.put_seq<double>(0.0, 6);
    w.put_seq<double>(0.0, 6);
    w.put_seq<double>(0.0, 6);
    w.put<std::uint8_t>(0);
    w.put<std::uint8_t>(0);
    w.bytes.resize(feedback::kMatchOffset, 0);
    w.put<std::uint32_t>(feedback::kMatchingWord);
    // clang-format on

    feedback::RawFrame out{};
    std::copy(w.bytes.begin(), w.bytes.end(), out.begin());
    return out;
}

}  // namespace ec::test
