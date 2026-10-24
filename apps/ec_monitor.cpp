// ec_monitor: connect to the controller's monitoring port (8056) and print its
// frames as the manual lays them out, in the controller's units (deg, mm), to
// compare with the teach pendant. The driver's SI view of the same frames is
// ec::State (ec_record --h5 records it).
//
//   ec_monitor [host] [--port N] [--rate HZ | --all] [--frames N]
//
// Every frame is received and decoded, so nothing piles up in the socket
// buffer; the rate only decides which frames are printed. Data goes to stdout,
// status and errors to stderr, so `ec_monitor --all > run.txt` captures only
// data. Read-only: it never sends anything to the robot. Runs until Ctrl+C, or
// until --frames frames have arrived. Exit codes: 0 stopped; 2 bad command line.

#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <format>
#include <string>
#include <string_view>
#include <thread>

#include "cli.hpp"
#include "ec/feedback/frame.hpp"
#include "ec/feedback/framer.hpp"
#include "ec/feedback/reader.hpp"
#include "ec/net/tcp_socket.hpp"

namespace {

using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
using cli::err;
using cli::out;
namespace fb = ec::feedback;

volatile std::sig_atomic_t g_stop = 0;
extern "C" void on_signal(int) { g_stop = 1; }

// Lets a sample through once per period on a fixed time grid, so the printed
// rate is what was asked for even if the frame rate changes. It picks samples;
// it does not average them.
class Downsampler {
public:
    explicit Downsampler(double rate_hz)
        : period_(rate_hz > 0
                      ? std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / rate_hz))
                      : Clock::duration::zero()) {}

    bool accept(Clock::time_point t) {
        if (period_ == Clock::duration::zero()) return true;  // --all
        if (!started_) {
            started_ = true;
            next_ = t + period_;
            return true;
        }
        if (t < next_) return false;
        next_ += period_;
        if (t >= next_) next_ = t + period_;  // fell far behind (reconnect): re-anchor
        return true;
    }

private:
    Clock::duration period_;
    Clock::time_point next_{};
    bool started_ = false;
};

// Frame spacing (min/mean/max) and rate over one-second windows.
class RateMeter {
public:
    void add(Clock::time_point t) {
        if (have_prev_) {
            const double dt = std::chrono::duration<double, std::milli>(t - prev_).count();
            if (n_ == 0 || dt < min_) min_ = dt;
            if (n_ == 0 || dt > max_) max_ = dt;
            sum_ += dt;
            ++n_;
        } else {
            window_start_ = t;
        }
        prev_ = t;
        have_prev_ = true;
        ++frames_;
        if (t - window_start_ >= 1s) {
            report_ =
                std::format("{:.1f} Hz, dt {:.2f}/{:.2f}/{:.2f} ms (min/mean/max)",
                            static_cast<double>(frames_) / std::chrono::duration<double>(t - window_start_).count(),
                            min_, n_ ? sum_ / static_cast<double>(n_) : 0.0, max_);
            window_start_ = t;
            frames_ = 0;
            n_ = 0;
            sum_ = 0;
        }
    }
    void restart() {
        have_prev_ = false;
        frames_ = 0;
        n_ = 0;
        sum_ = 0;
    }
    [[nodiscard]] const std::string& report() const { return report_; }

private:
    Clock::time_point prev_{}, window_start_{};
    bool have_prev_ = false;
    std::uint64_t frames_ = 0, n_ = 0;
    double min_ = 0, max_ = 0, sum_ = 0;
    std::string report_ = "measuring...";
};

std::string_view flag(bool on, std::string_view name) { return on ? name : ""; }

void print_frame(const fb::Frame& f, std::uint64_t seq, const std::string& rate) {
    out("---- #{}  ctrl {} ms  | {}\n", seq, f.timestamp_ms, rate);
    out("mode {}, state {}, cycle {}  {}{}{}{}{}{}\n", f.robot_mode, f.robot_state, f.cycle_mode,
        flag(f.servo_ready, "[servo on] "), flag(f.motors_synced, "[synced] "), flag(f.emergency_stop, "[E-STOP] "),
        flag(f.collision, "[COLLISION] "), flag(f.drag_enabled, "[drag] "), flag(f.force_sensor_connected, "[F/T] "));

    out("{:9} {:>9} {:>9} {:>9} {:>9} {:>9} {:>9}\n", "", "J1", "J2", "J3", "J4", "J5", "J6");
    auto row = [](std::string_view name, const auto& v, std::string_view unit) {
        out("{:9} {:9.3f} {:9.3f} {:9.3f} {:9.3f} {:9.3f} {:9.3f}  {}\n", name, v[0], v[1], v[2], v[3], v[4], v[5],
            unit);
    };
    row("pos", f.joint_pos_deg, "deg");
    row("vel", f.joint_vel_deg_s, "deg/s");
    row("torque", f.joint_torque_nm, "N*m");
    row("ext torq", f.joint_torque_ext_nm, "N*m");
    row("temp", f.joint_temp_c, "C");

    const auto& b = f.tcp_pose_base;
    out("TCP base  x {:.2f}  y {:.2f}  z {:.2f} mm | rx {:.4f}  ry {:.4f}  rz {:.4f} rad | v {:.2f} mm/s\n", b[0], b[1],
        b[2], b[3], b[4], b[5], f.tcp_speed_mm_s);
    const auto& w = f.tcp_wrench_ext;
    out("ext wrench F {:.2f} {:.2f} {:.2f} N | T {:.3f} {:.3f} {:.3f} N*m\n\n", w[0], w[1], w[2], w[3], w[4], w[5]);
    std::fflush(stdout);
}

int run(int argc, char** argv) {
    std::string host = "192.168.0.200";  // this tool's default; the library has none
    std::uint16_t port = 8056;
    double rate_hz = 2.0;
    bool all = false;
    std::uint64_t max_frames = 0;  // 0: no limit

    CLI::App app{"Print the robot state streamed on the controller's monitoring port (8056)."};
    app.add_option("host", host, "Controller address")->capture_default_str();
    app.add_option("--port", port, "Monitoring port")->capture_default_str()->check(CLI::Range(1, 65535));
    auto* rate = app.add_option("--rate", rate_hz, "Frames printed per second")
                     ->capture_default_str()
                     ->check(CLI::PositiveNumber);
    app.add_flag("--all", all, "Print every frame (about 125 per second)")->excludes(rate);
    app.add_option("--frames", max_frames, "Stop after this many frames (0: run until Ctrl+C)")->capture_default_str();
    if (const auto code = cli::parse(app, argc, argv)) return *code;

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    if (all)
        err("monitoring {}:{}, printing every frame (Ctrl+C to quit)\n", host, port);
    else
        err("monitoring {}:{}, printing {:.2f} frames/s (Ctrl+C to quit)\n", host, port, rate_hz);

    Downsampler printer(all ? 0.0 : rate_hz);
    RateMeter meter;
    std::uint64_t frames = 0;
    std::uint64_t decode_errors = 0;
    fb::Framer::Stats framing;  // summed over connections
    const auto done = [&] { return g_stop != 0 || (max_frames != 0 && frames >= max_frames); };

    while (!done()) {
        auto reader = fb::Reader::connect(host, port, 2000ms);
        if (!reader) {
            err("{}, retrying in 1 s\n", reader.error().message());
            std::this_thread::sleep_for(1s);
            continue;
        }
        err("connected\n");
        meter.restart();

        bool silent = false;  // "no data" reported once per silence, not every 200 ms
        while (!done()) {
            const auto sample = reader->next(200ms);
            if (!sample) {
                if (sample.error().kind == ec::net::NetError::Kind::Timeout) {
                    if (!silent) err("no data for 200 ms\n");
                    silent = true;
                    continue;
                }
                err("{}, reconnecting\n", sample.error().message());
                break;
            }
            silent = false;
            ++frames;
            const auto frame = fb::decode(sample->raw);
            if (!frame) {
                err("decode error #{}: {}\n", ++decode_errors, fb::to_string(frame.error()));
                continue;
            }
            meter.add(sample->arrival);
            if (printer.accept(sample->arrival)) print_frame(*frame, frames, meter.report());
        }
        framing.resyncs += reader->stats().resyncs;
        framing.bytes_discarded += reader->stats().bytes_discarded;
    }

    err("stopped: {} frames, {} resyncs, {} bytes discarded, {} decode errors\n", frames, framing.resyncs,
        framing.bytes_discarded, decode_errors);
    return cli::kOk;
}

}  // namespace

int main(int argc, char** argv) {
    return cli::guarded("ec_monitor", [&] { return run(argc, argv); });
}
