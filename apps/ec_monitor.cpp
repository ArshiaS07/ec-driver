// ec_monitor: connect to port 8056 and print the decoded robot state.
//
//   usage: ec_monitor [host] [--port N] [--rate HZ | --all]
//
//     host        controller address              (default 192.168.0.200)
//     --port N    feedback port                   (default 8056)
//     --rate HZ   print at most HZ samples/second (default 2, readable by eye)
//     --all       print every frame (~125/s), e.g. to redirect into a file
//
// Every frame is received and decoded, so nothing piles up in the socket
// buffer; the downsampler only decides which ones are printed. Data goes to
// stdout, status and errors to stderr, so `ec_monitor --all > run.txt` captures
// only data. Read-only: it never sends anything to the robot.

#include <array>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <format>
#include <string>
#include <string_view>
#include <thread>

#include "ec/feedback/frame.hpp"
#include "ec/feedback/framer.hpp"
#include "ec/net/tcp_socket.hpp"

namespace {

using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
namespace fb = ec::feedback;

volatile std::sig_atomic_t g_stop = 0;
extern "C" void on_signal(int) { g_stop = 1; }

template <typename... Args>
void out(std::format_string<Args...> fmt, Args&&... args) {
    const std::string s = std::format(fmt, std::forward<Args>(args)...);
    std::fwrite(s.data(), 1, s.size(), stdout);
}
template <typename... Args>
void err(std::format_string<Args...> fmt, Args&&... args) {
    const std::string s = std::format(fmt, std::forward<Args>(args)...);
    std::fwrite(s.data(), 1, s.size(), stderr);
}

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

struct Options {
    std::string host = "192.168.0.200";
    std::uint16_t port = 8056;
    double rate_hz = 2.0;
};

bool parse_args(int argc, char** argv, Options& o) {
    for (int i = 1; i < argc; ++i) {
        const std::string_view a = argv[i];
        if (a == "--all") {
            o.rate_hz = 0;
        } else if (a == "--rate" && i + 1 < argc) {
            o.rate_hz = std::atof(argv[++i]);
        } else if (a == "--port" && i + 1 < argc) {
            o.port = static_cast<std::uint16_t>(std::atoi(argv[++i]));
        } else if (!a.empty() && a[0] != '-') {
            o.host = a;
        } else {
            return false;  // --help or anything unknown
        }
    }
    return o.rate_hz >= 0;
}

std::string_view flag(bool on, std::string_view name) { return on ? name : ""; }

void print_frame(const fb::Frame& f, std::uint64_t seq, const std::string& rate) {
    out("---- #{}  ctrl {} ms  | {}\n", seq, f.timestamp_ms, rate);
    out("mode {}, state {}, cycle {}  {}{}{}{}{}{}\n", f.robot_mode, f.robot_state, f.cycle_mode,
        flag(f.servo_ready, "[brakes released] "), flag(f.motors_synced, "[synced] "),
        flag(f.emergency_stop, "[E-STOP] "), flag(f.collision, "[COLLISION] "), flag(f.drag_enabled, "[drag] "),
        flag(f.force_sensor_connected, "[F/T] "));

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

}  // namespace

int main(int argc, char** argv) {
    Options opt;
    if (!parse_args(argc, argv, opt)) {
        err("usage: ec_monitor [host] [--port N] [--rate HZ | --all]\n");
        return 2;
    }
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    if (opt.rate_hz > 0)
        err("monitoring {}:{}, printing {:.2f} samples/s (Ctrl+C to quit)\n", opt.host, opt.port, opt.rate_hz);
    else
        err("monitoring {}:{}, printing every frame (Ctrl+C to quit)\n", opt.host, opt.port);

    fb::Framer framer;
    Downsampler printer(opt.rate_hz);
    RateMeter meter;
    std::uint64_t decode_errors = 0;
    std::array<std::uint8_t, 4096> chunk{};

    while (!g_stop) {
        auto sock = ec::net::TcpSocket::connect(opt.host, opt.port, 2000ms);
        if (!sock) {
            err("{}, retrying in 1 s\n", sock.error().message());
            std::this_thread::sleep_for(1s);
            continue;
        }
        err("connected\n");
        framer.reset();
        meter.restart();

        while (!g_stop) {
            const auto n = sock->receive(chunk, 200ms);
            if (!n) {
                if (n.error().kind == ec::net::NetError::Kind::Timeout) {
                    if (!g_stop) err("no data for 200 ms\n");
                    continue;
                }
                err("{}, reconnecting\n", n.error().message());
                break;
            }
            const auto t_rx = Clock::now();
            framer.push(std::span(chunk.data(), *n));

            while (const auto raw = framer.pop()) {
                const auto frame = fb::decode(*raw);
                if (!frame) {
                    err("decode error #{}: {}\n", ++decode_errors, fb::to_string(frame.error()));
                    continue;
                }
                meter.add(t_rx);
                if (printer.accept(t_rx)) print_frame(*frame, framer.stats().frames, meter.report());
            }
        }
    }

    const auto& st = framer.stats();
    err("stopped: {} frames, {} resyncs, {} bytes discarded, {} decode errors\n", st.frames, st.resyncs,
        st.bytes_discarded, decode_errors);
    return 0;
}
