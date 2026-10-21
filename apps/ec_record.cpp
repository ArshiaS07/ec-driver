// ec_record: record the controller's monitoring stream (port 8056), or convert
// a raw recording.
//
//   ec_record [host] [--port N] [--raw FILE] [--h5 FILE] [--frames N] [--seconds S]
//   ec_record --replay FILE --h5 FILE
//
// Live, every frame goes to the raw recording (--raw: the bytes as received)
// and/or the HDF5 recording (--h5: SI columns under /feedback), until Ctrl+C, --frames
// or --seconds. Replay reads a raw recording instead and keeps its arrival
// times; that is how a raw recording becomes an HDF5 recording later. Formats:
// docs/recording-format.md. Read-only: it never sends anything to the robot.
//
// Exit codes: 0 done; 1 a file could not be written; 2 bad command line.

#include <chrono>
#include <csignal>
#include <cstdint>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "cli.hpp"
#include "ec/feedback/reader.hpp"
#include "ec/record/raw.hpp"
#include "ec/record/sink.hpp"
#if defined(EC_WITH_HDF5)
#include "ec/record/hdf5.hpp"
#endif

namespace {

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
using Sink = ec::record::Sink<ec::feedback::Sample>;

volatile std::sig_atomic_t g_stop = 0;
extern "C" void on_signal(int) { g_stop = 1; }

int run(int argc, char** argv) {
    std::string host = "192.168.0.200";  // this tool's default; the library has none
    std::uint16_t port = 8056;
    std::string raw_path;
    std::string h5_path;
    std::string replay_path;
    std::uint64_t max_frames = 0;
    double seconds = 0.0;

    CLI::App app{"Record the controller's monitoring stream (8056), or convert a raw recording."};
    auto* host_option = app.add_option("host", host, "Controller address")->capture_default_str();
    auto* port_option =
        app.add_option("--port", port, "Monitoring port")->capture_default_str()->check(CLI::Range(1, 65535));
    auto* raw_option = app.add_option("--raw", raw_path, "Raw recording to write: every frame as received");
#if defined(EC_WITH_HDF5)
    app.add_option("--h5", h5_path, "HDF5 recording to write: SI columns under /feedback");
#endif
    app.add_option("--frames", max_frames, "Stop after this many frames (0: no limit)")->capture_default_str();
    app.add_option("--seconds", seconds, "Stop after this long (0: no limit)")
        ->capture_default_str()
        ->check(CLI::NonNegativeNumber);
    app.add_option("--replay", replay_path, "Read this raw recording instead of the controller")
        ->check(CLI::ExistingFile)
        ->excludes(host_option)
        ->excludes(port_option)
        ->excludes(raw_option);
    if (const auto code = cli::parse(app, argc, argv)) return *code;
    if (raw_path.empty() && h5_path.empty()) {
        cli::err("nothing to write: give --raw{}\n", app.get_option_no_throw("--h5") ? " and/or --h5" : "");
        return cli::kUsage;
    }

    // The sinks. Each keeps its own error; the loop stops at the first one.
    std::vector<Sink*> sinks;
    std::optional<ec::record::RawWriter> raw;
    if (!raw_path.empty()) {
        auto writer = ec::record::RawWriter::create(raw_path);
        if (!writer) {
            cli::err("{}\n", writer.error());
            return cli::kFailed;
        }
        raw.emplace(std::move(*writer));
        sinks.push_back(&*raw);
    }
#if defined(EC_WITH_HDF5)
    std::optional<ec::record::H5File> h5_file;
    std::optional<ec::record::H5FeedbackWriter> h5;  // declared after the file: destroyed before it
    if (!h5_path.empty()) {
        auto file = ec::record::H5File::create(h5_path);
        if (!file) {
            cli::err("{}\n", file.error());
            return cli::kFailed;
        }
        h5_file.emplace(std::move(*file));
        auto writer = ec::record::H5FeedbackWriter::create(*h5_file, "/feedback");
        if (!writer) {
            cli::err("{}\n", writer.error());
            return cli::kFailed;
        }
        h5.emplace(std::move(*writer));
        sinks.push_back(&*h5);
    }
#endif
    const auto failed = [&] {
        bool any = raw && raw->error();
#if defined(EC_WITH_HDF5)
        any = any || (h5 && h5->error());
#endif
        return any;
    };

    const auto deadline =
        seconds > 0.0
            ? Clock::now() + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(seconds))
            : Clock::time_point::max();
    std::uint64_t frames = 0;
    const auto done = [&] {
        return g_stop != 0 || (max_frames != 0 && frames >= max_frames) || Clock::now() >= deadline || failed();
    };
    const auto record = [&](const ec::feedback::Sample& sample) {
        for (Sink* sink : sinks) sink->write(sample);
        ++frames;
    };

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    if (!replay_path.empty()) {
        auto reader = ec::record::RawReader::open(replay_path);
        if (!reader) {
            cli::err("{}\n", reader.error());
            return cli::kFailed;
        }
        while (!done()) {
            const auto sample = reader->next();
            if (!sample) break;
            record(*sample);
        }
        if (reader->truncated()) cli::err("{}: the last record is cut short; left out\n", replay_path);
    } else {
        cli::err("recording {}:{} (Ctrl+C to stop)\n", host, port);
        while (!done()) {
            auto reader = ec::feedback::Reader::connect(host, port, 2000ms);
            if (!reader) {
                cli::err("{}, retrying in 1 s\n", reader.error().message());
                std::this_thread::sleep_for(1s);
                continue;
            }
            cli::err("connected\n");
            bool silent = false;  // "no data" reported once per silence
            while (!done()) {
                const auto sample = reader->next(200ms);
                if (!sample) {
                    if (sample.error().kind == ec::net::NetError::Kind::Timeout) {
                        if (!silent) cli::err("no data for 200 ms\n");
                        silent = true;
                        continue;
                    }
                    cli::err("{}, reconnecting\n", sample.error().message());
                    break;
                }
                silent = false;
                record(*sample);
            }
        }
    }

    int status = cli::kOk;
    if (raw) {
        if (const auto closed = raw->close(); !closed) {
            cli::err("{}\n", closed.error());
            status = cli::kFailed;
        }
    }
#if defined(EC_WITH_HDF5)
    if (h5) {
        if (const auto flushed = h5->flush(); !flushed) {
            cli::err("{}\n", flushed.error());
            status = cli::kFailed;
        }
        if (h5->skipped() != 0)
            cli::err("{} frames did not decode; the HDF5 recording leaves them out\n", h5->skipped());
    }
#endif
    cli::err("stopped: {} frames\n", frames);
    return status;
}

}  // namespace

int main(int argc, char** argv) {
    return cli::guarded("ec_record", [&] { return run(argc, argv); });
}
