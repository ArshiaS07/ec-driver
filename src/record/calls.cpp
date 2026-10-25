#include "ec/record/calls.hpp"

#include <exception>
#include <format>
#include <utility>

#include "ec/record/clock.hpp"

namespace ec::record {

CallWriter::CallWriter(std::ofstream out, std::filesystem::path path) : out_(std::move(out)), path_(std::move(path)) {}

std::expected<CallWriter, std::string> CallWriter::create(const std::filesystem::path& path) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) return std::unexpected(std::format("{}: cannot create the file", path.string()));
    const ClockPair clocks = ClockPair::now();
    const sdk::Json header{{"format", std::string(kCallsFormat)},
                           {"steady_ns_at_start", clocks.steady_ns},
                           {"system_ns_at_start", clocks.system_ns}};
    out << header.dump() << '\n' << std::flush;
    if (!out) return std::unexpected(std::format("{}: cannot write the header", path.string()));
    return CallWriter(std::move(out), path);
}

void CallWriter::write(const CallRecord& call) {
    if (error_) return;
    try {
        sdk::Json line{{"sent", to_ns(call.sent)},
                       {"answered", to_ns(call.answered)},
                       {"method", call.method},
                       {"params", call.params}};
        if (call.reply)
            line["result"] = *call.reply;
        else
            line["error"] = {{"kind", std::string(to_string(call.reply.error().kind))},
                             {"message", call.reply.error().message}};
        // Text that is not UTF-8 (a controller's message) is kept, with replacement characters.
        out_ << line.dump(-1, ' ', false, sdk::Json::error_handler_t::replace) << '\n' << std::flush;
    } catch (const std::exception& e) {
        error_ = std::format("{}: {}", path_.string(), e.what());
        return;
    }
    if (!out_) {
        error_ = std::format("{}: write failed", path_.string());
        return;
    }
    ++written_;
}

}  // namespace ec::record
