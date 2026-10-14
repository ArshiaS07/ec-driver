#pragma once
/// @file
/// The contract of everything the control thread runs: an activity, a state
/// machine without I/O that is advanced once per cycle (ADR 0014,
/// docs/motion.md 5.3). Activities write commands into an Outbox; the control
/// thread sends them and hands the replies back in the next cycle.

#include <chrono>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <format>
#include <memory>
#include <span>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

#include "ec/command/error.hpp"
#include "ec/command/json.hpp"
#include "ec/command/request.hpp"
#include "ec/feedback/state.hpp"
#include "ec/motion/conditions.hpp"
#include "ec/motion/error.hpp"
#include "ec/stamped.hpp"

namespace ec::motion {

using ec::Clock;  // the driver's one clock (ec/stamped.hpp)

/// The controller's answer to one command sent the cycle before.
struct Reply {
    std::string_view method;                              ///< the command's method name
    std::expected<command::Json, command::Error> result;  ///< its unwrapped result, or why there is none
};

/// One command to send, and whether the controller will answer it.
struct Outgoing {
    command::Request request;
    bool expects_reply = true;  ///< false for commands sent after asking the controller not to reply
};

/// The commands an activity wants sent at the end of this cycle, in order.
/// Whether they go out one by one, pipelined or over another connection is the
/// control thread's business; activities only state what to send. Every command
/// that expects a reply gets one in the next cycle's Cycle::replies: the
/// controller's answer, or the error that stood for it.
class Outbox {
public:
    template <command::Command C>
    void send(const C& command) {
        items_.push_back(Outgoing{command::to_request(command), true});
    }
    /// A request already erased, e.g. one prepared when the activity was built.
    void send(command::Request request) { items_.push_back(Outgoing{std::move(request), true}); }

    /// A command the controller was told not to answer (transparent_transmission_init
    /// with response_enable 0, start_push_pos with ret_flag 0): it gets no Reply.
    template <command::Command C>
    void send_without_reply(const C& command) {
        items_.push_back(Outgoing{command::to_request(command), false});
    }

    [[nodiscard]] std::span<const Outgoing> items() const noexcept { return items_; }
    [[nodiscard]] bool empty() const noexcept { return items_.empty(); }
    [[nodiscard]] std::size_t size() const noexcept { return items_.size(); }
    void clear() noexcept { items_.clear(); }

private:
    std::vector<Outgoing> items_;
};

/// Everything an activity sees in one cycle, copied at the cycle's start so that
/// the whole cycle works on one snapshot.
struct Cycle {
    Clock::time_point now{};
    std::chrono::nanoseconds period{std::chrono::milliseconds{8}};  ///< the control loop's nominal period
    feedback::State state{};                                        ///< the newest controller state
    std::uint64_t state_seq = 0;                                    ///< its sequence number; 0: no state yet
    std::chrono::nanoseconds state_age{};                           ///< now minus its arrival
    Conditions conditions{};                                        ///< what holds now (observe())
    std::span<const Reply> replies{};  ///< answers to the previous cycle's commands, in order
};

/// What step() reports.
enum class Phase {
    Running,  ///< step again next cycle
    Done,     ///< finished; close() follows
};

/// Why close() is called.
enum class End {
    Done,       ///< step() reported Done
    Cancelled,  ///< ended early on request, or after an error that does not latch: close gracefully
    Stopped,    ///< stop() was called: stop the arm as fast as the family allows
    Faulted,    ///< a fault: stop the arm as fast as the family allows
};

/// The conditions an activity needs: to be started, and on every cycle while it
/// runs. A missing start condition refuses the start; a missing running
/// condition ends the activity with a fault.
struct Needs {
    Conditions to_start{};
    Conditions while_running{};
    /// It must send to the controller every cycle (a stream): cycles that start
    /// late are a fault while it runs. A planned motion, which the controller
    /// times, or a procedure only runs later.
    bool every_cycle = false;
};

/// An activity (docs/motion.md 5.3):
///   - `needs() const`: its conditions; may change as it advances;
///   - `step(cycle, out)`: called every cycle until it returns Done or an error;
///     the first steps open the family's protocol;
///   - `close(cycle, out, why)`: called every cycle after that, also after a
///     cancel, a stop or a fault, until it returns true. Its contract: leave the
///     arm at rest and the controller's protocol clean, whatever `why` is.
/// Optional: `name() const`, for messages.
/// Activities do no I/O and read no clocks: time comes in the Cycle.
template <typename A>
concept Activity = std::move_constructible<A> && requires(A& a, const A& ca, const Cycle& c, Outbox& out, End why) {
    { ca.needs() } -> std::convertible_to<Needs>;
    { a.step(c, out) } -> std::same_as<std::expected<Phase, Error>>;
    { a.close(c, out, why) } -> std::same_as<bool>;
};

/// A description of an activity, as plain data (like a command struct): the
/// activity that runs it comes from `make_activity(spec)`, found by
/// argument-dependent lookup. The library's motions are specs, so that
/// `robot.run(MoveJoint{.target = q, .speed = 0.2})` reads like a datasheet.
template <typename S>
concept Spec = requires(const S& spec) {
    { make_activity(spec) } -> Activity;
};

/// The activity's name(), or "activity" if it has none.
template <Activity A>
[[nodiscard]] std::string_view name_of(const A& a) {
    if constexpr (requires {
                      { a.name() } -> std::convertible_to<std::string_view>;
                  })
        return a.name();
    else
        return "activity";
}

/// Any activity, held by value: what the control thread stores. Move-only. A
/// moved-from AnyActivity may only be assigned to or destroyed.
class AnyActivity {
public:
    template <Activity A>
        requires(!std::same_as<std::remove_cvref_t<A>, AnyActivity>)
    AnyActivity(A activity)  // NOLINT: implicit on purpose, so any activity converts
        : self_(std::make_unique<Model<A>>(std::move(activity))) {}

    /// The activity a spec describes.
    template <Spec S>
        requires(!Activity<S>)
    AnyActivity(const S& spec)  // NOLINT: implicit on purpose, so any spec converts
        : AnyActivity(make_activity(spec)) {}

    [[nodiscard]] Needs needs() const { return self_->needs(); }
    [[nodiscard]] std::expected<Phase, Error> step(const Cycle& c, Outbox& out) { return self_->step(c, out); }
    [[nodiscard]] bool close(const Cycle& c, Outbox& out, End why) { return self_->close(c, out, why); }
    [[nodiscard]] std::string_view name() const { return self_->name(); }

private:
    struct Interface {
        Interface() = default;
        Interface(const Interface&) = delete;
        Interface& operator=(const Interface&) = delete;
        Interface(Interface&&) = delete;
        Interface& operator=(Interface&&) = delete;
        virtual ~Interface() = default;
        [[nodiscard]] virtual Needs needs() const = 0;
        [[nodiscard]] virtual std::expected<Phase, Error> step(const Cycle& c, Outbox& out) = 0;
        [[nodiscard]] virtual bool close(const Cycle& c, Outbox& out, End why) = 0;
        [[nodiscard]] virtual std::string_view name() const = 0;
    };

    template <Activity A>
    struct Model final : Interface {
        explicit Model(A a) : activity(std::move(a)) {}
        [[nodiscard]] Needs needs() const override { return activity.needs(); }
        [[nodiscard]] std::expected<Phase, Error> step(const Cycle& c, Outbox& out) override {
            return activity.step(c, out);
        }
        [[nodiscard]] bool close(const Cycle& c, Outbox& out, End why) override { return activity.close(c, out, why); }
        [[nodiscard]] std::string_view name() const override { return name_of(activity); }
        A activity;
    };

    std::unique_ptr<Interface> self_;
};

static_assert(Activity<AnyActivity>);

/// The first reply to a command of type C among `replies`, or nullptr.
template <command::Command C>
[[nodiscard]] const Reply* find_reply(std::span<const Reply> replies) noexcept {
    for (const Reply& r : replies)
        if (r.method == C::method) return &r;
    return nullptr;
}

/// Decodes `reply` as the reply of command C.
template <command::Command C>
[[nodiscard]] std::expected<typename C::Reply, command::Error> decode(const Reply& reply) {
    if (reply.method != C::method)
        return command::bad_reply(std::format("expected a reply to {}, got one to {}", C::method, reply.method));
    if (!reply.result) return std::unexpected(reply.result.error());
    return command::decode_reply<C>(*reply.result);
}

/// The Error that ends an activity because command `method` failed.
[[nodiscard]] inline Error command_failed(std::string_view method, const command::Error& e) {
    if (e.kind == command::Error::Kind::Disconnected)
        return Error{Error::Kind::Disconnected, std::format("{}: {}", method, e.message), {}};
    return Error{Error::Kind::Command, std::format("{}: {}: {}", method, command::to_string(e.kind), e.message), {}};
}

}  // namespace ec::motion
