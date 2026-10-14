#include "ec/robot.hpp"

#include <format>
#include <thread>

#include "ec/command/kinematics.hpp"
#include "ec/feedback/frame.hpp"
#include "ec/latest.hpp"

namespace ec {

struct Robot::Impl {
    RobotOptions options;
    std::shared_ptr<Latest<feedback::Sample>> latest = std::make_shared<Latest<feedback::Sample>>();
    std::unique_ptr<feedback::Receiver> receiver;
    std::unique_ptr<motion::ControlLoop> loop;  // after the receiver: destroyed first, while frames still come
};

namespace {

std::optional<Snapshot> snapshot(const std::optional<Latest<feedback::Sample>::Entry>& entry) {
    if (!entry) return std::nullopt;
    const auto frame = feedback::decode(entry->value.raw);
    if (!frame) return std::nullopt;
    return Snapshot{entry->seq, feedback::to_si(*frame), entry->value.arrival};
}

// How long servo(), velocity() and jog() wait for their stream to open.
constexpr auto kOpenWithin = std::chrono::seconds{3};

}  // namespace

std::expected<Robot, motion::Error> Robot::connect(RobotOptions options) {
    auto impl = std::make_unique<Impl>();
    impl->options = std::move(options);
    const RobotOptions& o = impl->options;
    const command::Client::Options client_options{
        .port = o.command_port, .connect_timeout = o.connect_timeout, .reply_timeout = o.control.reply_timeout};
    auto client = command::Client::connect(o.host, client_options);
    if (!client)
        return std::unexpected(
            motion::Error{motion::Error::Kind::Disconnected,
                          std::format("command port {}:{}: {}", o.host, o.command_port, client.error().message),
                          {}});

    feedback::Receiver::Options feedback = o.feedback;
    feedback.port = o.monitor_port;
    impl->receiver = std::make_unique<feedback::Receiver>(
        o.host, feedback, [latest = impl->latest, extra = o.on_sample](const feedback::Sample& s) {
            latest->publish(s);
            if (extra) extra(s);
        });
    if (!impl->latest->wait_newer(0, o.first_state_within)) {
        const auto status = impl->receiver->status();
        return std::unexpected(
            motion::Error{motion::Error::Kind::Disconnected,
                          std::format("monitor port {}:{}: no frame within {} ms{}", o.host, o.monitor_port,
                                      o.first_state_within.count(),
                                      status.last_error ? " (" + status.last_error->message() + ")" : std::string{}),
                          {}});
    }

    auto reconnect = [host = o.host, client_options] { return command::Client::connect(host, client_options); };
    impl->loop =
        std::make_unique<motion::ControlLoop>(std::move(*client), std::move(reconnect), impl->latest, o.control);
    // Its first cycle publishes the conditions and the mode, so that status() means something at once.
    static_cast<void>(impl->loop->next_status(0, std::chrono::seconds{1}));
    return Robot(std::move(impl));
}

Robot::Robot(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
Robot::Robot(Robot&&) noexcept = default;
Robot& Robot::operator=(Robot&&) noexcept = default;

Robot::~Robot() {
    if (!impl_) return;
    impl_->loop.reset();  // stops what runs while the receiver still feeds it
    impl_->receiver.reset();
}

std::optional<Snapshot> Robot::state() const { return snapshot(impl_->latest->get()); }

std::optional<Snapshot> Robot::next_state(std::uint64_t seq, std::chrono::nanoseconds timeout) const {
    return snapshot(impl_->latest->wait_newer(seq, timeout));
}

motion::Status Robot::status() const { return impl_->loop->status(); }

feedback::Receiver::Status Robot::feedback_status() const { return impl_->receiver->status(); }

motion::Job Robot::run(motion::AnyActivity activity) { return impl_->loop->run(std::move(activity)); }

motion::Job Robot::bring_up(const motion::BringUp& spec) { return run(spec); }

template <typename T>
std::expected<motion::Session<T>, motion::Error> Robot::open(std::shared_ptr<motion::Channel<T>> channel,
                                                             motion::Job job) {
    const auto deadline = std::chrono::steady_clock::now() + kOpenWithin;
    while (!channel->is_open()) {
        if (const auto ended = job.wait_for(std::chrono::milliseconds{2})) {
            if (!*ended) return std::unexpected(ended->error());
            return std::unexpected(motion::Error{motion::Error::Kind::Refused, "the stream ended at once", {}});
        }
        if (std::chrono::steady_clock::now() > deadline) {
            job.cancel();
            return std::unexpected(motion::Error{motion::Error::Kind::Timeout, "the stream did not open", {}});
        }
    }
    return motion::Session<T>(std::move(channel), std::move(job));
}

std::expected<motion::Session<Joints>, motion::Error> Robot::servo(const motion::ServoOptions& options) {
    auto channel = std::make_shared<motion::Channel<Joints>>();
    auto job = run(motion::ServoStream(options, channel));
    return open(std::move(channel), std::move(job));
}

std::expected<motion::Session<Joints>, motion::Error> Robot::velocity(const motion::VelocityOptions& options) {
    auto channel = std::make_shared<motion::Channel<Joints>>();
    auto job = run(motion::VelocityStream(options, channel));
    return open(std::move(channel), std::move(job));
}

std::expected<Joints, motion::Error> Robot::ik(const Eigen::Isometry3d& T_base_tcp, const Joints& near) {
    auto joints = call(command::InverseKinematic{.target = to_mm_rad(T_base_tcp), .reference = rad_to_deg(near)});
    if (!joints) return std::unexpected(std::move(joints.error()));
    return nearest_turn(deg_to_rad(*joints), near, kEc66JointLimit);  // in case it comes within one turn
}

std::expected<Eigen::Isometry3d, motion::Error> Robot::fk(const Joints& joints) {
    auto pose = call(command::PositiveKinematic{.joints = rad_to_deg(joints)});
    if (!pose) return std::unexpected(std::move(pose.error()));
    return from_mm_rad(*pose);
}

motion::Job Robot::move_joints(const motion::MoveJoint& move) { return run(move); }

std::expected<motion::JointTrajectory, motion::Error> Robot::plan(const PoseTrajectory& motion,
                                                                  const motion::PlanOptions& options) {
    const auto now = state();
    if (!now) return std::unexpected(motion::Error{motion::Error::Kind::Refused, "no state yet", {}});
    return motion::plan(
        motion, now->state.joint_position,
        [this](const Eigen::Isometry3d& T_base_tcp, const Joints& near) { return ik(T_base_tcp, near); }, options);
}

motion::Job Robot::play(motion::JointTrajectory trajectory, double speed, const motion::TrajectoryLimits& limits) {
    return run(motion::PlayTrajectory{.path = std::move(trajectory), .speed = speed, .limits = limits});
}

void Robot::stop() noexcept { impl_->loop->stop(); }

std::expected<void, motion::Error> Robot::acknowledge() { return impl_->loop->acknowledge(); }

std::expected<command::Json, motion::Error> Robot::call_json(command::Request request, command::Kind kind) {
    return impl_->loop->call(std::move(request), kind);
}

}  // namespace ec
