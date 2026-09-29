#include <spd_native/physics.hpp>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <std_msgs/msg/string.hpp>
#include <tianji_spd_interfaces/msg/joint_command.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace py = pybind11;

namespace spd_native {
namespace {
constexpr int kJointCount = 54;
constexpr int kReadyMask = 7;
constexpr std::int64_t kMaxAgeNs = 100'000'000;
constexpr std::int64_t kMaxFutureNs = 5'000'000;
constexpr std::int64_t kTransitionNs = 1'000'000'000;
constexpr const char *kRobotConfig = "tianji_wuji2_v1";
constexpr const char *kCommandTopic = "/spd/tianji_wuji2/v1/joint_command";
using Values = std::array<double, kJointCount>;
using Names = std::array<std::string, kJointCount>;
using WireCommand = tianji_spd_interfaces::msg::JointCommand;

struct CommandError : std::invalid_argument {
    using std::invalid_argument::invalid_argument;
};

const Names &canonical_names() {
    static const Names names = [] {
        Names result;
        std::size_t index = 0;
        for (const char *side : {"L", "R"}) {
            for (int joint = 1; joint <= 7; ++joint) {
                result[index++] = "Joint" + std::to_string(joint) + "_" + side;
            }
        }
        constexpr std::array<const char *, 20> hand_names = {
            "thumb_cmc_flex", "thumb_cmc_abd", "thumb_mcp", "thumb_ip",
            "index_finger_mcp_flex", "index_finger_mcp_abd", "index_finger_pip", "index_finger_dip",
            "middle_finger_mcp_flex", "middle_finger_mcp_abd", "middle_finger_pip", "middle_finger_dip",
            "ring_finger_mcp_flex", "ring_finger_mcp_abd", "ring_finger_pip", "ring_finger_dip",
            "pinky_mcp_flex", "pinky_mcp_abd", "pinky_pip", "pinky_dip",
        };
        for (const char *side : {"l", "r"}) {
            for (const char *name : hand_names) {
                result[index++] = std::string(side) + "_" + name;
            }
        }
        return result;
    }();
    return names;
}

struct Group {
    int bit;
    std::size_t begin;
    std::size_t end;
};
constexpr std::array<Group, 3> kGroups = {{{1, 0, 14}, {4, 14, 34}, {2, 34, 54}}};

std::int64_t system_time_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

std::int64_t steady_time_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::int64_t clock_time(const py::object &clock, bool monotonic = false) {
    if (!clock.is_none()) {
        return py::cast<std::int64_t>(clock());
    }
    return monotonic ? steady_time_ns() : system_time_ns();
}

void validate_clock(const py::object &clock) {
    if (!clock.is_none() && !PyCallable_Check(clock.ptr())) {
        throw py::type_error("clock must be callable or None");
    }
}

py::tuple names_tuple() {
    py::tuple result(kJointCount);
    const auto &names = canonical_names();
    for (std::size_t i = 0; i < names.size(); ++i) {
        result[i] = names[i];
    }
    return result;
}

py::tuple values_tuple(const Values &values) {
    py::tuple result(kJointCount);
    for (std::size_t i = 0; i < values.size(); ++i) {
        result[i] = values[i];
    }
    return result;
}

void validate_python_names(const py::iterable &names) {
    std::size_t i = 0;
    for (const auto name : names) {
        if (i == kJointCount || py::str(name).cast<std::string>() != canonical_names()[i]) {
            throw CommandError("joint_names do not match the canonical 54-DoF order");
        }
        ++i;
    }
    if (i != kJointCount) {
        throw CommandError("joint_names do not match the canonical 54-DoF order");
    }
}

Values python_values(const py::iterable &values) {
    Values result;
    std::size_t i = 0;
    for (const auto value : values) {
        if (i == result.size()) {
            throw CommandError("position_rad must contain 54 finite values");
        }
        result[i++] = py::cast<double>(value);
    }
    if (i != result.size()) {
        throw CommandError("position_rad must contain 54 finite values");
    }
    return result;
}

std::int64_t stamp_from_python(const py::handle &stamp) {
    try {
        const auto seconds = py::cast<std::int64_t>(stamp.attr("sec"));
        const auto nanos = py::cast<std::int64_t>(stamp.attr("nanosec"));
        if (seconds < 0 || seconds > 2'147'483'647 || nanos < 0 || nanos >= 1'000'000'000) {
            throw CommandError("invalid ROS timestamp");
        }
        return seconds * 1'000'000'000 + nanos;
    } catch (const py::error_already_set &) {
        throw CommandError("invalid ROS timestamp");
    } catch (const py::cast_error &) {
        throw CommandError("invalid ROS timestamp");
    }
}

struct JointCommandSnapshot {
    int schema_version;
    std::string robot_config;
    std::string session_id;
    std::uint64_t sequence;
    std::int64_t stamp_ns;
    int ready_mask;
    Values position_rad;

    void validate(std::optional<std::int64_t> now_ns = {},
                  std::optional<std::uint64_t> previous_sequence = {},
                  std::optional<std::int64_t> previous_stamp_ns = {}) const {
        if (schema_version != 1) {
            throw CommandError("unsupported schema_version: " + std::to_string(schema_version));
        }
        if (robot_config != kRobotConfig) {
            throw CommandError("unsupported robot_config: " + robot_config);
        }
        if (session_id.empty()) {
            throw CommandError("session_id must be non-empty");
        }
        if (sequence == 0) {
            throw CommandError("sequence must be a positive integer");
        }
        if (stamp_ns <= 0) {
            throw CommandError("stamp_ns must be a positive integer");
        }
        if (ready_mask & ~kReadyMask) {
            throw CommandError("ready_mask contains reserved bits");
        }
        for (double value : position_rad) {
            if (!std::isfinite(value)) {
                throw CommandError("position_rad must contain 54 finite values");
            }
        }
        if (previous_sequence && sequence <= *previous_sequence) {
            throw CommandError("sequence is not strictly increasing");
        }
        if (previous_stamp_ns && stamp_ns < *previous_stamp_ns) {
            throw CommandError("stamp_ns rolled back");
        }
        if (now_ns) {
            // Compare unsigned distances only after ordering, avoiding signed overflow.
            if (*now_ns >= stamp_ns &&
                static_cast<std::uint64_t>(*now_ns) - static_cast<std::uint64_t>(stamp_ns) > kMaxAgeNs) {
                throw CommandError("command is older than 100 ms");
            }
            if (*now_ns < stamp_ns &&
                static_cast<std::uint64_t>(stamp_ns) - static_cast<std::uint64_t>(*now_ns) > kMaxFutureNs) {
                throw CommandError("command is more than 5 ms in the future");
            }
        }
    }
};

using SnapshotPtr = std::shared_ptr<JointCommandSnapshot>;

SnapshotPtr snapshot_from_python(const py::handle &message) {
    if (py::isinstance<JointCommandSnapshot>(message)) {
        return py::cast<SnapshotPtr>(message);
    }
    validate_python_names(py::reinterpret_borrow<py::iterable>(message.attr("joint_names")));
    if (py::isinstance<py::bool_>(message.attr("sequence"))) {
        throw CommandError("sequence must be a positive integer");
    }
    std::int64_t stamp;
    if (py::hasattr(message, "stamp_ns")) {
        if (py::isinstance<py::bool_>(message.attr("stamp_ns"))) {
            throw CommandError("stamp_ns must be a positive integer");
        }
        stamp = py::cast<std::int64_t>(message.attr("stamp_ns"));
    } else {
        stamp = stamp_from_python(message.attr("stamp"));
    }
    return std::make_shared<JointCommandSnapshot>(JointCommandSnapshot{
        py::cast<int>(message.attr("schema_version")),
        py::str(message.attr("robot_config")).cast<std::string>(),
        py::str(message.attr("session_id")).cast<std::string>(),
        py::cast<std::uint64_t>(message.attr("sequence")),
        stamp,
        py::cast<int>(message.attr("ready_mask")),
        python_values(py::reinterpret_borrow<py::iterable>(message.attr("position_rad"))),
    });
}

SnapshotPtr snapshot_from_wire(const WireCommand &message) {
    if (message.joint_names != canonical_names()) {
        throw CommandError("joint_names do not match the canonical 54-DoF order");
    }
    if (message.stamp.sec < 0 || message.stamp.nanosec >= 1'000'000'000) {
        throw CommandError("invalid ROS timestamp");
    }
    return std::make_shared<JointCommandSnapshot>(JointCommandSnapshot{
        message.schema_version, message.robot_config, message.session_id, message.sequence,
        static_cast<std::int64_t>(message.stamp.sec) * 1'000'000'000 + message.stamp.nanosec,
        message.ready_mask, message.position_rad,
    });
}

struct AppliedCommand {
    SnapshotPtr snapshot;
    std::int64_t applied_sim_time_ns;
    int hold_mask;
    Values position_rad;
};

class RosJointCommandExecutor;

class JointCommandMailbox {
public:
    explicit JointCommandMailbox(py::object clock = py::none(), std::shared_ptr<Physics> physics = {})
        : physics_(std::move(physics)), clock_(std::move(clock)) {
        validate_clock(clock_);
    }

    bool receive(const py::object &message, std::optional<std::int64_t> now_ns) {
        const auto now = now_ns ? *now_ns : clock_time(clock_);
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        ++received_;
        try {
            return receive_locked(snapshot_from_python(message), now);
        } catch (const std::exception &error) {
            return reject_locked(error.what());
        }
    }

    void receive(const WireCommand &message) {
        const auto now = clock_time(clock_);
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        ++received_;
        try {
            receive_locked(snapshot_from_wire(message), now);
        } catch (const std::exception &error) {
            reject_locked(error.what());
        }
    }

    bool authorize(bool enabled, const std::optional<std::string> &session_id = {},
                   std::optional<std::int64_t> now_ns = {}) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        disable_locked();
        if (!enabled) {
            return true;
        }
        if (!latest_) {
            last_reject_reason_ = "no legal command candidate";
            return false;
        }
        try {
            latest_->validate(now_ns ? *now_ns : clock_time(clock_));
            if (latest_->ready_mask == 0) {
                throw CommandError("candidate has no ready groups");
            }
            if (session_id && latest_->session_id != *session_id) {
                throw CommandError("candidate session does not match authorization");
            }
        } catch (const std::exception &error) {
            last_reject_reason_ = error.what();
            return false;
        }
        enabled_ = true;
        authorized_session_ = latest_->session_id;
        pending_ = latest_;
        return true;
    }

    void clear() {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        disable_locked();
        latest_.reset();
        last_session_.reset();
        last_sequence_.reset();
        last_stamp_ns_.reset();
        last_reject_reason_.clear();
    }

    SnapshotPtr take_pending() {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        return std::exchange(pending_, {});
    }

    SnapshotPtr latest() const {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        return latest_;
    }

    bool enabled() const {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        return enabled_;
    }

    std::optional<std::string> authorized_session() const {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        return authorized_session_;
    }

    std::uint64_t received() const {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        return received_;
    }

    std::uint64_t accepted() const {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        return accepted_;
    }

    std::uint64_t rejected() const {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        return rejected_;
    }

    std::string last_reject_reason() const {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        return last_reject_reason_;
    }

private:
    friend class RosJointCommandExecutor;

    bool receive_locked(SnapshotPtr snapshot, std::int64_t now) {
        const bool same_session = last_session_ && *last_session_ == snapshot->session_id;
        snapshot->validate(now, same_session ? last_sequence_ : std::nullopt,
                           same_session ? last_stamp_ns_ : std::nullopt);
        if (physics_) {
            const auto targets = physics_->validate_values(snapshot->position_rad, snapshot->ready_mask);
            if (targets != snapshot->position_rad) {
                // A Python caller may retain this immutable source snapshot.
                if (!snapshot.unique()) {
                    snapshot = std::make_shared<JointCommandSnapshot>(*snapshot);
                }
                snapshot->position_rad = targets;
            }
        }
        if (authorized_session_ && snapshot->session_id != *authorized_session_) {
            disable_locked();
            last_reject_reason_ = "session changed; explicit authorization required";
        } else if (transition_ready_mask_ && snapshot->ready_mask != transition_ready_mask_) {
            disable_locked();
            last_reject_reason_ = "ready groups changed during transition; explicit authorization required";
        }
        latest_ = std::move(snapshot);
        last_session_ = latest_->session_id;
        last_sequence_ = latest_->sequence;
        last_stamp_ns_ = latest_->stamp_ns;
        ++accepted_;
        if (enabled_) {
            pending_ = latest_;
        }
        return true;
    }

    void disable_locked() {
        enabled_ = false;
        authorized_session_.reset();
        pending_.reset();
        transition_ready_mask_ = 0;
    }

    bool reject_locked(const std::string &reason) {
        ++rejected_;
        last_reject_reason_ = reason;
        if (transition_ready_mask_) {
            disable_locked();
        }
        return false;
    }

    mutable std::recursive_mutex mutex_;
    std::shared_ptr<Physics> physics_;
    py::object clock_;
    SnapshotPtr latest_;
    SnapshotPtr pending_;
    bool enabled_ = false;
    std::optional<std::string> authorized_session_;
    int transition_ready_mask_ = 0;
    std::optional<std::string> last_session_;
    std::optional<std::uint64_t> last_sequence_;
    std::optional<std::int64_t> last_stamp_ns_;
    std::uint64_t received_ = 0;
    std::uint64_t accepted_ = 0;
    std::uint64_t rejected_ = 0;
    std::string last_reject_reason_;
};

class RosJointCommandExecutor {
public:
    RosJointCommandExecutor(py::object plant, double max_enable_delta_rad, bool subscribe,
                            py::object clock, py::object monotonic, std::optional<std::size_t> domain_id)
        : plant_(std::move(plant)), physics_(plant_.attr("_physics").cast<std::shared_ptr<Physics>>()),
          clock_(std::move(clock)), monotonic_(std::move(monotonic)),
          mailbox_(std::make_shared<JointCommandMailbox>(clock_, physics_)),
          held_targets_(physics_->targets()), max_enable_delta_rad_(max_enable_delta_rad),
          owner_(std::this_thread::get_id()) {
        if (!std::isfinite(max_enable_delta_rad_) || max_enable_delta_rad_ < 0) {
            throw std::invalid_argument("max_enable_delta_rad must be finite and nonnegative");
        }
        validate_clock(monotonic_);
        if (!subscribe) {
            return;
        }
        context_ = std::make_shared<rclcpp::Context>();
        try {
            rclcpp::InitOptions init_options;
            if (domain_id) {
                init_options.set_domain_id(*domain_id);
            }
            context_->init(0, nullptr, init_options);
            rclcpp::NodeOptions options;
            options.context(context_);
            static std::atomic<unsigned long> next_node{0};
            node_ = std::make_shared<rclcpp::Node>(
                "spd_joint_command_executor_" + std::to_string(next_node++), options);
            rclcpp::ExecutorOptions executor_options;
            executor_options.context = context_;
            executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executor_options);
            subscription_ = node_->create_subscription<WireCommand>(
                kCommandTopic, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
                [mailbox = mailbox_](WireCommand::ConstSharedPtr message) { mailbox->receive(*message); });
            status_publisher_ = node_->create_publisher<std_msgs::msg::String>(
                "/spd/collection/status", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
            executor_->add_node(node_);
        } catch (...) {
            close();
            throw;
        }
    }

    ~RosJointCommandExecutor() {
        try {
            close();
        } catch (...) {
            // Destruction cannot propagate an RMW shutdown failure.
        }
    }

    std::shared_ptr<JointCommandMailbox> mailbox() const { return mailbox_; }

    int hold_mask() const {
        std::lock_guard<std::recursive_mutex> lock(mailbox_->mutex_);
        return mailbox_->enabled_ ? hold_mask_ : kReadyMask;
    }

    std::string state() const {
        std::lock_guard<std::recursive_mutex> lock(mailbox_->mutex_);
        if (!mailbox_->enabled_) {
            return mailbox_->latest_ ? "candidate" : "disabled";
        }
        return hold_mask_ ? "holding" : "enabled";
    }

    bool transition_active() const {
        std::lock_guard<std::recursive_mutex> lock(mailbox_->mutex_);
        return mailbox_->enabled_ && mailbox_->transition_ready_mask_ != 0;
    }

    bool transition_frame() const {
        std::lock_guard<std::recursive_mutex> lock(mailbox_->mutex_);
        return transition_frame_;
    }

    bool authorize(bool enabled) {
        require_owner();
        std::lock_guard<std::recursive_mutex> lock(mailbox_->mutex_);
        cancel_transition();
        hold_mask_ = latched_hold_ = kReadyMask;
        mailbox_->disable_locked();
        if (!enabled) {
            return true;
        }
        const auto &candidate = mailbox_->latest_;
        if (!candidate) {
            mailbox_->last_reject_reason_ = "no legal command candidate";
            return false;
        }
        for (const auto &group : kGroups) {
            if (candidate->ready_mask & group.bit) {
                for (auto i = group.begin; i < group.end; ++i) {
                    if (std::abs(candidate->position_rad[i] - held_targets_[i]) > max_enable_delta_rad_) {
                        mailbox_->last_reject_reason_ = "candidate exceeds enable delta from retained target";
                        return false;
                    }
                }
            }
        }
        if (!mailbox_->authorize(true)) {
            return false;
        }
        const auto now = clock_time(monotonic_, true);
        last_ready_ns_.fill(now);
        latched_hold_ = 0;
        hold_mask_ = kReadyMask ^ candidate->ready_mask;
        return true;
    }

    bool authorize_transition() {
        require_owner();
        std::lock_guard<std::recursive_mutex> lock(mailbox_->mutex_);
        cancel_transition();
        hold_mask_ = latched_hold_ = kReadyMask;
        if (!mailbox_->authorize(true)) {
            return false;
        }
        try {
            physics_->start_positions();
        } catch (const std::exception &error) {
            mailbox_->reject_locked(error.what());
            mailbox_->disable_locked();
            return false;
        }
        mailbox_->transition_ready_mask_ = mailbox_->latest_->ready_mask;
        last_ready_ns_.fill({});
        latched_hold_ = 0;
        hold_mask_ = kReadyMask ^ mailbox_->latest_->ready_mask;
        return true;
    }

    void clear() {
        require_owner();
        std::lock_guard<std::recursive_mutex> lock(mailbox_->mutex_);
        cancel_transition();
        mailbox_->clear();
        held_targets_ = physics_->targets();
        last_ready_ns_.fill({});
        hold_mask_ = latched_hold_ = kReadyMask;
    }

    std::optional<AppliedCommand> apply_pending(std::optional<std::int64_t> now_ns) {
        require_owner();
        const auto now = now_ns ? *now_ns : clock_time(monotonic_, true);
        std::lock_guard<std::recursive_mutex> lock(mailbox_->mutex_);
        transition_frame_ = false;
        if (!mailbox_->enabled_) {
            cancel_transition();
            hold_mask_ = kReadyMask;
            physics_->set_hold(hold_mask_);
            return {};
        }
        if (mailbox_->transition_ready_mask_) {
            return apply_transition(now);
        }
        cancel_transition();
        for (std::size_t i = 0; i < kGroups.size(); ++i) {
            if (last_ready_ns_[i] && now > *last_ready_ns_[i] &&
                static_cast<std::uint64_t>(now) - static_cast<std::uint64_t>(*last_ready_ns_[i]) > kMaxAgeNs) {
                latched_hold_ |= kGroups[i].bit;
            }
        }
        hold_mask_ |= latched_hold_;
        physics_->set_hold(hold_mask_);
        auto snapshot = mailbox_->take_pending();
        if (!snapshot) {
            return {};
        }
        const auto utc_now = clock_time(clock_);
        try {
            snapshot->validate(utc_now);
            physics_->submit_values(snapshot->position_rad, snapshot->ready_mask, latched_hold_);
        } catch (const std::exception &error) {
            mailbox_->reject_locked(error.what());
            return {};
        }
        hold_mask_ = (kReadyMask ^ snapshot->ready_mask) | latched_hold_;
        update_freshness(*snapshot, now, utc_now);
        held_targets_ = physics_->targets();
        return AppliedCommand{std::move(snapshot), physics_->sim_time_ns(), hold_mask_, held_targets_};
    }

    void spin_once() {
        require_owner();
        if (executor_ && context_->is_valid()) {
            executor_->spin_some(std::chrono::nanoseconds(0));
        }
    }

    void publish_status(const std::string &json) {
        require_owner();
        if (status_publisher_ && context_->is_valid()) {
            std_msgs::msg::String message;
            message.data = json;
            status_publisher_->publish(message);
        }
    }

    bool context_ok() const {
        return !closed_ && (!context_ || context_->is_valid());
    }

    void close() {
        if (closed_) {
            return;
        }
        closed_ = true;
        {
            std::lock_guard<std::recursive_mutex> lock(mailbox_->mutex_);
            mailbox_->disable_locked();
            cancel_transition();
            hold_mask_ = latched_hold_ = kReadyMask;
            mailbox_->clock_ = py::none();
        }
        clock_ = py::none();
        monotonic_ = py::none();
        plant_ = py::none();
        subscription_.reset();
        status_publisher_.reset();
        executor_.reset();
        node_.reset();
        if (context_ && context_->is_valid()) {
            context_->shutdown("SPD executor closed");
        }
        context_.reset();
    }

private:
    void require_owner() const {
        if (std::this_thread::get_id() != owner_) {
            throw std::runtime_error("executor must run on its owning physics thread");
        }
        if (closed_) {
            throw std::runtime_error("executor is closed");
        }
    }

    void cancel_transition() {
        mailbox_->transition_ready_mask_ = 0;
        transition_start_ns_.reset();
        transition_frame_ = false;
    }

    void update_freshness(const JointCommandSnapshot &snapshot, std::int64_t now, std::int64_t utc_now) {
        const auto source_monotonic = now - std::max<std::int64_t>(0, utc_now - snapshot.stamp_ns);
        for (std::size_t i = 0; i < kGroups.size(); ++i) {
            if ((snapshot.ready_mask & kGroups[i].bit) && !(latched_hold_ & kGroups[i].bit)) {
                last_ready_ns_[i] = source_monotonic;
            }
        }
    }

    std::optional<AppliedCommand> apply_transition(std::int64_t now) {
        const auto snapshot = mailbox_->latest_;
        double fraction = 0;
        std::int64_t utc_now;
        try {
            utc_now = clock_time(clock_);
            if (!snapshot || !mailbox_->authorized_session_ ||
                snapshot->session_id != *mailbox_->authorized_session_) {
                throw CommandError("transition lost its authorized session");
            }
            snapshot->validate(utc_now);
            if (snapshot->ready_mask != mailbox_->transition_ready_mask_) {
                throw CommandError("ready groups changed during transition");
            }
            if (!transition_start_ns_) {
                transition_origin_ = physics_->start_positions();
                transition_start_ns_ = now;
            }
            const auto elapsed = now > *transition_start_ns_
                ? static_cast<std::uint64_t>(now) - static_cast<std::uint64_t>(*transition_start_ns_) : 0;
            fraction = std::min(1.0, static_cast<double>(elapsed) / static_cast<double>(kTransitionNs));
            if (fraction == 1.0) {
                physics_->submit_values(snapshot->position_rad, snapshot->ready_mask);
            } else {
                const auto weight = fraction * fraction * fraction * (10.0 + fraction * (-15.0 + 6.0 * fraction));
                Values targets;
                for (std::size_t i = 0; i < targets.size(); ++i) {
                    targets[i] = transition_origin_[i] + weight * (snapshot->position_rad[i] - transition_origin_[i]);
                }
                physics_->submit_values(targets, snapshot->ready_mask);
            }
        } catch (const std::exception &error) {
            mailbox_->reject_locked(error.what());
            mailbox_->disable_locked();
            cancel_transition();
            hold_mask_ = latched_hold_ = kReadyMask;
            physics_->set_hold(kReadyMask);
            return {};
        }
        mailbox_->pending_.reset();
        hold_mask_ = kReadyMask ^ snapshot->ready_mask;
        update_freshness(*snapshot, now, utc_now);
        held_targets_ = physics_->targets();
        if (fraction == 1.0) {
            cancel_transition();
        }
        transition_frame_ = true;
        return AppliedCommand{snapshot, physics_->sim_time_ns(), hold_mask_, held_targets_};
    }

    py::object plant_;
    std::shared_ptr<Physics> physics_;
    py::object clock_;
    py::object monotonic_;
    std::shared_ptr<JointCommandMailbox> mailbox_;
    Values held_targets_;
    Values transition_origin_{};
    std::array<std::optional<std::int64_t>, 3> last_ready_ns_{};
    std::optional<std::int64_t> transition_start_ns_;
    double max_enable_delta_rad_;
    int latched_hold_ = kReadyMask;
    int hold_mask_ = kReadyMask;
    bool transition_frame_ = false;
    bool closed_ = false;
    std::thread::id owner_;
    std::shared_ptr<rclcpp::Context> context_;
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    rclcpp::Subscription<WireCommand>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
};
}  // namespace

std::pair<JointValues, int> decode_joint_command(py::handle message) {
    const auto snapshot = snapshot_from_python(message);
    snapshot->validate();
    return {snapshot->position_rad, snapshot->ready_mask};
}

}  // namespace spd_native

void bind_executor(py::module_ &m) {
    using namespace spd_native;
    py::register_exception<CommandError>(m, "JointCommandError", PyExc_ValueError);
    m.attr("SCHEMA_VERSION") = 1;
    m.attr("ROBOT_CONFIG") = kRobotConfig;
    m.attr("TOPIC") = kCommandTopic;
    m.attr("QOS_DEPTH") = 1;
    m.attr("MAX_AGE_NS") = kMaxAgeNs;
    m.attr("MAX_FUTURE_NS") = kMaxFutureNs;
    m.attr("ARMS_READY") = 1;
    m.attr("RIGHT_HAND_READY") = 2;
    m.attr("LEFT_HAND_READY") = 4;
    m.attr("VALID_READY_MASK") = kReadyMask;
    const auto names = names_tuple();
    m.attr("JOINT_NAMES") = names;
    m.attr("JOINT_NAME_TUPLE") = names;
    py::tuple arm_names(14);
    for (int i = 0; i < 14; ++i) {
        arm_names[i] = names[i];
    }
    m.attr("ARM_NAMES") = arm_names;

    py::class_<JointCommandSnapshot, SnapshotPtr>(m, "JointCommandSnapshot")
        .def(py::init([](int schema_version, std::string robot_config, std::string session_id,
                         std::uint64_t sequence, std::int64_t stamp_ns, int ready_mask,
                         py::iterable joint_names, py::iterable position_rad) {
            validate_python_names(joint_names);
            auto result = std::make_shared<JointCommandSnapshot>(JointCommandSnapshot{
                schema_version, std::move(robot_config), std::move(session_id), sequence,
                stamp_ns, ready_mask, python_values(position_rad)});
            result->validate();
            return result;
        }), py::arg("schema_version"), py::arg("robot_config"), py::arg("session_id"),
            py::arg("sequence"), py::arg("stamp_ns"), py::arg("ready_mask"),
            py::arg("joint_names"), py::arg("position_rad"))
        .def_readonly("schema_version", &JointCommandSnapshot::schema_version)
        .def_readonly("robot_config", &JointCommandSnapshot::robot_config)
        .def_readonly("session_id", &JointCommandSnapshot::session_id)
        .def_readonly("sequence", &JointCommandSnapshot::sequence)
        .def_readonly("stamp_ns", &JointCommandSnapshot::stamp_ns)
        .def_readonly("ready_mask", &JointCommandSnapshot::ready_mask)
        .def_property_readonly("joint_names", [](const JointCommandSnapshot &) { return names_tuple(); })
        .def_property_readonly("position_rad", [](const JointCommandSnapshot &s) { return values_tuple(s.position_rad); })
        .def("validate", [](SnapshotPtr snapshot, std::optional<std::int64_t> now_ns,
                            std::optional<std::uint64_t> previous_sequence,
                            std::optional<std::int64_t> previous_stamp_ns) {
            snapshot->validate(now_ns, previous_sequence, previous_stamp_ns);
            return snapshot;
        }, py::kw_only(), py::arg("now_ns") = py::none(), py::arg("previous_sequence") = py::none(),
            py::arg("previous_stamp_ns") = py::none())
        .def_static("from_values", [](std::string session_id, std::uint64_t sequence, int ready_mask,
                                      py::iterable position_rad, std::optional<std::int64_t> stamp_ns,
                                      std::string robot_config) {
            auto result = std::make_shared<JointCommandSnapshot>(JointCommandSnapshot{
                1, std::move(robot_config), std::move(session_id), sequence,
                stamp_ns ? *stamp_ns : system_time_ns(), ready_mask, python_values(position_rad)});
            result->validate();
            return result;
        }, py::kw_only(), py::arg("session_id"), py::arg("sequence"), py::arg("ready_mask"),
            py::arg("position_rad"), py::arg("stamp_ns") = py::none(), py::arg("robot_config") = kRobotConfig);

    m.def("ros_time_to_ns", &stamp_from_python, py::arg("stamp"));
    m.def("snapshot_from_ros", [](py::object message, std::optional<std::int64_t> now_ns,
                                  std::optional<std::uint64_t> previous_sequence,
                                  std::optional<std::int64_t> previous_stamp_ns) {
        auto snapshot = snapshot_from_python(message);
        snapshot->validate(now_ns ? *now_ns : system_time_ns(), previous_sequence, previous_stamp_ns);
        return snapshot;
    }, py::arg("message"), py::kw_only(), py::arg("now_ns") = py::none(),
        py::arg("previous_sequence") = py::none(), py::arg("previous_stamp_ns") = py::none());

    py::class_<AppliedCommand>(m, "AppliedCommand")
        .def_readonly("snapshot", &AppliedCommand::snapshot)
        .def_readonly("applied_sim_time_ns", &AppliedCommand::applied_sim_time_ns)
        .def_readonly("hold_mask", &AppliedCommand::hold_mask)
        .def_property_readonly("position_rad", [](const AppliedCommand &command) { return values_tuple(command.position_rad); });

    py::class_<JointCommandMailbox, std::shared_ptr<JointCommandMailbox>>(m, "JointCommandMailbox")
        .def(py::init([](py::object clock) { return std::make_shared<JointCommandMailbox>(std::move(clock)); }),
             py::kw_only(), py::arg("clock_ns") = py::none())
        .def("receive", py::overload_cast<const py::object &, std::optional<std::int64_t>>(&JointCommandMailbox::receive),
             py::arg("message"), py::kw_only(), py::arg("now_ns") = py::none())
        .def("authorize", &JointCommandMailbox::authorize, py::arg("enabled"), py::kw_only(),
             py::arg("session_id") = py::none(), py::arg("now_ns") = py::none())
        .def("clear", &JointCommandMailbox::clear)
        .def("take_pending", &JointCommandMailbox::take_pending)
        .def_property_readonly("latest", &JointCommandMailbox::latest)
        .def_property_readonly("enabled", &JointCommandMailbox::enabled)
        .def_property_readonly("authorized_session", &JointCommandMailbox::authorized_session)
        .def_property_readonly("received", &JointCommandMailbox::received)
        .def_property_readonly("accepted", &JointCommandMailbox::accepted)
        .def_property_readonly("rejected", &JointCommandMailbox::rejected)
        .def_property_readonly("last_reject_reason", &JointCommandMailbox::last_reject_reason);

    py::class_<RosJointCommandExecutor, std::shared_ptr<RosJointCommandExecutor>>(m, "RosJointCommandExecutor")
        .def(py::init<py::object, double, bool, py::object, py::object, std::optional<std::size_t>>(),
             py::arg("plant"), py::arg("max_enable_delta_rad") = 0.15, py::arg("subscribe") = true,
             py::arg("clock_ns") = py::none(), py::arg("monotonic_ns") = py::none(),
             py::arg("domain_id") = py::none())
        .def_property_readonly("mailbox", &RosJointCommandExecutor::mailbox)
        .def_property_readonly("hold_mask", &RosJointCommandExecutor::hold_mask)
        .def_property_readonly("state", &RosJointCommandExecutor::state)
        .def_property_readonly("transition_active", &RosJointCommandExecutor::transition_active)
        .def_property_readonly("transition_frame", &RosJointCommandExecutor::transition_frame)
        .def_property_readonly("context_ok", &RosJointCommandExecutor::context_ok)
        .def("authorize", &RosJointCommandExecutor::authorize, py::arg("enabled"))
        .def("authorize_transition", &RosJointCommandExecutor::authorize_transition)
        .def("clear", &RosJointCommandExecutor::clear)
        .def("apply_pending", &RosJointCommandExecutor::apply_pending,
             py::kw_only(), py::arg("now_ns") = py::none())
        .def("spin_once", &RosJointCommandExecutor::spin_once)
        .def("publish_status", &RosJointCommandExecutor::publish_status, py::arg("json"))
        .def("close", &RosJointCommandExecutor::close);
}
