// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Robot-local command/state ROS 2/DrDDS adapter for the Deep Robotics Lynx M20.
// High-bandwidth lidar and IMU ingress belongs to the M20 Point-LIO process.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>

#include <drdds/msg/nav_cmd.hpp>
#include <drdds/msg/gait.hpp>
#include <drdds/msg/motion_state.hpp>
#include <drdds/msg/motion_info.hpp>
#include <drdds/msg/std_msg_int32.hpp>
#include <rclcpp/rclcpp.hpp>

#include "dimos/native.hpp"

#include "geometry_msgs/Twist.hpp"
#include "std_msgs/Bool.hpp"
#include "std_msgs/Int32.hpp"
#include "std_msgs/UInt32.hpp"

using dimos::native::Builder;
using dimos::native::Config;
using dimos::native::Module;
using dimos::native::Output;
namespace logging = dimos::native::log;

namespace {

using Clock = std::chrono::steady_clock;
constexpr int64_t kNanosecondsPerSecond = 1'000'000'000LL;
constexpr int kMotionRlControl = 17;

void require_nonempty(const std::string& value, const char* name) {
    if (value.empty()) {
        throw std::runtime_error(std::string(name) + " must not be empty");
    }
}

double clamp(double value, double limit) {
    return std::max(-limit, std::min(limit, value));
}

geometry_msgs::Twist zero_twist() {
    geometry_msgs::Twist result;
    result.linear.x = 0.0;
    result.linear.y = 0.0;
    result.linear.z = 0.0;
    result.angular.x = 0.0;
    result.angular.y = 0.0;
    result.angular.z = 0.0;
    return result;
}

template <class T, class = void>
struct HasSec : std::false_type {};
template <class T>
struct HasSec<T, std::void_t<decltype(std::declval<T&>().sec)>> : std::true_type {};

template <class T, class = void>
struct HasNanosec : std::false_type {};
template <class T>
struct HasNanosec<T, std::void_t<decltype(std::declval<T&>().nanosec)>> : std::true_type {};

template <class T, class = void>
struct HasNsec : std::false_type {};
template <class T>
struct HasNsec<T, std::void_t<decltype(std::declval<T&>().nsec)>> : std::true_type {};

template <class T, class = void>
struct HasFrameId : std::false_type {};
template <class T>
struct HasFrameId<T, std::void_t<decltype(std::declval<T&>().frame_id)>> : std::true_type {};

template <class T, class = void>
struct HasTimestamp : std::false_type {};
template <class T>
struct HasTimestamp<T, std::void_t<decltype(std::declval<T&>().timestamp)>>
    : std::true_type {};

template <class T, class = void>
struct HasStamp : std::false_type {};
template <class T>
struct HasStamp<T, std::void_t<decltype(std::declval<T&>().stamp)>> : std::true_type {};

template <class T, class = void>
struct HasValue : std::false_type {};
template <class T>
struct HasValue<T, std::void_t<decltype(std::declval<const T&>().value)>> : std::true_type {};

template <class T, class = void>
struct HasData : std::false_type {};
template <class T>
struct HasData<T, std::void_t<decltype(std::declval<const T&>().data)>> : std::true_type {};

template <class Stamp>
void set_vendor_stamp(Stamp& stamp, int32_t sec, uint32_t nsec) {
    static_assert(HasSec<Stamp>::value, "M20 vendor Timestamp must expose sec");
    static_assert(HasNanosec<Stamp>::value || HasNsec<Stamp>::value,
                  "M20 vendor Timestamp must expose nanosec or nsec");
    stamp.sec = sec;
    if constexpr (HasNanosec<Stamp>::value) {
        stamp.nanosec = nsec;
    } else {
        stamp.nsec = nsec;
    }
}

// Released M20 message packages have used both `stamp` and `timestamp` in
// MetaType. Keep the bridge source-compatible with either installed version.
template <class Header>
void set_vendor_header(Header& header, uint64_t frame_id, const rclcpp::Time& now) {
    static_assert(HasFrameId<Header>::value, "M20 vendor MetaType must expose frame_id");
    static_assert(HasTimestamp<Header>::value || HasStamp<Header>::value,
                  "M20 vendor MetaType must expose timestamp or stamp");
    header.frame_id = frame_id;
    const int64_t total_ns = now.nanoseconds();
    const auto sec = static_cast<int32_t>(total_ns / kNanosecondsPerSecond);
    const auto nsec = static_cast<uint32_t>(total_ns % kNanosecondsPerSecond);
    if constexpr (HasTimestamp<Header>::value) {
        set_vendor_stamp(header.timestamp, sec, nsec);
    } else {
        set_vendor_stamp(header.stamp, sec, nsec);
    }
}

template <class Status>
int vendor_int32_value(const Status& status) {
    static_assert(HasValue<Status>::value || HasData<Status>::value,
                  "M20 vendor int32 status must expose value or data");
    if constexpr (HasValue<Status>::value) {
        return static_cast<int>(status.value);
    } else {
        return static_cast<int>(status.data);
    }
}

}  // namespace

struct M20ROSBridgeConfig {
    std::string nav_cmd_topic;
    std::string motion_state_topic;
    std::string motion_info_topic;
    std::string gait_topic;
    std::string hes_status_topic;
    std::string node_name;
    bool enable_command_output;
    double command_rate_hz;
    double command_timeout_s;
    double safety_timeout_s;
    double max_linear_x;
    double max_linear_y;
    double max_angular_z;

    void validate() const {
        require_nonempty(nav_cmd_topic, "nav_cmd_topic");
        require_nonempty(motion_state_topic, "motion_state_topic");
        require_nonempty(motion_info_topic, "motion_info_topic");
        require_nonempty(gait_topic, "gait_topic");
        require_nonempty(hes_status_topic, "hes_status_topic");
        require_nonempty(node_name, "node_name");
        dimos::native::require_positive(command_rate_hz, "command_rate_hz");
        dimos::native::require_positive(command_timeout_s, "command_timeout_s");
        dimos::native::require_positive(safety_timeout_s, "safety_timeout_s");
        dimos::native::require_positive(max_linear_x, "max_linear_x");
        dimos::native::require_positive(max_linear_y, "max_linear_y");
        dimos::native::require_positive(max_angular_z, "max_angular_z");
    }
};

M20ROSBridgeConfig parse_m20_config(Config& config) {
    M20ROSBridgeConfig result{};
    result.nav_cmd_topic = config.take<std::string>("nav_cmd_topic");
    result.motion_state_topic = config.take<std::string>("motion_state_topic");
    result.motion_info_topic = config.take<std::string>("motion_info_topic");
    result.gait_topic = config.take<std::string>("gait_topic");
    result.hes_status_topic = config.take<std::string>("hes_status_topic");
    result.node_name = config.take<std::string>("node_name");
    result.enable_command_output = config.take<bool>("enable_command_output");
    result.command_rate_hz = config.take<double>("command_rate_hz");
    result.command_timeout_s = config.take<double>("command_timeout_s");
    result.safety_timeout_s = config.take<double>("safety_timeout_s");
    result.max_linear_x = config.take<double>("max_linear_x");
    result.max_linear_y = config.take<double>("max_linear_y");
    result.max_angular_z = config.take<double>("max_angular_z");
    config.enforce_all_consumed();
    result.validate();
    return result;
}

class M20ROSBridge : public Module {
public:
    void build(Builder& builder, Config& config) override {
        cfg_ = parse_m20_config(config);
        builder.input<geometry_msgs::Twist>("safe_cmd_vel", &M20ROSBridge::on_command, this);
        builder.input<std_msgs::Int32>("motion_state_cmd",
                                      &M20ROSBridge::on_motion_state_command, this);
        builder.input<std_msgs::UInt32>("gait_cmd", &M20ROSBridge::on_gait_command, this);
        command_ready_ = builder.output<std_msgs::Bool>("command_ready");
        motion_state_ = builder.output<std_msgs::Int32>("motion_state");
        gait_state_ = builder.output<std_msgs::UInt32>("gait_state");
    }

    void setup() override {
        rclcpp::init(0, nullptr);
        // rclcpp installs process signal handlers during init. Restore the
        // NativeModule handlers so coordinator SIGTERM exits Module::handle();
        // teardown below then cancels the executor and shuts rclcpp down.
        dimos::native::install_signal_handlers();
        node_ = std::make_shared<rclcpp::Node>(cfg_.node_name);

        // The advertised M20 endpoint is RELIABLE/TRANSIENT_LOCAL. Some M20
        // firmware revisions expose the endpoint without actually emitting
        // its documented 1 Hz samples, so HES is a veto when observed rather
        // than the command-path heartbeat. The physical stop remains enforced
        // below this API by the robot controller.
        const auto hes_qos =
            rclcpp::QoS(rclcpp::KeepLast(2)).reliable().transient_local();
        hes_subscription_ = node_->create_subscription<drdds::msg::StdMsgInt32>(
            cfg_.hes_status_topic, hes_qos,
            [this](drdds::msg::StdMsgInt32::SharedPtr msg) { on_hes_status(*msg); });
        motion_info_subscription_ = node_->create_subscription<drdds::msg::MotionInfo>(
            cfg_.motion_info_topic,
            rclcpp::QoS(rclcpp::KeepLast(20)).reliable().durability_volatile(),
            [this](drdds::msg::MotionInfo::SharedPtr msg) { on_motion_info(*msg); });

        if (cfg_.enable_command_output) {
            nav_cmd_publisher_ = node_->create_publisher<drdds::msg::NavCmd>(
                cfg_.nav_cmd_topic, rclcpp::QoS(rclcpp::KeepLast(2)).reliable());
            motion_state_publisher_ = node_->create_publisher<drdds::msg::MotionState>(
                cfg_.motion_state_topic, rclcpp::QoS(rclcpp::KeepLast(2)).reliable());
            gait_publisher_ = node_->create_publisher<drdds::msg::Gait>(
                cfg_.gait_topic, rclcpp::QoS(rclcpp::KeepLast(2)).reliable());
        }

        const auto period = std::chrono::duration<double>(1.0 / cfg_.command_rate_hz);
        timer_ = node_->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            [this]() { publish_cycle(false); });

        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(node_);
        spin_thread_ = std::thread([this]() { executor_->spin(); });

        logging::info("M20 command/state ROS bridge started",
                      {logging::Field("command_output", cfg_.enable_command_output)});
    }

    void teardown() override {
        stopping_.store(true, std::memory_order_release);
        timer_.reset();
        if (nav_cmd_publisher_ != nullptr && rclcpp::ok()) {
            publish_cycle(true);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            publish_cycle(true);
        }
        if (executor_ != nullptr) {
            executor_->cancel();
        }
        if (spin_thread_.joinable()) {
            spin_thread_.join();
        }
        nav_cmd_publisher_.reset();
        motion_state_publisher_.reset();
        gait_publisher_.reset();
        hes_subscription_.reset();
        motion_info_subscription_.reset();
        if (executor_ != nullptr && node_ != nullptr) {
            executor_->remove_node(node_);
        }
        node_.reset();
        executor_.reset();
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

private:
    void on_command(const geometry_msgs::Twist& source) {
        geometry_msgs::Twist bounded = zero_twist();
        if (std::isfinite(source.linear.x) && std::isfinite(source.linear.y) &&
            std::isfinite(source.angular.z)) {
            bounded.linear.x = clamp(source.linear.x, cfg_.max_linear_x);
            bounded.linear.y = clamp(source.linear.y, cfg_.max_linear_y);
            bounded.angular.z = clamp(source.angular.z, cfg_.max_angular_z);
        }
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_command_ = bounded;
        command_received_at_ = Clock::now();
        have_command_ = true;
    }

    void on_motion_state_command(const std_msgs::Int32& source) {
        if (motion_state_publisher_ == nullptr || !rclcpp::ok()) return;
        drdds::msg::MotionState output;
        set_vendor_header(output.header, command_sequence_.fetch_add(1), node_->now());
        output.data.state = source.data;
        motion_state_publisher_->publish(output);
        logging::warn("published M20 motion-state command",
                      {logging::Field("state", static_cast<int64_t>(source.data))});
    }

    void on_gait_command(const std_msgs::UInt32& source) {
        if (gait_publisher_ == nullptr || !rclcpp::ok()) return;
        drdds::msg::Gait output;
        set_vendor_header(output.header, command_sequence_.fetch_add(1), node_->now());
        output.data.gait = source.data;
        gait_publisher_->publish(output);
        logging::info("published M20 gait command",
                      {logging::Field("gait", static_cast<int64_t>(source.data))});
    }

    void on_hes_status(const drdds::msg::StdMsgInt32& source) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        hes_status_ = vendor_int32_value(source);
        have_hes_ = true;
    }

    void on_motion_info(const drdds::msg::MotionInfo& source) {
        const int motion_state = source.data.motion_state.state;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            motion_state_value_ = motion_state;
            motion_info_received_at_ = Clock::now();
            have_motion_info_ = true;
        }
        std_msgs::Int32 motion;
        motion.data = motion_state;
        motion_state_.publish(motion);
        std_msgs::UInt32 gait;
        gait.data = source.data.gait_state.gait;
        gait_state_.publish(gait);
    }

    bool safety_ready(Clock::time_point now) const {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!cfg_.enable_command_output || !have_motion_info_) {
            return false;
        }
        const auto timeout = std::chrono::duration<double>(cfg_.safety_timeout_s);
        return now - motion_info_received_at_ <= timeout &&
               motion_state_value_ == kMotionRlControl &&
               (!have_hes_ || hes_status_ == 0);
    }

    geometry_msgs::Twist fresh_command_or_zero(Clock::time_point now) const {
        std::lock_guard<std::mutex> lock(state_mutex_);
        const auto timeout = std::chrono::duration<double>(cfg_.command_timeout_s);
        if (!have_command_ || now - command_received_at_ > timeout) {
            return zero_twist();
        }
        return latest_command_;
    }

    void publish_cycle(bool force_zero) {
        const auto now = Clock::now();
        const bool ready = !force_zero && !stopping_.load(std::memory_order_acquire) &&
                           nav_cmd_publisher_ != nullptr &&
                           nav_cmd_publisher_->get_subscription_count() > 0 &&
                           safety_ready(now);
        std_msgs::Bool ready_message;
        ready_message.data = static_cast<int8_t>(ready);
        command_ready_.publish(ready_message);

        if (nav_cmd_publisher_ == nullptr) {
            return;
        }
        const geometry_msgs::Twist command = ready ? fresh_command_or_zero(now) : zero_twist();
        drdds::msg::NavCmd output;
        set_vendor_header(output.header, command_sequence_.fetch_add(1), node_->now());
        output.data.x_vel = static_cast<float>(command.linear.x);
        output.data.y_vel = static_cast<float>(command.linear.y);
        output.data.yaw_vel = static_cast<float>(command.angular.z);
        nav_cmd_publisher_->publish(output);
    }

    M20ROSBridgeConfig cfg_;
    Output<std_msgs::Bool> command_ready_;
    Output<std_msgs::Int32> motion_state_;
    Output<std_msgs::UInt32> gait_state_;

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    rclcpp::Subscription<drdds::msg::StdMsgInt32>::SharedPtr hes_subscription_;
    rclcpp::Subscription<drdds::msg::MotionInfo>::SharedPtr motion_info_subscription_;
    rclcpp::Publisher<drdds::msg::NavCmd>::SharedPtr nav_cmd_publisher_;
    rclcpp::Publisher<drdds::msg::MotionState>::SharedPtr motion_state_publisher_;
    rclcpp::Publisher<drdds::msg::Gait>::SharedPtr gait_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::thread spin_thread_;

    mutable std::mutex state_mutex_;
    geometry_msgs::Twist latest_command_ = zero_twist();
    Clock::time_point command_received_at_{};
    Clock::time_point motion_info_received_at_{};
    bool have_command_ = false;
    bool have_hes_ = false;
    bool have_motion_info_ = false;
    int motion_state_value_ = 0;
    int hes_status_ = 1;
    std::atomic<bool> stopping_{false};
    std::atomic<uint64_t> command_sequence_{0};
};

int main() {
    dimos::native::run_with_transport<M20ROSBridge>();
    return 0;
}
