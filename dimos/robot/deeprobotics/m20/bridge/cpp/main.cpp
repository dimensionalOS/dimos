// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Robot-local ROS 2/DrDDS adapter for the Deep Robotics Lynx M20.
// DimOS and this process both run on GOS. ROS 2 is used only to reach the
// vendor topics; typed DimOS streams use the native SDK's local LCM transport.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <drdds/msg/location_status.hpp>
#include <drdds/msg/nav_cmd.hpp>
#include <drdds/msg/std_msg_int32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "dimos/native.hpp"

#include "geometry_msgs/PoseStamped.hpp"
#include "geometry_msgs/TransformStamped.hpp"
#include "geometry_msgs/Twist.hpp"
#include "nav_msgs/Odometry.hpp"
#include "sensor_msgs/PointCloud2.hpp"
#include "sensor_msgs/PointField.hpp"
#include "std_msgs/Bool.hpp"
#include "std_msgs/Header.hpp"
#include "tf2_msgs/TFMessage.hpp"

using dimos::native::Builder;
using dimos::native::Config;
using dimos::native::Module;
using dimos::native::Output;
namespace logging = dimos::native::log;

namespace {

using Clock = std::chrono::steady_clock;
constexpr int64_t kNanosecondsPerSecond = 1'000'000'000LL;

void require_nonempty(const std::string& value, const char* name) {
    if (value.empty()) {
        throw std::runtime_error(std::string(name) + " must not be empty");
    }
}

int32_t checked_i32(std::size_t value, const char* name) {
    if (value > static_cast<std::size_t>(std::numeric_limits<int32_t>::max())) {
        throw std::runtime_error(std::string(name) + " exceeds the DimOS message limit");
    }
    return static_cast<int32_t>(value);
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

template <class Stamp>
void set_vendor_stamp(Stamp& stamp, int32_t sec, uint32_t nsec) {
    static_assert(requires(Stamp value) { value.sec = int32_t{}; },
                  "M20 vendor Timestamp must expose sec");
    static_assert(requires(Stamp value) { value.nanosec = uint32_t{}; } ||
                      requires(Stamp value) { value.nsec = uint32_t{}; },
                  "M20 vendor Timestamp must expose nanosec or nsec");
    stamp.sec = sec;
    if constexpr (requires { stamp.nanosec = nsec; }) {
        stamp.nanosec = nsec;
    } else {
        stamp.nsec = nsec;
    }
}

// Released M20 message packages have used both `stamp` and `timestamp` in
// MetaType. Keep the bridge source-compatible with either installed version.
template <class Header>
void set_vendor_header(Header& header, uint64_t frame_id, const rclcpp::Time& now) {
    static_assert(requires(Header value) { value.frame_id = uint64_t{}; },
                  "M20 vendor MetaType must expose frame_id");
    static_assert(requires(Header value) { value.timestamp; } ||
                      requires(Header value) { value.stamp; },
                  "M20 vendor MetaType must expose timestamp or stamp");
    header.frame_id = frame_id;
    const int64_t total_ns = now.nanoseconds();
    const auto sec = static_cast<int32_t>(total_ns / kNanosecondsPerSecond);
    const auto nsec = static_cast<uint32_t>(total_ns % kNanosecondsPerSecond);
    if constexpr (requires { header.timestamp; }) {
        set_vendor_stamp(header.timestamp, sec, nsec);
    } else {
        set_vendor_stamp(header.stamp, sec, nsec);
    }
}

std_msgs::Header to_dimos_header(const std_msgs::msg::Header& source,
                                 const std::string& frame_id) {
    static std::atomic<int32_t> sequence{0};
    std_msgs::Header result;
    result.seq = sequence.fetch_add(1, std::memory_order_relaxed);
    result.stamp.sec = source.stamp.sec;
    result.stamp.nsec = static_cast<int32_t>(source.stamp.nanosec);
    result.frame_id = frame_id;
    return result;
}

}  // namespace

struct M20ROSBridgeConfig {
    std::string lidar_topic;
    std::string odom_topic;
    std::string nav_cmd_topic;
    std::string location_status_topic;
    std::string hes_status_topic;
    std::string node_name;
    std::string cloud_frame;
    std::string world_frame;
    std::string base_frame;
    bool enable_command_output;
    double command_rate_hz;
    double command_timeout_s;
    double safety_timeout_s;
    double max_linear_x;
    double max_linear_y;
    double max_angular_z;

    void validate() const {
        require_nonempty(lidar_topic, "lidar_topic");
        require_nonempty(odom_topic, "odom_topic");
        require_nonempty(nav_cmd_topic, "nav_cmd_topic");
        require_nonempty(location_status_topic, "location_status_topic");
        require_nonempty(hes_status_topic, "hes_status_topic");
        require_nonempty(node_name, "node_name");
        require_nonempty(cloud_frame, "cloud_frame");
        require_nonempty(world_frame, "world_frame");
        require_nonempty(base_frame, "base_frame");
        dimos::native::require_positive(command_rate_hz, "command_rate_hz");
        dimos::native::require_positive(command_timeout_s, "command_timeout_s");
        dimos::native::require_positive(safety_timeout_s, "safety_timeout_s");
        dimos::native::require_positive(max_linear_x, "max_linear_x");
        dimos::native::require_positive(max_linear_y, "max_linear_y");
        dimos::native::require_positive(max_angular_z, "max_angular_z");
    }
};

class M20ROSBridge : public Module {
public:
    void build(Builder& builder, Config& config) override {
        cfg_ = config.parse<M20ROSBridgeConfig>();
        builder.input<geometry_msgs::Twist>("safe_cmd_vel", &M20ROSBridge::on_command, this);
        command_ready_ = builder.output<std_msgs::Bool>("command_ready");
        lidar_ = builder.output<sensor_msgs::PointCloud2>("lidar");
        odom_ = builder.output<geometry_msgs::PoseStamped>("odom");
        odometry_ = builder.output<nav_msgs::Odometry>("odometry");
        tf_ = builder.output<tf2_msgs::TFMessage>("tf");
    }

    void setup() override {
        rclcpp::init(0, nullptr);
        // rclcpp installs process signal handlers during init. Restore the
        // NativeModule handlers so coordinator SIGTERM exits Module::handle();
        // teardown below then cancels the executor and shuts rclcpp down.
        dimos::native::install_signal_handlers();
        node_ = std::make_shared<rclcpp::Node>(cfg_.node_name);

        const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(2);
        lidar_subscription_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            cfg_.lidar_topic, sensor_qos,
            [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { on_lidar(*msg); });
        odom_subscription_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            cfg_.odom_topic, sensor_qos,
            [this](nav_msgs::msg::Odometry::SharedPtr msg) { on_odometry(*msg); });
        location_subscription_ = node_->create_subscription<drdds::msg::LocationStatus>(
            cfg_.location_status_topic, sensor_qos,
            [this](drdds::msg::LocationStatus::SharedPtr msg) { on_location_status(*msg); });
        hes_subscription_ = node_->create_subscription<drdds::msg::StdMsgInt32>(
            cfg_.hes_status_topic, sensor_qos,
            [this](drdds::msg::StdMsgInt32::SharedPtr msg) { on_hes_status(*msg); });

        if (cfg_.enable_command_output) {
            nav_cmd_publisher_ = node_->create_publisher<drdds::msg::NavCmd>(
                cfg_.nav_cmd_topic, rclcpp::QoS(rclcpp::KeepLast(2)).reliable());
        }

        const auto period = std::chrono::duration<double>(1.0 / cfg_.command_rate_hz);
        timer_ = node_->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            [this]() { publish_cycle(false); });

        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(node_);
        spin_thread_ = std::thread([this]() { executor_->spin(); });

        logging::info(
            "M20 ROS bridge started",
            {logging::Field("lidar_topic", cfg_.lidar_topic),
             logging::Field("odom_topic", cfg_.odom_topic),
             logging::Field("command_output", cfg_.enable_command_output)});
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
        hes_subscription_.reset();
        location_subscription_.reset();
        odom_subscription_.reset();
        lidar_subscription_.reset();
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
    void on_lidar(const sensor_msgs::msg::PointCloud2& source) {
        try {
            sensor_msgs::PointCloud2 result;
            result.header = to_dimos_header(source.header, cfg_.cloud_frame);
            result.height = checked_i32(source.height, "point-cloud height");
            result.width = checked_i32(source.width, "point-cloud width");
            result.fields_length = checked_i32(source.fields.size(), "point-cloud field count");
            result.fields.reserve(source.fields.size());
            for (const auto& source_field : source.fields) {
                sensor_msgs::PointField field;
                field.name = source_field.name;
                field.offset = checked_i32(source_field.offset, "point-field offset");
                field.datatype = source_field.datatype;
                field.count = checked_i32(source_field.count, "point-field count");
                result.fields.push_back(std::move(field));
            }
            result.is_bigendian = static_cast<int8_t>(source.is_bigendian);
            result.point_step = checked_i32(source.point_step, "point-cloud point step");
            result.row_step = checked_i32(source.row_step, "point-cloud row step");
            result.data_length = checked_i32(source.data.size(), "point-cloud byte count");
            result.data = source.data;
            result.is_dense = static_cast<int8_t>(source.is_dense);
            lidar_.publish(result);
        } catch (const std::exception& error) {
            logging::error("dropping invalid M20 point cloud",
                           {logging::Field("error", std::string(error.what()))});
        }
    }

    void on_odometry(const nav_msgs::msg::Odometry& source) {
        nav_msgs::Odometry result;
        result.header = to_dimos_header(source.header, cfg_.world_frame);
        result.child_frame_id = cfg_.base_frame;

        result.pose.pose.position.x = source.pose.pose.position.x;
        result.pose.pose.position.y = source.pose.pose.position.y;
        result.pose.pose.position.z = source.pose.pose.position.z;
        result.pose.pose.orientation.x = source.pose.pose.orientation.x;
        result.pose.pose.orientation.y = source.pose.pose.orientation.y;
        result.pose.pose.orientation.z = source.pose.pose.orientation.z;
        result.pose.pose.orientation.w = source.pose.pose.orientation.w;
        result.twist.twist.linear.x = source.twist.twist.linear.x;
        result.twist.twist.linear.y = source.twist.twist.linear.y;
        result.twist.twist.linear.z = source.twist.twist.linear.z;
        result.twist.twist.angular.x = source.twist.twist.angular.x;
        result.twist.twist.angular.y = source.twist.twist.angular.y;
        result.twist.twist.angular.z = source.twist.twist.angular.z;
        for (std::size_t i = 0; i < source.pose.covariance.size(); ++i) {
            result.pose.covariance[i] = source.pose.covariance[i];
            result.twist.covariance[i] = source.twist.covariance[i];
        }

        geometry_msgs::PoseStamped pose;
        pose.header = result.header;
        pose.pose = result.pose.pose;

        geometry_msgs::TransformStamped transform;
        transform.header = result.header;
        transform.child_frame_id = cfg_.base_frame;
        transform.transform.translation.x = source.pose.pose.position.x;
        transform.transform.translation.y = source.pose.pose.position.y;
        transform.transform.translation.z = source.pose.pose.position.z;
        transform.transform.rotation = result.pose.pose.orientation;
        tf2_msgs::TFMessage transforms;
        transforms.transforms_length = 1;
        transforms.transforms.push_back(std::move(transform));

        odometry_.publish(result);
        odom_.publish(pose);
        tf_.publish(transforms);
    }

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

    void on_location_status(const drdds::msg::LocationStatus& source) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        location_status_ = static_cast<int>(source.data.total_status);
        location_received_at_ = Clock::now();
        have_location_ = true;
    }

    void on_hes_status(const drdds::msg::StdMsgInt32& source) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        hes_status_ = static_cast<int>(source.data);
        hes_received_at_ = Clock::now();
        have_hes_ = true;
    }

    bool safety_ready(Clock::time_point now) const {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!cfg_.enable_command_output || !have_location_ || !have_hes_) {
            return false;
        }
        const auto timeout = std::chrono::duration<double>(cfg_.safety_timeout_s);
        return now - location_received_at_ <= timeout && now - hes_received_at_ <= timeout &&
               location_status_ == 1 && hes_status_ == 0;
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
    Output<sensor_msgs::PointCloud2> lidar_;
    Output<geometry_msgs::PoseStamped> odom_;
    Output<nav_msgs::Odometry> odometry_;
    Output<tf2_msgs::TFMessage> tf_;

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<drdds::msg::LocationStatus>::SharedPtr location_subscription_;
    rclcpp::Subscription<drdds::msg::StdMsgInt32>::SharedPtr hes_subscription_;
    rclcpp::Publisher<drdds::msg::NavCmd>::SharedPtr nav_cmd_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::thread spin_thread_;

    mutable std::mutex state_mutex_;
    geometry_msgs::Twist latest_command_ = zero_twist();
    Clock::time_point command_received_at_{};
    Clock::time_point location_received_at_{};
    Clock::time_point hes_received_at_{};
    bool have_command_ = false;
    bool have_location_ = false;
    bool have_hes_ = false;
    int location_status_ = 0;
    int hes_status_ = 1;
    std::atomic<bool> stopping_{false};
    std::atomic<uint64_t> command_sequence_{0};
};

int main() {
    dimos::native::run_with_transport<M20ROSBridge>();
    return 0;
}
