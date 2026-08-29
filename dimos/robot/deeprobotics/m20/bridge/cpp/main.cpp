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
#include <cstring>
#include <limits>
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
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "dimos/native.hpp"

#include "geometry_msgs/Twist.hpp"
#include "sensor_msgs/Imu.hpp"
#include "sensor_msgs/PointCloud2.hpp"
#include "sensor_msgs/PointField.hpp"
#include "std_msgs/Bool.hpp"
#include "std_msgs/Header.hpp"
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

int32_t checked_i32(std::size_t value, const char* name) {
    if (value > static_cast<std::size_t>(std::numeric_limits<int32_t>::max())) {
        throw std::runtime_error(std::string(name) + " exceeds the DimOS message limit");
    }
    return static_cast<int32_t>(value);
}

std::size_t checked_product(std::size_t left, std::size_t right, const char* name) {
    if (left != 0 && right > std::numeric_limits<std::size_t>::max() / left) {
        throw std::runtime_error(std::string(name) + " overflows size_t");
    }
    return left * right;
}

struct XYZOffsets {
    std::size_t x;
    std::size_t y;
    std::size_t z;
};

XYZOffsets validate_cloud_for_mapping(const sensor_msgs::msg::PointCloud2& cloud) {
    if (cloud.width == 0 || cloud.height == 0) {
        throw std::runtime_error("point cloud is empty");
    }
    if (cloud.is_bigendian) {
        throw std::runtime_error("big-endian point clouds are not supported by the mapper");
    }
    if (cloud.point_step == 0) {
        throw std::runtime_error("point-cloud point_step is zero");
    }
    if (cloud.point_step < sizeof(float)) {
        throw std::runtime_error("point-cloud point_step is shorter than float32");
    }

    const auto row_bytes = checked_product(static_cast<std::size_t>(cloud.width),
                                           static_cast<std::size_t>(cloud.point_step),
                                           "point-cloud row size");
    if (cloud.row_step != row_bytes) {
        throw std::runtime_error("point cloud contains unsupported row padding");
    }
    const auto required_bytes = checked_product(row_bytes, static_cast<std::size_t>(cloud.height),
                                                "point-cloud byte count");
    if (cloud.data.size() < required_bytes) {
        throw std::runtime_error("point-cloud data is shorter than its dimensions");
    }

    XYZOffsets offsets{std::numeric_limits<std::size_t>::max(),
                       std::numeric_limits<std::size_t>::max(),
                       std::numeric_limits<std::size_t>::max()};
    for (const auto& field : cloud.fields) {
        if (field.datatype != sensor_msgs::msg::PointField::FLOAT32 || field.count == 0) {
            continue;
        }
        const auto offset = static_cast<std::size_t>(field.offset);
        if (offset > static_cast<std::size_t>(cloud.point_step) - sizeof(float)) {
            continue;
        }
        if (field.name == "x") {
            offsets.x = offset;
        } else if (field.name == "y") {
            offsets.y = offset;
        } else if (field.name == "z") {
            offsets.z = offset;
        }
    }
    const auto missing = std::numeric_limits<std::size_t>::max();
    if (offsets.x == missing || offsets.y == missing || offsets.z == missing) {
        throw std::runtime_error("point cloud lacks mapper-compatible float32 x/y/z fields");
    }
    return offsets;
}

bool has_finite_xyz(const sensor_msgs::msg::PointCloud2& cloud, const XYZOffsets& offsets) {
    const auto point_count = checked_product(static_cast<std::size_t>(cloud.width),
                                             static_cast<std::size_t>(cloud.height),
                                             "point-cloud point count");
    const auto point_step = static_cast<std::size_t>(cloud.point_step);
    for (std::size_t index = 0; index < point_count; ++index) {
        const auto base = index * point_step;
        float x = 0.0F;
        float y = 0.0F;
        float z = 0.0F;
        std::memcpy(&x, cloud.data.data() + base + offsets.x, sizeof(float));
        std::memcpy(&y, cloud.data.data() + base + offsets.y, sizeof(float));
        std::memcpy(&z, cloud.data.data() + base + offsets.z, sizeof(float));
        if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
            return true;
        }
    }
    return false;
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
    std::string imu_topic;
    std::string nav_cmd_topic;
    std::string motion_state_topic;
    std::string motion_info_topic;
    std::string gait_topic;
    std::string hes_status_topic;
    std::string node_name;
    std::string cloud_frame;
    std::string base_frame;
    bool enable_command_output;
    double command_rate_hz;
    double command_timeout_s;
    double safety_timeout_s;
    double lidar_timeout_s;
    double max_linear_x;
    double max_linear_y;
    double max_angular_z;

    void validate() const {
        require_nonempty(lidar_topic, "lidar_topic");
        require_nonempty(imu_topic, "imu_topic");
        require_nonempty(nav_cmd_topic, "nav_cmd_topic");
        require_nonempty(motion_state_topic, "motion_state_topic");
        require_nonempty(motion_info_topic, "motion_info_topic");
        require_nonempty(gait_topic, "gait_topic");
        require_nonempty(hes_status_topic, "hes_status_topic");
        require_nonempty(node_name, "node_name");
        require_nonempty(cloud_frame, "cloud_frame");
        require_nonempty(base_frame, "base_frame");
        dimos::native::require_positive(command_rate_hz, "command_rate_hz");
        dimos::native::require_positive(command_timeout_s, "command_timeout_s");
        dimos::native::require_positive(safety_timeout_s, "safety_timeout_s");
        dimos::native::require_positive(lidar_timeout_s, "lidar_timeout_s");
        dimos::native::require_positive(max_linear_x, "max_linear_x");
        dimos::native::require_positive(max_linear_y, "max_linear_y");
        dimos::native::require_positive(max_angular_z, "max_angular_z");
    }
};

M20ROSBridgeConfig parse_m20_config(Config& config) {
    M20ROSBridgeConfig result{};
    result.lidar_topic = config.take<std::string>("lidar_topic");
    result.imu_topic = config.take<std::string>("imu_topic");
    result.nav_cmd_topic = config.take<std::string>("nav_cmd_topic");
    result.motion_state_topic = config.take<std::string>("motion_state_topic");
    result.motion_info_topic = config.take<std::string>("motion_info_topic");
    result.gait_topic = config.take<std::string>("gait_topic");
    result.hes_status_topic = config.take<std::string>("hes_status_topic");
    result.node_name = config.take<std::string>("node_name");
    result.cloud_frame = config.take<std::string>("cloud_frame");
    result.base_frame = config.take<std::string>("base_frame");
    result.enable_command_output = config.take<bool>("enable_command_output");
    result.command_rate_hz = config.take<double>("command_rate_hz");
    result.command_timeout_s = config.take<double>("command_timeout_s");
    result.safety_timeout_s = config.take<double>("safety_timeout_s");
    result.lidar_timeout_s = config.take<double>("lidar_timeout_s");
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
        builder.input<std_msgs::Bool>("localization_ready",
                                      &M20ROSBridge::on_localization_ready, this);
        builder.input<std_msgs::Int32>("motion_state_cmd",
                                      &M20ROSBridge::on_motion_state_command, this);
        builder.input<std_msgs::UInt32>("gait_cmd", &M20ROSBridge::on_gait_command, this);
        command_ready_ = builder.output<std_msgs::Bool>("command_ready");
        lidar_ready_ = builder.output<std_msgs::Bool>("lidar_ready");
        motion_state_ = builder.output<std_msgs::Int32>("motion_state");
        gait_state_ = builder.output<std_msgs::UInt32>("gait_state");
        raw_lidar_ = builder.output<sensor_msgs::PointCloud2>("raw_lidar");
        imu_ = builder.output<sensor_msgs::Imu>("imu");
    }

    void setup() override {
        rclcpp::init(0, nullptr);
        // rclcpp installs process signal handlers during init. Restore the
        // NativeModule handlers so coordinator SIGTERM exits Module::handle();
        // teardown below then cancels the executor and shuts rclcpp down.
        dimos::native::install_signal_handlers();
        node_ = std::make_shared<rclcpp::Node>(cfg_.node_name);

        // Both inspected bare-DDS M20 publishers offer RELIABLE/VOLATILE. Pin
        // the cloud subscription to that verified contract so DDS detects a
        // future incompatible vendor QoS change instead of silently dropping.
        const auto lidar_qos =
            rclcpp::QoS(rclcpp::KeepLast(2)).reliable().durability_volatile();
        // The advertised M20 endpoint is RELIABLE/TRANSIENT_LOCAL. Some M20
        // firmware revisions expose the endpoint without actually emitting
        // its documented 1 Hz samples, so HES is a veto when observed rather
        // than the command-path heartbeat. The physical stop remains enforced
        // below this API by the robot controller.
        const auto hes_qos =
            rclcpp::QoS(rclcpp::KeepLast(2)).reliable().transient_local();
        lidar_subscription_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            cfg_.lidar_topic, lidar_qos,
            [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { on_lidar(*msg); });
        imu_subscription_ = node_->create_subscription<sensor_msgs::msg::Imu>(
            cfg_.imu_topic,
            rclcpp::QoS(rclcpp::KeepLast(20)).reliable().durability_volatile(),
            [this](sensor_msgs::msg::Imu::SharedPtr msg) { on_imu(*msg); });
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

        logging::info(
            "M20 ROS bridge started",
            {logging::Field("lidar_topic", cfg_.lidar_topic),
             logging::Field("imu_topic", cfg_.imu_topic),
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
        motion_state_publisher_.reset();
        gait_publisher_.reset();
        hes_subscription_.reset();
        motion_info_subscription_.reset();
        imu_subscription_.reset();
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
            const XYZOffsets offsets = validate_cloud_for_mapping(source);
            if (!has_finite_xyz(source, offsets)) {
                throw std::runtime_error("point cloud contains no finite XYZ return");
            }
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
            raw_lidar_.publish(result);
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                lidar_received_at_ = Clock::now();
                last_lidar_width_ = source.width;
                have_lidar_ = true;
            }
        } catch (const std::exception& error) {
            logging::error("dropping invalid M20 point cloud",
                           {logging::Field("error", std::string(error.what()))});
        }
    }

    void on_imu(const sensor_msgs::msg::Imu& source) {
        sensor_msgs::Imu result;
        result.header = to_dimos_header(source.header, cfg_.base_frame);
        result.orientation.x = source.orientation.x;
        result.orientation.y = source.orientation.y;
        result.orientation.z = source.orientation.z;
        result.orientation.w = source.orientation.w;
        result.angular_velocity.x = source.angular_velocity.x;
        result.angular_velocity.y = source.angular_velocity.y;
        result.angular_velocity.z = source.angular_velocity.z;
        result.linear_acceleration.x = source.linear_acceleration.x;
        result.linear_acceleration.y = source.linear_acceleration.y;
        result.linear_acceleration.z = source.linear_acceleration.z;
        for (std::size_t index = 0; index < 9; ++index) {
            result.orientation_covariance[index] = source.orientation_covariance[index];
            result.angular_velocity_covariance[index] =
                source.angular_velocity_covariance[index];
            result.linear_acceleration_covariance[index] =
                source.linear_acceleration_covariance[index];
        }
        imu_.publish(result);
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

    void on_localization_ready(const std_msgs::Bool& source) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        localization_ready_state_ = source.data != 0;
        localization_received_at_ = Clock::now();
        have_localization_ = true;
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

    bool lidar_fresh(Clock::time_point now) const {
        std::lock_guard<std::mutex> lock(state_mutex_);
        const auto timeout = std::chrono::duration<double>(cfg_.lidar_timeout_s);
        return have_lidar_ && now - lidar_received_at_ <= timeout;
    }

    std::pair<double, uint32_t> lidar_diagnostics(Clock::time_point now) const {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!have_lidar_) {
            return {-1.0, 0};
        }
        return {std::chrono::duration<double>(now - lidar_received_at_).count(),
                last_lidar_width_};
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
        const bool cloud_ready = !force_zero && !stopping_.load(std::memory_order_acquire) &&
                                 lidar_fresh(now);
        std_msgs::Bool lidar_ready_message;
        lidar_ready_message.data = static_cast<int8_t>(cloud_ready);
        lidar_ready_.publish(lidar_ready_message);

        const int8_t new_lidar_state = cloud_ready ? 1 : 0;
        const int8_t previous_lidar_state =
            lidar_health_state_.exchange(new_lidar_state, std::memory_order_acq_rel);
        if (previous_lidar_state != new_lidar_state) {
            const auto [age_s, width] = lidar_diagnostics(now);
            if (cloud_ready) {
                logging::info("M20 lidar stream is healthy",
                              {logging::Field("cloud_age_s", age_s),
                               logging::Field("cloud_width", static_cast<int64_t>(width))});
            } else {
                logging::warn("M20 lidar stream is missing or stale",
                              {logging::Field("cloud_age_s", age_s),
                               logging::Field("timeout_s", cfg_.lidar_timeout_s)});
            }
        }

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
    Output<std_msgs::Bool> lidar_ready_;
    Output<std_msgs::Int32> motion_state_;
    Output<std_msgs::UInt32> gait_state_;
    Output<sensor_msgs::PointCloud2> raw_lidar_;
    Output<sensor_msgs::Imu> imu_;

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
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
    Clock::time_point localization_received_at_{};
    Clock::time_point motion_info_received_at_{};
    Clock::time_point lidar_received_at_{};
    bool have_command_ = false;
    bool have_localization_ = false;
    bool have_hes_ = false;
    bool have_motion_info_ = false;
    bool have_lidar_ = false;
    bool localization_ready_state_ = false;
    uint32_t last_lidar_width_ = 0;
    int motion_state_value_ = 0;
    int hes_status_ = 1;
    std::atomic<bool> stopping_{false};
    std::atomic<int8_t> lidar_health_state_{-1};
    std::atomic<uint64_t> command_sequence_{0};
};

int main() {
    dimos::native::run_with_transport<M20ROSBridge>();
    return 0;
}
