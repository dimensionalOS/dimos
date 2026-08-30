// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// M20 Point-LIO adapter. This process subscribes directly to the robot's public
// merged RoboSense PointCloud2 and base-aligned IMU over ROS 2, converts them
// into the pinned Point-LIO core, and owns odom -> base_link.

#include <algorithm>
#include <atomic>
#include <boost/make_shared.hpp>
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
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "dimos/native.hpp"

#include "geometry_msgs/PoseStamped.hpp"
#include "geometry_msgs/TransformStamped.hpp"
#include "nav_msgs/Odometry.hpp"
#include "sensor_msgs/Imu.hpp"
#include "sensor_msgs/PointCloud2.hpp"
#include "sensor_msgs/PointField.hpp"
#include "std_msgs/Bool.hpp"
#include "tf2_msgs/TFMessage.hpp"

#include "estimator_pose.hpp"
#include "point_cloud_utils.hpp"

// Existing DimOS Point-LIO core, pinned by CMake.
#include "pointlio.hpp"
#include "pointlio_debug.hpp"

using dimos::native::Builder;
using dimos::native::Config;
using dimos::native::Module;
using dimos::native::Output;
namespace logging = dimos::native::log;

namespace {

using Clock = std::chrono::steady_clock;
constexpr double kStandardGravityMps2 = 9.80665;
constexpr std::size_t kPointLioStaticPointLimit = 100'000;
constexpr std::size_t kM20RawPointLimit = 500'000;
constexpr std::size_t kMaxInitializationLidarFrames = 20;

void require_nonempty(const std::string& value, const char* name) {
    if (value.empty()) {
        throw std::runtime_error(std::string(name) + " must not be empty");
    }
}

void require_vector_size(const std::vector<double>& value, std::size_t expected,
                         const char* name) {
    if (value.size() != expected) {
        throw std::runtime_error(std::string(name) + " must contain " +
                                 std::to_string(expected) + " values");
    }
}

int ivox_nearby_code(const std::string& name) {
    if (name == "center") return 0;
    if (name == "nearby6") return 6;
    if (name == "nearby18") return 18;
    if (name == "nearby26") return 26;
    throw std::runtime_error(
        "ivox_nearby_type must be one of: center nearby6 nearby18 nearby26, got '" +
        name + "'");
}

double header_seconds(const std_msgs::msg::Header& header) {
    return static_cast<double>(header.stamp.sec) +
           static_cast<double>(header.stamp.nanosec) / 1e9;
}

template <typename T>
T read_unaligned(const std::vector<uint8_t>& data, std::size_t offset) {
    T result{};
    std::memcpy(&result, data.data() + offset, sizeof(T));
    return result;
}

struct M20FieldOffsets {
    std::size_t x;
    std::size_t y;
    std::size_t z;
    std::size_t intensity;
    std::size_t ring;
    std::size_t timestamp;
};

M20FieldOffsets validate_m20_cloud(const sensor_msgs::msg::PointCloud2& cloud) {
    if (cloud.height == 0 || cloud.width == 0) {
        throw std::runtime_error("M20 point cloud is empty");
    }
    if (cloud.is_bigendian) {
        throw std::runtime_error("M20 Point-LIO requires a little-endian cloud");
    }
    if (cloud.point_step == 0 || cloud.row_step == 0) {
        throw std::runtime_error("M20 point cloud has an invalid stride");
    }
    const auto point_count = static_cast<std::size_t>(cloud.width) *
                             static_cast<std::size_t>(cloud.height);
    if (point_count > kM20RawPointLimit) {
        throw std::runtime_error("M20 point cloud exceeds the raw input sanity limit");
    }
    const auto required_bytes = point_count * static_cast<std::size_t>(cloud.point_step);
    if (required_bytes > cloud.data.size()) {
        throw std::runtime_error("M20 point cloud data is shorter than its dimensions");
    }

    constexpr auto missing = std::numeric_limits<std::size_t>::max();
    M20FieldOffsets offsets{missing, missing, missing, missing, missing, missing};
    for (const auto& field : cloud.fields) {
        const auto offset = static_cast<std::size_t>(field.offset);
        if (field.count == 0 || offset >= static_cast<std::size_t>(cloud.point_step)) {
            continue;
        }
        if (field.name == "x" &&
            field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
            offsets.x = offset;
        } else if (field.name == "y" &&
                   field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
            offsets.y = offset;
        } else if (field.name == "z" &&
                   field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
            offsets.z = offset;
        } else if (field.name == "intensity" &&
                   field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
            offsets.intensity = offset;
        } else if (field.name == "ring" &&
                   field.datatype == sensor_msgs::msg::PointField::UINT16) {
            offsets.ring = offset;
        } else if (field.name == "timestamp" &&
                   field.datatype == sensor_msgs::msg::PointField::FLOAT64) {
            offsets.timestamp = offset;
        }
    }
    if (offsets.x == missing || offsets.y == missing || offsets.z == missing ||
        offsets.intensity == missing || offsets.ring == missing ||
        offsets.timestamp == missing) {
        throw std::runtime_error(
            "M20 Point-LIO requires float32 x/y/z/intensity, uint16 ring, and "
            "float64 timestamp fields");
    }
    if (offsets.timestamp + sizeof(double) > static_cast<std::size_t>(cloud.point_step) ||
        offsets.ring + sizeof(uint16_t) > static_cast<std::size_t>(cloud.point_step) ||
        offsets.intensity + sizeof(float) > static_cast<std::size_t>(cloud.point_step)) {
        throw std::runtime_error("M20 point fields extend past point_step");
    }
    return offsets;
}

struct TimedPoint {
    double timestamp;
    custom_messages::CustomPoint point;
};

}  // namespace

struct M20PointLioConfig {
    std::string lidar_topic;
    std::string imu_topic;
    std::string node_name;
    std::string world_frame;
    std::string base_frame;
    double processing_rate_hz;
    double pointcloud_rate_hz;
    double odometry_rate_hz;
    double readiness_rate_hz;
    double lidar_timeout_s;
    double imu_timeout_s;
    double estimate_timeout_s;
    double max_scan_duration_s;
    int max_cloud_points;
    double msr_freq;
    double main_freq;
    bool con_frame;
    int con_frame_num;
    bool cut_frame;
    double cut_frame_time_interval;
    double time_lag_imu_to_lidar;
    int scan_line;
    int scan_rate;
    double blind;
    int point_filter_num;
    bool use_imu_as_input;
    bool prop_at_freq_of_imu;
    bool check_satu;
    int init_map_size;
    bool space_down_sample;
    double satu_acc;
    double satu_gyro;
    double acc_norm;
    double plane_thr;
    double filter_size_surf;
    double filter_size_map;
    double ivox_grid_resolution;
    std::string ivox_nearby_type;
    double cube_side_length;
    double det_range;
    double fov_degree;
    bool imu_en;
    bool start_in_aggressive_motion;
    bool extrinsic_est_en;
    double imu_time_inte;
    double lidar_meas_cov;
    double acc_cov_input;
    double vel_cov;
    double gyr_cov_input;
    double gyr_cov_output;
    double acc_cov_output;
    double b_gyr_cov;
    double b_acc_cov;
    double imu_meas_acc_cov;
    double imu_meas_omg_cov;
    double match_s;
    bool gravity_align;
    std::vector<double> gravity;
    std::vector<double> gravity_init;
    std::vector<double> extrinsic_t;
    std::vector<double> extrinsic_r;
    bool publish_odometry_without_downsample;
    bool odom_only;
    bool debug;

    void validate() const {
        require_nonempty(lidar_topic, "lidar_topic");
        require_nonempty(imu_topic, "imu_topic");
        require_nonempty(node_name, "node_name");
        require_nonempty(world_frame, "world_frame");
        require_nonempty(base_frame, "base_frame");
        dimos::native::require_positive(processing_rate_hz, "processing_rate_hz");
        dimos::native::require_positive(pointcloud_rate_hz, "pointcloud_rate_hz");
        dimos::native::require_positive(odometry_rate_hz, "odometry_rate_hz");
        dimos::native::require_positive(readiness_rate_hz, "readiness_rate_hz");
        dimos::native::require_positive(lidar_timeout_s, "lidar_timeout_s");
        dimos::native::require_positive(imu_timeout_s, "imu_timeout_s");
        dimos::native::require_positive(estimate_timeout_s, "estimate_timeout_s");
        dimos::native::require_positive(max_scan_duration_s, "max_scan_duration_s");
        dimos::native::require_positive(msr_freq, "msr_freq");
        dimos::native::require_positive(main_freq, "main_freq");
        if (max_cloud_points <= 0 ||
            max_cloud_points > static_cast<int>(kPointLioStaticPointLimit)) {
            throw std::runtime_error("max_cloud_points must be in [1, 100000]");
        }
        if (scan_line <= 0 || scan_line > std::numeric_limits<uint16_t>::max()) {
            throw std::runtime_error("scan_line must be in [1, 65535]");
        }
        if (point_filter_num <= 0 || init_map_size <= 0 || con_frame_num <= 0) {
            throw std::runtime_error(
                "point_filter_num, init_map_size, and con_frame_num must be positive");
        }
        require_vector_size(gravity, 3, "gravity");
        require_vector_size(gravity_init, 3, "gravity_init");
        require_vector_size(extrinsic_t, 3, "extrinsic_t");
        require_vector_size(extrinsic_r, 9, "extrinsic_r");
        (void)ivox_nearby_code(ivox_nearby_type);
    }
};

// GOS ships GCC 9. Its C++20 implementation is sufficient for the native SDK
// and Point-LIO, but not for Boost.PFR's compile-time field-name extraction.
// Parse keys explicitly so the source builds with the robot's stock toolchain
// while retaining strict unknown/missing-key validation.
M20PointLioConfig parse_m20_pointlio_config(Config& config) {
    M20PointLioConfig result{};
    result.lidar_topic = config.take<std::string>("lidar_topic");
    result.imu_topic = config.take<std::string>("imu_topic");
    result.node_name = config.take<std::string>("node_name");
    result.world_frame = config.take<std::string>("world_frame");
    result.base_frame = config.take<std::string>("base_frame");
    result.processing_rate_hz = config.take<double>("processing_rate_hz");
    result.pointcloud_rate_hz = config.take<double>("pointcloud_rate_hz");
    result.odometry_rate_hz = config.take<double>("odometry_rate_hz");
    result.readiness_rate_hz = config.take<double>("readiness_rate_hz");
    result.lidar_timeout_s = config.take<double>("lidar_timeout_s");
    result.imu_timeout_s = config.take<double>("imu_timeout_s");
    result.estimate_timeout_s = config.take<double>("estimate_timeout_s");
    result.max_scan_duration_s = config.take<double>("max_scan_duration_s");
    result.max_cloud_points = config.take<int>("max_cloud_points");
    result.msr_freq = config.take<double>("msr_freq");
    result.main_freq = config.take<double>("main_freq");
    result.con_frame = config.take<bool>("con_frame");
    result.con_frame_num = config.take<int>("con_frame_num");
    result.cut_frame = config.take<bool>("cut_frame");
    result.cut_frame_time_interval = config.take<double>("cut_frame_time_interval");
    result.time_lag_imu_to_lidar = config.take<double>("time_lag_imu_to_lidar");
    result.scan_line = config.take<int>("scan_line");
    result.scan_rate = config.take<int>("scan_rate");
    result.blind = config.take<double>("blind");
    result.point_filter_num = config.take<int>("point_filter_num");
    result.use_imu_as_input = config.take<bool>("use_imu_as_input");
    result.prop_at_freq_of_imu = config.take<bool>("prop_at_freq_of_imu");
    result.check_satu = config.take<bool>("check_satu");
    result.init_map_size = config.take<int>("init_map_size");
    result.space_down_sample = config.take<bool>("space_down_sample");
    result.satu_acc = config.take<double>("satu_acc");
    result.satu_gyro = config.take<double>("satu_gyro");
    result.acc_norm = config.take<double>("acc_norm");
    result.plane_thr = config.take<double>("plane_thr");
    result.filter_size_surf = config.take<double>("filter_size_surf");
    result.filter_size_map = config.take<double>("filter_size_map");
    result.ivox_grid_resolution = config.take<double>("ivox_grid_resolution");
    result.ivox_nearby_type = config.take<std::string>("ivox_nearby_type");
    result.cube_side_length = config.take<double>("cube_side_length");
    result.det_range = config.take<double>("det_range");
    result.fov_degree = config.take<double>("fov_degree");
    result.imu_en = config.take<bool>("imu_en");
    result.start_in_aggressive_motion = config.take<bool>("start_in_aggressive_motion");
    result.extrinsic_est_en = config.take<bool>("extrinsic_est_en");
    result.imu_time_inte = config.take<double>("imu_time_inte");
    result.lidar_meas_cov = config.take<double>("lidar_meas_cov");
    result.acc_cov_input = config.take<double>("acc_cov_input");
    result.vel_cov = config.take<double>("vel_cov");
    result.gyr_cov_input = config.take<double>("gyr_cov_input");
    result.gyr_cov_output = config.take<double>("gyr_cov_output");
    result.acc_cov_output = config.take<double>("acc_cov_output");
    result.b_gyr_cov = config.take<double>("b_gyr_cov");
    result.b_acc_cov = config.take<double>("b_acc_cov");
    result.imu_meas_acc_cov = config.take<double>("imu_meas_acc_cov");
    result.imu_meas_omg_cov = config.take<double>("imu_meas_omg_cov");
    result.match_s = config.take<double>("match_s");
    result.gravity_align = config.take<bool>("gravity_align");
    result.gravity = config.take<std::vector<double>>("gravity");
    result.gravity_init = config.take<std::vector<double>>("gravity_init");
    result.extrinsic_t = config.take<std::vector<double>>("extrinsic_t");
    result.extrinsic_r = config.take<std::vector<double>>("extrinsic_r");
    result.publish_odometry_without_downsample =
        config.take<bool>("publish_odometry_without_downsample");
    result.odom_only = config.take<bool>("odom_only");
    result.debug = config.take<bool>("debug");
    config.enforce_all_consumed();
    result.validate();
    return result;
}

class M20PointLio : public Module {
public:
    void build(Builder& builder, Config& config) override {
        cfg_ = parse_m20_pointlio_config(config);

        lidar_ready_ = builder.output<std_msgs::Bool>("lidar_ready");
        localization_ready_ = builder.output<std_msgs::Bool>("localization_ready");
        lidar_ = builder.output<sensor_msgs::PointCloud2>("lidar");
        odom_ = builder.output<geometry_msgs::PoseStamped>("odom");
        odometry_ = builder.output<nav_msgs::Odometry>("odometry");
        tf_ = builder.output<tf2_msgs::TFMessage>("tf");

        process_period_ = std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(1.0 / cfg_.processing_rate_hz));
        pointcloud_period_ = std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(1.0 / cfg_.pointcloud_rate_hz));
        odometry_period_ = std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(1.0 / cfg_.odometry_rate_hz));
        readiness_period_ = std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(1.0 / cfg_.readiness_rate_hz));
    }

    void setup() override {
        pointlio_debug = cfg_.debug;

        PointLioParams params;
        params.odom_header_frame_id = cfg_.world_frame;
        params.odom_child_frame_id = cfg_.base_frame;
        params.con_frame = cfg_.con_frame;
        params.con_frame_num = cfg_.con_frame_num;
        params.cut_frame = cfg_.cut_frame;
        params.cut_frame_time_interval = cfg_.cut_frame_time_interval;
        params.time_lag_imu_to_lidar = cfg_.time_lag_imu_to_lidar;
        // The M20 converter emits the same timestamped CustomMsg shape as the
        // existing Mid-360 adapter, so the core intentionally stays in AVIA mode.
        params.lidar_type = 1;
        params.scan_line = cfg_.scan_line;
        params.scan_rate = cfg_.scan_rate;
        params.timestamp_unit = 3;
        params.blind = cfg_.blind;
        params.point_filter_num = cfg_.point_filter_num;
        params.use_imu_as_input = cfg_.use_imu_as_input;
        params.prop_at_freq_of_imu = cfg_.prop_at_freq_of_imu;
        params.check_satu = cfg_.check_satu;
        params.init_map_size = cfg_.init_map_size;
        params.space_down_sample = cfg_.space_down_sample;
        params.satu_acc = cfg_.satu_acc;
        params.satu_gyro = cfg_.satu_gyro;
        params.acc_norm = cfg_.acc_norm;
        params.plane_thr = static_cast<float>(cfg_.plane_thr);
        params.filter_size_surf = cfg_.filter_size_surf;
        params.filter_size_map = cfg_.filter_size_map;
        params.ivox_grid_resolution = static_cast<float>(cfg_.ivox_grid_resolution);
        params.ivox_nearby_type = ivox_nearby_code(cfg_.ivox_nearby_type);
        params.cube_side_length = cfg_.cube_side_length;
        params.det_range = static_cast<float>(cfg_.det_range);
        params.fov_degree = cfg_.fov_degree;
        params.imu_en = cfg_.imu_en;
        params.start_in_aggressive_motion = cfg_.start_in_aggressive_motion;
        params.extrinsic_est_en = cfg_.extrinsic_est_en;
        params.imu_time_inte = cfg_.imu_time_inte;
        params.lidar_meas_cov = cfg_.lidar_meas_cov;
        params.acc_cov_input = cfg_.acc_cov_input;
        params.vel_cov = cfg_.vel_cov;
        params.gyr_cov_input = cfg_.gyr_cov_input;
        params.gyr_cov_output = cfg_.gyr_cov_output;
        params.acc_cov_output = cfg_.acc_cov_output;
        params.b_gyr_cov = cfg_.b_gyr_cov;
        params.b_acc_cov = cfg_.b_acc_cov;
        params.imu_meas_acc_cov = cfg_.imu_meas_acc_cov;
        params.imu_meas_omg_cov = cfg_.imu_meas_omg_cov;
        params.match_s = cfg_.match_s;
        params.gravity_align = cfg_.gravity_align;
        params.gravity = cfg_.gravity;
        params.gravity_init = cfg_.gravity_init;
        params.extrinsic_T = cfg_.extrinsic_t;
        params.extrinsic_R = cfg_.extrinsic_r;
        params.publish_odometry_without_downsample =
            cfg_.publish_odometry_without_downsample;
        params.odom_only = cfg_.odom_only;

        point_lio_ = std::make_unique<PointLio>(params, cfg_.msr_freq, cfg_.main_freq);

        rclcpp::init(0, nullptr);
        dimos::native::install_signal_handlers();
        node_ = std::make_shared<rclcpp::Node>(cfg_.node_name);
        lidar_callback_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        imu_callback_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions lidar_options;
        lidar_options.callback_group = lidar_callback_group_;
        rclcpp::SubscriptionOptions imu_options;
        imu_options.callback_group = imu_callback_group_;
        const auto lidar_qos =
            rclcpp::QoS(rclcpp::KeepLast(2)).reliable().durability_volatile();
        const auto imu_qos =
            rclcpp::QoS(rclcpp::KeepLast(20)).reliable().durability_volatile();
        lidar_subscription_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            cfg_.lidar_topic, lidar_qos,
            [this](sensor_msgs::msg::PointCloud2::SharedPtr message) {
                on_lidar(*message);
            },
            lidar_options);
        imu_subscription_ = node_->create_subscription<sensor_msgs::msg::Imu>(
            cfg_.imu_topic, imu_qos,
            [this](sensor_msgs::msg::Imu::SharedPtr message) { on_imu(*message); },
            imu_options);
        executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
            rclcpp::ExecutorOptions(), 2);
        executor_->add_node(node_);

        const auto now = Clock::now();
        last_pointcloud_publish_ = now;
        last_odometry_publish_ = now;
        last_readiness_publish_ = now - readiness_period_;
        processing_thread_ = std::thread([this]() { processing_loop(); });
        spin_thread_ = std::thread([this]() { executor_->spin(); });
        logging::info("M20 Point-LIO started",
                      {logging::Field("world_frame", cfg_.world_frame),
                       logging::Field("base_frame", cfg_.base_frame),
                       logging::Field("scan_lines", static_cast<int64_t>(cfg_.scan_line)),
                       logging::Field("lidar_topic", cfg_.lidar_topic),
                       logging::Field("imu_topic", cfg_.imu_topic)});
    }

    void teardown() override {
        stopping_.store(true, std::memory_order_release);
        if (executor_ != nullptr) {
            executor_->cancel();
        }
        if (spin_thread_.joinable()) {
            spin_thread_.join();
        }
        lidar_subscription_.reset();
        imu_subscription_.reset();
        lidar_callback_group_.reset();
        imu_callback_group_.reset();
        executor_.reset();
        node_.reset();
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        if (processing_thread_.joinable()) {
            processing_thread_.join();
        }
        std_msgs::Bool ready;
        ready.data = 0;
        lidar_ready_.publish(ready);
        localization_ready_.publish(ready);
        point_lio_.reset();
    }

private:
    void on_lidar(const sensor_msgs::msg::PointCloud2& source) {
        if (stopping_.load(std::memory_order_acquire) || point_lio_ == nullptr) return;
        const auto received_at = Clock::now();
        bool feed_reserved = false;

        try {
            const auto offsets = validate_m20_cloud(source);
            {
                std::lock_guard<std::mutex> lock(health_mutex_);
                last_lidar_received_at_ = received_at;
                have_lidar_ = true;
            }

            std::unique_lock<std::mutex> callback_lock(lidar_callback_mutex_,
                                                       std::try_to_lock);
            if (!callback_lock.owns_lock() || point_lio_is_processing_lidar()) {
                log_busy_lidar_drop();
                return;
            }

            const auto point_count = static_cast<std::size_t>(source.width) *
                                     static_cast<std::size_t>(source.height);
            const auto point_step = static_cast<std::size_t>(source.point_step);
            const auto point_limit = static_cast<std::size_t>(cfg_.max_cloud_points);
            const auto source_sample_count = std::min(point_count, point_limit);
            std::vector<TimedPoint> points;
            points.reserve(source_sample_count);
            uint16_t min_ring = std::numeric_limits<uint16_t>::max();
            uint16_t max_ring = 0;

            for (std::size_t sample_index = 0; sample_index < source_sample_count;
                 ++sample_index) {
                const std::size_t index =
                    source_sample_count == point_count
                        ? sample_index
                        : (source_sample_count == 1
                               ? point_count / 2
                               : sample_index * (point_count - 1) /
                                     (source_sample_count - 1));
                const auto base = index * point_step;
                const float x = read_unaligned<float>(source.data, base + offsets.x);
                const float y = read_unaligned<float>(source.data, base + offsets.y);
                const float z = read_unaligned<float>(source.data, base + offsets.z);
                const float intensity =
                    read_unaligned<float>(source.data, base + offsets.intensity);
                const uint16_t ring =
                    read_unaligned<uint16_t>(source.data, base + offsets.ring);
                const double timestamp =
                    read_unaligned<double>(source.data, base + offsets.timestamp);
                if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
                    !std::isfinite(timestamp) || ring >= cfg_.scan_line) {
                    continue;
                }

                custom_messages::CustomPoint point{};
                point.x = x;
                point.y = y;
                point.z = z;
                point.reflectivity = static_cast<uint16_t>(std::clamp(
                    std::isfinite(intensity) ? static_cast<double>(intensity) : 0.0, 0.0,
                    255.0));
                point.tag = 0;
                point.line = ring;
                points.push_back({timestamp, point});
                min_ring = std::min(min_ring, ring);
                max_ring = std::max(max_ring, ring);
            }
            if (points.empty()) {
                throw std::runtime_error("M20 cloud has no finite Point-LIO returns");
            }

            if (point_count > source_sample_count &&
                !logged_cloud_sampling_.exchange(true, std::memory_order_acq_rel)) {
                logging::info(
                    "uniformly sampled M20 cloud before Point-LIO preprocessing",
                    {logging::Field("input_points", static_cast<int64_t>(point_count)),
                     logging::Field("sampled_source_points",
                                    static_cast<int64_t>(source_sample_count)),
                     logging::Field("selected_valid_points",
                                    static_cast<int64_t>(points.size()))});
            }

            std::sort(points.begin(), points.end(),
                      [](const TimedPoint& left, const TimedPoint& right) {
                          return left.timestamp < right.timestamp;
                      });
            const auto valid_point_count = points.size();
            const double first_point_time = points.front().timestamp;
            const double last_point_time = points.back().timestamp;
            const double scan_duration = last_point_time - first_point_time;
            if (scan_duration < 0.0 || scan_duration > cfg_.max_scan_duration_s) {
                throw std::runtime_error("M20 per-point timestamp span is outside the "
                                         "configured scan-duration limit");
            }

            const double source_header_time = header_seconds(source.header);
            // rsdriver uses absolute PTP seconds today. Accept a relative
            // per-scan timestamp too, but anchor that explicitly to the header.
            const bool absolute_point_time = first_point_time > 100'000'000.0;
            const double frame_time = absolute_point_time
                                          ? first_point_time
                                          : source_header_time + first_point_time;
            if (!std::isfinite(frame_time) || frame_time <= 0.0) {
                throw std::runtime_error("M20 cloud has no usable sensor timestamp");
            }
            if (last_lidar_sensor_time_ > 0.0 && frame_time <= last_lidar_sensor_time_) {
                logging::warn("dropping non-monotonic M20 lidar frame",
                              {logging::Field("stamp", frame_time),
                               logging::Field("previous_stamp", last_lidar_sensor_time_)});
                return;
            }

            auto message = boost::make_shared<custom_messages::CustomMsg>();
            message->header.seq = 0;
            message->header.stamp = custom_messages::Time().fromSec(frame_time);
            message->header.frame_id = cfg_.base_frame;
            message->timebase = static_cast<ulli>(std::llround(frame_time * 1e9));
            message->lidar_id = 0;
            for (auto& reserved : message->rsvd) reserved = 0;
            message->points.reserve(points.size());
            for (auto& timed : points) {
                const double offset_ns = (timed.timestamp - first_point_time) * 1e9;
                timed.point.offset_time =
                    static_cast<uli>(std::max(0.0, std::round(offset_ns)));
                message->points.push_back(timed.point);
            }
            message->point_num = static_cast<uli>(message->points.size());

            // Point-LIO's feeder callbacks take its internal buffer mutex and
            // are designed to run concurrently with process(). An outer lock
            // here would block sensor ingestion for the full estimator step.
            {
                std::lock_guard<std::mutex> lock(lidar_feed_mutex_);
                if (estimator_initialized_) {
                    lidar_feed_pending_ = true;
                } else {
                    ++initialization_lidar_frames_;
                }
                feed_reserved = true;
            }
            point_lio_->feed_lidar(message);
            last_lidar_sensor_time_ = frame_time;

            if (!logged_cloud_contract_.exchange(true, std::memory_order_acq_rel)) {
                logging::info(
                    "M20 Point-LIO accepted cloud contract",
                    {logging::Field("input_points", static_cast<int64_t>(point_count)),
                     logging::Field("valid_points",
                                    static_cast<int64_t>(valid_point_count)),
                     logging::Field("selected_points",
                                    static_cast<int64_t>(points.size())),
                     logging::Field("min_ring", static_cast<int64_t>(min_ring)),
                     logging::Field("max_ring", static_cast<int64_t>(max_ring)),
                     logging::Field("scan_duration_s", scan_duration),
                     logging::Field("absolute_point_time", absolute_point_time)});
            }
        } catch (const std::exception& error) {
            if (feed_reserved) {
                std::lock_guard<std::mutex> lock(lidar_feed_mutex_);
                if (estimator_initialized_) {
                    lidar_feed_pending_ = false;
                } else if (initialization_lidar_frames_ > 0) {
                    --initialization_lidar_frames_;
                }
            }
            logging::error("dropping M20 cloud before Point-LIO",
                           {logging::Field("error", std::string(error.what()))});
        }
    }

    bool point_lio_is_processing_lidar() const {
        std::lock_guard<std::mutex> lock(lidar_feed_mutex_);
        if (!estimator_initialized_) {
            return initialization_lidar_frames_ >= kMaxInitializationLidarFrames;
        }
        return lidar_feed_pending_;
    }

    void log_busy_lidar_drop() {
        const auto dropped =
            busy_lidar_drops_.fetch_add(1, std::memory_order_acq_rel) + 1;
        if (dropped == 1 || dropped % 500 == 0) {
            logging::info(
                "shedding M20 lidar frame while Point-LIO processes the previous frame",
                {logging::Field("dropped", static_cast<int64_t>(dropped))});
        }
    }

    void on_imu(const sensor_msgs::msg::Imu& source) {
        if (stopping_.load(std::memory_order_acquire) || point_lio_ == nullptr) return;
        const auto received_at = Clock::now();
        const double timestamp = header_seconds(source.header);
        if (!std::isfinite(timestamp) || timestamp <= 0.0 ||
            !std::isfinite(source.angular_velocity.x) ||
            !std::isfinite(source.angular_velocity.y) ||
            !std::isfinite(source.angular_velocity.z) ||
            !std::isfinite(source.linear_acceleration.x) ||
            !std::isfinite(source.linear_acceleration.y) ||
            !std::isfinite(source.linear_acceleration.z)) {
            logging::error("dropping invalid M20 IMU sample");
            return;
        }
        if (last_imu_sensor_time_ > 0.0 && timestamp <= last_imu_sensor_time_) {
            logging::warn("dropping non-monotonic M20 IMU sample",
                          {logging::Field("stamp", timestamp),
                           logging::Field("previous_stamp", last_imu_sensor_time_)});
            return;
        }

        auto message = boost::make_shared<custom_messages::Imu>();
        message->header.seq = 0;
        message->header.stamp = custom_messages::Time().fromSec(timestamp);
        message->header.frame_id = cfg_.base_frame;
        message->orientation.x = source.orientation.x;
        message->orientation.y = source.orientation.y;
        message->orientation.z = source.orientation.z;
        message->orientation.w = source.orientation.w;
        message->angular_velocity.x = source.angular_velocity.x;
        message->angular_velocity.y = source.angular_velocity.y;
        message->angular_velocity.z = source.angular_velocity.z;
        // ROS sensor_msgs/Imu is m/s^2; this Point-LIO core expects g.
        message->linear_acceleration.x = source.linear_acceleration.x / kStandardGravityMps2;
        message->linear_acceleration.y = source.linear_acceleration.y / kStandardGravityMps2;
        message->linear_acceleration.z = source.linear_acceleration.z / kStandardGravityMps2;
        for (int index = 0; index < 9; ++index) {
            message->orientation_covariance[index] = source.orientation_covariance[index];
            message->angular_velocity_covariance[index] =
                source.angular_velocity_covariance[index];
            message->linear_acceleration_covariance[index] =
                source.linear_acceleration_covariance[index] /
                (kStandardGravityMps2 * kStandardGravityMps2);
        }

        {
            std::lock_guard<std::mutex> lock(health_mutex_);
            last_imu_received_at_ = received_at;
            have_imu_ = true;
        }
        point_lio_->feed_imu(message);
        last_imu_sensor_time_ = timestamp;
    }

    bool lidar_health_is_fresh(Clock::time_point now) const {
        std::lock_guard<std::mutex> lock(health_mutex_);
        return have_lidar_ &&
               now - last_lidar_received_at_ <=
                   std::chrono::duration<double>(cfg_.lidar_timeout_s);
    }

    bool localization_health_is_fresh(Clock::time_point now) const {
        std::lock_guard<std::mutex> lock(health_mutex_);
        return have_lidar_ && have_imu_ && have_estimate_ &&
               now - last_lidar_received_at_ <=
                   std::chrono::duration<double>(cfg_.lidar_timeout_s) &&
               now - last_imu_received_at_ <=
                   std::chrono::duration<double>(cfg_.imu_timeout_s) &&
               now - last_estimate_advanced_at_ <=
                   std::chrono::duration<double>(cfg_.estimate_timeout_s);
    }

    void processing_loop() {
        while (!stopping_.load(std::memory_order_acquire)) {
            const auto iteration_started = Clock::now();
            bool have_estimate = false;
            double estimate_stamp = 0.0;
            point_lio_->process();
            const auto pose = point_lio_->get_pose();
            have_estimate = dimos::has_estimate(pose);
            if (have_estimate) {
                const auto& source_odom = point_lio_->get_odometry();
                estimate_stamp = source_odom.header.stamp.toSec();
                const auto now = Clock::now();
                if (std::isfinite(estimate_stamp) && estimate_stamp > 0.0) {
                    if (now - last_pointcloud_publish_ >= pointcloud_period_ &&
                        estimate_stamp > last_pointcloud_stamp_) {
                        const auto cloud = point_lio_->get_body_cloud();
                        if (cloud != nullptr && !cloud->empty()) {
                            publish_pointcloud(cloud, estimate_stamp);
                            last_pointcloud_stamp_ = estimate_stamp;
                            last_pointcloud_publish_ = now;
                        }
                    }
                    if (now - last_odometry_publish_ >= odometry_period_ &&
                        estimate_stamp > last_odometry_stamp_) {
                        publish_odometry(source_odom, estimate_stamp);
                        last_odometry_stamp_ = estimate_stamp;
                        last_odometry_publish_ = now;
                    }
                }
            }

            const auto now = Clock::now();
            bool estimate_advanced = false;
            if (have_estimate && std::isfinite(estimate_stamp) && estimate_stamp > 0.0) {
                std::lock_guard<std::mutex> lock(health_mutex_);
                if (!have_estimate_ || estimate_stamp > last_estimate_sensor_time_) {
                    last_estimate_sensor_time_ = estimate_stamp;
                    last_estimate_advanced_at_ = now;
                    have_estimate_ = true;
                    estimate_advanced = true;
                }
            }
            if (estimate_advanced) {
                std::lock_guard<std::mutex> lock(lidar_feed_mutex_);
                estimator_initialized_ = true;
                lidar_feed_pending_ = false;
            }
            if (now - last_readiness_publish_ >= readiness_period_) {
                publish_readiness(now);
                last_readiness_publish_ = now;
            }

            const auto elapsed = Clock::now() - iteration_started;
            if (elapsed < process_period_) {
                std::this_thread::sleep_for(process_period_ - elapsed);
            }
        }
    }

    void publish_readiness(Clock::time_point now) {
        const bool lidar_ready = lidar_health_is_fresh(now);
        const bool localization_ready = localization_health_is_fresh(now);
        std_msgs::Bool lidar_message;
        lidar_message.data = static_cast<int8_t>(lidar_ready);
        lidar_ready_.publish(lidar_message);
        std_msgs::Bool localization_message;
        localization_message.data = static_cast<int8_t>(localization_ready);
        localization_ready_.publish(localization_message);

        const int8_t current_lidar = lidar_ready ? 1 : 0;
        const int8_t previous_lidar =
            lidar_readiness_state_.exchange(current_lidar, std::memory_order_acq_rel);
        if (current_lidar != previous_lidar) {
            if (lidar_ready) {
                logging::info("M20 Point-LIO lidar input is ready");
            } else {
                logging::warn("M20 Point-LIO lidar input is not ready");
            }
        }

        const int8_t current = localization_ready ? 1 : 0;
        const int8_t previous =
            readiness_state_.exchange(current, std::memory_order_acq_rel);
        if (current != previous) {
            if (localization_ready) {
                logging::info("M20 Point-LIO localization is ready");
            } else {
                logging::warn("M20 Point-LIO localization is not ready");
            }
        }
    }

    void publish_pointcloud(const PointCloudXYZI::Ptr& cloud, double timestamp) {
        const auto count = static_cast<int>(cloud->size());
        auto output = dimos::make_xyzi_cloud(cfg_.base_frame, timestamp, count);
        for (int index = 0; index < count; ++index) {
            float* point = dimos::xyzi_point(output, index);
            point[0] = cloud->points[index].x;
            point[1] = cloud->points[index].y;
            point[2] = cloud->points[index].z;
            point[3] = cloud->points[index].intensity;
        }
        lidar_.publish(output);
    }

    void publish_odometry(const custom_messages::Odometry& source, double timestamp) {
        nav_msgs::Odometry output;
        output.header = dimos::make_header(cfg_.world_frame, timestamp);
        output.child_frame_id = cfg_.base_frame;
        output.pose.pose.position.x = source.pose.pose.position.x;
        output.pose.pose.position.y = source.pose.pose.position.y;
        output.pose.pose.position.z = source.pose.pose.position.z;
        output.pose.pose.orientation.x = source.pose.pose.orientation.x;
        output.pose.pose.orientation.y = source.pose.pose.orientation.y;
        output.pose.pose.orientation.z = source.pose.pose.orientation.z;
        output.pose.pose.orientation.w = source.pose.pose.orientation.w;
        output.twist.twist.linear.x = source.twist.twist.linear.x;
        output.twist.twist.linear.y = source.twist.twist.linear.y;
        output.twist.twist.linear.z = source.twist.twist.linear.z;
        output.twist.twist.angular.x = source.twist.twist.angular.x;
        output.twist.twist.angular.y = source.twist.twist.angular.y;
        output.twist.twist.angular.z = source.twist.twist.angular.z;
        for (int index = 0; index < 36; ++index) {
            output.pose.covariance[index] = source.pose.covariance[index];
            output.twist.covariance[index] = source.twist.covariance[index];
        }

        geometry_msgs::PoseStamped pose;
        pose.header = output.header;
        pose.pose = output.pose.pose;

        geometry_msgs::TransformStamped transform;
        transform.header = output.header;
        transform.child_frame_id = cfg_.base_frame;
        transform.transform.translation.x = output.pose.pose.position.x;
        transform.transform.translation.y = output.pose.pose.position.y;
        transform.transform.translation.z = output.pose.pose.position.z;
        transform.transform.rotation = output.pose.pose.orientation;
        tf2_msgs::TFMessage transforms;
        transforms.transforms_length = 1;
        transforms.transforms.push_back(std::move(transform));

        odometry_.publish(output);
        odom_.publish(pose);
        tf_.publish(transforms);
    }

    M20PointLioConfig cfg_;
    Output<std_msgs::Bool> lidar_ready_;
    Output<std_msgs::Bool> localization_ready_;
    Output<sensor_msgs::PointCloud2> lidar_;
    Output<geometry_msgs::PoseStamped> odom_;
    Output<nav_msgs::Odometry> odometry_;
    Output<tf2_msgs::TFMessage> tf_;
    std::unique_ptr<PointLio> point_lio_;

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    rclcpp::CallbackGroup::SharedPtr lidar_callback_group_;
    rclcpp::CallbackGroup::SharedPtr imu_callback_group_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    std::thread spin_thread_;
    std::thread processing_thread_;

    Clock::duration process_period_{};
    Clock::duration pointcloud_period_{};
    Clock::duration odometry_period_{};
    Clock::duration readiness_period_{};
    Clock::time_point last_pointcloud_publish_{};
    Clock::time_point last_odometry_publish_{};
    Clock::time_point last_readiness_publish_{};
    double last_pointcloud_stamp_ = 0.0;
    double last_odometry_stamp_ = 0.0;
    double last_lidar_sensor_time_ = 0.0;
    double last_imu_sensor_time_ = 0.0;

    mutable std::mutex health_mutex_;
    Clock::time_point last_lidar_received_at_{};
    Clock::time_point last_imu_received_at_{};
    Clock::time_point last_estimate_advanced_at_{};
    double last_estimate_sensor_time_ = 0.0;
    bool have_lidar_ = false;
    bool have_imu_ = false;
    bool have_estimate_ = false;
    std::atomic<bool> stopping_{false};
    mutable std::mutex lidar_feed_mutex_;
    std::mutex lidar_callback_mutex_;
    std::size_t initialization_lidar_frames_ = 0;
    bool estimator_initialized_ = false;
    bool lidar_feed_pending_ = false;
    std::atomic<uint64_t> busy_lidar_drops_{0};
    std::atomic<bool> logged_cloud_contract_{false};
    std::atomic<bool> logged_cloud_sampling_{false};
    std::atomic<int8_t> lidar_readiness_state_{-1};
    std::atomic<int8_t> readiness_state_{-1};
};

int main() {
    dimos::native::run_with_transport<M20PointLio>();
    return 0;
}
