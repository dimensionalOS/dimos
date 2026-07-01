// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Point-LIO + Livox Mid-360 native module for dimos NativeModule framework.
//
// Binds Livox SDK2 directly into the Point-LIO core: SDK callbacks feed
// CustomMsg/Imu to the IESKF estimator, which performs LiDAR-inertial SLAM.
// Sensor-frame point clouds and odometry are published on LCM.
//
// Usage:
//   ./pointlio_native \
//       --lidar '/lidar#sensor_msgs.PointCloud2' \
//       --odometry '/odometry#nav_msgs.Odometry' \
//       --filter_size_surf 0.2 --ivox_grid_resolution 2.0 ... \   # tuning as plain CLI args
//       --host_ip 192.168.1.5 --lidar_ip 192.168.1.155

#include <lcm/lcm-cpp.hpp>

#include <Eigen/Dense>

#include <atomic>
#include <boost/make_shared.hpp>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "livox_sdk_config.hpp"

#include "dimos_native_module.hpp"

#include "geometry_msgs/Quaternion.hpp"
#include "geometry_msgs/Vector3.hpp"
#include "nav_msgs/Odometry.hpp"
#include "sensor_msgs/Imu.hpp"
#include "sensor_msgs/PointCloud2.hpp"
#include "sensor_msgs/PointField.hpp"

// Point-LIO (header-only core, compiled sources linked via CMake)
#include "pointlio.hpp"
#include "pointlio_debug.hpp"

using livox_common::GRAVITY_MS2;
using livox_common::DATA_TYPE_IMU;
using livox_common::DATA_TYPE_CARTESIAN_HIGH;
using livox_common::DATA_TYPE_CARTESIAN_LOW;

static std::atomic<bool> g_running{true};
static lcm::LCM* g_lcm = nullptr;
static PointLio* g_point_lio = nullptr;

static double get_publish_ts() {
    return std::chrono::duration<double>( std::chrono::system_clock::now().time_since_epoch()).count();
}

// Parse a comma-separated list of doubles (CLI vector args); empty on bad input.
static std::vector<double> parse_doubles(const std::string& csv) {
    std::vector<double> out;
    size_t i = 0;
    while (i < csv.size()) {
        size_t j = csv.find(',', i);
        if (j == std::string::npos) { j = csv.size(); }
        try {
            out.push_back(std::stod(csv.substr(i, j - i)));
        } catch (...) {
            return {};
        }
        i = j + 1;
    }
    return out;
}

static std::string g_lidar_topic;
static std::string g_odometry_topic;
static std::string g_frame_id;          // required via --frame_id
static std::string g_child_frame_id;     // required via --child_frame_id
static std::string g_sensor_frame_id;    // required via --sensor_frame_id
static float g_frequency = 10.0f;

// Optional rigid transform applied to the published cloud + odometry (and, via
// the Python TF republisher that reads the odometry, the odom->body TF). Lets a
// physically-tilted mount be reported as a different mount without touching the
// SLAM core. Identity unless --transform supplies a row-major 4x4 (16 doubles).
// Cloud points are moved p' = C @ p; the body pose is conjugated T' = C @ T @
// inv(C) and the twist rotated by C's rotation, so cloud and odometry stay
// mutually consistent.
static bool g_has_transform = false;
static Eigen::Matrix4d g_transform = Eigen::Matrix4d::Identity();
static Eigen::Matrix4d g_transform_inv = Eigen::Matrix4d::Identity();

// Frame accumulator (Livox SDK raw → CustomMsg)
static std::mutex g_pc_mutex;
// Serializes all Point-LIO EKF access. The SDK delivers IMU on its own callback
// thread (on_imu_data → feed_imu) while the main loop runs feed_lidar/process/
// get_* — Point-LIO's estimator is not thread-safe, so without this the two
// threads race on the EKF state and occasionally emit a corrupt 2nd trajectory.
// Distinct from g_pc_mutex (which only guards the point accumulator) so incoming
// point packets can still accumulate while the EKF is processing.
static std::mutex g_lio_mutex;
static std::vector<custom_messages::CustomPoint> g_accumulated_points;
static uint64_t g_frame_start_ns = 0;
static bool g_frame_has_timestamp = false;

static uint64_t get_timestamp_ns(const LivoxLidarEthernetPacket* pkt) {
    uint64_t ns = 0;
    std::memcpy(&ns, pkt->timestamp, sizeof(uint64_t));
    return ns;
}

using dimos::time_from_seconds;
using dimos::make_header;

// Publish the lidar point cloud in the sensor frame (g_sensor_frame_id).
// `cloud` is Point-LIO's undistorted scan in the sensor's own frame
// (get_body_cloud), so points are published as-is with no world registration.
static void publish_lidar(PointCloudXYZI::Ptr cloud, double timestamp, const std::string& topic = "") {
    const std::string& chan = topic.empty() ? g_lidar_topic : topic;
    if (!g_lcm || !cloud || cloud->empty() || chan.empty()) { return; }

    int num_points = static_cast<int>(cloud->size());

    sensor_msgs::PointCloud2 pc;
    pc.header = make_header(g_sensor_frame_id, timestamp);
    pc.height = 1;
    pc.width = num_points;
    pc.is_bigendian = 0;
    pc.is_dense = 1;

    pc.fields_length = 4;
    pc.fields.resize(4);

    auto make_field = [](const std::string& name, int32_t offset) {
        sensor_msgs::PointField field;
        field.name = name;
        field.offset = offset;
        field.datatype = sensor_msgs::PointField::FLOAT32;
        field.count = 1;
        return field;
    };

    pc.fields[0] = make_field("x", 0);
    pc.fields[1] = make_field("y", 4);
    pc.fields[2] = make_field("z", 8);
    pc.fields[3] = make_field("intensity", 12);

    pc.point_step = 16;
    pc.row_step = pc.point_step * num_points;

    pc.data_length = pc.row_step;
    pc.data.resize(pc.data_length);

    const Eigen::Matrix3d transform_rotation = g_transform.block<3, 3>(0, 0);
    const Eigen::Vector3d transform_translation = g_transform.block<3, 1>(0, 3);
    for (int point_idx = 0; point_idx < num_points; ++point_idx) {
        float* dst = reinterpret_cast<float*>(pc.data.data() + point_idx * 16);
        double x = cloud->points[point_idx].x;
        double y = cloud->points[point_idx].y;
        double z = cloud->points[point_idx].z;
        if (g_has_transform) {
            const Eigen::Vector3d moved = transform_rotation * Eigen::Vector3d(x, y, z) + transform_translation;
            x = moved.x();
            y = moved.y();
            z = moved.z();
        }
        dst[0] = static_cast<float>(x);
        dst[1] = static_cast<float>(y);
        dst[2] = static_cast<float>(z);
        dst[3] = cloud->points[point_idx].intensity;
    }

    g_lcm->publish(chan, &pc);
}

static void publish_odometry(const custom_messages::Odometry& odom, double timestamp) {
    if (!g_lcm) { return; }

    nav_msgs::Odometry msg;
    msg.header = make_header(g_frame_id, timestamp);
    msg.child_frame_id = g_child_frame_id;

    // Pose in the SLAM/sensor frame.
    Eigen::Vector3d position(
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
    Eigen::Quaterniond orientation(
        odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);
    Eigen::Vector3d linear(
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z);
    Eigen::Vector3d angular(
        odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z);

    if (g_has_transform) {
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        pose.block<3, 3>(0, 0) = orientation.toRotationMatrix();
        pose.block<3, 1>(0, 3) = position;
        const Eigen::Matrix4d faked = g_transform * pose * g_transform_inv;
        const Eigen::Matrix3d transform_rotation = g_transform.block<3, 3>(0, 0);
        position = faked.block<3, 1>(0, 3);
        orientation = Eigen::Quaterniond(faked.block<3, 3>(0, 0));
        orientation.normalize();
        linear = transform_rotation * linear;
        angular = transform_rotation * angular;
    }

    msg.pose.pose.position.x = position.x();
    msg.pose.pose.position.y = position.y();
    msg.pose.pose.position.z = position.z();
    msg.pose.pose.orientation.x = orientation.x();
    msg.pose.pose.orientation.y = orientation.y();
    msg.pose.pose.orientation.z = orientation.z();
    msg.pose.pose.orientation.w = orientation.w();

    for (int idx = 0; idx < 36; ++idx) {
        msg.pose.covariance[idx] = odom.pose.covariance[idx];
    }

    // Velocity from Point-LIO's IESKF state (its key output over FAST-LIO).
    msg.twist.twist.linear.x = linear.x();
    msg.twist.twist.linear.y = linear.y();
    msg.twist.twist.linear.z = linear.z();
    msg.twist.twist.angular.x = angular.x();
    msg.twist.twist.angular.y = angular.y();
    msg.twist.twist.angular.z = angular.z();
    std::memset(msg.twist.covariance, 0, sizeof(msg.twist.covariance));

    g_lcm->publish(g_odometry_topic, &msg);
}


static void on_point_cloud(const uint32_t /*handle*/, const uint8_t /*dev_type*/, LivoxLidarEthernetPacket* data, void* /*client_data*/) {
    if (!g_running.load() || data == nullptr) { return; }

    uint64_t ts_ns = get_timestamp_ns(data);
    uint16_t dot_num = data->dot_num;

    // Per-point intra-packet offset (matches livox_ros_driver2). Without it all
    // points share one timestamp and per-point deskew is lost. time_interval
    // unit is 0.1us, so *100 → ns.
    const uint64_t point_interval_ns = dot_num > 0 ? static_cast<uint64_t>(data->time_interval) * 100 / dot_num : 0;

    std::lock_guard<std::mutex> lock(g_pc_mutex);

    if (!g_frame_has_timestamp) {
        g_frame_start_ns = ts_ns;
        g_frame_has_timestamp = true;
    }

    if (data->data_type == DATA_TYPE_CARTESIAN_HIGH) {
        auto* pts = reinterpret_cast<const LivoxLidarCartesianHighRawPoint*>(data->data);
        for (uint16_t point_idx = 0; point_idx < dot_num; ++point_idx) {
            custom_messages::CustomPoint cp;
            cp.x = static_cast<double>(pts[point_idx].x) / 1000.0;   // mm → m
            cp.y = static_cast<double>(pts[point_idx].y) / 1000.0;
            cp.z = static_cast<double>(pts[point_idx].z) / 1000.0;
            cp.reflectivity = pts[point_idx].reflectivity;
            cp.tag = pts[point_idx].tag;
            cp.line = 0;  // Mid-360: single line
            cp.offset_time = static_cast<uli>((ts_ns - g_frame_start_ns) + point_idx * point_interval_ns);
            g_accumulated_points.push_back(cp);
        }
    } else if (data->data_type == DATA_TYPE_CARTESIAN_LOW) {
        auto* pts = reinterpret_cast<const LivoxLidarCartesianLowRawPoint*>(data->data);
        for (uint16_t point_idx = 0; point_idx < dot_num; ++point_idx) {
            custom_messages::CustomPoint cp;
            cp.x = static_cast<double>(pts[point_idx].x) / 100.0;   // cm → m
            cp.y = static_cast<double>(pts[point_idx].y) / 100.0;
            cp.z = static_cast<double>(pts[point_idx].z) / 100.0;
            cp.reflectivity = pts[point_idx].reflectivity;
            cp.tag = pts[point_idx].tag;
            cp.line = 0;
            cp.offset_time = static_cast<uli>((ts_ns - g_frame_start_ns) + point_idx * point_interval_ns);
            g_accumulated_points.push_back(cp);
        }
    }
}

static void on_imu_data(const uint32_t /*handle*/, const uint8_t /*dev_type*/, LivoxLidarEthernetPacket* data, void* /*client_data*/) {
    if (!g_running.load() || data == nullptr || !g_point_lio) { return; }

    uint64_t pkt_ts_ns = get_timestamp_ns(data);
    double ts = static_cast<double>(pkt_ts_ns) / 1e9;
    auto* imu_pts = reinterpret_cast<const LivoxLidarImuRawPoint*>(data->data);
    uint16_t dot_num = data->dot_num;

    // Serialize EKF access against the main loop (run_main_iter). Held across the
    // whole packet so its samples feed atomically.
    std::lock_guard<std::mutex> lio_lock(g_lio_mutex);
    for (uint16_t point_idx = 0; point_idx < dot_num; ++point_idx) {
        auto imu_msg = boost::make_shared<custom_messages::Imu>();
        imu_msg->header.stamp = custom_messages::Time().fromSec(ts);
        imu_msg->header.seq = 0;
        imu_msg->header.frame_id = "livox_frame";

        imu_msg->orientation.x = 0.0;
        imu_msg->orientation.y = 0.0;
        imu_msg->orientation.z = 0.0;
        imu_msg->orientation.w = 1.0;
        for (int cov_idx = 0; cov_idx < 9; ++cov_idx) {
            imu_msg->orientation_covariance[cov_idx] = 0.0;
        }

        imu_msg->angular_velocity.x = static_cast<double>(imu_pts[point_idx].gyro_x);
        imu_msg->angular_velocity.y = static_cast<double>(imu_pts[point_idx].gyro_y);
        imu_msg->angular_velocity.z = static_cast<double>(imu_pts[point_idx].gyro_z);
        for (int cov_idx = 0; cov_idx < 9; ++cov_idx) {
            imu_msg->angular_velocity_covariance[cov_idx] = 0.0;
        }

        // Point-LIO expects accel in g (EKF does its own scaling). SDK already
        // reports g, so feed raw — scaling by GRAVITY_MS2 would double-scale and
        // trip the satu_acc check at rest.
        imu_msg->linear_acceleration.x = static_cast<double>(imu_pts[point_idx].acc_x);
        imu_msg->linear_acceleration.y = static_cast<double>(imu_pts[point_idx].acc_y);
        imu_msg->linear_acceleration.z = static_cast<double>(imu_pts[point_idx].acc_z);
        for (int cov_idx = 0; cov_idx < 9; ++cov_idx) {
            imu_msg->linear_acceleration_covariance[cov_idx] = 0.0;
        }

        g_point_lio->feed_imu(imu_msg);
    }
}

static void on_info_change(const uint32_t handle, const LivoxLidarInfo* info, void* /*client_data*/) {
    if (info == nullptr) { return; }

    char sn[17] = {};
    std::memcpy(sn, info->sn, 16);
    char ip[17] = {};
    std::memcpy(ip, info->lidar_ip, 16);

    if (pointlio_debug) {
        printf("[pointlio] Device connected: handle=%u type=%u sn=%s ip=%s\n", handle, info->dev_type, sn, ip);
    }

    SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, nullptr, nullptr);
    EnableLivoxLidarImuData(handle, nullptr, nullptr);
}

static void signal_handler(int /*sig*/) {
    g_running.store(false);
}

int main(int argc, char** argv) {
    dimos::NativeModule mod(argc, argv);

    g_lidar_topic = mod.has("lidar") ? mod.topic("lidar") : "";
    g_odometry_topic = mod.has("odometry") ? mod.topic("odometry") : "";

    if (g_lidar_topic.empty() && g_odometry_topic.empty()) {
        fprintf(stderr, "Error: at least one of --lidar or --odometry is required\n");
        return 1;
    }

    // Point-LIO tuning, passed as CLI args by the dimos module (no YAML).
    PointLioParams params;
    // common
    params.con_frame = mod.arg_bool("con_frame", params.con_frame);
    params.con_frame_num = mod.arg_int("con_frame_num", params.con_frame_num);
    params.cut_frame = mod.arg_bool("cut_frame", params.cut_frame);
    params.cut_frame_time_interval = mod.arg_float("cut_frame_time_interval", params.cut_frame_time_interval);
    params.time_lag_imu_to_lidar = mod.arg_float("time_lag_imu_to_lidar", params.time_lag_imu_to_lidar);
    // preprocess
    params.scan_line = mod.arg_int("scan_line", params.scan_line);
    params.scan_rate = mod.arg_int("scan_rate", params.scan_rate);
    params.blind = mod.arg_float("blind", params.blind);
    params.point_filter_num = mod.arg_int("point_filter_num", params.point_filter_num);
    std::string lidar_type = mod.arg("lidar_type", "avia");
    params.lidar_type = lidar_type == "velodyne" ? 2 : lidar_type == "ouster" ? 3 :
                        lidar_type == "hesai" ? 4 : lidar_type == "unilidar" ? 5 : 1;
    std::string ts_unit = mod.arg("timestamp_unit", "nanosecond");
    params.timestamp_unit = ts_unit == "second" ? 0 : ts_unit == "millisecond" ? 1 :
                            ts_unit == "microsecond" ? 2 : 3;
    // mapping
    params.use_imu_as_input = mod.arg_bool("use_imu_as_input", params.use_imu_as_input);
    params.prop_at_freq_of_imu = mod.arg_bool("prop_at_freq_of_imu", params.prop_at_freq_of_imu);
    params.check_satu = mod.arg_bool("check_satu", params.check_satu);
    params.init_map_size = mod.arg_int("init_map_size", params.init_map_size);
    params.space_down_sample = mod.arg_bool("space_down_sample", params.space_down_sample);
    params.satu_acc = mod.arg_float("satu_acc", params.satu_acc);
    params.satu_gyro = mod.arg_float("satu_gyro", params.satu_gyro);
    params.acc_norm = mod.arg_float("acc_norm", params.acc_norm);
    params.plane_thr = mod.arg_float("plane_thr", params.plane_thr);
    params.filter_size_surf = mod.arg_float("filter_size_surf", params.filter_size_surf);
    params.filter_size_map = mod.arg_float("filter_size_map", params.filter_size_map);
    params.ivox_grid_resolution = mod.arg_float("ivox_grid_resolution", params.ivox_grid_resolution);
    std::string ivox_nearby = mod.arg("ivox_nearby_type", "nearby6");
    params.ivox_nearby_type = ivox_nearby == "center" ? 0 : ivox_nearby == "nearby18" ? 18 :
                              ivox_nearby == "nearby26" ? 26 : 6;
    params.cube_side_length = mod.arg_float("cube_side_length", params.cube_side_length);
    params.det_range = mod.arg_float("det_range", params.det_range);
    params.fov_degree = mod.arg_float("fov_degree", params.fov_degree);
    params.imu_en = mod.arg_bool("imu_en", params.imu_en);
    params.start_in_aggressive_motion = mod.arg_bool("start_in_aggressive_motion", params.start_in_aggressive_motion);
    params.extrinsic_est_en = mod.arg_bool("extrinsic_est_en", params.extrinsic_est_en);
    params.imu_time_inte = mod.arg_float("imu_time_inte", params.imu_time_inte);
    params.lidar_meas_cov = mod.arg_float("lidar_meas_cov", params.lidar_meas_cov);
    params.acc_cov_input = mod.arg_float("acc_cov_input", params.acc_cov_input);
    params.vel_cov = mod.arg_float("vel_cov", params.vel_cov);
    params.gyr_cov_input = mod.arg_float("gyr_cov_input", params.gyr_cov_input);
    params.gyr_cov_output = mod.arg_float("gyr_cov_output", params.gyr_cov_output);
    params.acc_cov_output = mod.arg_float("acc_cov_output", params.acc_cov_output);
    params.b_gyr_cov = mod.arg_float("b_gyr_cov", params.b_gyr_cov);
    params.b_acc_cov = mod.arg_float("b_acc_cov", params.b_acc_cov);
    params.imu_meas_acc_cov = mod.arg_float("imu_meas_acc_cov", params.imu_meas_acc_cov);
    params.imu_meas_omg_cov = mod.arg_float("imu_meas_omg_cov", params.imu_meas_omg_cov);
    params.match_s = mod.arg_float("match_s", params.match_s);
    params.gravity_align = mod.arg_bool("gravity_align", params.gravity_align);
    if (auto g = parse_doubles(mod.arg("gravity", "")); !g.empty()) params.gravity = g;
    if (auto gi = parse_doubles(mod.arg("gravity_init", "")); !gi.empty()) params.gravity_init = gi;
    if (auto et = parse_doubles(mod.arg("extrinsic_t", "")); !et.empty()) params.extrinsic_T = et;
    if (auto er = parse_doubles(mod.arg("extrinsic_r", "")); !er.empty()) params.extrinsic_R = er;
    // Optional post-SLAM mount correction (row-major 4x4); identity when absent.
    if (auto transform = parse_doubles(mod.arg("transform", "")); transform.size() == 16) {
        g_has_transform = true;
        for (int row = 0; row < 4; ++row) {
            for (int col = 0; col < 4; ++col) {
                g_transform(row, col) = transform[row * 4 + col];
            }
        }
        g_transform_inv = g_transform.inverse();
    }
    // odometry
    params.publish_odometry_without_downsample =
        mod.arg_bool("publish_odometry_without_downsample", params.publish_odometry_without_downsample);
    params.odom_only = mod.arg_bool("odom_only", params.odom_only);

    // Point-LIO internal processing rates
    double msr_freq = mod.arg_float("msr_freq", 50.0f);
    double main_freq = mod.arg_float("main_freq", 5000.0f);

    // Livox hardware config
    std::string host_ip = mod.arg("host_ip", "192.168.1.5");
    std::string lidar_ip = mod.arg("lidar_ip", "192.168.1.155");
    g_frequency = mod.arg_float("frequency", 10.0f);
    g_frame_id = mod.arg_required("frame_id");
    g_child_frame_id = mod.arg_required("child_frame_id");
    g_sensor_frame_id = mod.arg_required("sensor_frame_id");
    float pointcloud_freq = mod.arg_float("pointcloud_freq", 5.0f);
    float odom_freq = mod.arg_float("odom_freq", 50.0f);

    // Propagates to the Point-LIO core via the `pointlio_debug` global.
    bool debug = mod.arg_bool("debug", false);
    pointlio_debug = debug;

    // SDK network ports (defaults from SdkPorts struct in livox_sdk_config.hpp)
    livox_common::SdkPorts ports;
    const livox_common::SdkPorts port_defaults;
    ports.cmd_data        = mod.arg_int("cmd_data_port", port_defaults.cmd_data);
    ports.push_msg        = mod.arg_int("push_msg_port", port_defaults.push_msg);
    ports.point_data      = mod.arg_int("point_data_port", port_defaults.point_data);
    ports.imu_data        = mod.arg_int("imu_data_port", port_defaults.imu_data);
    ports.log_data        = mod.arg_int("log_data_port", port_defaults.log_data);
    ports.host_cmd_data   = mod.arg_int("host_cmd_data_port", port_defaults.host_cmd_data);
    ports.host_push_msg   = mod.arg_int("host_push_msg_port", port_defaults.host_push_msg);
    ports.host_point_data = mod.arg_int("host_point_data_port", port_defaults.host_point_data);
    ports.host_imu_data   = mod.arg_int("host_imu_data_port", port_defaults.host_imu_data);
    ports.host_log_data   = mod.arg_int("host_log_data_port", port_defaults.host_log_data);

    if (debug) {
        printf("[pointlio] Starting Point-LIO + Livox Mid-360 native module\n");
        printf("[pointlio] lidar topic: %s\n", g_lidar_topic.empty() ? "(disabled)" : g_lidar_topic.c_str());
        printf("[pointlio] odometry topic: %s\n", g_odometry_topic.empty() ? "(disabled)" : g_odometry_topic.c_str());
        printf("[pointlio] tuning: filter_size_surf=%.3f ivox_res=%.3f lidar_type=%d\n", params.filter_size_surf, params.ivox_grid_resolution, params.lidar_type);
        printf("[pointlio] host_ip: %s  lidar_ip: %s  frequency: %.1f Hz\n", host_ip.c_str(), lidar_ip.c_str(), g_frequency);
        printf("[pointlio] pointcloud_freq: %.1f Hz  odom_freq: %.1f Hz\n", pointcloud_freq, odom_freq);
    }

    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);

    lcm::LCM lcm;
    if (!lcm.good()) {
        fprintf(stderr, "Error: LCM init failed\n");
        return 1;
    }
    g_lcm = &lcm;

    if (debug) { printf("[pointlio] Initializing Point-LIO...\n"); }
    PointLio point_lio(params, msr_freq, main_freq);
    g_point_lio = &point_lio;
    if (debug) { printf("[pointlio] Point-LIO initialized.\n"); }

    // Main-loop state. Body lives in `run_main_iter`, driven by the wall-paced
    // main thread.
    auto frame_interval = std::chrono::microseconds( static_cast<int64_t>(1e6 / g_frequency));
    std::optional<std::chrono::steady_clock::time_point> last_emit;
    const double process_period_ms = 1000.0 / main_freq;

    auto pc_interval = std::chrono::microseconds( static_cast<int64_t>(1e6 / pointcloud_freq));
    auto odom_interval = std::chrono::microseconds( static_cast<int64_t>(1e6 / odom_freq));
    std::optional<std::chrono::steady_clock::time_point> last_pc_publish;
    std::optional<std::chrono::steady_clock::time_point> last_odom_publish;


    auto run_main_iter = [&](std::chrono::steady_clock::time_point now) {
        // Lazy-seed rate-limit bookmarks on the first iteration so they align
        // with the wall clock.
        if (!last_emit.has_value()) {
            last_emit = now;
        }
        if (!last_pc_publish.has_value()) {
            last_pc_publish = now;
        }
        if (!last_odom_publish.has_value()) {
            last_odom_publish = now;
        }

        // At frame rate: drain accumulated points into a CustomMsg and feed
        // Point-LIO. Hold g_pc_mutex across the rate-limit check AND swap so the
        // clock + accumulator are observed atomically (no packet slips between).
        std::vector<custom_messages::CustomPoint> points;
        uint64_t frame_start = 0;
        {
            std::lock_guard<std::mutex> lock(g_pc_mutex);
            if (now - *last_emit >= frame_interval) {
                if (!g_accumulated_points.empty()) {
                    points.swap(g_accumulated_points);
                    frame_start = g_frame_start_ns;
                    g_frame_has_timestamp = false;
                }
                last_emit = now;
            }
        }
        // Serialize EKF access against the SDK IMU callback (on_imu_data) for the
        // rest of the iteration — feed_lidar/process/get_* all touch the estimator.
        std::lock_guard<std::mutex> lio_lock(g_lio_mutex);
        if (!points.empty()) {
            const size_t num_points = points.size();
            auto lidar_msg = boost::make_shared<custom_messages::CustomMsg>();
            lidar_msg->header.seq = 0;
            lidar_msg->header.stamp = custom_messages::Time().fromSec( static_cast<double>(frame_start) / 1e9);
            lidar_msg->header.frame_id = "livox_frame";
            lidar_msg->timebase = frame_start;
            lidar_msg->lidar_id = 0;
            for (int idx = 0; idx < 3; idx++) { lidar_msg->rsvd[idx] = 0; }
            lidar_msg->point_num = static_cast<uli>(num_points);
            lidar_msg->points = std::move(points);
            if (pointlio_debug) {
                fprintf(stderr, "[pointlio] feed_lidar frame: %zu points\n", num_points);
            }
            point_lio.feed_lidar(lidar_msg);
        }

        // One Point-LIO IESKF step (cheap when queues empty).
        point_lio.process();

        auto pose = point_lio.get_pose();
        if (!pose.empty() && (pose[0] != 0.0 || pose[1] != 0.0 || pose[2] != 0.0)) {
            double ts = get_publish_ts();

            const bool lidar_due = !g_lidar_topic.empty() && now - *last_pc_publish >= pc_interval;

            // get_body_cloud is the loop's costliest step, so build it only when
            // a publish is due.
            if (lidar_due) {
                auto body_cloud = point_lio.get_body_cloud();
                if (body_cloud && !body_cloud->empty()) {
                    publish_lidar(body_cloud, ts);
                    last_pc_publish = now;
                    if (pointlio_debug) {
                        fprintf(stderr, "[pointlio] publish lidar: %zu points  pose=(%.3f, %.3f, %.3f)\n", body_cloud->size(), pose[0], pose[1], pose[2]);
                    }
                }
            }

            // Pose + covariance at odom_freq.
            if (!g_odometry_topic.empty() && now - *last_odom_publish >= odom_interval) {
                publish_odometry(point_lio.get_odometry(), ts);
                last_odom_publish = now;
                if (pointlio_debug) {
                    fprintf(stderr, "[pointlio] publish odom: pose=(%.3f, %.3f, %.3f)\n", pose[0], pose[1], pose[2]);
                }
            }
        }
    };

    // Packet source: Livox SDK callbacks from its own threads feed the
    // accumulator/EKF; the main thread below owns run_main_iter.
    if (!livox_common::init_livox_sdk(host_ip, lidar_ip, ports, debug)) {
        return 1;
    }
    SetLivoxLidarPointCloudCallBack(on_point_cloud, nullptr);
    SetLivoxLidarImuDataCallback(on_imu_data, nullptr);
    SetLivoxLidarInfoChangeCallback(on_info_change, nullptr);
    if (!LivoxLidarSdkStart()) {
        fprintf(stderr, "Error: LivoxLidarSdkStart failed\n");
        LivoxLidarSdkUninit();
        return 1;
    }
    if (debug) { printf("[pointlio] SDK started, waiting for device...\n"); }

    while (g_running.load()) {
        auto loop_start = std::chrono::high_resolution_clock::now();
        run_main_iter(std::chrono::steady_clock::now());

        lcm.handleTimeout(0);

        // Rate control (~main_freq, 5kHz default).
        auto loop_end = std::chrono::high_resolution_clock::now();
        auto elapsed_ms = std::chrono::duration<double, std::milli>(loop_end - loop_start).count();
        if (elapsed_ms < process_period_ms) {
            std::this_thread::sleep_for(std::chrono::microseconds( static_cast<int64_t>((process_period_ms - elapsed_ms) * 1000)));
        }
    }

    if (debug) { printf("[pointlio] Shutting down...\n"); }
    // Uninit (stops + joins the SDK callback threads) BEFORE clearing the
    // pointers those callbacks read, so an in-flight on_imu_data/on_point_cloud
    // can't race the assignment and dereference a null g_point_lio / g_lcm.
    LivoxLidarSdkUninit();
    g_point_lio = nullptr;
    g_lcm = nullptr;

    if (debug) { printf("[pointlio] Done.\n"); }
    return 0;
}
