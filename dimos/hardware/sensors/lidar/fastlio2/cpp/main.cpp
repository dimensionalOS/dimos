// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// FAST-LIO2 + Livox Mid-360 native module for dimos NativeModule framework.
//
// Binds Livox SDK2 directly into FAST-LIO-NON-ROS: SDK callbacks feed
// CustomMsg/Imu to FastLio, which performs EKF-LOAM SLAM.  Registered
// (world-frame) point clouds and odometry are published on LCM.
//
// Usage:
//   ./fastlio2_native \
//       --lidar '/lidar#sensor_msgs.PointCloud2' \
//       --odometry '/odometry#nav_msgs.Odometry' \
//       --config_path /path/to/mid360.yaml \
//       --host_ip 192.168.1.5 --lidar_ip 192.168.1.155

#include <lcm/lcm-cpp.hpp>

#include <atomic>
#include <boost/make_shared.hpp>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "livox_sdk_config.hpp"

#include "cloud_filter.hpp"
#include "dimos_native_module.hpp"
#include "voxel_map.hpp"

// dimos LCM message headers
#include "geometry_msgs/Quaternion.hpp"
#include "geometry_msgs/Vector3.hpp"
#include "nav_msgs/Odometry.hpp"
#include "sensor_msgs/Imu.hpp"
#include "sensor_msgs/PointCloud2.hpp"
#include "sensor_msgs/PointField.hpp"

// FAST-LIO (header-only core, compiled sources linked via CMake)
#include "fast_lio.hpp"

using livox_common::GRAVITY_MS2;
using livox_common::DATA_TYPE_IMU;
using livox_common::DATA_TYPE_CARTESIAN_HIGH;
using livox_common::DATA_TYPE_CARTESIAN_LOW;

// ---------------------------------------------------------------------------
// Global state
// ---------------------------------------------------------------------------

static std::atomic<bool> g_running{true};
static lcm::LCM* g_lcm = nullptr;
static FastLio* g_fastlio = nullptr;

static std::string g_lidar_topic;
static std::string g_odometry_topic;
static std::string g_map_topic;
static std::string g_frame_id = "map";
static std::string g_child_frame_id = "body";
static float g_frequency = 10.0f;

// Frame accumulator (Livox SDK raw → CustomMsg)
static std::mutex g_pc_mutex;
static std::vector<custom_messages::CustomPoint> g_accumulated_points;
static uint64_t g_frame_start_ns = 0;
static bool g_frame_has_timestamp = false;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static uint64_t get_timestamp_ns(const LivoxLidarEthernetPacket* pkt) {
    uint64_t ns = 0;
    std::memcpy(&ns, pkt->timestamp, sizeof(uint64_t));
    return ns;
}

using dimos::time_from_seconds;
using dimos::make_header;

// ---------------------------------------------------------------------------
// Publish lidar (world-frame point cloud)
// ---------------------------------------------------------------------------

static void publish_lidar(PointCloudXYZI::Ptr cloud, double timestamp,
                          const std::string& topic = "") {
    const std::string& chan = topic.empty() ? g_lidar_topic : topic;
    if (!g_lcm || !cloud || cloud->empty() || chan.empty()) return;

    int num_points = static_cast<int>(cloud->size());

    sensor_msgs::PointCloud2 pc;
    pc.header = make_header(g_frame_id, timestamp);
    pc.height = 1;
    pc.width = num_points;
    pc.is_bigendian = 0;
    pc.is_dense = 1;

    // Fields: x, y, z, intensity (float32 each)
    pc.fields_length = 4;
    pc.fields.resize(4);

    auto make_field = [](const std::string& name, int32_t offset) {
        sensor_msgs::PointField f;
        f.name = name;
        f.offset = offset;
        f.datatype = sensor_msgs::PointField::FLOAT32;
        f.count = 1;
        return f;
    };

    pc.fields[0] = make_field("x", 0);
    pc.fields[1] = make_field("y", 4);
    pc.fields[2] = make_field("z", 8);
    pc.fields[3] = make_field("intensity", 12);

    pc.point_step = 16;
    pc.row_step = pc.point_step * num_points;

    pc.data_length = pc.row_step;
    pc.data.resize(pc.data_length);

    for (int i = 0; i < num_points; ++i) {
        float* dst = reinterpret_cast<float*>(pc.data.data() + i * 16);
        dst[0] = cloud->points[i].x;
        dst[1] = cloud->points[i].y;
        dst[2] = cloud->points[i].z;
        dst[3] = cloud->points[i].intensity;
    }

    g_lcm->publish(chan, &pc);
}

// ---------------------------------------------------------------------------
// Publish odometry
// ---------------------------------------------------------------------------

static void publish_odometry(const custom_messages::Odometry& odom, double timestamp) {
    if (!g_lcm) return;

    nav_msgs::Odometry msg;
    msg.header = make_header(g_frame_id, timestamp);
    msg.child_frame_id = g_child_frame_id;

    // Pose
    msg.pose.pose.position.x = odom.pose.pose.position.x;
    msg.pose.pose.position.y = odom.pose.pose.position.y;
    msg.pose.pose.position.z = odom.pose.pose.position.z;
    msg.pose.pose.orientation.x = odom.pose.pose.orientation.x;
    msg.pose.pose.orientation.y = odom.pose.pose.orientation.y;
    msg.pose.pose.orientation.z = odom.pose.pose.orientation.z;
    msg.pose.pose.orientation.w = odom.pose.pose.orientation.w;

    // Covariance (fixed-size double[36])
    for (int i = 0; i < 36; ++i) {
        msg.pose.covariance[i] = odom.pose.covariance[i];
    }

    // Twist (zero — FAST-LIO doesn't output velocity directly)
    msg.twist.twist.linear.x = 0;
    msg.twist.twist.linear.y = 0;
    msg.twist.twist.linear.z = 0;
    msg.twist.twist.angular.x = 0;
    msg.twist.twist.angular.y = 0;
    msg.twist.twist.angular.z = 0;
    std::memset(msg.twist.covariance, 0, sizeof(msg.twist.covariance));

    g_lcm->publish(g_odometry_topic, &msg);
}

// ---------------------------------------------------------------------------
// Livox SDK callbacks
// ---------------------------------------------------------------------------

static void on_point_cloud(const uint32_t /*handle*/, const uint8_t /*dev_type*/,
                           LivoxLidarEthernetPacket* data, void* /*client_data*/) {
    if (!g_running.load() || data == nullptr) return;

    uint64_t ts_ns = get_timestamp_ns(data);
    uint16_t dot_num = data->dot_num;

    std::lock_guard<std::mutex> lock(g_pc_mutex);

    if (!g_frame_has_timestamp) {
        g_frame_start_ns = ts_ns;
        g_frame_has_timestamp = true;
    }

    if (data->data_type == DATA_TYPE_CARTESIAN_HIGH) {
        auto* pts = reinterpret_cast<const LivoxLidarCartesianHighRawPoint*>(data->data);
        for (uint16_t i = 0; i < dot_num; ++i) {
            custom_messages::CustomPoint cp;
            cp.x = static_cast<double>(pts[i].x) / 1000.0;   // mm → m
            cp.y = static_cast<double>(pts[i].y) / 1000.0;
            cp.z = static_cast<double>(pts[i].z) / 1000.0;
            cp.reflectivity = pts[i].reflectivity;
            cp.tag = pts[i].tag;
            cp.line = 0;  // Mid-360: non-repetitive, single "line"
            cp.offset_time = static_cast<uli>(ts_ns - g_frame_start_ns);
            g_accumulated_points.push_back(cp);
        }
    } else if (data->data_type == DATA_TYPE_CARTESIAN_LOW) {
        auto* pts = reinterpret_cast<const LivoxLidarCartesianLowRawPoint*>(data->data);
        for (uint16_t i = 0; i < dot_num; ++i) {
            custom_messages::CustomPoint cp;
            cp.x = static_cast<double>(pts[i].x) / 100.0;   // cm → m
            cp.y = static_cast<double>(pts[i].y) / 100.0;
            cp.z = static_cast<double>(pts[i].z) / 100.0;
            cp.reflectivity = pts[i].reflectivity;
            cp.tag = pts[i].tag;
            cp.line = 0;
            cp.offset_time = static_cast<uli>(ts_ns - g_frame_start_ns);
            g_accumulated_points.push_back(cp);
        }
    }
}

static void on_imu_data(const uint32_t /*handle*/, const uint8_t /*dev_type*/,
                        LivoxLidarEthernetPacket* data, void* /*client_data*/) {
    if (!g_running.load() || data == nullptr || !g_fastlio) return;

    double ts = static_cast<double>(get_timestamp_ns(data)) / 1e9;
    auto* imu_pts = reinterpret_cast<const LivoxLidarImuRawPoint*>(data->data);
    uint16_t dot_num = data->dot_num;

    for (uint16_t i = 0; i < dot_num; ++i) {
        auto imu_msg = boost::make_shared<custom_messages::Imu>();
        imu_msg->header.stamp = custom_messages::Time().fromSec(ts);
        imu_msg->header.seq = 0;
        imu_msg->header.frame_id = "livox_frame";

        imu_msg->orientation.x = 0.0;
        imu_msg->orientation.y = 0.0;
        imu_msg->orientation.z = 0.0;
        imu_msg->orientation.w = 1.0;
        for (int j = 0; j < 9; ++j)
            imu_msg->orientation_covariance[j] = 0.0;

        imu_msg->angular_velocity.x = static_cast<double>(imu_pts[i].gyro_x);
        imu_msg->angular_velocity.y = static_cast<double>(imu_pts[i].gyro_y);
        imu_msg->angular_velocity.z = static_cast<double>(imu_pts[i].gyro_z);
        for (int j = 0; j < 9; ++j)
            imu_msg->angular_velocity_covariance[j] = 0.0;

        imu_msg->linear_acceleration.x = static_cast<double>(imu_pts[i].acc_x) * GRAVITY_MS2;
        imu_msg->linear_acceleration.y = static_cast<double>(imu_pts[i].acc_y) * GRAVITY_MS2;
        imu_msg->linear_acceleration.z = static_cast<double>(imu_pts[i].acc_z) * GRAVITY_MS2;
        for (int j = 0; j < 9; ++j)
            imu_msg->linear_acceleration_covariance[j] = 0.0;

        g_fastlio->feed_imu(imu_msg);
    }
}

static void on_info_change(const uint32_t handle, const LivoxLidarInfo* info,
                           void* /*client_data*/) {
    if (info == nullptr) return;

    char sn[17] = {};
    std::memcpy(sn, info->sn, 16);
    char ip[17] = {};
    std::memcpy(ip, info->lidar_ip, 16);

    printf("[fastlio2] Device connected: handle=%u type=%u sn=%s ip=%s\n",
           handle, info->dev_type, sn, ip);

    SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, nullptr, nullptr);
    EnableLivoxLidarImuData(handle, nullptr, nullptr);
}

// ---------------------------------------------------------------------------
// Signal handling
// ---------------------------------------------------------------------------

static void signal_handler(int /*sig*/) {
    g_running.store(false);
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(int argc, char** argv) {
    dimos::NativeModule mod(argc, argv);

    // Required: LCM topics for output ports
    g_lidar_topic = mod.has("lidar") ? mod.topic("lidar") : "";
    g_odometry_topic = mod.has("odometry") ? mod.topic("odometry") : "";
    g_map_topic = mod.has("global_map") ? mod.topic("global_map") : "";

    if (g_lidar_topic.empty() && g_odometry_topic.empty()) {
        fprintf(stderr, "Error: at least one of --lidar or --odometry is required\n");
        return 1;
    }

    // FAST-LIO config path
    std::string config_path = mod.arg("config_path", "");
    if (config_path.empty()) {
        fprintf(stderr, "Error: --config_path <path> is required\n");
        return 1;
    }

    // FAST-LIO internal processing rates
    double msr_freq = mod.arg_float("msr_freq", 50.0f);
    double main_freq = mod.arg_float("main_freq", 5000.0f);

    // Livox hardware config
    std::string host_ip = mod.arg("host_ip", "192.168.1.5");
    std::string lidar_ip = mod.arg("lidar_ip", "192.168.1.155");
    g_frequency = mod.arg_float("frequency", 10.0f);
    g_frame_id = mod.arg("frame_id", "map");
    g_child_frame_id = mod.arg("child_frame_id", "body");
    float pointcloud_freq = mod.arg_float("pointcloud_freq", 5.0f);
    float odom_freq = mod.arg_float("odom_freq", 50.0f);
    CloudFilterConfig filter_cfg;
    filter_cfg.voxel_size = mod.arg_float("voxel_size", 0.1f);
    filter_cfg.sor_mean_k = mod.arg_int("sor_mean_k", 50);
    filter_cfg.sor_stddev = mod.arg_float("sor_stddev", 1.0f);
    float map_voxel_size = mod.arg_float("map_voxel_size", 0.1f);
    float map_max_range = mod.arg_float("map_max_range", 100.0f);
    float map_freq = mod.arg_float("map_freq", 0.0f);

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

    printf("[fastlio2] Starting FAST-LIO2 + Livox Mid-360 native module\n");
    printf("[fastlio2] lidar topic: %s\n",
           g_lidar_topic.empty() ? "(disabled)" : g_lidar_topic.c_str());
    printf("[fastlio2] odometry topic: %s\n",
           g_odometry_topic.empty() ? "(disabled)" : g_odometry_topic.c_str());
    printf("[fastlio2] global_map topic: %s\n",
           g_map_topic.empty() ? "(disabled)" : g_map_topic.c_str());
    printf("[fastlio2] config: %s\n", config_path.c_str());
    printf("[fastlio2] host_ip: %s  lidar_ip: %s  frequency: %.1f Hz\n",
           host_ip.c_str(), lidar_ip.c_str(), g_frequency);
    printf("[fastlio2] pointcloud_freq: %.1f Hz  odom_freq: %.1f Hz\n",
           pointcloud_freq, odom_freq);
    printf("[fastlio2] voxel_size: %.3f  sor_mean_k: %d  sor_stddev: %.1f\n",
           filter_cfg.voxel_size, filter_cfg.sor_mean_k, filter_cfg.sor_stddev);
    if (!g_map_topic.empty())
        printf("[fastlio2] map_voxel_size: %.3f  map_max_range: %.1f  map_freq: %.1f Hz\n",
               map_voxel_size, map_max_range, map_freq);

    // Signal handlers
    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);

    // Init LCM
    lcm::LCM lcm;
    if (!lcm.good()) {
        fprintf(stderr, "Error: LCM init failed\n");
        return 1;
    }
    g_lcm = &lcm;

    // Init FAST-LIO with config
    printf("[fastlio2] Initializing FAST-LIO...\n");
    FastLio fast_lio(config_path, msr_freq, main_freq);
    g_fastlio = &fast_lio;
    printf("[fastlio2] FAST-LIO initialized.\n");

    // Init Livox SDK (in-memory config, no temp files)
    if (!livox_common::init_livox_sdk(host_ip, lidar_ip, ports)) {
        return 1;
    }

    // Register SDK callbacks
    SetLivoxLidarPointCloudCallBack(on_point_cloud, nullptr);
    SetLivoxLidarImuDataCallback(on_imu_data, nullptr);
    SetLivoxLidarInfoChangeCallback(on_info_change, nullptr);

    // Start SDK
    if (!LivoxLidarSdkStart()) {
        fprintf(stderr, "Error: LivoxLidarSdkStart failed\n");
        LivoxLidarSdkUninit();
        return 1;
    }

    printf("[fastlio2] SDK started, waiting for device...\n");

    // Main loop
    auto frame_interval = std::chrono::microseconds(
        static_cast<int64_t>(1e6 / g_frequency));
    auto last_emit = std::chrono::steady_clock::now();
    const double process_period_ms = 1000.0 / main_freq;

    // Rate limiters for output publishing
    auto pc_interval = std::chrono::microseconds(
        static_cast<int64_t>(1e6 / pointcloud_freq));
    auto odom_interval = std::chrono::microseconds(
        static_cast<int64_t>(1e6 / odom_freq));
    auto last_pc_publish = std::chrono::steady_clock::now();
    auto last_odom_publish = std::chrono::steady_clock::now();

    // Global voxel map (only if map topic is configured AND map_freq > 0)
    std::unique_ptr<VoxelMap> global_map;
    std::chrono::microseconds map_interval{0};
    auto last_map_publish = std::chrono::steady_clock::now();
    if (!g_map_topic.empty() && map_freq > 0.0f) {
        global_map = std::make_unique<VoxelMap>(map_voxel_size, map_max_range);
        map_interval = std::chrono::microseconds(
            static_cast<int64_t>(1e6 / map_freq));
    }

    while (g_running.load()) {
        auto loop_start = std::chrono::high_resolution_clock::now();

        // At frame rate: build CustomMsg from accumulated points and feed to FAST-LIO
        auto now = std::chrono::steady_clock::now();
        if (now - last_emit >= frame_interval) {
            std::vector<custom_messages::CustomPoint> points;
            uint64_t frame_start = 0;

            {
                std::lock_guard<std::mutex> lock(g_pc_mutex);
                if (!g_accumulated_points.empty()) {
                    points.swap(g_accumulated_points);
                    frame_start = g_frame_start_ns;
                    g_frame_has_timestamp = false;
                }
            }

            if (!points.empty()) {
                // Build CustomMsg
                auto lidar_msg = boost::make_shared<custom_messages::CustomMsg>();
                lidar_msg->header.seq = 0;
                lidar_msg->header.stamp = custom_messages::Time().fromSec(
                    static_cast<double>(frame_start) / 1e9);
                lidar_msg->header.frame_id = "livox_frame";
                lidar_msg->timebase = frame_start;
                lidar_msg->lidar_id = 0;
                for (int i = 0; i < 3; i++)
                    lidar_msg->rsvd[i] = 0;
                lidar_msg->point_num = static_cast<uli>(points.size());
                lidar_msg->points = std::move(points);

                fast_lio.feed_lidar(lidar_msg);
            }

            last_emit = now;
        }

        // Run FAST-LIO processing step (high frequency)
        fast_lio.process();

        // Check for new results and accumulate/publish (rate-limited)
        auto pose = fast_lio.get_pose();
        if (!pose.empty() && (pose[0] != 0.0 || pose[1] != 0.0 || pose[2] != 0.0)) {
            double ts = std::chrono::duration<double>(
                std::chrono::system_clock::now().time_since_epoch()).count();

            auto world_cloud = fast_lio.get_world_cloud();
            if (world_cloud && !world_cloud->empty()) {
                auto filtered = filter_cloud<PointType>(world_cloud, filter_cfg);

                // Per-scan publish at pointcloud_freq
                if (!g_lidar_topic.empty() && now - last_pc_publish >= pc_interval) {
                    publish_lidar(filtered, ts);
                    last_pc_publish = now;
                }

                // Global map: insert, prune, and publish at map_freq
                if (global_map) {
                    global_map->insert<PointType>(filtered);

                    if (now - last_map_publish >= map_interval) {
                        global_map->prune(
                            static_cast<float>(pose[0]),
                            static_cast<float>(pose[1]),
                            static_cast<float>(pose[2]));
                        auto map_cloud = global_map->to_cloud<PointType>();
                        publish_lidar(map_cloud, ts, g_map_topic);
                        last_map_publish = now;
                    }
                }
            }

            // Publish odometry (rate-limited to odom_freq)
            if (!g_odometry_topic.empty() && (now - last_odom_publish >= odom_interval)) {
                publish_odometry(fast_lio.get_odometry(), ts);
                last_odom_publish = now;
            }
        }

        // Handle LCM messages
        lcm.handleTimeout(0);

        // Rate control (~5kHz processing)
        auto loop_end = std::chrono::high_resolution_clock::now();
        auto elapsed_ms = std::chrono::duration<double, std::milli>(loop_end - loop_start).count();
        if (elapsed_ms < process_period_ms) {
            std::this_thread::sleep_for(std::chrono::microseconds(
                static_cast<int64_t>((process_period_ms - elapsed_ms) * 1000)));
        }
    }

    // Cleanup
    printf("[fastlio2] Shutting down...\n");
    g_fastlio = nullptr;
    LivoxLidarSdkUninit();
    g_lcm = nullptr;

    printf("[fastlio2] Done.\n");
    return 0;
}
