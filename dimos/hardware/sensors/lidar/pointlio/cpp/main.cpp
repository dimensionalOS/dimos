// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// FAST-LIO2 + Livox Mid-360 native module for dimos NativeModule framework.
//
// Binds Livox SDK2 directly into FAST-LIO-NON-ROS: SDK callbacks feed
// CustomMsg/Imu to FastLio, which performs EKF-LOAM SLAM.  Sensor-frame
// (mid360_link) point clouds and odometry are published on LCM.
//
// Usage:
//   ./fastlio2_native \
//       --lidar '/lidar#sensor_msgs.PointCloud2' \
//       --odometry '/odometry#nav_msgs.Odometry' \
//       --config_path /path/to/default.yaml \
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
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "livox_sdk_config.hpp"

#include "cloud_filter.hpp"
#include "dimos_native_module.hpp"
#include "timing.hpp"

// dimos LCM message headers
#include "geometry_msgs/Quaternion.hpp"
#include "geometry_msgs/Vector3.hpp"
#include "nav_msgs/Odometry.hpp"
#include "sensor_msgs/Imu.hpp"
#include "sensor_msgs/PointCloud2.hpp"
#include "sensor_msgs/PointField.hpp"

// FAST-LIO (header-only core, compiled sources linked via CMake)
#include "fast_lio.hpp"
#include "fast_lio_debug.hpp"

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

static double get_publish_ts() {
    return std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

static std::string g_lidar_topic;
static std::string g_odometry_topic;
static std::string g_frame_id;        // required via --frame_id
static std::string g_child_frame_id;   // required via --child_frame_id
static float g_frequency = 10.0f;

// Initial pose offset (applied to all SLAM outputs)
static double g_init_x = 0.0;
static double g_init_y = 0.0;
static double g_init_z = 0.0;
static double g_init_qx = 0.0;
static double g_init_qy = 0.0;
static double g_init_qz = 0.0;
static double g_init_qw = 1.0;

// Hamilton product: q_out = q1 * q2
static void quat_mul(double ax, double ay, double az, double aw,
                     double bx, double by, double bz, double bw,
                     double& ox, double& oy, double& oz, double& ow) {
    ow = aw*bw - ax*bx - ay*by - az*bz;
    ox = aw*bx + ax*bw + ay*bz - az*by;
    oy = aw*by - ax*bz + ay*bw + az*bx;
    oz = aw*bz + ax*by - ay*bx + az*bw;
}

// Rotate vector by quaternion: v_out = q * v * q_inv
static void quat_rotate(double qx, double qy, double qz, double qw,
                        double vx, double vy, double vz,
                        double& ox, double& oy, double& oz) {
    double tx = 2.0 * (qy*vz - qz*vy);
    double ty = 2.0 * (qz*vx - qx*vz);
    double tz = 2.0 * (qx*vy - qy*vx);
    ox = vx + qw*tx + (qy*tz - qz*ty);
    oy = vy + qw*ty + (qz*tx - qx*tz);
    oz = vz + qw*tz + (qx*ty - qy*tx);
}

static bool has_init_pose() {
    return g_init_x != 0.0 || g_init_y != 0.0 || g_init_z != 0.0 ||
           g_init_qx != 0.0 || g_init_qy != 0.0 || g_init_qz != 0.0 || g_init_qw != 1.0;
}

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
// Publish lidar point cloud in the sensor body frame (g_frame_id / mid360_link)
// ---------------------------------------------------------------------------
//
// `cloud` is FAST-LIO's undistorted scan in the sensor's own frame
// (get_body_cloud), so points are published as-is with no world registration.
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

    // x, y, z, intensity (float32 each)
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

    // p_out = R_init * p_slam + t_init
    if (has_init_pose()) {
        double rx, ry, rz;
        quat_rotate(g_init_qx, g_init_qy, g_init_qz, g_init_qw,
                    odom.pose.pose.position.x,
                    odom.pose.pose.position.y,
                    odom.pose.pose.position.z,
                    rx, ry, rz);
        msg.pose.pose.position.x = rx + g_init_x;
        msg.pose.pose.position.y = ry + g_init_y;
        msg.pose.pose.position.z = rz + g_init_z;

        double ox, oy, oz, ow;
        quat_mul(g_init_qx, g_init_qy, g_init_qz, g_init_qw,
                 odom.pose.pose.orientation.x,
                 odom.pose.pose.orientation.y,
                 odom.pose.pose.orientation.z,
                 odom.pose.pose.orientation.w,
                 ox, oy, oz, ow);
        msg.pose.pose.orientation.x = ox;
        msg.pose.pose.orientation.y = oy;
        msg.pose.pose.orientation.z = oz;
        msg.pose.pose.orientation.w = ow;
    } else {
        msg.pose.pose.position.x = odom.pose.pose.position.x;
        msg.pose.pose.position.y = odom.pose.pose.position.y;
        msg.pose.pose.position.z = odom.pose.pose.position.z;
        msg.pose.pose.orientation.x = odom.pose.pose.orientation.x;
        msg.pose.pose.orientation.y = odom.pose.pose.orientation.y;
        msg.pose.pose.orientation.z = odom.pose.pose.orientation.z;
        msg.pose.pose.orientation.w = odom.pose.pose.orientation.w;
    }

    for (int i = 0; i < 36; ++i) {
        msg.pose.covariance[i] = odom.pose.covariance[i];
    }

    // Twist zeroed — FAST-LIO doesn't output velocity.
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

    // Per-point intra-packet offset (matches livox_ros_driver2). Without it all
    // points share one timestamp and per-point deskew is lost. time_interval
    // unit is 0.1us, so *100 → ns.
    const uint64_t point_interval_ns =
        dot_num > 0 ? static_cast<uint64_t>(data->time_interval) * 100 / dot_num : 0;

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
            cp.line = 0;  // Mid-360: single line
            cp.offset_time = static_cast<uli>((ts_ns - g_frame_start_ns) + i * point_interval_ns);
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
            cp.offset_time = static_cast<uli>((ts_ns - g_frame_start_ns) + i * point_interval_ns);
            g_accumulated_points.push_back(cp);
        }
    }
}

static void on_imu_data(const uint32_t /*handle*/, const uint8_t /*dev_type*/,
                        LivoxLidarEthernetPacket* data, void* /*client_data*/) {
    if (!g_running.load() || data == nullptr || !g_fastlio) return;

    uint64_t pkt_ts_ns = get_timestamp_ns(data);
    // Live IMU-drop instrumentation: a dropped datagram shows as a sensor-ts
    // jump; wall gaps exceeding sensor gaps mean callback starvation.
    {
        static std::atomic<uint64_t> last_pkt_ts_ns{0};
        static std::atomic<uint64_t> imu_pkt_count{0};
        static std::atomic<uint64_t> imu_gap_count{0};
        static std::atomic<uint64_t> max_sensor_gap_us{0};
        using clk = std::chrono::steady_clock;
        static auto last_wall = clk::now();
        auto now_wall = clk::now();
        uint64_t prev = last_pkt_ts_ns.exchange(pkt_ts_ns);
        uint64_t n = imu_pkt_count.fetch_add(1) + 1;
        if (prev != 0 && pkt_ts_ns > prev) {
            uint64_t sensor_gap_us = (pkt_ts_ns - prev) / 1000;
            uint64_t wall_gap_us = std::chrono::duration_cast<std::chrono::microseconds>(
                                       now_wall - last_wall).count();
            uint64_t cur_max = max_sensor_gap_us.load();
            while (sensor_gap_us > cur_max &&
                   !max_sensor_gap_us.compare_exchange_weak(cur_max, sensor_gap_us)) {}
            if (sensor_gap_us > 15000) {
                imu_gap_count.fetch_add(1);
                fprintf(stderr, "[imu-gap] sensor_gap=%.1fms wall_gap=%.1fms pkt#%llu\n",
                        sensor_gap_us / 1000.0, wall_gap_us / 1000.0,
                        static_cast<unsigned long long>(n));
            }
        }
        last_wall = now_wall;
        if (n % 1000 == 0) {
            fprintf(stderr, "[imu-stats] pkts=%llu gaps>15ms=%llu max_sensor_gap=%.1fms\n",
                    static_cast<unsigned long long>(n),
                    static_cast<unsigned long long>(imu_gap_count.load()),
                    max_sensor_gap_us.load() / 1000.0);
        }
    }

    double ts = static_cast<double>(pkt_ts_ns) / 1e9;
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

        // Point-LIO expects accel in g (EKF does its own scaling). SDK already
        // reports g, so feed raw — scaling by GRAVITY_MS2 would double-scale and
        // trip the satu_acc check at rest.
        imu_msg->linear_acceleration.x = static_cast<double>(imu_pts[i].acc_x);
        imu_msg->linear_acceleration.y = static_cast<double>(imu_pts[i].acc_y);
        imu_msg->linear_acceleration.z = static_cast<double>(imu_pts[i].acc_z);
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

    if (fastlio_debug) {
        printf("[fastlio2] Device connected: handle=%u type=%u sn=%s ip=%s\n",
               handle, info->dev_type, sn, ip);
    }

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
    g_frame_id = mod.arg_required("frame_id");
    g_child_frame_id = mod.arg_required("odom_frame_id");
    float pointcloud_freq = mod.arg_float("pointcloud_freq", 5.0f);
    float odom_freq = mod.arg_float("odom_freq", 50.0f);
    CloudFilterConfig filter_cfg;
    filter_cfg.voxel_size = mod.arg_float("voxel_size", 0.1f);
    filter_cfg.sor_mean_k = mod.arg_int("sor_mean_k", 50);
    filter_cfg.sor_stddev = mod.arg_float("sor_stddev", 1.0f);

    // Propagates to the FAST-LIO core via the `fastlio_debug` global.
    bool debug = mod.arg_bool("debug", false);
    fastlio_debug = debug;

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

    // Initial pose offset [x, y, z, qx, qy, qz, qw]
    {
        std::string init_str = mod.arg("init_pose", "");
        if (!init_str.empty()) {
            double vals[7] = {0, 0, 0, 0, 0, 0, 1};
            int n = 0;
            size_t pos = 0;
            while (pos < init_str.size() && n < 7) {
                size_t comma = init_str.find(',', pos);
                if (comma == std::string::npos) comma = init_str.size();
                vals[n++] = std::stod(init_str.substr(pos, comma - pos));
                pos = comma + 1;
            }
            g_init_x = vals[0]; g_init_y = vals[1]; g_init_z = vals[2];
            g_init_qx = vals[3]; g_init_qy = vals[4]; g_init_qz = vals[5]; g_init_qw = vals[6];
        }
    }

    if (debug) {
        printf("[fastlio2] Starting FAST-LIO2 + Livox Mid-360 native module\n");
        if (has_init_pose()) {
            printf("[fastlio2] init_pose: xyz=(%.3f, %.3f, %.3f) quat=(%.4f, %.4f, %.4f, %.4f)\n",
                   g_init_x, g_init_y, g_init_z, g_init_qx, g_init_qy, g_init_qz, g_init_qw);
        }
        printf("[fastlio2] lidar topic: %s\n",
               g_lidar_topic.empty() ? "(disabled)" : g_lidar_topic.c_str());
        printf("[fastlio2] odometry topic: %s\n",
               g_odometry_topic.empty() ? "(disabled)" : g_odometry_topic.c_str());
        printf("[fastlio2] config: %s\n", config_path.c_str());
        printf("[fastlio2] host_ip: %s  lidar_ip: %s  frequency: %.1f Hz\n",
               host_ip.c_str(), lidar_ip.c_str(), g_frequency);
        printf("[fastlio2] pointcloud_freq: %.1f Hz  odom_freq: %.1f Hz\n",
               pointcloud_freq, odom_freq);
        printf("[fastlio2] voxel_size: %.3f  sor_mean_k: %d  sor_stddev: %.1f\n",
               filter_cfg.voxel_size, filter_cfg.sor_mean_k, filter_cfg.sor_stddev);
    }

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

    if (debug) printf("[fastlio2] Initializing FAST-LIO...\n");
    FastLio fast_lio(config_path, msr_freq, main_freq);
    g_fastlio = &fast_lio;
    if (debug) printf("[fastlio2] FAST-LIO initialized.\n");

    // Main-loop state. Body lives in `run_main_iter`, driven by the wall-paced
    // main thread.
    auto frame_interval = std::chrono::microseconds(
        static_cast<int64_t>(1e6 / g_frequency));
    std::optional<std::chrono::steady_clock::time_point> last_emit;
    const double process_period_ms = 1000.0 / main_freq;

    auto pc_interval = std::chrono::microseconds(
        static_cast<int64_t>(1e6 / pointcloud_freq));
    auto odom_interval = std::chrono::microseconds(
        static_cast<int64_t>(1e6 / odom_freq));
    std::optional<std::chrono::steady_clock::time_point> last_pc_publish;
    std::optional<std::chrono::steady_clock::time_point> last_odom_publish;


    // Per-section timing for `run_main_iter`, active only with --debug.
    // maybe_flush() below prints a summary every second.
    static timing::Section t_iter{"run_main_iter"};
    static timing::Section t_emit_check{"emit.lock+swap"};
    static timing::Section t_feed_lidar{"fast_lio.feed_lidar"};
    static timing::Section t_process{"fast_lio.process"};
    static timing::Section t_get_world_cloud{"fast_lio.get_body_cloud"};
    static timing::Section t_filter_cloud{"filter_cloud"};
    static timing::Section t_publish_lidar{"publish_lidar"};
    static timing::Section t_publish_odom{"publish_odometry"};

    auto run_main_iter = [&](std::chrono::steady_clock::time_point now) {
        timing::Scope iter_scope(t_iter);
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
        // FAST-LIO. Hold g_pc_mutex across the rate-limit check AND swap so the
        // clock + accumulator are observed atomically (no packet slips between).
        std::vector<custom_messages::CustomPoint> points;
        uint64_t frame_start = 0;
        {
            timing::Scope s(t_emit_check);
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
        if (!points.empty()) {
            auto lidar_msg = boost::make_shared<custom_messages::CustomMsg>();
            lidar_msg->header.seq = 0;
            lidar_msg->header.stamp = custom_messages::Time().fromSec(
                static_cast<double>(frame_start) / 1e9);
            lidar_msg->header.frame_id = "livox_frame";
            lidar_msg->timebase = frame_start;
            lidar_msg->lidar_id = 0;
            for (int i = 0; i < 3; i++) lidar_msg->rsvd[i] = 0;
            lidar_msg->point_num = static_cast<uli>(points.size());
            lidar_msg->points = std::move(points);
            timing::Scope s(t_feed_lidar);
            fast_lio.feed_lidar(lidar_msg);
        }

        // One FAST-LIO IESKF step (cheap when queues empty).
        {
            timing::Scope s(t_process);
            fast_lio.process();
        }

        auto pose = fast_lio.get_pose();
        if (!pose.empty() && (pose[0] != 0.0 || pose[1] != 0.0 || pose[2] != 0.0)) {
            double ts = get_publish_ts();

            const bool lidar_due =
                !g_lidar_topic.empty() && now - *last_pc_publish >= pc_interval;

            // get_body_cloud + filter_cloud (SOR) is the loop's costliest step,
            // so build it only when a publish is due.
            if (lidar_due) {
                auto body_cloud = ([&]() {
                    timing::Scope s(t_get_world_cloud);
                    return fast_lio.get_body_cloud();
                })();
                if (body_cloud && !body_cloud->empty()) {
                    auto filtered = ([&]() {
                        timing::Scope s(t_filter_cloud);
                        return filter_cloud<PointType>(body_cloud, filter_cfg);
                    })();
                    timing::Scope s(t_publish_lidar);
                    publish_lidar(filtered, ts);
                    last_pc_publish = now;
                }
            }

            // Pose + covariance at odom_freq.
            if (!g_odometry_topic.empty() && now - *last_odom_publish >= odom_interval) {
                timing::Scope s(t_publish_odom);
                publish_odometry(fast_lio.get_odometry(), ts);
                last_odom_publish = now;
            }
        }

        timing::maybe_flush(std::chrono::steady_clock::now());
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
    if (debug) printf("[fastlio2] SDK started, waiting for device...\n");

    while (g_running.load()) {
        auto loop_start = std::chrono::high_resolution_clock::now();
        run_main_iter(std::chrono::steady_clock::now());

        lcm.handleTimeout(0);

        // Rate control (~main_freq, 5kHz default).
        auto loop_end = std::chrono::high_resolution_clock::now();
        auto elapsed_ms = std::chrono::duration<double, std::milli>(loop_end - loop_start).count();
        if (elapsed_ms < process_period_ms) {
            std::this_thread::sleep_for(std::chrono::microseconds(
                static_cast<int64_t>((process_period_ms - elapsed_ms) * 1000)));
        }
    }

    // Cleanup
    if (debug) printf("[fastlio2] Shutting down...\n");
    g_fastlio = nullptr;
    LivoxLidarSdkUninit();
    g_lcm = nullptr;

    if (debug) printf("[fastlio2] Done.\n");
    return 0;
}
