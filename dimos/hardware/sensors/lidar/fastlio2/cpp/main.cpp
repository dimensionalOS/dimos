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
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "livox_sdk_config.hpp"

#include "cloud_filter.hpp"
#include "dimos_native_module.hpp"
#include "pcap_replay.hpp"
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

// Virtual clock: in replay mode, tracks the pcap timestamp of the packet
// currently being fed so publish_*() reports the original capture time
// instead of replay wall time. Live mode leaves it at 0 and publish_*()
// falls back to system_clock::now().
static std::atomic<bool> g_replay_mode{false};
static std::atomic<uint64_t> g_virtual_clock_ns{0};

// Deterministic clock mode. When set, both live and replay drive
// g_virtual_clock_ns from the packet's sensor-clock pkt->timestamp (which
// is identical bit-for-bit between SDK delivery and pcap), and use it as
// the source for scan-boundary rate limits and publish timestamps. This
// removes wall-clock jitter from scan boundaries → live and replay produce
// the same algorithm state. Trade-off: published header.stamp values
// become sensor-boot-relative seconds instead of unix wall time, so this
// is off by default and only flipped on by the record/replay demos.
static std::atomic<bool> g_deterministic_clock{false};

// First packet's sensor ts (deterministic mode only). Used to seed the
// main loop's rate-limit bookmarks at exactly the first delivered packet,
// independent of when the main loop's first iteration happens to run.
static std::atomic<uint64_t> g_first_packet_clock_ns{0};

// First-packet marker. Used by record/replay tooling to align the SDK's
// warmup-induced packet drop with replay. The C++ binary writes the wall
// clock of the first on_point_cloud / on_imu_data callback (live mode
// only) to this file; demo_replay reads it back and passes the value as
// --replay_skip_until_ns so pcap_replay drops the same SDK-eaten prefix.
static std::string g_first_packet_marker_path;
static std::atomic<bool> g_first_packet_marker_written{false};

// The packet's sensor-clock timestamp (pkt->timestamp) is identical bit-for-bit
// between the live SDK delivery path and the recorded pcap, so writing it from
// the first SDK callback gives replay an exact boundary to skip on. Wall clock
// would only let us match within delivery latency (sub-ms).
static void mark_first_packet(uint64_t pkt_timestamp_ns) {
    if (g_first_packet_marker_path.empty()) {
        return;
    }
    bool expected = false;
    if (!g_first_packet_marker_written.compare_exchange_strong(expected, true)) {
        return;
    }
    FILE* f = std::fopen(g_first_packet_marker_path.c_str(), "w");
    if (f) {
        std::fprintf(f, "%lu\n", static_cast<unsigned long>(pkt_timestamp_ns));
        std::fclose(f);
    }
}

static double get_publish_ts() {
    if (g_deterministic_clock.load() || g_replay_mode.load()) {
        return static_cast<double>(g_virtual_clock_ns.load()) / 1e9;
    }
    return std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

// Virtualized clock for the main loop's frame/publish rate limiters. In
// replay mode this returns a time_point derived from g_virtual_clock_ns so
// scan boundaries are determined by packet arrival, not by wall-clock thread
// scheduling jitter. Returns nullopt if replay hasn't yet seen a packet
// (caller should skip rate-limit checks in that case).
static std::optional<std::chrono::steady_clock::time_point> virtual_now() {
    if (g_deterministic_clock.load() || g_replay_mode.load()) {
        uint64_t ns = g_virtual_clock_ns.load();
        if (ns == 0) {
            return std::nullopt;
        }
        return std::chrono::steady_clock::time_point(std::chrono::nanoseconds(ns));
    }
    return std::chrono::steady_clock::now();
}

static std::string g_lidar_topic;
static std::string g_odometry_topic;
static std::string g_map_topic;
static std::string g_frame_id;        // required via --frame_id
static std::string g_child_frame_id;   // required via --child_frame_id
static float g_frequency = 10.0f;

// Initial pose offset (applied to all SLAM outputs)
// Position offset
static double g_init_x = 0.0;
static double g_init_y = 0.0;
static double g_init_z = 0.0;
// Orientation offset as quaternion (identity = no rotation)
static double g_init_qx = 0.0;
static double g_init_qy = 0.0;
static double g_init_qz = 0.0;
static double g_init_qw = 1.0;

// Helper: quaternion multiply (Hamilton product)  q_out = q1 * q2
static void quat_mul(double ax, double ay, double az, double aw,
                     double bx, double by, double bz, double bw,
                     double& ox, double& oy, double& oz, double& ow) {
    ow = aw*bw - ax*bx - ay*by - az*bz;
    ox = aw*bx + ax*bw + ay*bz - az*by;
    oy = aw*by - ax*bz + ay*bw + az*bx;
    oz = aw*bz + ax*by - ay*bx + az*bw;
}

// Helper: rotate a vector by a quaternion  v_out = q * v * q_inv
static void quat_rotate(double qx, double qy, double qz, double qw,
                        double vx, double vy, double vz,
                        double& ox, double& oy, double& oz) {
    // t = 2 * cross(q_xyz, v)
    double tx = 2.0 * (qy*vz - qz*vy);
    double ty = 2.0 * (qz*vx - qx*vz);
    double tz = 2.0 * (qx*vy - qy*vx);
    // v_out = v + qw*t + cross(q_xyz, t)
    ox = vx + qw*tx + (qy*tz - qz*ty);
    oy = vy + qw*ty + (qz*tx - qx*tz);
    oz = vz + qw*tz + (qx*ty - qy*tx);
}

// Check if initial pose is non-identity
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

    // Apply the full init_pose transform (rotation + translation) to point clouds.
    // FAST-LIO's map origin is at the sensor's initial position.  The rotation
    // corrects axis direction (e.g. 180° X for upside-down mount) and the
    // translation shifts the origin so that ground sits at z≈0 (e.g. z=1.2
    // for a sensor mounted 1.2m above ground).  This matches the odometry
    // frame, which also gets the full init_pose applied.
    const bool apply_init_pose = has_init_pose();
    for (int i = 0; i < num_points; ++i) {
        float* dst = reinterpret_cast<float*>(pc.data.data() + i * 16);
        if (apply_init_pose) {
            double rx, ry, rz;
            quat_rotate(g_init_qx, g_init_qy, g_init_qz, g_init_qw,
                        cloud->points[i].x, cloud->points[i].y, cloud->points[i].z,
                        rx, ry, rz);
            dst[0] = static_cast<float>(rx + g_init_x);
            dst[1] = static_cast<float>(ry + g_init_y);
            dst[2] = static_cast<float>(rz + g_init_z);
        } else {
            dst[0] = cloud->points[i].x;
            dst[1] = cloud->points[i].y;
            dst[2] = cloud->points[i].z;
        }
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

    // Pose (apply initial pose offset: p_out = R_init * p_slam + t_init)
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

    if (!g_replay_mode.load()) {
        mark_first_packet(ts_ns);
    }

    std::lock_guard<std::mutex> lock(g_pc_mutex);

    // Update the deterministic-mode virtual clock INSIDE the accumulator
    // mutex so the main loop can never observe a clock advance without
    // also seeing the matching points (race that drifts scan composition).
    // Monotonic update: SDK threads can deliver packets slightly out of
    // sensor-ts order, and we must not let a later store(lower_ts) undo
    // a previous store(higher_ts).
    if (g_deterministic_clock.load()) {
        uint64_t expected = 0;
        g_first_packet_clock_ns.compare_exchange_strong(expected, ts_ns);
        uint64_t cur = g_virtual_clock_ns.load();
        while (cur < ts_ns && !g_virtual_clock_ns.compare_exchange_weak(cur, ts_ns)) {}
    }

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

    uint64_t pkt_ts_ns = get_timestamp_ns(data);
    if (!g_replay_mode.load()) {
        mark_first_packet(pkt_ts_ns);
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

        imu_msg->linear_acceleration.x = static_cast<double>(imu_pts[i].acc_x) * GRAVITY_MS2;
        imu_msg->linear_acceleration.y = static_cast<double>(imu_pts[i].acc_y) * GRAVITY_MS2;
        imu_msg->linear_acceleration.z = static_cast<double>(imu_pts[i].acc_z) * GRAVITY_MS2;
        for (int j = 0; j < 9; ++j)
            imu_msg->linear_acceleration_covariance[j] = 0.0;

        g_fastlio->feed_imu(imu_msg);
    }

    // Advance the deterministic-mode virtual clock AFTER feed_imu has
    // queued the sample, holding g_pc_mutex so this is fully serialized
    // with on_point_cloud / the main-loop scan swap. Monotonic CAS so
    // out-of-order SDK delivery can't roll the clock backward.
    if (g_deterministic_clock.load()) {
        std::lock_guard<std::mutex> lock(g_pc_mutex);
        uint64_t expected = 0;
        g_first_packet_clock_ns.compare_exchange_strong(expected, pkt_ts_ns);
        uint64_t cur = g_virtual_clock_ns.load();
        while (cur < pkt_ts_ns && !g_virtual_clock_ns.compare_exchange_weak(cur, pkt_ts_ns)) {}
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
    g_frame_id = mod.arg_required("frame_id");
    g_child_frame_id = mod.arg_required("child_frame_id");
    float pointcloud_freq = mod.arg_float("pointcloud_freq", 5.0f);
    float odom_freq = mod.arg_float("odom_freq", 50.0f);
    CloudFilterConfig filter_cfg;
    filter_cfg.voxel_size = mod.arg_float("voxel_size", 0.1f);
    filter_cfg.sor_mean_k = mod.arg_int("sor_mean_k", 50);
    filter_cfg.sor_stddev = mod.arg_float("sor_stddev", 1.0f);
    float map_voxel_size = mod.arg_float("map_voxel_size", 0.1f);
    float map_max_range = mod.arg_float("map_max_range", 100.0f);
    float map_freq = mod.arg_float("map_freq", 0.0f);

    // Verbose logging — propagates to the FAST-LIO C++ core via the
    // `fastlio_debug` global. Default false → only real errors print.
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

    // Replay mode (offline). When --replay_pcap is given the SDK is not
    // initialized; a feeder thread reads the pcap and calls the existing
    // on_point_cloud / on_imu_data callbacks directly. publish_*() uses
    // the pcap timestamps as the clock so outputs match the original run.
    std::string replay_pcap = mod.arg("replay_pcap", "");
    g_replay_mode.store(!replay_pcap.empty());

    // Drop pcap packets with pcap_ts < this value. Used in replay to mimic
    // the SDK warmup discard that the live run experienced — so the
    // algorithm starts from the same first packet in both modes.
    uint64_t replay_skip_until_ns = 0;
    {
        std::string s = mod.arg("replay_skip_until_ns", "0");
        if (!s.empty()) {
            replay_skip_until_ns = std::stoull(s);
        }
    }

    // Live mode: write the wall_clock_ns of the first SDK callback to this
    // file. Pair with replay's --replay_skip_until_ns to align packet sets.
    g_first_packet_marker_path = mod.arg("first_packet_marker", "");

    // Drive virtual_now() and get_publish_ts() off the packet's sensor
    // clock in both live and replay. Eliminates wall-clock jitter from
    // scan boundaries and makes outputs bit-comparable across modes.
    // Changes header.stamp from unix wall time to sensor-boot seconds.
    g_deterministic_clock.store(mod.arg_bool("deterministic_clock", false));

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

    // Init FAST-LIO with config
    if (debug) printf("[fastlio2] Initializing FAST-LIO...\n");
    FastLio fast_lio(config_path, msr_freq, main_freq);
    g_fastlio = &fast_lio;
    if (debug) printf("[fastlio2] FAST-LIO initialized.\n");

    // Main-loop state. The body lives in `run_main_iter` below so it can be
    // invoked from either the wall-clock-paced main thread (live) or the
    // pcap-paced feeder thread (replay), with no race on accumulator
    // contents in the replay case.
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

    std::unique_ptr<VoxelMap> global_map;
    std::chrono::microseconds map_interval{0};
    std::optional<std::chrono::steady_clock::time_point> last_map_publish;
    if (!g_map_topic.empty() && map_freq > 0.0f) {
        global_map = std::make_unique<VoxelMap>(map_voxel_size, map_max_range);
        map_interval = std::chrono::microseconds(
            static_cast<int64_t>(1e6 / map_freq));
    }

    auto run_main_iter = [&](std::chrono::steady_clock::time_point now) {
        // Lazy-seed all rate-limit bookmarks on the first iteration so they
        // line up with the chosen clock (wall in live, pcap in replay) and
        // don't fire immediately based on an arbitrary "since program start"
        // delta. In deterministic mode we seed from the FIRST packet's
        // sensor ts (not the current ts) so both live and replay anchor
        // their first scan boundary at the same packet — required for
        // bit-for-bit live↔replay parity.
        auto seed = now;
        if (g_deterministic_clock.load()) {
            uint64_t first = g_first_packet_clock_ns.load();
            if (first != 0) {
                seed = std::chrono::steady_clock::time_point(
                    std::chrono::nanoseconds(first));
            }
        }
        if (!last_emit.has_value()) {
            last_emit = seed;
        }
        if (!last_pc_publish.has_value()) {
            last_pc_publish = seed;
        }
        if (!last_odom_publish.has_value()) {
            last_odom_publish = seed;
        }
        if (global_map && !last_map_publish.has_value()) {
            last_map_publish = seed;
        }

        // At frame rate: drain accumulated raw points into a CustomMsg
        // and feed to FAST-LIO. Hold g_pc_mutex across the rate-limit
        // CHECK as well as the swap, so in deterministic mode the
        // virtual clock + accumulator are observed atomically with no
        // other thread able to slip a packet in between the decision
        // and the swap.
        std::vector<custom_messages::CustomPoint> points;
        uint64_t frame_start = 0;
        {
            std::lock_guard<std::mutex> lock(g_pc_mutex);
            auto check_now = now;
            if (g_deterministic_clock.load()) {
                uint64_t ns = g_virtual_clock_ns.load();
                if (ns != 0) {
                    check_now = std::chrono::steady_clock::time_point(
                        std::chrono::nanoseconds(ns));
                }
            }
            if (check_now - *last_emit >= frame_interval) {
                if (!g_accumulated_points.empty()) {
                    points.swap(g_accumulated_points);
                    frame_start = g_frame_start_ns;
                    g_frame_has_timestamp = false;
                }
                last_emit = check_now;
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
            for (int i = 0; i < 3; i++) lidar_msg->rsvd[i] = 0;
            lidar_msg->point_num = static_cast<uli>(points.size());
            lidar_msg->points = std::move(points);
            fast_lio.feed_lidar(lidar_msg);
        }

        // Run one FAST-LIO IESKF step. Cheap when the IMU/lidar queues
        // are empty; the heavy work happens after a feed_lidar above.
        fast_lio.process();

        // Check for new SLAM results and publish (rate-limited).
        auto pose = fast_lio.get_pose();
        if (!pose.empty() && (pose[0] != 0.0 || pose[1] != 0.0 || pose[2] != 0.0)) {
            double ts = get_publish_ts();
            auto world_cloud = fast_lio.get_world_cloud();
            if (world_cloud && !world_cloud->empty()) {
                auto filtered = filter_cloud<PointType>(world_cloud, filter_cfg);

                // Per-scan world-frame cloud at pointcloud_freq.
                if (!g_lidar_topic.empty() && now - *last_pc_publish >= pc_interval) {
                    publish_lidar(filtered, ts);
                    last_pc_publish = now;
                }

                // Global voxel map: insert this scan, prune, then publish
                // a snapshot at map_freq.
                if (global_map) {
                    global_map->insert<PointType>(filtered);
                    if (now - *last_map_publish >= map_interval) {
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

            // Pose + covariance, rate-limited to odom_freq.
            if (!g_odometry_topic.empty() && now - *last_odom_publish >= odom_interval) {
                publish_odometry(fast_lio.get_odometry(), ts);
                last_odom_publish = now;
            }
        }
    };

    // Source of point/IMU packets:
    //   live mode  -> Livox SDK opens UDP sockets + dispatches via callbacks
    //                 and the main thread drives run_main_iter at main_freq.
    //   replay mode -> the feeder thread reads the pcap, calls the same
    //                  callbacks, and invokes run_main_iter inline after
    //                  each packet. SDK is not initialized; main thread
    //                  just keeps LCM alive while the feeder owns processing.
    std::thread replay_thread;
    if (g_replay_mode.load()) {
        if (debug) printf("[fastlio2] REPLAY mode, pcap=%s\n", replay_pcap.c_str());
        replay_thread = std::thread([&]() {
            pcap_replay::Replayer rep;
            rep.path = replay_pcap;
            rep.host_point_port = static_cast<uint16_t>(ports.host_point_data);
            rep.host_imu_port = static_cast<uint16_t>(ports.host_imu_data);
            rep.on_point = [](LivoxLidarEthernetPacket* p) {
                on_point_cloud(0, 0, p, nullptr);
            };
            rep.on_imu = [](LivoxLidarEthernetPacket* p) {
                on_imu_data(0, 0, p, nullptr);
            };
            rep.on_clock = [](uint64_t pcap_ts_ns) {
                // In deterministic mode the callbacks already pushed the
                // sensor pkt->timestamp into g_virtual_clock_ns — don't
                // overwrite with the pcap (wall) ts.
                if (g_deterministic_clock.load()) {
                    return;
                }
                g_virtual_clock_ns.store(pcap_ts_ns);
            };
            rep.on_iter = [&]() {
                auto vn = virtual_now();
                if (vn.has_value()) {
                    run_main_iter(*vn);
                }
            };
            rep.running = &g_running;
            // Serialized replay drives the main loop per packet, so wall
            // pacing isn't needed for correctness — virtual_clock controls
            // rate limits. Run as fast as the algorithm can keep up.
            rep.realtime = false;
            rep.skip_until_ns = replay_skip_until_ns;
            rep.run();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            g_running.store(false);
        });
    } else {
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
    }

    while (g_running.load()) {
        if (g_replay_mode.load()) {
            // Feeder thread owns run_main_iter. Just keep LCM alive and idle.
            lcm.handleTimeout(20);
            continue;
        }

        auto loop_start = std::chrono::high_resolution_clock::now();
        auto now_opt = virtual_now();
        if (!now_opt.has_value()) {
            // No clock yet (live: shouldn't happen; replay: handled above).
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        run_main_iter(*now_opt);

        // Drain LCM messages.
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
    if (replay_thread.joinable()) {
        replay_thread.join();
    }
    if (!g_replay_mode.load()) {
        LivoxLidarSdkUninit();
    }
    g_lcm = nullptr;

    if (debug) printf("[fastlio2] Done.\n");
    return 0;
}
