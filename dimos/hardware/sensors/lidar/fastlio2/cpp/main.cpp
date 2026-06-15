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

// Clock for the main loop's frame/publish rate limiters. Returns an optional
// for backward-compatibility with callers that check has_value(); the live
// path always populates it.
static std::optional<std::chrono::steady_clock::time_point> virtual_now() {
    return std::chrono::steady_clock::now();
}

static std::string g_lidar_topic;
static std::string g_odometry_topic;
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

    uint64_t pkt_ts_ns = get_timestamp_ns(data);

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

    // Post-IESKF-update velocity cap. When |v_world| exceeds this value
    // the EKF state is restored to the last accepted scan with vel=0 and
    // map_incremental is skipped. Breaks the reinforcing-loop divergence
    // that gives FAST-LIO multi-km/s velocity runaway on aggressive
    // motion or large IMU gaps. Zero disables. Defaults sized to the
    // Go2 quadruped envelope (~3.1 m/s); raise for faster platforms.
    double max_velocity_norm_ms = mod.arg_float("max_velocity_norm_ms", 0.0f);

    // Livox hardware config
    std::string host_ip = mod.arg("host_ip", "192.168.1.5");
    std::string lidar_ip = mod.arg("lidar_ip", "192.168.1.155");
    g_frequency = mod.arg_float("frequency", 10.0f);
    g_frame_id = mod.arg_required("frame_id");
    g_child_frame_id = mod.arg_required("child_frame_id");
    float pointcloud_freq = mod.arg_float("pointcloud_freq", 5.0f);
    float odom_freq = mod.arg_float("odom_freq", 50.0f);

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

    // Main-loop state. The body lives in `run_main_iter` below, driven by the
    // wall-clock-paced main thread.
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

    // Per-section timing counters for `run_main_iter`. Active only when
    // --debug is on (Scope's constructor reads `fastlio_debug` and no-ops
    // otherwise). `timing::maybe_flush(now)` at the bottom prints a per-
    // section summary every second of wall clock so we can see both how
    // often each part fires and how long each call takes.
    static timing::Section t_iter{"run_main_iter"};
    static timing::Section t_emit_check{"emit.lock+swap"};
    static timing::Section t_feed_lidar{"fast_lio.feed_lidar"};
    static timing::Section t_process{"fast_lio.process"};
    static timing::Section t_get_world_cloud{"fast_lio.get_world_cloud"};
    static timing::Section t_publish_lidar{"publish_lidar"};
    static timing::Section t_publish_odom{"publish_odometry"};

    auto run_main_iter = [&](std::chrono::steady_clock::time_point now) {
        timing::Scope iter_scope(t_iter);
        // Lazy-seed all rate-limit bookmarks on the first iteration so they
        // line up with the wall clock and don't fire immediately based on an
        // arbitrary "since program start" delta.
        auto seed = now;
        if (!last_emit.has_value()) {
            last_emit = seed;
        }
        if (!last_pc_publish.has_value()) {
            last_pc_publish = seed;
        }
        if (!last_odom_publish.has_value()) {
            last_odom_publish = seed;
        }

        // At frame rate: drain accumulated raw points into a CustomMsg
        // and feed to FAST-LIO. Hold g_pc_mutex across the rate-limit
        // CHECK as well as the swap so the accumulator is observed
        // atomically with no other thread able to slip a packet in
        // between the decision and the swap.
        std::vector<custom_messages::CustomPoint> points;
        uint64_t frame_start = 0;
        {
            timing::Scope s(t_emit_check);
            std::lock_guard<std::mutex> lock(g_pc_mutex);
            auto check_now = now;
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
            timing::Scope s(t_feed_lidar);
            fast_lio.feed_lidar(lidar_msg);
        }

        // Run one FAST-LIO IESKF step. Cheap when the IMU/lidar queues
        // are empty; the heavy work happens after a feed_lidar above.
        {
            timing::Scope s(t_process);
            fast_lio.process();
        }

        // Check for new SLAM results and publish (rate-limited).
        auto pose = fast_lio.get_pose();
        if (!pose.empty() && (pose[0] != 0.0 || pose[1] != 0.0 || pose[2] != 0.0)) {
            double ts = get_publish_ts();
            auto world_cloud = ([&]() {
                timing::Scope s(t_get_world_cloud);
                return fast_lio.get_world_cloud();
            })();
            if (world_cloud && !world_cloud->empty()) {
                // Per-scan world-frame cloud at pointcloud_freq, published as-is
                // (no output-cloud voxel downsampling / outlier removal).
                if (!g_lidar_topic.empty() && now - *last_pc_publish >= pc_interval) {
                    timing::Scope s(t_publish_lidar);
                    publish_lidar(world_cloud, ts);
                    last_pc_publish = now;
                }
            }

            // Pose + covariance, rate-limited to odom_freq.
            if (!g_odometry_topic.empty() && now - *last_odom_publish >= odom_interval) {
                timing::Scope s(t_publish_odom);
                publish_odometry(fast_lio.get_odometry(), ts);
                last_odom_publish = now;
            }
        }

        // Periodic per-section summary to stderr (no-op when --debug off).
        timing::maybe_flush(std::chrono::steady_clock::now());
    };

    // Source of point/IMU packets: the Livox SDK opens UDP sockets and
    // dispatches via callbacks from its own threads. The main thread drives
    // run_main_iter at main_freq, consuming whatever the SDK queued.
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
        auto now_opt = virtual_now();
        if (!now_opt.has_value()) {
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
    LivoxLidarSdkUninit();
    g_lcm = nullptr;

    if (debug) printf("[fastlio2] Done.\n");
    return 0;
}
