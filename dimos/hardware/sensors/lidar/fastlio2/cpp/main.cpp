// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// FAST-LIO2 + Livox Mid-360 native module for dimos NativeModule framework.
//
// Binds Livox SDK2 directly into FAST-LIO-NON-ROS: SDK callbacks feed
// CustomMsg/Imu to FastLio, which performs EKF-LOAM SLAM.  Sensor/body-frame
// point clouds and odometry are published on LCM (consumers register the cloud
// via the odometry pose).
//
// Usage:
//   ./fastlio2_native \
//       --lidar '/lidar#sensor_msgs.PointCloud2' \
//       --odometry '/odometry#nav_msgs.Odometry' \
//       --acc_cov 1.0 --filter_size_surf 0.1 ... \   # tuning as plain CLI args
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

#include "dimos_native_module.hpp"

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

// Global state

static std::atomic<bool> g_running{true};
static lcm::LCM* g_lcm = nullptr;
static FastLio* g_fastlio = nullptr;

static std::string g_lidar_topic;
static std::string g_odometry_topic;
static std::string g_frame_id;  // required via --frame_id
static std::string g_sensor_frame_id;  // required via --sensor_frame_id
static float g_frequency = 10.0f;

// Frame accumulator (Livox SDK raw → CustomMsg)
static std::mutex g_pc_mutex;
static std::vector<custom_messages::CustomPoint> g_accumulated_points;
static uint64_t g_frame_start_ns = 0;
static bool g_frame_has_timestamp = false;

// Helpers

static uint64_t get_timestamp_ns(const LivoxLidarEthernetPacket* pkt) {
    uint64_t ns = 0;
    std::memcpy(&ns, pkt->timestamp, sizeof(uint64_t));
    return ns;
}

using dimos::time_from_seconds;
using dimos::make_header;

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

// Publish a lidar point cloud, stamped with `frame_id`.

static void publish_lidar(PointCloudXYZI::Ptr cloud, double timestamp, const std::string& frame_id, const std::string& topic = "") {
    const std::string& chan = topic.empty() ? g_lidar_topic : topic;
    if (!g_lcm || !cloud || cloud->empty() || chan.empty()) { return; }

    int num_points = static_cast<int>(cloud->size());

    sensor_msgs::PointCloud2 pc;
    pc.header = make_header(frame_id, timestamp);
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

// Publish odometry

static void publish_odometry(const custom_messages::Odometry& odom, double timestamp) {
    if (!g_lcm) { return; }

    nav_msgs::Odometry msg;
    msg.header = make_header(g_frame_id, timestamp);
    msg.child_frame_id = g_sensor_frame_id;

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


// Livox SDK callbacks

static void on_point_cloud(const uint32_t /*handle*/, const uint8_t /*dev_type*/, LivoxLidarEthernetPacket* data, void* /*client_data*/) {
    if (!g_running.load() || data == nullptr) { return; }

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
            cp.x = static_cast<double>(pts[i].x) / 1000.0;  // mm → m
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
            cp.x = static_cast<double>(pts[i].x) / 100.0;  // cm → m
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

static void on_imu_data(const uint32_t /*handle*/, const uint8_t /*dev_type*/, LivoxLidarEthernetPacket* data, void* /*client_data*/) {
    if (!g_running.load() || data == nullptr || !g_fastlio) { return; }

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
        for (int j = 0; j < 9; ++j) { imu_msg->orientation_covariance[j] = 0.0; }

        imu_msg->angular_velocity.x = static_cast<double>(imu_pts[i].gyro_x);
        imu_msg->angular_velocity.y = static_cast<double>(imu_pts[i].gyro_y);
        imu_msg->angular_velocity.z = static_cast<double>(imu_pts[i].gyro_z);
        for (int j = 0; j < 9; ++j) { imu_msg->angular_velocity_covariance[j] = 0.0; }

        imu_msg->linear_acceleration.x = static_cast<double>(imu_pts[i].acc_x) * GRAVITY_MS2;
        imu_msg->linear_acceleration.y = static_cast<double>(imu_pts[i].acc_y) * GRAVITY_MS2;
        imu_msg->linear_acceleration.z = static_cast<double>(imu_pts[i].acc_z) * GRAVITY_MS2;
        for (int j = 0; j < 9; ++j) { imu_msg->linear_acceleration_covariance[j] = 0.0; }

        g_fastlio->feed_imu(imu_msg);
    }
}

static void on_info_change(const uint32_t handle, const LivoxLidarInfo* info, void* /*client_data*/) {
    if (info == nullptr) { return; }

    char sn[17] = {};
    std::memcpy(sn, info->sn, 16);
    char ip[17] = {};
    std::memcpy(ip, info->lidar_ip, 16);

    if (fastlio_debug) {
        printf("[fastlio2] Device connected: handle=%u type=%u sn=%s ip=%s\n", handle, info->dev_type, sn, ip);
    }

    SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, nullptr, nullptr);
    EnableLivoxLidarImuData(handle, nullptr, nullptr);
}

// Signal handling

static void signal_handler(int /*sig*/) {
    g_running.store(false);
}

// Main

// One iteration of the main loop: drain accumulated points into a CustomMsg,
// run a FAST-LIO step, and publish results (rate-limited by the bookmarks).
static void run_main_iter(
    std::chrono::steady_clock::time_point now,
    FastLio& fast_lio,
    std::chrono::steady_clock::time_point& last_emit,
    std::chrono::steady_clock::time_point& last_pc_publish,
    std::chrono::steady_clock::time_point& last_odom_publish,
    std::chrono::microseconds frame_interval,
    std::chrono::microseconds pc_interval,
    std::chrono::microseconds odom_interval,
    bool scan_publish_en,
    bool dense_publish_en
) {
    // At frame rate, drain accumulated raw points into a CustomMsg and feed
    // FAST-LIO. Hold g_pc_mutex across the rate-limit check + swap so a
    // callback can't slip a packet in between the decision and the swap.
    std::vector<custom_messages::CustomPoint> points;
    uint64_t frame_start = 0;
    {
        std::lock_guard<std::mutex> lock(g_pc_mutex);
        if (now - last_emit >= frame_interval) {
            if (!g_accumulated_points.empty()) {
                points.swap(g_accumulated_points);
                frame_start = g_frame_start_ns;
                g_frame_has_timestamp = false;
            }
            last_emit = now;
        }
    }
    if (!points.empty()) {
        // Build CustomMsg
        auto lidar_msg = boost::make_shared<custom_messages::CustomMsg>();
        lidar_msg->header.seq = 0;
        lidar_msg->header.stamp = custom_messages::Time().fromSec(static_cast<double>(frame_start) / 1e9);
        lidar_msg->header.frame_id = "livox_frame";
        lidar_msg->timebase = frame_start;
        lidar_msg->lidar_id = 0;
        for (int i = 0; i < 3; i++) { lidar_msg->rsvd[i] = 0; }
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
        double ts = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
        if (scan_publish_en && !g_lidar_topic.empty() && now - last_pc_publish >= pc_interval) {
            // Sensor-frame cloud; register downstream via the odom pose.
            // dense_publish_en false -> FAST-LIO's IESKF-downsampled scan.
            auto cloud = dense_publish_en ? fast_lio.get_body_cloud() : fast_lio.get_body_cloud_down();
            if (cloud && !cloud->empty()) {
                publish_lidar(cloud, ts, g_sensor_frame_id);
            }
            last_pc_publish = now;
        }

        // Pose + covariance, rate-limited to odom_freq.
        if (!g_odometry_topic.empty() && now - last_odom_publish >= odom_interval) {
            publish_odometry(fast_lio.get_odometry(), ts);
            last_odom_publish = now;
        }
    }
}

int main(int argc, char** argv) {
    dimos::NativeModule mod(argc, argv);

    // Required: LCM topics for output ports
    g_lidar_topic = mod.has("lidar") ? mod.topic("lidar") : "";
    g_odometry_topic = mod.has("odometry") ? mod.topic("odometry") : "";

    if (g_lidar_topic.empty() && g_odometry_topic.empty()) {
        fprintf(stderr, "Error: at least one of --lidar or --odometry is required\n");
        return 1;
    }

    // FAST-LIO tuning, passed as CLI args by the dimos module (no YAML).
    FastLioParams params;
    params.acc_cov = mod.arg_float("acc_cov", params.acc_cov);
    params.gyr_cov = mod.arg_float("gyr_cov", params.gyr_cov);
    params.b_acc_cov = mod.arg_float("b_acc_cov", params.b_acc_cov);
    params.b_gyr_cov = mod.arg_float("b_gyr_cov", params.b_gyr_cov);
    params.filter_size_surf = mod.arg_float("filter_size_surf", params.filter_size_surf);
    params.filter_size_map = mod.arg_float("filter_size_map", params.filter_size_map);
    params.det_range = mod.arg_float("det_range", params.det_range);
    params.blind = mod.arg_float("blind", params.blind);
    params.time_offset_lidar_to_imu = mod.arg_float("time_offset_lidar_to_imu", params.time_offset_lidar_to_imu);
    params.fov_degree = mod.arg_int("fov_degree", params.fov_degree);
    params.scan_line = mod.arg_int("scan_line", params.scan_line);
    params.scan_rate = mod.arg_int("scan_rate", params.scan_rate);
    params.time_sync_en = mod.arg_bool("time_sync_en", params.time_sync_en);
    params.extrinsic_est_en = mod.arg_bool("extrinsic_est_en", params.extrinsic_est_en);
    std::string lidar_type = mod.arg("lidar_type", "livox");
    params.lidar_type = lidar_type == "velodyne" ? 2 : lidar_type == "ouster" ? 3 : 1;
    std::string ts_unit = mod.arg("timestamp_unit", "microsecond");
    params.timestamp_unit = ts_unit == "second" ? 0 : ts_unit == "millisecond" ? 1 : ts_unit == "nanosecond" ? 3 : 2;
    if (auto et = parse_doubles(mod.arg("extrinsic_t", "")); !et.empty()) { params.extrinsic_T = et; }
    if (auto er = parse_doubles(mod.arg("extrinsic_r", "")); !er.empty()) { params.extrinsic_R = er; }

    // FAST-LIO internal processing rates
    double msr_freq = mod.arg_float("msr_freq", 50.0f);
    double main_freq = mod.arg_float("main_freq", 5000.0f);

    // Livox hardware config
    std::string host_ip = mod.arg("host_ip", "192.168.1.5");
    std::string lidar_ip = mod.arg("lidar_ip", "192.168.1.155");
    g_frequency = mod.arg_float("frequency", 10.0f);
    g_frame_id = mod.arg_required("frame_id");
    g_sensor_frame_id = mod.arg_required("sensor_frame_id");
    float pointcloud_freq = mod.arg_float("pointcloud_freq", 5.0f);
    float odom_freq = mod.arg_float("odom_freq", 50.0f);

    // Cloud-publish behaviour: scan_publish_en gates the lidar output;
    // dense_publish_en false voxel-downsamples the published cloud.
    bool scan_publish_en = mod.arg_bool("scan_publish_en", true);
    bool dense_publish_en = mod.arg_bool("dense_publish_en", true);

    // Verbose logging — propagates to the FAST-LIO C++ core via the
    // `fastlio_debug` global. Default false → only real errors print.
    bool debug = mod.arg_bool("debug", false);
    fastlio_debug = debug;

    // SDK network ports (defaults from SdkPorts struct in livox_sdk_config.hpp)
    livox_common::SdkPorts ports;
    const livox_common::SdkPorts port_defaults;
    ports.cmd_data = mod.arg_int("cmd_data_port", port_defaults.cmd_data);
    ports.push_msg = mod.arg_int("push_msg_port", port_defaults.push_msg);
    ports.point_data = mod.arg_int("point_data_port", port_defaults.point_data);
    ports.imu_data = mod.arg_int("imu_data_port", port_defaults.imu_data);
    ports.log_data = mod.arg_int("log_data_port", port_defaults.log_data);
    ports.host_cmd_data = mod.arg_int("host_cmd_data_port", port_defaults.host_cmd_data);
    ports.host_push_msg = mod.arg_int("host_push_msg_port", port_defaults.host_push_msg);
    ports.host_point_data = mod.arg_int("host_point_data_port", port_defaults.host_point_data);
    ports.host_imu_data = mod.arg_int("host_imu_data_port", port_defaults.host_imu_data);
    ports.host_log_data = mod.arg_int("host_log_data_port", port_defaults.host_log_data);

    if (debug) {
        printf("[fastlio2] Starting FAST-LIO2 + Livox Mid-360 native module\n");
        printf("[fastlio2] lidar topic: %s\n", g_lidar_topic.empty() ? "(disabled)" : g_lidar_topic.c_str());
        printf("[fastlio2] odometry topic: %s\n", g_odometry_topic.empty() ? "(disabled)" : g_odometry_topic.c_str());
        printf("[fastlio2] acc_cov: %.3f  filter_size_surf: %.3f\n", params.acc_cov, params.filter_size_surf);
        printf("[fastlio2] host_ip: %s  lidar_ip: %s  frequency: %.1f Hz\n", host_ip.c_str(), lidar_ip.c_str(), g_frequency);
        printf("[fastlio2] pointcloud_freq: %.1f Hz  odom_freq: %.1f Hz\n", pointcloud_freq, odom_freq);
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
    if (debug) { printf("[fastlio2] Initializing FAST-LIO...\n"); }
    FastLio fast_lio(params, msr_freq, main_freq);
    g_fastlio = &fast_lio;
    if (debug) { printf("[fastlio2] FAST-LIO initialized.\n"); }

    // Main-loop rate-limit state (consumed by the loop below).
    auto frame_interval = std::chrono::microseconds(static_cast<int64_t>(1e6 / g_frequency));
    const double process_period_ms = 1000.0 / main_freq;

    auto pc_interval = std::chrono::microseconds(static_cast<int64_t>(1e6 / pointcloud_freq));
    auto odom_interval = std::chrono::microseconds(static_cast<int64_t>(1e6 / odom_freq));

    // Rate-limit bookmarks, seeded to now so they don't all fire on iteration 1.
    auto last_emit = std::chrono::steady_clock::now();
    auto last_pc_publish = last_emit;
    auto last_odom_publish = last_emit;

    // The Livox SDK opens UDP sockets and dispatches via its own callback
    // threads; the main loop below consumes what's queued.
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
    if (debug) { printf("[fastlio2] SDK started, waiting for device...\n"); }

    while (g_running.load()) {
        auto loop_start = std::chrono::high_resolution_clock::now();
        auto now = std::chrono::steady_clock::now();
        run_main_iter(
            now,
            fast_lio,
            last_emit,
            last_pc_publish,
            last_odom_publish,
            frame_interval,
            pc_interval,
            odom_interval,
            scan_publish_en,
            dense_publish_en
        );

        // Drain LCM messages.
        lcm.handleTimeout(0);

        // Rate control (~main_freq, 5kHz default).
        auto loop_end = std::chrono::high_resolution_clock::now();
        auto elapsed_ms = std::chrono::duration<double, std::milli>(loop_end - loop_start).count();
        if (elapsed_ms < process_period_ms) {
            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int64_t>((process_period_ms - elapsed_ms) * 1000)));
        }
    }

    // Cleanup. Uninit the SDK (stops + joins its callback threads) BEFORE
    // clearing the globals the callbacks read, so a late on_point/on_imu can't
    // race the assignment and dereference a null g_fastlio / g_lcm.
    if (debug) { printf("[fastlio2] Shutting down...\n"); }
    LivoxLidarSdkUninit();
    g_fastlio = nullptr;
    g_lcm = nullptr;

    if (debug) { printf("[fastlio2] Done.\n"); }
    return 0;
}
