// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Livox Mid-360 native module for dimos NativeModule framework.
//
// Publishes PointCloud2 and Imu messages on LCM topics received via CLI args.
// Usage: ./mid360_native --lidar <topic> --imu <topic> [--host_ip <ip>] [--lidar_ip <ip>] ...

#include <lcm/lcm-cpp.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "livox_sdk_config.hpp"

#include "dimos_native_module.hpp"

#include "geometry_msgs/Quaternion.hpp"
#include "geometry_msgs/Vector3.hpp"
#include "sensor_msgs/Imu.hpp"
#include "sensor_msgs/PointCloud2.hpp"
#include "sensor_msgs/PointField.hpp"

using livox_common::GRAVITY_MS2;
using livox_common::DATA_TYPE_IMU;
using livox_common::DATA_TYPE_CARTESIAN_HIGH;
using livox_common::DATA_TYPE_CARTESIAN_LOW;

// ---------------------------------------------------------------------------
// Global state
// ---------------------------------------------------------------------------

static std::atomic<bool> g_running{true};
static lcm::LCM* g_lcm = nullptr;
static std::string g_lidar_topic;
static std::string g_imu_topic;
static std::string g_frame_id = "lidar_link";
static std::string g_imu_frame_id = "imu_link";
static float g_frequency = 10.0f;

// Frame accumulator
static std::mutex g_pc_mutex;
static std::vector<float> g_accumulated_xyz;       // interleaved x,y,z
static std::vector<float> g_accumulated_intensity;  // per-point intensity
// Per-point time offsets (ns since frame start) and Livox tag bytes, matching
// what Point-LIO's CustomPoint carries (line is always 0 on the Mid-360).
static std::vector<uint32_t> g_accumulated_offset_ns;
static std::vector<uint8_t> g_accumulated_tag;
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
// Build and publish PointCloud2
// ---------------------------------------------------------------------------

// --point_format selects the wire layout:
//   minimal (default) x,y,z,offset_time                    — 16 B/point
//   full              x,y,z,intensity,offset_time,tag,line — 22 B/point
//   legacy            x,y,z,intensity                      — 16 B/point
// offset_time is uint32 ns since the header stamp; tag/line are the Livox
// per-point bytes (line is always 0 on the Mid-360). Scan-undistorting
// estimators (FAST-LIVO2 etc.) need offset_time.
enum class PointFormat { Full, Minimal, Legacy };
static PointFormat g_point_format = PointFormat::Minimal;

// Full-layout field offsets; minimal packs offset_time at 12 instead.
static constexpr int32_t OFFSET_TIME_OFFSET = 16;
static constexpr int32_t TAG_OFFSET = 20;
static constexpr int32_t LINE_OFFSET = 21;
static constexpr int32_t POINT_STEP = 22;
static constexpr int32_t COMPACT_POINT_STEP = 16;  // minimal and legacy layouts

static void publish_pointcloud(const std::vector<float>& xyz,
                               const std::vector<float>& intensity,
                               const std::vector<uint32_t>& offset_ns,
                               const std::vector<uint8_t>& tag,
                               double timestamp) {
    if (!g_lcm || xyz.empty()) { return; }

    int num_points = static_cast<int>(xyz.size()) / 3;

    sensor_msgs::PointCloud2 pc;
    pc.header = make_header(g_frame_id, timestamp);
    pc.height = 1;
    pc.width = num_points;
    pc.is_bigendian = 0;
    pc.is_dense = 1;

    const PointFormat format = g_point_format;
    const int32_t point_step = format == PointFormat::Full ? POINT_STEP : COMPACT_POINT_STEP;

    auto make_field = [](const std::string& name, int32_t offset, int8_t datatype) {
        sensor_msgs::PointField f;
        f.name = name;
        f.offset = offset;
        f.datatype = datatype;
        f.count = 1;
        return f;
    };

    pc.fields.clear();
    pc.fields.push_back(make_field("x", 0, sensor_msgs::PointField::FLOAT32));
    pc.fields.push_back(make_field("y", 4, sensor_msgs::PointField::FLOAT32));
    pc.fields.push_back(make_field("z", 8, sensor_msgs::PointField::FLOAT32));
    if (format != PointFormat::Minimal) {
        pc.fields.push_back(make_field("intensity", 12, sensor_msgs::PointField::FLOAT32));
    }
    if (format == PointFormat::Full) {
        pc.fields.push_back(
            make_field("offset_time", OFFSET_TIME_OFFSET, sensor_msgs::PointField::UINT32));
        pc.fields.push_back(make_field("tag", TAG_OFFSET, sensor_msgs::PointField::UINT8));
        pc.fields.push_back(make_field("line", LINE_OFFSET, sensor_msgs::PointField::UINT8));
    } else if (format == PointFormat::Minimal) {
        pc.fields.push_back(make_field("offset_time", 12, sensor_msgs::PointField::UINT32));
    }
    pc.fields_length = static_cast<int32_t>(pc.fields.size());

    pc.point_step = point_step;
    pc.row_step = pc.point_step * num_points;

    // Pack point data
    pc.data_length = pc.row_step;
    pc.data.resize(pc.data_length);

    for (int i = 0; i < num_points; ++i) {
        uint8_t* base = pc.data.data() + i * point_step;
        float* dst = reinterpret_cast<float*>(base);
        dst[0] = xyz[i * 3 + 0];
        dst[1] = xyz[i * 3 + 1];
        dst[2] = xyz[i * 3 + 2];
        switch (format) {
            case PointFormat::Full: {
                dst[3] = intensity[i];
                uint32_t offset_value = offset_ns[i];
                std::memcpy(base + OFFSET_TIME_OFFSET, &offset_value, sizeof(uint32_t));
                base[TAG_OFFSET] = tag[i];
                base[LINE_OFFSET] = 0;  // Mid-360: single line
                break;
            }
            case PointFormat::Minimal: {
                uint32_t offset_value = offset_ns[i];
                std::memcpy(base + 12, &offset_value, sizeof(uint32_t));
                break;
            }
            case PointFormat::Legacy:
                dst[3] = intensity[i];
                break;
        }
    }

    g_lcm->publish(g_lidar_topic, &pc);
}

// ---------------------------------------------------------------------------
// SDK callbacks
// ---------------------------------------------------------------------------

static void on_point_cloud(const uint32_t /*handle*/, const uint8_t /*dev_type*/,
                           LivoxLidarEthernetPacket* data, void* /*client_data*/) {
    if (!g_running.load() || data == nullptr) { return; }

    uint64_t ts_ns = get_timestamp_ns(data);
    uint16_t dot_num = data->dot_num;

    // Per-point intra-packet offset (matches livox_ros_driver2 and the pointlio
    // module). time_interval unit is 0.1us, so *100 → ns.
    const uint64_t point_interval_ns =
        dot_num > 0 ? static_cast<uint64_t>(data->time_interval) * 100 / dot_num : 0;

    std::lock_guard<std::mutex> lock(g_pc_mutex);

    if (!g_frame_has_timestamp) {
        g_frame_start_ns = ts_ns;
        g_frame_has_timestamp = true;
    }

    // Clamp instead of wrapping when a UDP packet arrives out of order with a
    // stamp older than the frame start (offset 0 = "at the header stamp").
    const uint64_t packet_offset_ns = ts_ns >= g_frame_start_ns ? ts_ns - g_frame_start_ns : 0;
    // Saturate at uint32 range (~4.29s) so a stalled emit loop degrades to a
    // clamped offset rather than a wrapped one.
    auto saturated_offset = [](uint64_t offset_ns_u64) {
        constexpr uint64_t kMax = std::numeric_limits<uint32_t>::max();
        return static_cast<uint32_t>(offset_ns_u64 < kMax ? offset_ns_u64 : kMax);
    };

    if (data->data_type == DATA_TYPE_CARTESIAN_HIGH) {
        auto* pts = reinterpret_cast<const LivoxLidarCartesianHighRawPoint*>(data->data);
        for (uint16_t i = 0; i < dot_num; ++i) {
            // Livox high-precision coordinates are in mm, convert to meters
            g_accumulated_xyz.push_back(static_cast<float>(pts[i].x) / 1000.0f);
            g_accumulated_xyz.push_back(static_cast<float>(pts[i].y) / 1000.0f);
            g_accumulated_xyz.push_back(static_cast<float>(pts[i].z) / 1000.0f);
            g_accumulated_intensity.push_back(static_cast<float>(pts[i].reflectivity) / 255.0f);
            g_accumulated_offset_ns.push_back(
                saturated_offset(packet_offset_ns + i * point_interval_ns));
            g_accumulated_tag.push_back(pts[i].tag);
        }
    } else if (data->data_type == DATA_TYPE_CARTESIAN_LOW) {
        auto* pts = reinterpret_cast<const LivoxLidarCartesianLowRawPoint*>(data->data);
        for (uint16_t i = 0; i < dot_num; ++i) {
            // Livox low-precision coordinates are in cm, convert to meters
            g_accumulated_xyz.push_back(static_cast<float>(pts[i].x) / 100.0f);
            g_accumulated_xyz.push_back(static_cast<float>(pts[i].y) / 100.0f);
            g_accumulated_xyz.push_back(static_cast<float>(pts[i].z) / 100.0f);
            g_accumulated_intensity.push_back(static_cast<float>(pts[i].reflectivity) / 255.0f);
            g_accumulated_offset_ns.push_back(
                saturated_offset(packet_offset_ns + i * point_interval_ns));
            g_accumulated_tag.push_back(pts[i].tag);
        }
    }
}

static void on_imu_data(const uint32_t /*handle*/, const uint8_t /*dev_type*/,
                        LivoxLidarEthernetPacket* data, void* /*client_data*/) {
    if (!g_running.load() || data == nullptr || !g_lcm) return;
    if (g_imu_topic.empty()) return;

    double ts = get_timestamp_ns(data) / 1e9;
    auto* imu_pts = reinterpret_cast<const LivoxLidarImuRawPoint*>(data->data);
    uint16_t dot_num = data->dot_num;

    for (uint16_t i = 0; i < dot_num; ++i) {
        sensor_msgs::Imu msg;
        msg.header = make_header(g_imu_frame_id, ts);

        // Orientation unknown — set to identity with high covariance
        msg.orientation.x = 0.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 1.0;
        msg.orientation_covariance[0] = -1.0;  // indicates unknown

        msg.angular_velocity.x = static_cast<double>(imu_pts[i].gyro_x);
        msg.angular_velocity.y = static_cast<double>(imu_pts[i].gyro_y);
        msg.angular_velocity.z = static_cast<double>(imu_pts[i].gyro_z);

        msg.linear_acceleration.x = static_cast<double>(imu_pts[i].acc_x) * GRAVITY_MS2;
        msg.linear_acceleration.y = static_cast<double>(imu_pts[i].acc_y) * GRAVITY_MS2;
        msg.linear_acceleration.z = static_cast<double>(imu_pts[i].acc_z) * GRAVITY_MS2;

        g_lcm->publish(g_imu_topic, &msg);
    }
}

static void on_info_change(const uint32_t handle, const LivoxLidarInfo* info,
                           void* /*client_data*/) {
    if (info == nullptr) return;

    char sn[17] = {};
    std::memcpy(sn, info->sn, 16);
    char ip[17] = {};
    std::memcpy(ip, info->lidar_ip, 16);

    printf("[mid360] Device connected: handle=%u type=%u sn=%s ip=%s\n",
           handle, info->dev_type, sn, ip);

    // Set to normal work mode
    SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, nullptr, nullptr);

    // Enable IMU
    if (!g_imu_topic.empty()) {
        EnableLivoxLidarImuData(handle, nullptr, nullptr);
    }
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

    // Required: LCM topics for ports
    g_lidar_topic = mod.has("lidar") ? mod.topic("lidar") : "";
    g_imu_topic = mod.has("imu") ? mod.topic("imu") : "";

    if (g_lidar_topic.empty()) {
        fprintf(stderr, "Error: --lidar <topic> is required\n");
        return 1;
    }

    // Optional config args
    std::string host_ip = mod.arg("host_ip", "192.168.1.5");
    std::string lidar_ip = mod.arg("lidar_ip", "192.168.1.155");
    g_frequency = mod.arg_float("frequency", 10.0f);
    g_frame_id = mod.arg("frame_id", "lidar_link");
    g_imu_frame_id = mod.arg("imu_frame_id", "imu_link");
    const std::string point_format = mod.arg("point_format", "minimal");
    if (point_format == "full") {
        g_point_format = PointFormat::Full;
    } else if (point_format == "minimal") {
        g_point_format = PointFormat::Minimal;
    } else if (point_format == "legacy") {
        g_point_format = PointFormat::Legacy;
    } else {
        fprintf(stderr, "[mid360] unknown --point_format '%s' (full|minimal|legacy)\n",
                point_format.c_str());
        return 1;
    }

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

    printf("[mid360] Starting native Livox Mid-360 module\n");
    printf("[mid360] lidar topic: %s\n", g_lidar_topic.c_str());
    printf("[mid360] imu topic: %s\n", g_imu_topic.empty() ? "(disabled)" : g_imu_topic.c_str());
    printf("[mid360] host_ip: %s  lidar_ip: %s  frequency: %.1f Hz\n",
           host_ip.c_str(), lidar_ip.c_str(), g_frequency);

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

    // Init Livox SDK (in-memory config, no temp files)
    if (!livox_common::init_livox_sdk(host_ip, lidar_ip, ports)) {
        return 1;
    }

    // Register callbacks
    SetLivoxLidarPointCloudCallBack(on_point_cloud, nullptr);
    if (!g_imu_topic.empty()) {
        SetLivoxLidarImuDataCallback(on_imu_data, nullptr);
    }
    SetLivoxLidarInfoChangeCallback(on_info_change, nullptr);

    // Start SDK
    if (!LivoxLidarSdkStart()) {
        fprintf(stderr, "Error: LivoxLidarSdkStart failed\n");
        LivoxLidarSdkUninit();
        return 1;
    }

    printf("[mid360] SDK started, waiting for device...\n");

    // Main loop: periodically emit accumulated point clouds
    auto frame_interval = std::chrono::microseconds(
        static_cast<int64_t>(1e6 / g_frequency));
    auto last_emit = std::chrono::steady_clock::now();

    while (g_running.load()) {
        // Handle LCM (for any subscriptions, though we mostly publish)
        lcm.handleTimeout(10);  // 10ms timeout

        auto now = std::chrono::steady_clock::now();
        if (now - last_emit >= frame_interval) {
            // Swap out the accumulated data
            std::vector<float> xyz;
            std::vector<float> intensity;
            std::vector<uint32_t> offset_ns;
            std::vector<uint8_t> tag;
            double ts = 0.0;

            {
                std::lock_guard<std::mutex> lock(g_pc_mutex);
                if (!g_accumulated_xyz.empty()) {
                    xyz.swap(g_accumulated_xyz);
                    intensity.swap(g_accumulated_intensity);
                    offset_ns.swap(g_accumulated_offset_ns);
                    tag.swap(g_accumulated_tag);
                    // Header stamp = frame start (the timebase offset_time is
                    // relative to), same as the pointlio module.
                    ts = static_cast<double>(g_frame_start_ns) / 1e9;
                    g_frame_has_timestamp = false;
                }
            }

            if (!xyz.empty()) {
                publish_pointcloud(xyz, intensity, offset_ns, tag, ts);
            }

            last_emit = now;
        }
    }

    // Cleanup
    printf("[mid360] Shutting down...\n");
    LivoxLidarSdkUninit();
    g_lcm = nullptr;

    printf("[mid360] Done.\n");
    return 0;
}
