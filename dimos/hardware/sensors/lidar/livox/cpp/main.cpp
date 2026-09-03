// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Livox Mid-360 native module. Drives Livox SDK2, accumulates points from its
// callbacks, and publishes a PointCloud2 at a fixed rate on lidar plus Imu
// samples on imu. No inputs, so it overrides handle() with its own emit loop.

#include <chrono>
#include <cstdint>
#include <cstring>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "livox_sdk_config.hpp"
#include "point_cloud_utils.hpp"

#include "dimos/native.hpp"

#include "sensor_msgs/Imu.hpp"
#include "sensor_msgs/PointCloud2.hpp"
#include "sensor_msgs/PointField.hpp"
#include "std_msgs/Header.hpp"

using dimos::native::Builder;
using dimos::native::Config;
using dimos::native::Module;
using dimos::native::Output;
namespace logging = dimos::native::log;

using livox_common::GRAVITY_MS2;
using livox_common::DATA_TYPE_CARTESIAN_HIGH;
using livox_common::DATA_TYPE_CARTESIAN_LOW;

struct Mid360Config {
    std::string host_ip;
    std::string lidar_ip;
    double frequency;
    bool enable_imu;
    std::string point_format;
    std::string frame_id;
    std::string imu_frame_id;
    int cmd_data_port;
    int push_msg_port;
    int point_data_port;
    int imu_data_port;
    int log_data_port;
    int host_cmd_data_port;
    int host_push_msg_port;
    int host_point_data_port;
    int host_imu_data_port;
    int host_log_data_port;

    void validate() const {
        dimos::native::require_positive(frequency, "frequency");
    }
};

namespace {

using dimos::make_header;

// How often the emit loop checks whether the frame interval has elapsed.
constexpr std::chrono::milliseconds kEmitPollInterval{10};

// Wire layout per point. offset_time is uint32 ns since the header stamp; tag
// and line are the Livox per-point bytes (line is always 0 on the Mid-360).
// Scan-undistorting estimators (FAST-LIVO2 etc.) need offset_time, and no LIO
// in the stack reads intensity, so minimal is the default.
//   Minimal x,y,z,offset_time                    — 16 B/point
//   Full    x,y,z,intensity,offset_time,tag,line — 22 B/point
//   Legacy  x,y,z,intensity                      — 16 B/point
enum class PointFormat { Full, Minimal, Legacy };

// Full-layout field offsets; minimal packs offset_time at 12 instead.
constexpr int32_t kOffsetTimeOffset = 16;
constexpr int32_t kTagOffset = 20;
constexpr int32_t kLineOffset = 21;
constexpr int32_t kFullPointStep = 22;
constexpr int32_t kCompactPointStep = 16;  // minimal and legacy layouts

PointFormat parse_point_format(const std::string& name) {
    if (name == "full") return PointFormat::Full;
    if (name == "minimal") return PointFormat::Minimal;
    if (name == "legacy") return PointFormat::Legacy;
    throw std::runtime_error("point_format must be full, minimal or legacy, got '" + name + "'");
}

uint64_t packet_timestamp_ns(const LivoxLidarEthernetPacket* pkt) {
    uint64_t ns = 0;
    std::memcpy(&ns, pkt->timestamp, sizeof(uint64_t));
    return ns;
}

double packet_timestamp(const LivoxLidarEthernetPacket* pkt) {
    return static_cast<double>(packet_timestamp_ns(pkt)) / 1e9;
}

// Saturate at the uint32 range (~4.29 s) so a stalled emit loop degrades to a
// clamped offset rather than a wrapped one.
uint32_t saturated_offset(uint64_t offset_ns) {
    constexpr uint64_t kMax = std::numeric_limits<uint32_t>::max();
    return static_cast<uint32_t>(offset_ns < kMax ? offset_ns : kMax);
}

sensor_msgs::PointCloud2 make_cloud(PointFormat format, const std::string& frame_id, double ts,
                                    int num_points) {
    sensor_msgs::PointCloud2 pc;
    pc.header = make_header(frame_id, ts);
    pc.height = 1;
    pc.width = num_points;
    pc.is_bigendian = 0;
    pc.is_dense = 1;

    auto make_field = [](const std::string& name, int32_t offset, int8_t datatype) {
        sensor_msgs::PointField f;
        f.name = name;
        f.offset = offset;
        f.datatype = datatype;
        f.count = 1;
        return f;
    };

    pc.fields.push_back(make_field("x", 0, sensor_msgs::PointField::FLOAT32));
    pc.fields.push_back(make_field("y", 4, sensor_msgs::PointField::FLOAT32));
    pc.fields.push_back(make_field("z", 8, sensor_msgs::PointField::FLOAT32));
    if (format != PointFormat::Minimal) {
        pc.fields.push_back(make_field("intensity", 12, sensor_msgs::PointField::FLOAT32));
    }
    if (format == PointFormat::Full) {
        pc.fields.push_back(
            make_field("offset_time", kOffsetTimeOffset, sensor_msgs::PointField::UINT32));
        pc.fields.push_back(make_field("tag", kTagOffset, sensor_msgs::PointField::UINT8));
        pc.fields.push_back(make_field("line", kLineOffset, sensor_msgs::PointField::UINT8));
    } else if (format == PointFormat::Minimal) {
        pc.fields.push_back(make_field("offset_time", 12, sensor_msgs::PointField::UINT32));
    }
    pc.fields_length = static_cast<int32_t>(pc.fields.size());

    pc.point_step = format == PointFormat::Full ? kFullPointStep : kCompactPointStep;
    pc.row_step = pc.point_step * num_points;
    pc.data_length = pc.row_step;
    pc.data.resize(pc.data_length);
    return pc;
}

}  // namespace

class Mid360 : public Module {
public:
    void build(Builder& builder, Config& config) override {
        cfg_ = config.parse<Mid360Config>();
        format_ = parse_point_format(cfg_.point_format);
        lidar_ = builder.output<sensor_msgs::PointCloud2>("lidar");
        if (cfg_.enable_imu) {
            imu_ = builder.output<sensor_msgs::Imu>("imu");
        }
        frame_interval_ =
            std::chrono::microseconds(static_cast<int64_t>(1e6 / cfg_.frequency));
    }

    void setup() override {
        livox_common::SdkPorts ports;
        ports.cmd_data = cfg_.cmd_data_port;
        ports.push_msg = cfg_.push_msg_port;
        ports.point_data = cfg_.point_data_port;
        ports.imu_data = cfg_.imu_data_port;
        ports.log_data = cfg_.log_data_port;
        ports.host_cmd_data = cfg_.host_cmd_data_port;
        ports.host_push_msg = cfg_.host_push_msg_port;
        ports.host_point_data = cfg_.host_point_data_port;
        ports.host_imu_data = cfg_.host_imu_data_port;
        ports.host_log_data = cfg_.host_log_data_port;

        if (!livox_common::init_livox_sdk(cfg_.host_ip, cfg_.lidar_ip, ports)) {
            throw std::runtime_error("init_livox_sdk failed");
        }

        SetLivoxLidarPointCloudCallBack(&Mid360::point_cloud_cb, this);
        if (cfg_.enable_imu) {
            SetLivoxLidarImuDataCallback(&Mid360::imu_cb, this);
        }
        SetLivoxLidarInfoChangeCallback(&Mid360::info_cb, this);

        if (!LivoxLidarSdkStart()) {
            LivoxLidarSdkUninit();
            throw std::runtime_error("LivoxLidarSdkStart failed");
        }
        logging::info("mid360 SDK started, waiting for device",
                      {logging::Field("lidar_ip", cfg_.lidar_ip),
                       logging::Field("host_ip", cfg_.host_ip)});
    }

    // Own emit loop: swap out the accumulated frame and publish at frequency.
    void handle() override {
        auto last_emit = std::chrono::steady_clock::now();
        while (!shutdown_requested()) {
            std::this_thread::sleep_for(kEmitPollInterval);
            auto now = std::chrono::steady_clock::now();
            if (now - last_emit >= frame_interval_) {
                emit_frame();
                last_emit = now;
            }
        }
    }

    void teardown() override {
        LivoxLidarSdkUninit();
    }

private:
    static void point_cloud_cb(const uint32_t, const uint8_t,
                               LivoxLidarEthernetPacket* data, void* ctx) {
        static_cast<Mid360*>(ctx)->on_point_cloud(data);
    }

    static void imu_cb(const uint32_t, const uint8_t, LivoxLidarEthernetPacket* data,
                       void* ctx) {
        static_cast<Mid360*>(ctx)->on_imu(data);
    }

    static void info_cb(const uint32_t handle, const LivoxLidarInfo* info, void* ctx) {
        static_cast<Mid360*>(ctx)->on_info(handle, info);
    }

    void on_point_cloud(LivoxLidarEthernetPacket* data) {
        if (shutdown_requested() || data == nullptr) return;

        uint64_t ts_ns = packet_timestamp_ns(data);
        uint16_t dot_num = data->dot_num;

        // Per-point intra-packet spacing, matching livox_ros_driver2 and the
        // pointlio module. time_interval is in units of 0.1 us, so *100 -> ns.
        const uint64_t point_interval_ns =
            dot_num > 0 ? static_cast<uint64_t>(data->time_interval) * 100 / dot_num : 0;

        std::lock_guard<std::mutex> lock(pc_mutex_);
        if (!frame_has_ts_) {
            frame_start_ns_ = ts_ns;
            frame_has_ts_ = true;
        }

        // Clamp rather than wrap when a UDP packet arrives out of order with a
        // stamp older than the frame start (offset 0 = "at the header stamp").
        const uint64_t packet_offset_ns =
            ts_ns >= frame_start_ns_ ? ts_ns - frame_start_ns_ : 0;

        if (data->data_type == DATA_TYPE_CARTESIAN_HIGH) {
            auto* pts = reinterpret_cast<const LivoxLidarCartesianHighRawPoint*>(data->data);
            for (uint16_t i = 0; i < dot_num; ++i) {
                // High-precision coordinates are in mm.
                xyz_.push_back(static_cast<float>(pts[i].x) / 1000.0f);
                xyz_.push_back(static_cast<float>(pts[i].y) / 1000.0f);
                xyz_.push_back(static_cast<float>(pts[i].z) / 1000.0f);
                intensity_.push_back(static_cast<float>(pts[i].reflectivity) / 255.0f);
                offset_ns_.push_back(saturated_offset(packet_offset_ns + i * point_interval_ns));
                tag_.push_back(pts[i].tag);
            }
        } else if (data->data_type == DATA_TYPE_CARTESIAN_LOW) {
            auto* pts = reinterpret_cast<const LivoxLidarCartesianLowRawPoint*>(data->data);
            for (uint16_t i = 0; i < dot_num; ++i) {
                // Low-precision coordinates are in cm.
                xyz_.push_back(static_cast<float>(pts[i].x) / 100.0f);
                xyz_.push_back(static_cast<float>(pts[i].y) / 100.0f);
                xyz_.push_back(static_cast<float>(pts[i].z) / 100.0f);
                intensity_.push_back(static_cast<float>(pts[i].reflectivity) / 255.0f);
                offset_ns_.push_back(saturated_offset(packet_offset_ns + i * point_interval_ns));
                tag_.push_back(pts[i].tag);
            }
        }
    }

    void on_imu(LivoxLidarEthernetPacket* data) {
        if (shutdown_requested() || data == nullptr) return;

        double ts = packet_timestamp(data);
        auto* imu_pts = reinterpret_cast<const LivoxLidarImuRawPoint*>(data->data);
        uint16_t dot_num = data->dot_num;

        for (uint16_t i = 0; i < dot_num; ++i) {
            sensor_msgs::Imu msg;
            msg.header = make_header(cfg_.imu_frame_id, ts);

            // Orientation unknown: identity with -1 covariance to flag it.
            msg.orientation.x = 0.0;
            msg.orientation.y = 0.0;
            msg.orientation.z = 0.0;
            msg.orientation.w = 1.0;
            msg.orientation_covariance[0] = -1.0;

            msg.angular_velocity.x = static_cast<double>(imu_pts[i].gyro_x);
            msg.angular_velocity.y = static_cast<double>(imu_pts[i].gyro_y);
            msg.angular_velocity.z = static_cast<double>(imu_pts[i].gyro_z);

            msg.linear_acceleration.x = static_cast<double>(imu_pts[i].acc_x) * GRAVITY_MS2;
            msg.linear_acceleration.y = static_cast<double>(imu_pts[i].acc_y) * GRAVITY_MS2;
            msg.linear_acceleration.z = static_cast<double>(imu_pts[i].acc_z) * GRAVITY_MS2;

            imu_.publish(msg);
        }
    }

    void on_info(uint32_t handle, const LivoxLidarInfo* info) {
        if (info == nullptr) return;

        char sn[livox_common::kInfoFieldLen + 1] = {};
        std::memcpy(sn, info->sn, livox_common::kInfoFieldLen);
        char ip[livox_common::kInfoFieldLen + 1] = {};
        std::memcpy(ip, info->lidar_ip, livox_common::kInfoFieldLen);
        logging::info("mid360 device connected",
                      {logging::Field("sn", std::string(sn)),
                       logging::Field("ip", std::string(ip))});

        SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, nullptr, nullptr);
        if (cfg_.enable_imu) {
            EnableLivoxLidarImuData(handle, nullptr, nullptr);
        }
    }

    void emit_frame() {
        std::vector<float> xyz;
        std::vector<float> intensity;
        std::vector<uint32_t> offset_ns;
        std::vector<uint8_t> tag;
        double ts = 0.0;
        {
            std::lock_guard<std::mutex> lock(pc_mutex_);
            if (xyz_.empty()) return;
            xyz.swap(xyz_);
            intensity.swap(intensity_);
            offset_ns.swap(offset_ns_);
            tag.swap(tag_);
            // Header stamp is the frame start, the timebase offset_time is
            // relative to, same as the pointlio module.
            ts = static_cast<double>(frame_start_ns_) / 1e9;
            frame_has_ts_ = false;
        }
        publish_pointcloud(xyz, intensity, offset_ns, tag, ts);
    }

    void publish_pointcloud(const std::vector<float>& xyz, const std::vector<float>& intensity,
                            const std::vector<uint32_t>& offset_ns,
                            const std::vector<uint8_t>& tag, double ts) {
        int num_points = static_cast<int>(xyz.size()) / 3;

        sensor_msgs::PointCloud2 pc = make_cloud(format_, cfg_.frame_id, ts, num_points);

        for (int i = 0; i < num_points; ++i) {
            uint8_t* base = pc.data.data() + i * pc.point_step;
            float* dst = reinterpret_cast<float*>(base);
            dst[0] = xyz[i * 3 + 0];
            dst[1] = xyz[i * 3 + 1];
            dst[2] = xyz[i * 3 + 2];
            switch (format_) {
                case PointFormat::Full: {
                    dst[3] = intensity[i];
                    uint32_t offset_value = offset_ns[i];
                    std::memcpy(base + kOffsetTimeOffset, &offset_value, sizeof(uint32_t));
                    base[kTagOffset] = tag[i];
                    base[kLineOffset] = 0;  // Mid-360: single line
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

        lidar_.publish(pc);
    }

    Mid360Config cfg_;
    Output<sensor_msgs::PointCloud2> lidar_;
    Output<sensor_msgs::Imu> imu_;
    // From config, set in build().
    std::chrono::microseconds frame_interval_{};
    PointFormat format_ = PointFormat::Minimal;

    std::mutex pc_mutex_;
    std::vector<float> xyz_;
    std::vector<float> intensity_;
    // Per-point time offsets (ns since frame start) and Livox tag bytes,
    // matching what Point-LIO's CustomPoint carries.
    std::vector<uint32_t> offset_ns_;
    std::vector<uint8_t> tag_;
    uint64_t frame_start_ns_ = 0;
    bool frame_has_ts_ = false;
};

int main() {
    dimos::native::run_with_transport<Mid360>();
    return 0;
}
