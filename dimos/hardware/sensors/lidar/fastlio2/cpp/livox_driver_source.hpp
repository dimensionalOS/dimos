// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// LivoxDriverSource — owns the Livox SDK2 connection that feeds FAST-LIO:
//   * initialises the SDK in start()
//   * registers point-cloud / IMU / info-change callbacks
//   * accumulates raw points into a custom_messages::CustomMsg and pushes
//     one frame per 1/frequency seconds via tick()
//   * pushes IMU samples to FAST-LIO directly from the SDK callback thread
// This keeps main.cpp ignorant of the SDK and makes it trivial to add a
// second source (e.g. an LCM stream subscriber) alongside it.

#ifndef LIVOX_DRIVER_SOURCE_HPP_
#define LIVOX_DRIVER_SOURCE_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <boost/make_shared.hpp>

#include "dimos_native_module.hpp"
#include "fast_lio.hpp"
#include "fast_lio_debug.hpp"
#include "livox_sdk_config.hpp"

class LivoxDriverSource {
public:
    // Everything the source needs that comes from the CLI / outer config.
    // Defaults mirror the binary's documented defaults.
    struct Config {
        std::string host_ip  = "192.168.1.5";
        std::string lidar_ip = "192.168.1.155";
        livox_common::SdkPorts ports;   // struct defaults from livox_sdk_config.hpp
        float frame_frequency_hz = 10.0f;
        bool  debug = false;

        static Config from_args(const dimos::NativeModule& mod) {
            const livox_common::SdkPorts defaults;
            Config c;
            c.host_ip            = mod.arg("host_ip",  c.host_ip);
            c.lidar_ip           = mod.arg("lidar_ip", c.lidar_ip);
            c.frame_frequency_hz = mod.arg_float("frequency", c.frame_frequency_hz);
            c.debug              = mod.arg_bool("debug", c.debug);
            c.ports.cmd_data        = mod.arg_int("cmd_data_port",        defaults.cmd_data);
            c.ports.push_msg        = mod.arg_int("push_msg_port",        defaults.push_msg);
            c.ports.point_data      = mod.arg_int("point_data_port",      defaults.point_data);
            c.ports.imu_data        = mod.arg_int("imu_data_port",        defaults.imu_data);
            c.ports.log_data        = mod.arg_int("log_data_port",        defaults.log_data);
            c.ports.host_cmd_data   = mod.arg_int("host_cmd_data_port",   defaults.host_cmd_data);
            c.ports.host_push_msg   = mod.arg_int("host_push_msg_port",   defaults.host_push_msg);
            c.ports.host_point_data = mod.arg_int("host_point_data_port", defaults.host_point_data);
            c.ports.host_imu_data   = mod.arg_int("host_imu_data_port",   defaults.host_imu_data);
            c.ports.host_log_data   = mod.arg_int("host_log_data_port",   defaults.host_log_data);
            return c;
        }
    };

    LivoxDriverSource(FastLio* fast_lio,
                      Config cfg,
                      std::atomic<bool>& running)
        : fast_lio_(fast_lio),
          cfg_(std::move(cfg)),
          frame_interval_(std::chrono::microseconds(
              static_cast<int64_t>(1e6 / cfg_.frame_frequency_hz))),
          running_(running),
          last_emit_(std::chrono::steady_clock::now()) {}

    ~LivoxDriverSource() { stop(); }

    LivoxDriverSource(const LivoxDriverSource&) = delete;
    LivoxDriverSource& operator=(const LivoxDriverSource&) = delete;

    bool start() {
        if (started_) return true;
        if (!livox_common::init_livox_sdk(cfg_.host_ip, cfg_.lidar_ip, cfg_.ports, cfg_.debug)) {
            return false;
        }
        SetLivoxLidarPointCloudCallBack(
            [](uint32_t handle, uint8_t dev_type,
               LivoxLidarEthernetPacket* data, void* client_data) {
                static_cast<LivoxDriverSource*>(client_data)->on_point_cloud(handle, dev_type, data);
            },
            this);
        SetLivoxLidarImuDataCallback(
            [](uint32_t handle, uint8_t dev_type,
               LivoxLidarEthernetPacket* data, void* client_data) {
                static_cast<LivoxDriverSource*>(client_data)->on_imu(handle, dev_type, data);
            },
            this);
        SetLivoxLidarInfoChangeCallback(&LivoxDriverSource::info_thunk, this);
        if (!LivoxLidarSdkStart()) {
            std::fprintf(stderr, "Error: LivoxLidarSdkStart failed\n");
            LivoxLidarSdkUninit();
            return false;
        }
        started_ = true;
        return true;
    }

    void stop() {
        if (!started_) return;
        LivoxLidarSdkUninit();
        started_ = false;
    }

    // Called once per main-loop iteration. Builds a CustomMsg from points
    // accumulated by on_point_cloud and feeds it to FAST-LIO at the
    // configured frame rate.
    void tick(std::chrono::steady_clock::time_point now) {
        if (now - last_emit_ < frame_interval_) return;

        std::vector<custom_messages::CustomPoint> points;
        uint64_t frame_start = 0;
        {
            std::lock_guard<std::mutex> lock(pc_mutex_);
            if (!accumulated_.empty()) {
                points.swap(accumulated_);
                frame_start = frame_start_ns_;
                frame_has_timestamp_ = false;
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
            for (int i = 0; i < 3; ++i) lidar_msg->rsvd[i] = 0;
            lidar_msg->point_num = static_cast<uli>(points.size());
            lidar_msg->points = std::move(points);
            fast_lio_->feed_lidar(lidar_msg);
        }

        last_emit_ = now;
    }

private:

    static void info_thunk(const uint32_t handle, const LivoxLidarInfo* info,
                           void* /*client_data*/) {
        if (info == nullptr) return;

        char sn[17] = {};
        std::memcpy(sn, info->sn, 16);
        char ip[17] = {};
        std::memcpy(ip, info->lidar_ip, 16);

        if (fastlio_debug) {
            std::printf("[fastlio2] Device connected: handle=%u type=%u sn=%s ip=%s\n",
                        handle, info->dev_type, sn, ip);
        }

        SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, nullptr, nullptr);
        EnableLivoxLidarImuData(handle, nullptr, nullptr);
    }

    static uint64_t timestamp_ns(const LivoxLidarEthernetPacket* pkt) {
        uint64_t ns = 0;
        std::memcpy(&ns, pkt->timestamp, sizeof(uint64_t));
        return ns;
    }

    void on_point_cloud(uint32_t /*handle*/, uint8_t /*dev_type*/,
                        LivoxLidarEthernetPacket* data) {
        if (!running_.load() || data == nullptr) return;

        const uint64_t ts_ns = timestamp_ns(data);
        const uint16_t dot_num = data->dot_num;

        std::lock_guard<std::mutex> lock(pc_mutex_);

        if (!frame_has_timestamp_) {
            frame_start_ns_ = ts_ns;
            frame_has_timestamp_ = true;
        }

        if (data->data_type == livox_common::DATA_TYPE_CARTESIAN_HIGH) {
            const auto* pts = reinterpret_cast<const LivoxLidarCartesianHighRawPoint*>(data->data);
            for (uint16_t i = 0; i < dot_num; ++i) {
                custom_messages::CustomPoint cp;
                cp.x = static_cast<double>(pts[i].x) / 1000.0;   // mm → m
                cp.y = static_cast<double>(pts[i].y) / 1000.0;
                cp.z = static_cast<double>(pts[i].z) / 1000.0;
                cp.reflectivity = pts[i].reflectivity;
                cp.tag = pts[i].tag;
                cp.line = 0;
                cp.offset_time = static_cast<uli>(ts_ns - frame_start_ns_);
                accumulated_.push_back(cp);
            }
        } else if (data->data_type == livox_common::DATA_TYPE_CARTESIAN_LOW) {
            const auto* pts = reinterpret_cast<const LivoxLidarCartesianLowRawPoint*>(data->data);
            for (uint16_t i = 0; i < dot_num; ++i) {
                custom_messages::CustomPoint cp;
                cp.x = static_cast<double>(pts[i].x) / 100.0;    // cm → m
                cp.y = static_cast<double>(pts[i].y) / 100.0;
                cp.z = static_cast<double>(pts[i].z) / 100.0;
                cp.reflectivity = pts[i].reflectivity;
                cp.tag = pts[i].tag;
                cp.line = 0;
                cp.offset_time = static_cast<uli>(ts_ns - frame_start_ns_);
                accumulated_.push_back(cp);
            }
        }
    }

    void on_imu(uint32_t /*handle*/, uint8_t /*dev_type*/,
                LivoxLidarEthernetPacket* data) {
        if (!running_.load() || data == nullptr || !fast_lio_) return;

        const double ts = static_cast<double>(timestamp_ns(data)) / 1e9;
        const auto* imu_pts = reinterpret_cast<const LivoxLidarImuRawPoint*>(data->data);
        const uint16_t dot_num = data->dot_num;

        for (uint16_t i = 0; i < dot_num; ++i) {
            auto imu_msg = boost::make_shared<custom_messages::Imu>();
            imu_msg->header.stamp = custom_messages::Time().fromSec(ts);
            imu_msg->header.seq = 0;
            imu_msg->header.frame_id = "livox_frame";

            imu_msg->orientation.x = 0.0;
            imu_msg->orientation.y = 0.0;
            imu_msg->orientation.z = 0.0;
            imu_msg->orientation.w = 1.0;
            for (int j = 0; j < 9; ++j) imu_msg->orientation_covariance[j] = 0.0;

            imu_msg->angular_velocity.x = static_cast<double>(imu_pts[i].gyro_x);
            imu_msg->angular_velocity.y = static_cast<double>(imu_pts[i].gyro_y);
            imu_msg->angular_velocity.z = static_cast<double>(imu_pts[i].gyro_z);
            for (int j = 0; j < 9; ++j) imu_msg->angular_velocity_covariance[j] = 0.0;

            imu_msg->linear_acceleration.x =
                static_cast<double>(imu_pts[i].acc_x) * livox_common::GRAVITY_MS2;
            imu_msg->linear_acceleration.y =
                static_cast<double>(imu_pts[i].acc_y) * livox_common::GRAVITY_MS2;
            imu_msg->linear_acceleration.z =
                static_cast<double>(imu_pts[i].acc_z) * livox_common::GRAVITY_MS2;
            for (int j = 0; j < 9; ++j) imu_msg->linear_acceleration_covariance[j] = 0.0;

            fast_lio_->feed_imu(imu_msg);
        }
    }

    FastLio* fast_lio_;
    Config cfg_;
    std::chrono::microseconds frame_interval_;
    std::atomic<bool>& running_;

    bool started_ = false;
    std::chrono::steady_clock::time_point last_emit_;

    std::mutex pc_mutex_;
    std::vector<custom_messages::CustomPoint> accumulated_;
    uint64_t frame_start_ns_ = 0;
    bool frame_has_timestamp_ = false;
};

#endif  // LIVOX_DRIVER_SOURCE_HPP_
