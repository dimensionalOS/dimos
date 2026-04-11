// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// DrddsLidarBridge — NativeModule that reads lidar + IMU from POSIX shared
// memory (written by drdds_recv on the M20 host) and publishes them as LCM
// PointCloud2 + Imu messages for consumption by AriseSLAM and SmartNav.
//
// This replaces ros2_pub.cpp — instead of publishing to ROS2 topics inside a
// Docker container, it publishes to LCM on the host so the entire nav stack
// can run natively via nix (no Docker container needed).
//
// Usage: ./drdds_lidar_bridge --lidar <topic> --imu <topic>

#include <lcm/lcm-cpp.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <semaphore.h>
#include <thread>
#include <vector>

#include "shm_transport.h"

#include "dimos_native_module.hpp"

#include "geometry_msgs/Quaternion.hpp"
#include "geometry_msgs/Vector3.hpp"
#include "sensor_msgs/Imu.hpp"
#include "sensor_msgs/PointCloud2.hpp"
#include "sensor_msgs/PointField.hpp"

// ---------------------------------------------------------------------------
// Global state
// ---------------------------------------------------------------------------

static std::atomic<bool> g_running{true};
static lcm::LCM* g_lcm = nullptr;
static std::string g_lidar_topic;
static std::string g_imu_topic;

using dimos::time_from_seconds;
using dimos::make_header;

// ---------------------------------------------------------------------------
// Signal handling
// ---------------------------------------------------------------------------

static void signal_handler(int) { g_running = false; }

// ---------------------------------------------------------------------------
// RSAIRY PointCloud2 fields — all 6 preserved for ARISE SLAM.
// ARISE needs ring (scan line grouping) and time (motion compensation).
// ---------------------------------------------------------------------------

static void set_rsairy_fields(sensor_msgs::PointCloud2& pc) {
    pc.fields_length = 6;
    pc.fields.resize(6);

    auto set = [&](int i, const std::string& name, int32_t offset, uint8_t dtype) {
        pc.fields[i].name = name;
        pc.fields[i].offset = offset;
        pc.fields[i].datatype = dtype;
        pc.fields[i].count = 1;
    };

    set(0, "x",         0,  sensor_msgs::PointField::FLOAT32);
    set(1, "y",         4,  sensor_msgs::PointField::FLOAT32);
    set(2, "z",         8,  sensor_msgs::PointField::FLOAT32);
    set(3, "intensity", 12, sensor_msgs::PointField::FLOAT32);
    set(4, "ring",      16, sensor_msgs::PointField::UINT16);
    set(5, "time",      18, sensor_msgs::PointField::FLOAT32);
}

// ---------------------------------------------------------------------------
// Lidar: read SHM, remap rings, relativize time, publish LCM PointCloud2
// ---------------------------------------------------------------------------

static void lidar_loop() {
    drdds_bridge::ShmReader reader(drdds_bridge::SHM_LIDAR_NAME);

    // Open notification semaphore (created by drdds_recv)
    sem_t* notify_sem = sem_open(drdds_bridge::SHM_NOTIFY_NAME, O_CREAT, 0666, 0);
    if (notify_sem == SEM_FAILED) {
        fprintf(stderr, "[drdds_bridge] sem_open failed, falling back to 1ms polling\n");
        notify_sem = nullptr;
    }

    uint64_t count = 0;

    while (g_running) {
        if (!reader.is_open()) {
            if (!reader.try_open()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            fprintf(stderr, "[drdds_bridge] Lidar SHM connected\n");
        }

        auto* slot = reader.poll();
        if (!slot) {
            if (notify_sem) {
                struct timespec ts;
                clock_gettime(CLOCK_REALTIME, &ts);
                ts.tv_sec += 1;
                sem_timedwait(notify_sem, &ts);
                while (sem_trywait(notify_sem) == 0) {}  // drain
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            continue;
        }
        if (slot->msg_type != 0) continue;

        uint32_t data_size = slot->data_size;
        if (data_size + sizeof(drdds_bridge::SlotHeader) > drdds_bridge::LIDAR_SLOT_SIZE)
            continue;

        // Build LCM PointCloud2
        sensor_msgs::PointCloud2 pc;
        double stamp = slot->stamp_sec + slot->stamp_nsec * 1e-9;
        pc.header = make_header("lidar_link", stamp);
        pc.height = slot->height;
        pc.width = slot->width;
        pc.point_step = slot->point_step;
        pc.row_step = slot->row_step;
        pc.is_dense = slot->is_dense;
        pc.is_bigendian = slot->is_bigendian;
        set_rsairy_fields(pc);

        // Copy point data
        pc.data_length = data_size;
        pc.data.resize(data_size);
        const uint8_t* src = reader.slot_data(slot);
        std::memcpy(pc.data.data(), src, data_size);

        // Post-process for ARISE SLAM compatibility:
        // 1. Remap rings: 192-channel RSAIRY → 64 bins (ARISE N_SCANS=64)
        // 2. Make time relative: absolute f64 → relative f32
        {
            const uint32_t pt_step = pc.point_step;  // 26 bytes from drdds
            const uint32_t n_pts = pc.width * pc.height;
            constexpr size_t ring_off = 16;
            constexpr size_t time_off = 18;

            if (n_pts > 0 && pt_step > 0) {
                // Read first point's time for relative offset
                double first_t;
                std::memcpy(&first_t, pc.data.data() + time_off, sizeof(double));

                for (uint32_t i = 0; i < n_pts; i++) {
                    uint8_t* pt = pc.data.data() + i * pt_step;

                    // Remap ring: 0-191 → 0-63
                    uint16_t ring;
                    std::memcpy(&ring, pt + ring_off, sizeof(uint16_t));
                    ring = static_cast<uint16_t>(ring * 64 / 192);
                    if (ring > 63) ring = 63;
                    std::memcpy(pt + ring_off, &ring, sizeof(uint16_t));

                    // Make time relative, write as float32
                    double t;
                    std::memcpy(&t, pt + time_off, sizeof(double));
                    float t_f32 = static_cast<float>(t - first_t);
                    std::memcpy(pt + time_off, &t_f32, sizeof(float));
                }
            }
        }

        g_lcm->publish(g_lidar_topic, &pc);
        count++;

        if (count % 100 == 1) {
            fprintf(stderr, "[drdds_bridge] lidar #%lu pts=%u bytes=%u\n",
                    count, slot->width * slot->height, data_size);
        }
    }

    if (notify_sem) sem_close(notify_sem);
}

// ---------------------------------------------------------------------------
// IMU: read SHM, publish LCM Imu
// ---------------------------------------------------------------------------

static void imu_loop() {
    drdds_bridge::ShmReader reader(drdds_bridge::SHM_IMU_NAME);
    uint64_t count = 0;

    while (g_running) {
        if (!reader.is_open()) {
            if (!reader.try_open()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            fprintf(stderr, "[drdds_bridge] IMU SHM connected\n");
        }

        auto* slot = reader.poll();
        if (!slot) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        if (slot->msg_type != 1 || slot->data_size < 80) continue;

        sensor_msgs::Imu imu;
        double stamp = slot->stamp_sec + slot->stamp_nsec * 1e-9;
        imu.header = make_header("imu_link", stamp);

        const uint8_t* d = reader.slot_data(slot);
        size_t off = 0;
        auto read_d = [&]() -> double {
            double v;
            std::memcpy(&v, d + off, 8);
            off += 8;
            return v;
        };

        imu.orientation.x = read_d();
        imu.orientation.y = read_d();
        imu.orientation.z = read_d();
        imu.orientation.w = read_d();
        imu.angular_velocity.x = read_d();
        imu.angular_velocity.y = read_d();
        imu.angular_velocity.z = read_d();
        imu.linear_acceleration.x = read_d();
        imu.linear_acceleration.y = read_d();
        imu.linear_acceleration.z = read_d();

        // Zero covariance (unknown)
        for (int i = 0; i < 9; i++) {
            imu.orientation_covariance[i] = 0.0;
            imu.angular_velocity_covariance[i] = 0.0;
            imu.linear_acceleration_covariance[i] = 0.0;
        }

        g_lcm->publish(g_imu_topic, &imu);
        count++;
    }
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    dimos::NativeModule mod(argc, argv);
    g_lidar_topic = mod.has("lidar") ? mod.topic("lidar") : "";
    g_imu_topic = mod.has("imu") ? mod.topic("imu") : "";

    if (g_lidar_topic.empty() && g_imu_topic.empty()) {
        fprintf(stderr, "Usage: %s --lidar <topic> --imu <topic>\n", argv[0]);
        return 1;
    }

    lcm::LCM lcm;
    if (!lcm.good()) {
        fprintf(stderr, "[drdds_bridge] LCM init failed\n");
        return 1;
    }
    g_lcm = &lcm;

    fprintf(stderr, "[drdds_bridge] Starting (lidar=%s, imu=%s)\n",
            g_lidar_topic.c_str(), g_imu_topic.c_str());

    std::thread lidar_thread, imu_thread;
    if (!g_lidar_topic.empty()) lidar_thread = std::thread(lidar_loop);
    if (!g_imu_topic.empty()) imu_thread = std::thread(imu_loop);

    // Main thread handles LCM (required for publishing)
    while (g_running) {
        lcm.handleTimeout(100);
    }

    if (lidar_thread.joinable()) lidar_thread.join();
    if (imu_thread.joinable()) imu_thread.join();

    fprintf(stderr, "[drdds_bridge] Shutting down\n");
    return 0;
}
