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
//        [--body_crop xmin,xmax,ymin,ymax,zmin,zmax]  (base_link AABB, in meters)
//
// Body crop: drops points whose (x,y,z) in base_link fall inside the AABB.
// rsdriver merged mode publishes clouds in base_link (identity extrinsic),
// so this is a per-point coordinate comparison — no transform needed.
// See FASTLIO2_LOG Finding #34.

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

// Body-frame AABB for robot self-filtering. Drops points that fall inside
// the box (x in [xmin,xmax], y in [ymin,ymax], z in [zmin,zmax]) before
// publishing the cloud. Zero-initialized → disabled by default; populated
// from --body_crop CLI arg at startup. Applies to the lidar stream only.
// Default values for M20 (from URDF + leg-swing padding):
//   x: [-0.454, +0.454], y: [-0.354, +0.354], z: [-0.740, +0.220]
struct BodyCrop {
    bool enabled = false;
    float xmin = 0, xmax = 0, ymin = 0, ymax = 0, zmin = 0, zmax = 0;
};
static BodyCrop g_body_crop;

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

        // Build LCM PointCloud2. We re-stamp the header below using the
        // first point's time (scan_start) once point data is available.
        sensor_msgs::PointCloud2 pc;
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

        // Post-process:
        //   1. Set pc.header.stamp = scan_START time (first point's time).
        //      drdds's slot->stamp_{sec,nsec} is empirically the scan_END
        //      time (= max of per-point times). ROS and FAST-LIO2 expect
        //      header.stamp to be scan_START; FAST-LIO2 then adds
        //      max_offset_s to recover scan_end for the syncPackage gate.
        //      Using scan_end as header.stamp causes FAST-LIO2 to hunt for
        //      IMU samples 100 ms in the FUTURE of the scan, which
        //      manifests as `imu_vs_frame_end = -100 ms` even in steady
        //      state. At 1 rad/s rotation that is 6° of forced IMU
        //      extrapolation per scan, enough to push ICP into the wrong
        //      local minimum and cause catastrophic SLAM drift during yaw.
        //      See FASTLIO2_LOG Finding #28.
        //   2. Make time relative: absolute f64 → relative f32 (0 .. span).
        //
        // Previously the loop also collapsed the 192-ring RSAIRY cloud to
        // 64 rings via `ring * 64 / 192`. That was a vestigial
        // AriseSLAM-era accommodation and has been verified inert in our
        // FAST-LIO2 path (feature_enabled=false so ring-indexed neighbour
        // math is never triggered; see FASTLIO2_LOG Finding #25). Removed
        // for hygiene so downstream consumers see the raw 0-191 rings and
        // can use them if they ever need feature extraction.
        {
            const uint32_t pt_step = pc.point_step;  // 26 bytes from drdds
            const uint32_t n_pts = pc.width * pc.height;
            constexpr size_t xyz_off = 0;    // x,y,z are the first 3 f32 fields
            constexpr size_t time_off = 18;

            double scan_start = slot->stamp_sec + slot->stamp_nsec * 1e-9;
            uint32_t n_kept = 0;
            uint32_t n_cropped = 0;
            if (n_pts > 0 && pt_step > 0) {
                // Per-point times in the raw drdds cloud are absolute
                // seconds (same PTP-UTC clock as the IMU). Empirically the
                // first in-memory point has the minimum time (verified via
                // SHM probe — no descending steps), so use it as scan_start.
                double first_t;
                std::memcpy(&first_t, pc.data.data() + time_off, sizeof(double));
                scan_start = first_t;

                // In-place compaction: iterate all points, test body-crop
                // AABB, rewrite the time field (f64 absolute → f32 relative),
                // and keep only accepted points. Dropped points' slots are
                // overwritten in-place. Using an output cursor (n_kept)
                // decoupled from the input index (i) makes this O(n) with no
                // extra allocation — we just copy forward over the gaps.
                for (uint32_t i = 0; i < n_pts; i++) {
                    uint8_t* src = pc.data.data() + i * pt_step;

                    if (g_body_crop.enabled) {
                        float x, y, z;
                        std::memcpy(&x, src + xyz_off,     sizeof(float));
                        std::memcpy(&y, src + xyz_off + 4, sizeof(float));
                        std::memcpy(&z, src + xyz_off + 8, sizeof(float));
                        // rsdriver merged mode publishes clouds in base_link,
                        // so (x,y,z) is already in body frame — direct AABB
                        // compare with no rotation needed.
                        if (x >= g_body_crop.xmin && x <= g_body_crop.xmax &&
                            y >= g_body_crop.ymin && y <= g_body_crop.ymax &&
                            z >= g_body_crop.zmin && z <= g_body_crop.zmax) {
                            n_cropped++;
                            continue;  // inside body box → drop
                        }
                    }

                    uint8_t* dst = pc.data.data() + n_kept * pt_step;
                    if (dst != src) {
                        std::memmove(dst, src, pt_step);
                    }

                    // Make time relative, write as float32.
                    double t;
                    std::memcpy(&t, dst + time_off, sizeof(double));
                    float t_f32 = static_cast<float>(t - first_t);
                    std::memcpy(dst + time_off, &t_f32, sizeof(float));

                    n_kept++;
                }
            } else {
                n_kept = n_pts;
            }

            // Shrink the payload to only the kept points.
            pc.width = n_kept;
            pc.height = 1;
            pc.row_step = pt_step * n_kept;
            pc.data_length = pt_step * n_kept;
            pc.data.resize(pt_step * n_kept);

            pc.header = make_header("lidar_link", scan_start);

            // Surface crop stats periodically so we can monitor it.
            if (g_body_crop.enabled && count % 100 == 0) {
                fprintf(stderr, "[drdds_bridge] body_crop: %u kept, %u dropped (%.1f%%)\n",
                        n_kept, n_cropped,
                        n_pts > 0 ? 100.0 * n_cropped / n_pts : 0.0);
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
        fprintf(stderr, "Usage: %s --lidar <topic> --imu <topic>"
                        " [--body_crop xmin,xmax,ymin,ymax,zmin,zmax]\n", argv[0]);
        return 1;
    }

    // Parse optional body-frame AABB crop. Values are in base_link meters.
    // Default (0,0,0,0,0,0 or unset) disables cropping.
    if (mod.has("body_crop")) {
        const std::string s = mod.arg("body_crop", "");
        float v[6] = {0};
        if (std::sscanf(s.c_str(), "%f,%f,%f,%f,%f,%f",
                        &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) == 6) {
            g_body_crop.enabled = true;
            g_body_crop.xmin = v[0]; g_body_crop.xmax = v[1];
            g_body_crop.ymin = v[2]; g_body_crop.ymax = v[3];
            g_body_crop.zmin = v[4]; g_body_crop.zmax = v[5];
            fprintf(stderr, "[drdds_bridge] body_crop ENABLED: "
                            "x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]\n",
                    v[0], v[1], v[2], v[3], v[4], v[5]);
        } else {
            fprintf(stderr, "[drdds_bridge] body_crop parse failed for '%s' — "
                            "expected 'xmin,xmax,ymin,ymax,zmin,zmax'\n", s.c_str());
        }
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
