// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// FAST-LIO2 native SLAM module for dimos NativeModule framework.
//
// Two input modes:
//   livox_sdk (default) — binds Livox SDK2 directly, SDK callbacks feed
//                         CustomMsg/Imu to FastLio. Used with Livox Mid-360.
//   lcm                 — subscribes to externally-published LCM PointCloud2
//                         + Imu streams (e.g. from DrddsLidarBridge on M20).
//                         No Livox hardware required.
//
// FastLio performs EKF-LOAM SLAM; registered (world-frame) point clouds +
// odometry are published on LCM.
//
// Usage (Livox mode, default):
//   ./fastlio2_native \
//       --lidar '/lidar#sensor_msgs.PointCloud2' \
//       --odometry '/odometry#nav_msgs.Odometry' \
//       --config_path /path/to/mid360.yaml \
//       --host_ip 192.168.1.5 --lidar_ip 192.168.1.155
//
// Usage (LCM input mode, e.g. M20 RSAIRY via DrddsLidarBridge):
//   ./fastlio2_native \
//       --input_mode lcm \
//       --raw_points '/raw_points#sensor_msgs.PointCloud2' \
//       --imu '/imu#sensor_msgs.Imu' \
//       --lidar '/registered_scan#sensor_msgs.PointCloud2' \
//       --odometry '/odometry#nav_msgs.Odometry' \
//       --config_path /path/to/velodyne.yaml

#include <lcm/lcm-cpp.hpp>

#include <atomic>
#include <boost/make_shared.hpp>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <deque>
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
// LCM input handlers (--input_mode lcm)
//
// Convert externally-published PointCloud2 + Imu (e.g. from DrddsLidarBridge
// on M20) into Livox-style CustomMsg + custom_messages::Imu and feed them
// into FAST-LIO. No Livox hardware / SDK required in this mode.
// ---------------------------------------------------------------------------

namespace {

// Resolved offsets of the fields we care about inside a PointCloud2 point.
struct Pc2FieldOffsets {
    int x = -1, y = -1, z = -1;
    int intensity = -1;
    int time = -1;
    int ring = -1;
    int8_t time_datatype = 0;  // FLOAT32 or FLOAT64
    int8_t ring_datatype = 0;
};

Pc2FieldOffsets resolve_pc2_fields(const sensor_msgs::PointCloud2& pc) {
    Pc2FieldOffsets o;
    for (const auto& f : pc.fields) {
        if (f.name == "x") o.x = f.offset;
        else if (f.name == "y") o.y = f.offset;
        else if (f.name == "z") o.z = f.offset;
        else if (f.name == "intensity") o.intensity = f.offset;
        else if (f.name == "time" || f.name == "t" || f.name == "timestamp") {
            o.time = f.offset;
            o.time_datatype = f.datatype;
        }
        else if (f.name == "ring") {
            o.ring = f.offset;
            o.ring_datatype = f.datatype;
        }
    }
    return o;
}

}  // namespace

// When true, trust the sensor-provided timestamps on lidar+IMU directly
// instead of applying the wall-clock-anchor + frame_ts-rewrite workarounds
// that were introduced to paper over the yesense↔rsdriver clock-domain
// mismatch (FASTLIO2_LOG Findings #7–#9). With the Airy IMU path, both
// lidar and IMU are PTP-locked to the same hardware clock, so the raw
// sensor_ts pair is already consistent and the rewrite would corrupt it.
// Set via `--native_clock true` CLI flag.
static std::atomic<bool> g_native_clock{false};

// Debug state for the first N frames, to diagnose timebase/IMU sync.
static std::atomic<int> g_lcm_pc_frames{0};
static std::atomic<double> g_last_imu_ts{0.0};

// Monotonicity guard: drdds bridge occasionally emits duplicate or
// out-of-order scans (dual-lidar merge artifact). Feeding non-monotonic
// scans to FAST-LIO's EKF makes it diverge immediately. Track the last
// accepted scan timestamp (seconds) and drop anything older or equal.
static std::atomic<double> g_last_scan_ts{0.0};
static std::atomic<uint64_t> g_dropped_dup_scans{0};
static std::atomic<uint64_t> g_dropped_stale_scans{0};

// Second monotonicity guard for the REWRITTEN (wall-clock anchored) stamp.
// Scans alternate between 100ms and 200ms spans (front lidar only vs both
// lidars merged). Anchoring at `imu_latest - max_offset` makes a 200ms scan
// timestamp 100ms EARLIER than the preceding 100ms scan even when the
// sensor-domain stamps are monotonic. FAST-LIO's syncPackage silently
// corrupts EKF state when this happens. Codex review 2026-04-18.
static std::atomic<double> g_last_frame_ts{0.0};
static std::atomic<uint64_t> g_dropped_regressed_rewritten{0};

// Stationary preroll gate. FAST-LIO's IMU_init averages ~20 samples (100ms
// @ 200Hz) to estimate gravity + gyro bias. M20 servo flutter during
// startup poisons both estimates permanently, producing ~1m/s² residual
// accel bias that integrates into hundreds of meters of drift within
// seconds. Buffer lidar scans until we see `PREROLL_STATIONARY_S` seconds
// of IMU with gyro and accel below the noise thresholds — only then feed
// scans to FAST-LIO so its IMU_init window falls on clean data.
static std::atomic<bool> g_seeded{false};
static std::atomic<double> g_stationary_since{0.0};
static std::atomic<uint64_t> g_imu_samples{0};
static constexpr double PREROLL_STATIONARY_S = 2.0;
static constexpr double STATIONARY_GYRO_RAD = 0.02;   // ~1.1 deg/s
static constexpr double STATIONARY_ACC_DEV  = 0.5;    // m/s² deviation from |g|

// ---------------------------------------------------------------------------
// Two-stage LCM transport (codex review 2026-04-21).
//
// Previously, on_lcm_point_cloud() did all the expensive work — max-offset
// scan, ring remap, time relativization, the full 2.6MB byte-copy into
// FAST-LIO's message type, and feed_lidar_pc2() — synchronously on LCM's
// single handle-loop thread. That same thread also dispatches on_lcm_imu().
// A 100ms lidar callback blocked ~20 IMU messages per cycle in the socket
// buffer; once the queue spilled, imu_latest stalled for seconds and
// FAST-LIO's EKF propagated on stale IMU → runaway drift after ~50s.
//
// Fix: subscriber callbacks now only COPY raw bytes into a small struct
// and push onto a bounded queue. A dedicated owner thread drains the
// queues (IMU priority: all pending IMU first, then one lidar frame) and
// does all the feed_imu/feed_lidar_pc2 work serially. This keeps
// LCM callback latency near-zero so IMU delivery tracks actual publish
// timestamps even during heavy lidar processing.
// ---------------------------------------------------------------------------

struct FieldDescr {
    std::string name;
    uint32_t offset;
    uint8_t datatype;
    uint32_t count;
};

struct LidarBuf {
    double sensor_ts;
    double wall_now;          // captured at callback entry for the legacy yesense anchor path
    uint32_t height, width, point_step, row_step;
    uint8_t is_dense, is_bigendian;
    std::vector<FieldDescr> fields;
    std::vector<uint8_t> data;
};

struct ImuBuf {
    double sensor_ts;
    double wall_now;          // captured at callback entry
    double ox, oy, oz, ow;
    double gx, gy, gz;
    double ax, ay, az;
};

static std::deque<LidarBuf> g_lidar_queue;
static std::deque<ImuBuf>   g_imu_queue;
static std::mutex           g_queue_mutex;
static std::condition_variable g_queue_cv;
static std::atomic<uint64_t> g_lidar_dropped_full{0};
static std::atomic<uint64_t> g_imu_dropped_full{0};
static constexpr size_t MAX_LIDAR_QUEUE = 3;       // ~300ms of headroom at 10Hz
static constexpr size_t MAX_IMU_QUEUE   = 400;     // 2s at 200Hz

// Thin subscriber: copy raw bytes + metadata, enqueue, return.
// All heavy work (max_offset scan, monotonic floor, feed_lidar_pc2) runs
// on the owner thread in process_lidar_buf().
static void on_lcm_point_cloud(const lcm::ReceiveBuffer* /*rbuf*/,
                               const std::string& /*channel*/,
                               const sensor_msgs::PointCloud2* pc) {
    if (!g_fastlio || !g_running.load() || pc == nullptr) return;
    if (pc->data_length == 0) return;

    LidarBuf buf;
    buf.sensor_ts = static_cast<double>(pc->header.stamp.sec) +
                    static_cast<double>(pc->header.stamp.nsec) * 1e-9;
    buf.wall_now = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count()) * 1e-9;
    buf.height = pc->height;
    buf.width = pc->width;
    buf.point_step = pc->point_step;
    buf.row_step = pc->row_step;
    buf.is_dense = pc->is_dense ? 1 : 0;
    buf.is_bigendian = pc->is_bigendian ? 1 : 0;
    buf.fields.reserve(pc->fields.size());
    for (const auto& f : pc->fields) {
        buf.fields.push_back({f.name,
                              static_cast<uint32_t>(f.offset),
                              static_cast<uint8_t>(f.datatype),
                              static_cast<uint32_t>(f.count)});
    }
    buf.data.assign(pc->data.begin(), pc->data.begin() + pc->data_length);

    {
        std::lock_guard<std::mutex> lk(g_queue_mutex);
        while (g_lidar_queue.size() >= MAX_LIDAR_QUEUE) {
            g_lidar_queue.pop_front();
            g_lidar_dropped_full.fetch_add(1);
        }
        g_lidar_queue.push_back(std::move(buf));
    }
    g_queue_cv.notify_one();
}

// Heavy lidar processing (ex-callback body). Runs on the owner thread.
static void process_lidar_buf(LidarBuf& buf) {
    // Rebuild a sensor_msgs::PointCloud2-style view so the existing logic
    // downstream still uses the same offsets / types. Most of the below
    // code is verbatim from the pre-refactor callback; only the message
    // ownership model changed (LidarBuf instead of the LCM pointer).
    const double sensor_ts = buf.sensor_ts;

    // Drop scans until the stationary preroll window has been observed.
    if (!g_seeded.load(std::memory_order_acquire)) {
        return;
    }

    // Resolve field offsets (x/y/z/intensity/ring/time) from the copied descriptors.
    Pc2FieldOffsets off;
    for (const auto& f : buf.fields) {
        if      (f.name == "x")         off.x = f.offset;
        else if (f.name == "y")         off.y = f.offset;
        else if (f.name == "z")         off.z = f.offset;
        else if (f.name == "intensity") off.intensity = f.offset;
        else if (f.name == "time" || f.name == "t" || f.name == "timestamp") {
            off.time = f.offset;
            off.time_datatype = f.datatype;
        } else if (f.name == "ring") {
            off.ring = f.offset;
            off.ring_datatype = f.datatype;
        }
    }
    if (off.x < 0 || off.y < 0 || off.z < 0) {
        static std::atomic<int> warn_count{0};
        if (warn_count.fetch_add(1) < 3) {
            fprintf(stderr, "[fastlio2] PointCloud2 missing x/y/z fields, skipping.\n");
        }
        return;
    }

    const int32_t npoints = static_cast<int32_t>(buf.width * buf.height);
    const int32_t step = static_cast<int32_t>(buf.point_step);
    if (npoints <= 0 || step <= 0 ||
        static_cast<int32_t>(buf.data.size()) < npoints * step) {
        return;
    }

    // Drop EXACT duplicate scans only (identical sensor_ts). drdds bridge
    // emits real scans with slightly non-monotonic sensor_ts due to
    // dual-lidar interleaving, but those are still valid lidar data — we
    // re-anchor them to wall_now below so FAST-LIO sees them as "current".
    // Being too strict here starved the EKF of data (only 5 scans in 15s).
    const double last_ts = g_last_scan_ts.load(std::memory_order_acquire);
    if (sensor_ts == last_ts) {
        g_dropped_dup_scans.fetch_add(1);
        return;
    }
    g_last_scan_ts.store(sensor_ts, std::memory_order_release);

    // First pass to compute max offset (needed for the native-clock path's
    // sanity check and for the wall-clock anchor used in the legacy path).
    const uint8_t* pc_data = buf.data.data();
    double max_offset_s = 0.0;
    if (off.time >= 0 && off.time_datatype == sensor_msgs::PointField::FLOAT32) {
        for (int32_t i = 0; i < npoints; ++i) {
            const uint8_t* p = pc_data + static_cast<size_t>(i) * step;
            float t_rel = *reinterpret_cast<const float*>(p + off.time);
            if (t_rel > max_offset_s) max_offset_s = t_rel;
        }
    }

    double frame_ts;
    double imu_latest;
    if (g_native_clock.load(std::memory_order_acquire)) {
        // Airy IMU path: sensor timestamps are PTP-locked and already in a
        // shared hardware-clock domain with the IMU. Trust `sensor_ts` as the
        // floor for `header.stamp`, but enforce a strict monotonic floor.
        //
        // Why the floor: rsdriver merges two async Airy lidars and stamps the
        // cloud with `ts_first_point = true` → first-point time. First points
        // from the two lidars interleave, so header.stamp can regress by
        // hundreds of ms across consecutive merged scans. FAST-LIO's EKF sees
        // lidar moving backward in time and blows up. Codex review 2026-04-19.
        imu_latest = g_last_imu_ts.load(std::memory_order_acquire);
        const double prev_frame_ts = g_last_frame_ts.load(std::memory_order_acquire);
        frame_ts = std::max(sensor_ts, prev_frame_ts + 1e-4);
        if (frame_ts > sensor_ts) {
            uint64_t n = g_dropped_regressed_rewritten.fetch_add(1);
            if (n < 5) {
                fprintf(stderr,
                        "[fastlio2] clamping regressed sensor_ts (native_clock): "
                        "sensor_ts=%.6f prev=%.6f → frame_ts=%.6f\n",
                        sensor_ts, prev_frame_ts, frame_ts);
            }
        }
        g_last_frame_ts.store(frame_ts, std::memory_order_release);
    } else {
        // Legacy yesense path: the raw drdds lidar header stamp is sensor
        // time (~0.8s behind wall clock), while the IMU stream is stamped at
        // wall clock in on_lcm_imu below. Comparing cross-domain values
        // diverges the EKF. Anchor scan end at imu_latest so FAST-LIO's
        // syncPackage sees IMU data bracketing the whole scan (Finding #8).
        // wall_now was captured at subscriber-callback time (not here in
        // the owner thread) so the fallback matches the old semantics when
        // IMU hasn't arrived yet.
        const double wall_now = buf.wall_now;
        imu_latest = g_last_imu_ts.load(std::memory_order_acquire);
        const double frame_end = (imu_latest > 0.0) ? imu_latest : wall_now;
        frame_ts = frame_end - max_offset_s;

        // Variable max_offset_s (100ms vs 200ms when scans alternate between
        // single-lidar and merged-dual) can make frame_ts jump backwards even
        // with monotonic sensor_ts. Drop regressed rewrites (Finding #9).
        const double prev_frame_ts = g_last_frame_ts.load(std::memory_order_acquire);
        if (frame_ts <= prev_frame_ts) {
            uint64_t n = g_dropped_regressed_rewritten.fetch_add(1);
            if (n < 5) {
                fprintf(stderr,
                        "[fastlio2] dropping regressed rewritten scan: frame_ts=%.6f prev=%.6f "
                        "max_offset=%.3fs imu_latest=%.6f\n",
                        frame_ts, prev_frame_ts, max_offset_s, imu_latest);
            }
            return;
        }
        g_last_frame_ts.store(frame_ts, std::memory_order_release);
    }

    // Build a custom_messages::PointCloud2 copy of the incoming dimos LCM
    // PointCloud2 and feed it to FAST-LIO's Velodyne/PC2 path (rather than
    // stuffing into a Livox CustomMsg). This lets FAST-LIO's preprocess pick
    // up our ring+time fields correctly under `lidar_type: 2` (Velodyne).
    //
    // Only `header.stamp` is rewritten (wall-clock anchor per the timing
    // analysis above); all other fields copy through 1:1 because
    // custom_messages::PointCloud2 is structurally identical to ROS PC2.
    auto lidar_msg = boost::make_shared<custom_messages::PointCloud2>();
    lidar_msg->header.seq = 0;
    lidar_msg->header.stamp = custom_messages::Time().fromSec(frame_ts);
    lidar_msg->header.frame_id = "lidar";
    lidar_msg->height = buf.height;
    lidar_msg->width = buf.width;
    lidar_msg->is_bigendian = buf.is_bigendian ? true : false;
    lidar_msg->point_step = buf.point_step;
    lidar_msg->row_step = buf.row_step;
    lidar_msg->is_dense = buf.is_dense ? true : false;

    // Copy PointField descriptors. FAST-LIO matches by NAME, not index, so
    // whatever ordering drdds bridge emits (x, y, z, intensity, ring, time)
    // is fine.
    lidar_msg->fields.clear();
    lidar_msg->fields.reserve(buf.fields.size());
    for (const auto& f : buf.fields) {
        custom_messages::PointField pf;
        pf.name = f.name;
        pf.offset = static_cast<uli>(f.offset);
        pf.datatype = static_cast<usi>(f.datatype);
        pf.count = static_cast<uli>(f.count);
        lidar_msg->fields.push_back(pf);
    }

    // Copy packed bytes. custom_messages::PointCloud2::data is unfortunately
    // typed as std::vector<usi> (unsigned short) even though it's meant for
    // byte-packed PointCloud2 data — FAST-LIO's pcl_custom::toPCL truncates
    // each element back to uint8 before feeding PCL. We replicate that
    // pattern byte-for-byte.
    const size_t bytes = buf.data.size();
    lidar_msg->data.resize(bytes);
    for (size_t i = 0; i < bytes; ++i) {
        lidar_msg->data[i] = static_cast<usi>(pc_data[i]);
    }

    // Log the first 30 frames for sync debugging.
    int frame_idx = g_lcm_pc_frames.fetch_add(1);
    if (frame_idx < 30) {
        double imu_ts = g_last_imu_ts.load();
        fprintf(stderr,
                "[fastlio2] frame #%d  path=pc2  timebase=%.6f  max_offset=%.3fs  imu_latest=%.6f  "
                "imu_vs_frame_end=%+.3fs  npts=%d\n",
                frame_idx, frame_ts, max_offset_s, imu_ts,
                (imu_ts - (frame_ts + max_offset_s)),
                npoints);
    }

    g_fastlio->feed_lidar_pc2(lidar_msg);
}

// Thin subscriber: copy IMU fields, enqueue, return. Heavy lifting (stationary
// preroll gate, timestamp selection, feed_imu) runs on the owner thread in
// process_imu_buf().
static void on_lcm_imu(const lcm::ReceiveBuffer* /*rbuf*/,
                       const std::string& /*channel*/,
                       const sensor_msgs::Imu* imu) {
    if (!g_fastlio || !g_running.load() || imu == nullptr) return;

    ImuBuf b;
    b.sensor_ts = static_cast<double>(imu->header.stamp.sec) +
                  static_cast<double>(imu->header.stamp.nsec) * 1e-9;
    b.wall_now = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count()) * 1e-9;
    b.ox = imu->orientation.x; b.oy = imu->orientation.y;
    b.oz = imu->orientation.z; b.ow = imu->orientation.w;
    b.gx = imu->angular_velocity.x; b.gy = imu->angular_velocity.y; b.gz = imu->angular_velocity.z;
    b.ax = imu->linear_acceleration.x; b.ay = imu->linear_acceleration.y; b.az = imu->linear_acceleration.z;

    {
        std::lock_guard<std::mutex> lk(g_queue_mutex);
        while (g_imu_queue.size() >= MAX_IMU_QUEUE) {
            g_imu_queue.pop_front();
            g_imu_dropped_full.fetch_add(1);
        }
        g_imu_queue.push_back(b);
    }
    g_queue_cv.notify_one();
}

// Heavy IMU processing (ex-callback body). Runs on the owner thread.
static void process_imu_buf(const ImuBuf& b) {
    // Timestamp source — same semantics as before: sensor stamp (PTP) in
    // native_clock mode, subscriber-side wall clock in legacy mode.
    const double ts = g_native_clock.load(std::memory_order_acquire)
                          ? b.sensor_ts
                          : b.wall_now;
    g_last_imu_ts.store(ts);

    // Stationarity tracker: if any axis exceeds the threshold, reset the
    // clock on "how long have we been still". Only once the robot has been
    // stationary for PREROLL_STATIONARY_S do we flip g_seeded, which
    // unblocks lidar feeding and lets FAST-LIO's IMU_init run on clean data.
    if (!g_seeded.load(std::memory_order_acquire)) {
        const double a_mag = std::sqrt(b.ax*b.ax + b.ay*b.ay + b.az*b.az);
        const bool moving = std::fabs(b.gx) > STATIONARY_GYRO_RAD ||
                            std::fabs(b.gy) > STATIONARY_GYRO_RAD ||
                            std::fabs(b.gz) > STATIONARY_GYRO_RAD ||
                            std::fabs(a_mag - 9.81) > STATIONARY_ACC_DEV;
        if (moving) {
            g_stationary_since.store(ts, std::memory_order_release);
        } else {
            double since = g_stationary_since.load(std::memory_order_acquire);
            if (since == 0.0) {
                g_stationary_since.store(ts, std::memory_order_release);
                since = ts;
            }
            if (ts - since >= PREROLL_STATIONARY_S) {
                g_seeded.store(true, std::memory_order_release);
                fprintf(stderr,
                        "[fastlio2] stationary preroll complete after %.2fs — "
                        "releasing lidar scans to FAST-LIO\n",
                        ts - since);
            }
        }
        uint64_t s = g_imu_samples.fetch_add(1);
        if (s == 0) {
            fprintf(stderr, "[fastlio2] first IMU sample received (ts=%.3f) — starting preroll\n", ts);
        }
    }

    auto imu_msg = boost::make_shared<custom_messages::Imu>();
    imu_msg->header.seq = 0;
    imu_msg->header.stamp = custom_messages::Time().fromSec(ts);
    imu_msg->header.frame_id = "imu_link";
    imu_msg->orientation.x = b.ox; imu_msg->orientation.y = b.oy;
    imu_msg->orientation.z = b.oz; imu_msg->orientation.w = b.ow;
    for (int j = 0; j < 9; ++j) imu_msg->orientation_covariance[j] = 0.0;
    imu_msg->angular_velocity.x = b.gx;
    imu_msg->angular_velocity.y = b.gy;
    imu_msg->angular_velocity.z = b.gz;
    for (int j = 0; j < 9; ++j) imu_msg->angular_velocity_covariance[j] = 0.0;
    imu_msg->linear_acceleration.x = b.ax;
    imu_msg->linear_acceleration.y = b.ay;
    imu_msg->linear_acceleration.z = b.az;
    for (int j = 0; j < 9; ++j) imu_msg->linear_acceleration_covariance[j] = 0.0;

    g_fastlio->feed_imu(imu_msg);
}

// Owner thread: drains the IMU + lidar queues. IMU priority — all pending
// IMU is drained before each lidar frame so g_last_imu_ts tracks the most
// recent publisher timestamp instead of stalling during heavy lidar work
// (previously ran synchronously on the LCM callback thread).
static void fastlio_owner_loop() {
    using namespace std::chrono_literals;
    std::unique_lock<std::mutex> lk(g_queue_mutex);
    while (g_running.load()) {
        g_queue_cv.wait_for(lk, 50ms, [] {
            return !g_imu_queue.empty() || !g_lidar_queue.empty() ||
                   !g_running.load();
        });
        if (!g_running.load()) break;

        // Priority 1: drain all pending IMU.
        while (!g_imu_queue.empty()) {
            ImuBuf b = std::move(g_imu_queue.front());
            g_imu_queue.pop_front();
            lk.unlock();
            process_imu_buf(b);
            lk.lock();
        }

        // Priority 2: process at most one lidar frame per wake, then go
        // around so any freshly-arrived IMU drains before the next scan.
        if (!g_lidar_queue.empty()) {
            LidarBuf b = std::move(g_lidar_queue.front());
            g_lidar_queue.pop_front();
            lk.unlock();
            process_lidar_buf(b);
            lk.lock();
        }
    }
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

    // Required: LCM topics for output ports.
    // Accept both `--lidar` (legacy / Livox wrapper) and `--registered_scan`
    // (smart_nav convention used by AriseSLAM and the LCM-input wrapper).
    g_lidar_topic = mod.has("lidar") ? mod.topic("lidar")
                  : mod.has("registered_scan") ? mod.topic("registered_scan")
                  : "";
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

    // Input mode: "livox_sdk" (default) or "lcm" (M20 / external PointCloud2+Imu)
    const std::string input_mode = mod.arg("input_mode", "livox_sdk");
    // Trust sensor timestamps on lidar+IMU instead of wall-clock-rewriting
    // them. Required when the IMU and lidar share a PTP-locked hardware
    // clock (e.g. the Airy integrated IMU path); wrong for the yesense
    // legacy path, which has separate clock domains.
    // Parse bool manually — the lidar-common dimos::NativeModule header lacks
    // arg_bool (unlike the smart_nav one). Match the Python blueprint, which
    // sends "true"/"false" lowercased.
    {
        std::string nc = mod.arg("native_clock", "false");
        g_native_clock.store(nc == "true" || nc == "1",
                             std::memory_order_release);
    }
    fprintf(stderr, "[fastlio2] native_clock = %s\n",
            g_native_clock.load() ? "true" : "false");
    if (input_mode != "livox_sdk" && input_mode != "lcm") {
        fprintf(stderr, "Error: --input_mode must be 'livox_sdk' or 'lcm' (got '%s')\n",
                input_mode.c_str());
        return 1;
    }

    // LCM-input topics (required when input_mode=lcm, ignored otherwise)
    std::string raw_points_topic;
    std::string imu_input_topic;
    if (input_mode == "lcm") {
        if (!mod.has("raw_points") || !mod.has("imu")) {
            fprintf(stderr, "Error: input_mode=lcm requires --raw_points and --imu topics\n");
            return 1;
        }
        raw_points_topic = mod.topic("raw_points");
        imu_input_topic = mod.topic("imu");
    }

    // Livox hardware config (ignored when input_mode=lcm)
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

    printf("[fastlio2] Starting FAST-LIO2 native module (input_mode=%s)\n",
           input_mode.c_str());
    if (has_init_pose()) {
        printf("[fastlio2] init_pose: xyz=(%.3f, %.3f, %.3f) quat=(%.4f, %.4f, %.4f, %.4f)\n",
               g_init_x, g_init_y, g_init_z, g_init_qx, g_init_qy, g_init_qz, g_init_qw);
    }
    if (input_mode == "lcm") {
        printf("[fastlio2] raw_points topic: %s\n", raw_points_topic.c_str());
        printf("[fastlio2] imu topic: %s\n", imu_input_topic.c_str());
    } else {
        printf("[fastlio2] host_ip: %s  lidar_ip: %s  frequency: %.1f Hz\n",
               host_ip.c_str(), lidar_ip.c_str(), g_frequency);
    }
    printf("[fastlio2] lidar topic: %s\n",
           g_lidar_topic.empty() ? "(disabled)" : g_lidar_topic.c_str());
    printf("[fastlio2] odometry topic: %s\n",
           g_odometry_topic.empty() ? "(disabled)" : g_odometry_topic.c_str());
    printf("[fastlio2] global_map topic: %s\n",
           g_map_topic.empty() ? "(disabled)" : g_map_topic.c_str());
    printf("[fastlio2] config: %s\n", config_path.c_str());
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

    if (input_mode == "livox_sdk") {
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

        printf("[fastlio2] Livox SDK started, waiting for device...\n");
    } else {
        // LCM input mode: subscribe to raw_points + imu streams.
        // Callbacks feed FastLio directly; no frame accumulation needed.
        // Explicit template params required: free-function pointers don't
        // deduce through LCM's std::function<HandlerFunction<T>> overload.
        lcm.subscribe<sensor_msgs::PointCloud2>(raw_points_topic, &on_lcm_point_cloud);
        lcm.subscribe<sensor_msgs::Imu>(imu_input_topic, &on_lcm_imu);
        printf("[fastlio2] LCM subscribers active, waiting for data...\n");
    }

    // Decouple LCM subscriber thread from FAST-LIO state updates: subscribers
    // just push bytes onto bounded queues; fastlio_owner_loop drains them
    // with IMU priority and calls feed_imu / feed_lidar_pc2 serially on its
    // own thread. Prevents slow lidar processing from starving IMU delivery
    // (codex review 2026-04-21).
    std::thread fastlio_owner_thread;
    if (input_mode == "lcm") {
        fastlio_owner_thread = std::thread(fastlio_owner_loop);
    }

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

        auto now = std::chrono::steady_clock::now();

        // Livox-mode frame accumulation: SDK packets arrive as small chunks;
        // batch them into full scans at frame_interval. LCM mode receives
        // complete PointCloud2 messages per frame and feeds directly from
        // the subscriber callback, so this block is skipped.
        if (input_mode == "livox_sdk" && now - last_emit >= frame_interval) {
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
    g_queue_cv.notify_all();
    if (fastlio_owner_thread.joinable()) {
        fastlio_owner_thread.join();
    }
    g_fastlio = nullptr;
    if (input_mode == "livox_sdk") {
        LivoxLidarSdkUninit();
    }
    g_lcm = nullptr;

    printf("[fastlio2] Done.\n");
    return 0;
}
