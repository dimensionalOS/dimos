// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// airy_imu_bridge — NativeModule that taps the RoboSense Airy lidar's
// integrated IMU directly from its UDP multicast stream, parses the 51-byte
// packet format (per RoboSense rs_driver decoder_RSAIRY.hpp), rotates the
// accel + gyro into the robot base_link frame, and publishes LCM
// sensor_msgs::Imu.
//
// Why bypass rsdriver: rsdriver's binary has IMU parser symbols linked but
// does NOT publish /imu/data on drdds despite the config key — dead code
// path. We subscribe directly to the multicast to avoid that.
//
// Clean pairing: the Airy integrated IMU shares a PTP-locked hardware clock
// with the Airy lidar optics. Combined with rsdriver's known lidar→base
// extrinsic (applied here as R_base_from_sensor), this gives FAST-LIO2 a
// single, physically consistent IMU stream in base_link frame.
//
// Usage: ./airy_imu_bridge --imu <topic> [--which front|rear]

#include <arpa/inet.h>
#include <lcm/lcm-cpp.hpp>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include "dimos_native_module.hpp"
#include "sensor_msgs/Imu.hpp"

namespace {

// ---------------------------------------------------------------------------
// RSAIRY IMU packet layout (big-endian int32 fields, 51 bytes total).
// Per rs_driver/src/rs_driver/driver/decoder/decoder_RSAIRY.hpp.
// ---------------------------------------------------------------------------

constexpr size_t AIRY_IMU_PACKET_SIZE = 51;
constexpr uint8_t AIRY_MAGIC[4] = {0xAA, 0x55, 0x5A, 0x05};

// Offsets within the packet.
constexpr size_t OFF_MAGIC     = 0;    // 4 bytes
constexpr size_t OFF_SEC       = 4;    // 6 bytes BE (UTC seconds)
constexpr size_t OFF_USEC      = 10;   // 4 bytes BE (microseconds)
constexpr size_t OFF_ACC_X     = 14;   // 4 bytes BE int32 (raw ADC)
constexpr size_t OFF_ACC_Y     = 18;
constexpr size_t OFF_ACC_Z     = 22;
constexpr size_t OFF_GYR_X     = 26;
constexpr size_t OFF_GYR_Y     = 30;
constexpr size_t OFF_GYR_Z     = 34;
constexpr size_t OFF_TEMP      = 38;   // 4 bytes
constexpr size_t OFF_ODR       = 42;   // 1 byte
constexpr size_t OFF_ACC_FSR   = 43;   // 1 byte: 0=2g, 1=4g, 2=8g, 3=16g
constexpr size_t OFF_GYR_FSR   = 44;   // 1 byte: 0=250, 1=500, 2=1000, 3=2000 dps

// PTP lock sanity gate: drop packets whose UTC seconds predate 2024-01-01.
// A free-running Airy without PTP reports seconds starting near 0.
constexpr int64_t PTP_MIN_SEC = 1704067200;  // 2024-01-01 00:00:00 UTC

constexpr double PI = 3.14159265358979323846;
constexpr double G_MSS = 9.80665;

// Multicast groups + ports from rsdriver config (/opt/robot/share/node_driver/config/config.yaml).
constexpr const char* MULTICAST_FRONT = "224.10.10.201";
constexpr uint16_t    PORT_FRONT      = 6681;
constexpr const char* MULTICAST_REAR  = "224.10.10.202";
constexpr uint16_t    PORT_REAR       = 6682;

// ---------------------------------------------------------------------------
// Big-endian readers. rs_driver decodes all multi-byte fields via ntohl/ntohs.
// ---------------------------------------------------------------------------

uint32_t rd_be_u32(const uint8_t* p) {
    return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | uint32_t(p[3]);
}

int32_t rd_be_i32(const uint8_t* p) {
    return static_cast<int32_t>(rd_be_u32(p));
}

uint64_t rd_be_u48(const uint8_t* p) {
    uint64_t v = 0;
    for (int i = 0; i < 6; ++i) v = (v << 8) | uint64_t(p[i]);
    return v;
}

// ---------------------------------------------------------------------------
// Rotation matrices — Airy sensor frame → M20 base_link.
//
// Derived empirically from the FRONT Airy stationary accel reading: raw
// sensor-frame accel at rest was (0.063, -9.859, 0.113) m/s², meaning the
// Airy IMU measures gravity (proper accel = +Z_world) along its own -Y axis.
// That pins the sensor axis mapping:
//   FRONT:  sensor X → base -Y, sensor Y → base -Z (down),
//           sensor Z (dome) → base +X (forward)
//   REAR:   the housing is 180° about Y relative to front (dome points -X),
//           so R_rear = R_front · Ry(π). sensor X → base +Y,
//           sensor Y → base -Z, sensor Z → base -X (backward).
//
// The rsdriver config's (yaw, pitch, roll) fields do NOT decode cleanly to
// these via the ZYX Tait-Bryan convention — either the driver uses a
// non-standard euler order or the sensor's own axis convention differs
// from the documented "X cable, Y perp, Z dome". Rather than guess, we
// pin the rotation to what the stationary-gravity measurement shows.
// Columns of R are the sensor axes expressed in base_link.
// ---------------------------------------------------------------------------

struct Mat3 { double m[3][3]; };

constexpr Mat3 R_BASE_FROM_FRONT = {{
    {0,  0,  1},
    {-1, 0,  0},
    {0, -1,  0},
}};

constexpr Mat3 R_BASE_FROM_REAR = {{
    {0,  0, -1},
    {1,  0,  0},
    {0, -1,  0},
}};

void rotate(const Mat3& R, double vx, double vy, double vz, double& ox, double& oy, double& oz) {
    ox = R.m[0][0] * vx + R.m[0][1] * vy + R.m[0][2] * vz;
    oy = R.m[1][0] * vx + R.m[1][1] * vy + R.m[1][2] * vz;
    oz = R.m[2][0] * vx + R.m[2][1] * vy + R.m[2][2] * vz;
}

// ---------------------------------------------------------------------------
// Parsed packet
// ---------------------------------------------------------------------------

struct AiryImuSample {
    double stamp_s;            // UTC seconds as double
    double accel_ms2[3];       // accel in sensor frame, m/s²
    double gyro_rads[3];       // gyro in sensor frame, rad/s
};

bool parse_airy_imu(const uint8_t* p, size_t n, AiryImuSample& out) {
    if (n < AIRY_IMU_PACKET_SIZE) return false;
    if (std::memcmp(p + OFF_MAGIC, AIRY_MAGIC, 4) != 0) return false;

    // Timestamp
    uint64_t sec  = rd_be_u48(p + OFF_SEC);
    uint32_t usec = rd_be_u32(p + OFF_USEC);
    if (static_cast<int64_t>(sec) < PTP_MIN_SEC) return false;  // PTP lock gate
    out.stamp_s = static_cast<double>(sec) + static_cast<double>(usec) * 1e-6;

    // FSR and unit conversion (rs_driver canonical)
    uint8_t acc_fsr = p[OFF_ACC_FSR];
    uint8_t gyr_fsr = p[OFF_GYR_FSR];
    if (acc_fsr > 3 || gyr_fsr > 3) return false;

    double accel_g_per_count = double(1u << (acc_fsr + 1)) / 32768.0;     // 2,4,8,16 g / 32768
    double gyro_dps_per_count = double(250u << gyr_fsr) / 32768.0;        // 250,500,1000,2000 dps / 32768
    double gyro_rad_per_count = gyro_dps_per_count * PI / 180.0;

    int32_t acc_x_raw = rd_be_i32(p + OFF_ACC_X);
    int32_t acc_y_raw = rd_be_i32(p + OFF_ACC_Y);
    int32_t acc_z_raw = rd_be_i32(p + OFF_ACC_Z);
    int32_t gyr_x_raw = rd_be_i32(p + OFF_GYR_X);
    int32_t gyr_y_raw = rd_be_i32(p + OFF_GYR_Y);
    int32_t gyr_z_raw = rd_be_i32(p + OFF_GYR_Z);

    out.accel_ms2[0] = double(acc_x_raw) * accel_g_per_count * G_MSS;
    out.accel_ms2[1] = double(acc_y_raw) * accel_g_per_count * G_MSS;
    out.accel_ms2[2] = double(acc_z_raw) * accel_g_per_count * G_MSS;
    out.gyro_rads[0] = double(gyr_x_raw) * gyro_rad_per_count;
    out.gyro_rads[1] = double(gyr_y_raw) * gyro_rad_per_count;
    out.gyro_rads[2] = double(gyr_z_raw) * gyro_rad_per_count;
    return true;
}

// ---------------------------------------------------------------------------
// Multicast socket
// ---------------------------------------------------------------------------

int open_multicast(const char* group, uint16_t port) {
    int fd = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (fd < 0) return -1;

    int reuse = 1;
    ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
#ifdef SO_REUSEPORT
    ::setsockopt(fd, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof(reuse));
#endif

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        ::close(fd);
        return -1;
    }

    ip_mreq mreq{};
    mreq.imr_multiaddr.s_addr = ::inet_addr(group);
    mreq.imr_interface.s_addr = htonl(INADDR_ANY);
    if (::setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0) {
        ::close(fd);
        return -1;
    }

    // 500ms recv timeout so we can poll g_running.
    timeval tv{};
    tv.tv_sec = 0;
    tv.tv_usec = 500000;
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    return fd;
}

// ---------------------------------------------------------------------------
// Globals + signal handling
// ---------------------------------------------------------------------------

std::atomic<bool> g_running{true};
void signal_handler(int) { g_running = false; }

}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    dimos::NativeModule mod(argc, argv);

    if (!mod.has("imu")) {
        std::fprintf(stderr, "Usage: %s --imu <topic> [--which front|rear]\n", argv[0]);
        return 1;
    }
    const std::string imu_topic = mod.topic("imu");
    const std::string which = mod.arg("which", "front");
    // When "sensor" is passed, publish accel+gyro in the Airy's own sensor
    // frame (no rotation). Used for the strict single-lidar clean test where
    // the lidar cloud is also kept in sensor frame (rsdriver extrinsic zeroed)
    // so FAST-LIO2's extrinsic is truly identity without any rotation-math
    // dependencies. Default "base_link" path applies R_base_from_{front|rear}.
    const std::string out_frame = mod.arg("frame", "base_link");

    const char*  mgroup;
    uint16_t     mport;
    const Mat3*  R_base_from_sensor;
    if (which == "front") {
        mgroup = MULTICAST_FRONT;
        mport  = PORT_FRONT;
        R_base_from_sensor = &R_BASE_FROM_FRONT;
    } else if (which == "rear") {
        mgroup = MULTICAST_REAR;
        mport  = PORT_REAR;
        R_base_from_sensor = &R_BASE_FROM_REAR;
    } else {
        std::fprintf(stderr, "--which must be 'front' or 'rear', got '%s'\n", which.c_str());
        return 1;
    }

    const bool rotate_to_base = (out_frame == "base_link");
    if (!rotate_to_base && out_frame != "sensor") {
        std::fprintf(stderr, "--frame must be 'base_link' or 'sensor', got '%s'\n", out_frame.c_str());
        return 1;
    }
    const char* header_frame_id = rotate_to_base ? "base_link" : "airy_sensor";

    lcm::LCM lcm;
    if (!lcm.good()) {
        std::fprintf(stderr, "[airy_imu_bridge] LCM init failed\n");
        return 1;
    }

    int sock = open_multicast(mgroup, mport);
    if (sock < 0) {
        std::fprintf(stderr, "[airy_imu_bridge] multicast bind failed on %s:%u: %s\n",
                     mgroup, mport, std::strerror(errno));
        return 1;
    }
    std::fprintf(stderr, "[airy_imu_bridge] %s Airy IMU → LCM topic '%s' (multicast %s:%u)\n",
                 which.c_str(), imu_topic.c_str(), mgroup, mport);

    uint8_t buf[128];
    uint64_t n_pkts = 0;
    uint64_t n_bad  = 0;
    uint64_t n_drop_ptp = 0;
    auto last_status = std::chrono::steady_clock::now();

    while (g_running) {
        ssize_t got = ::recv(sock, buf, sizeof(buf), 0);
        if (got < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
            std::fprintf(stderr, "[airy_imu_bridge] recv error: %s\n", std::strerror(errno));
            break;
        }
        if (got != static_cast<ssize_t>(AIRY_IMU_PACKET_SIZE)) {
            ++n_bad;
            continue;
        }

        AiryImuSample s;
        if (!parse_airy_imu(buf, got, s)) {
            ++n_drop_ptp;
            continue;
        }


        // Optionally rotate accel + gyro from sensor frame into base_link.
        // In `--frame sensor` mode, pass through unchanged so the downstream
        // FAST-LIO2 sees IMU and lidar cloud in the same raw Airy sensor
        // frame — the strict single-sensor clean test.
        //
        // NOTE: translation lever-arm term ω × (ω × r) + α × r is NOT subtracted
        // here when rotating to base_link. For front Airy at (+0.32, 0, -0.013)
        // m from base, centripetal at |ω|=2 rad/s contributes ~1.3 m/s² on
        // worst axis — FAST-LIO2's extrinsic_est_en should soak it up.
        double ax, ay, az, gx, gy, gz;
        if (rotate_to_base) {
            rotate(*R_base_from_sensor, s.accel_ms2[0], s.accel_ms2[1], s.accel_ms2[2], ax, ay, az);
            rotate(*R_base_from_sensor, s.gyro_rads[0], s.gyro_rads[1], s.gyro_rads[2], gx, gy, gz);
        } else {
            ax = s.accel_ms2[0]; ay = s.accel_ms2[1]; az = s.accel_ms2[2];
            gx = s.gyro_rads[0]; gy = s.gyro_rads[1]; gz = s.gyro_rads[2];
        }

        // Subtract persistent stationary accel bias in base_link frame.
        // Measured empirically on this M20 (robot at rest): front Airy rotated
        // into base_link gives ~(0.13, 0.35, 10.03) m/s² — a 0.13 m/s² x bias,
        // 0.35 y bias, and 0.22 z bias (= |a|-g). Integrating the y-axis bias
        // twice over 180s was producing thousands of meters of drift. FAST-
        // LIO's init averages mean_acc for its gravity estimate; subtracting
        // here before FAST-LIO sees the data leaves a cleaner gravity vector
        // and a smaller initial bias for the EKF's random-walk state to
        // track. Skip the correction in sensor frame since it's a base-frame
        // observation; revisit with a per-robot calibration file later.
        if (rotate_to_base && which == "front") {
            ax -= 0.13;
            ay -= 0.35;
            az -= 0.22;
        }

        sensor_msgs::Imu msg;
        msg.header = dimos::make_header(header_frame_id, s.stamp_s);
        // Airy IMU packets carry no orientation — mark unavailable per ROS convention.
        msg.orientation.x = 0;
        msg.orientation.y = 0;
        msg.orientation.z = 0;
        msg.orientation.w = 1;
        msg.orientation_covariance[0] = -1.0;
        for (int i = 1; i < 9; ++i) msg.orientation_covariance[i] = 0.0;
        msg.angular_velocity.x = gx;
        msg.angular_velocity.y = gy;
        msg.angular_velocity.z = gz;
        for (int i = 0; i < 9; ++i) msg.angular_velocity_covariance[i] = 0.0;
        msg.linear_acceleration.x = ax;
        msg.linear_acceleration.y = ay;
        msg.linear_acceleration.z = az;
        for (int i = 0; i < 9; ++i) msg.linear_acceleration_covariance[i] = 0.0;

        lcm.publish(imu_topic, &msg);
        ++n_pkts;

        auto now = std::chrono::steady_clock::now();
        if (now - last_status >= std::chrono::seconds(5)) {
            double rate = n_pkts / 5.0;
            // Also dump wall-clock-vs-packet-stamp skew so we can diagnose
            // whether imu_latest stalls seen downstream originate here (LCM
            // publish delay in the bridge) or in fastlio2's subscriber path.
            const double wall_now = static_cast<double>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count()) * 1e-9;
            const double pkt_vs_wall = s.stamp_s - wall_now;
            std::fprintf(stderr, "[airy_imu_bridge] %.1f Hz over 5s (pkts=%lu bad=%lu ptp_drop=%lu) "
                                 "last acc=(%.3f,%.3f,%.3f) gyro=(%.4f,%.4f,%.4f) "
                                 "pkt_vs_wall=%+.3fs\n",
                         rate,
                         static_cast<unsigned long>(n_pkts),
                         static_cast<unsigned long>(n_bad),
                         static_cast<unsigned long>(n_drop_ptp),
                         ax, ay, az, gx, gy, gz, pkt_vs_wall);
            n_pkts = 0;
            last_status = now;
        }
    }

    ::close(sock);
    std::fprintf(stderr, "[airy_imu_bridge] Shutting down\n");
    return 0;
}
