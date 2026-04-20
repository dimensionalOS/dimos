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
// with the Airy lidar optics. The IMU→base rotation is composed from two
// pieces: (1) R_imu_to_lidar, a per-unit factory calibration read from
// DIFOP register C.17 (quaternion at packet offset 1092), and (2)
// R_base_lidar, the physical mount rotation determined by the M20 chassis
// geometry (hardcoded per front/rear). DIFOP is ~1 Hz so we start in a
// degraded mode with R_imu_to_lidar = identity and hot-swap once the
// factory cal arrives. This gives FAST-LIO2 a single, physically
// consistent IMU stream in base_link frame.
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
#include <thread>

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

// DIFOP (device info) packets are ~1 Hz on a separate port. Register C.17
// (IMU_CALIB_DATA) holds the factory IMU→lidar quaternion + lever-arm at
// packet offset 1092.
constexpr uint16_t DIFOP_PORT_FRONT = 7781;
constexpr uint16_t DIFOP_PORT_REAR  = 7782;
constexpr size_t   DIFOP_PACKET_SIZE = 1248;
constexpr size_t   DIFOP_C17_OFFSET  = 1092;  // 7×uint32 BE → float32
constexpr uint8_t  DIFOP_MAGIC[8] = {0xA5, 0xFF, 0x00, 0x5A, 0x11, 0x11, 0x55, 0x55};
constexpr size_t   DIFOP_TAIL_OFFSET = 1246;
constexpr uint8_t  DIFOP_TAIL[2]  = {0x0F, 0xF0};

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
// Rotation matrices — Airy lidar frame → M20 base_link.
//
// These describe the PHYSICAL mount of the lidar housing on the M20 and are
// invariant across units / temperature / time. The IMU-to-lidar rotation is
// a separate per-unit factory calibration read from DIFOP register C.17 at
// runtime (see read_difop_c17()).
//
//   FRONT:  lidar dome → base +X (forward), cable port → base -Z (down).
//           Mapping: lidar X → base -Y, lidar Y → base -Z, lidar Z → base +X.
//   REAR:   housing 180° about Z relative to front: dome → base -X,
//           cable → base -Z. Mapping: lidar X → base +Y, lidar Y → base -Z,
//           lidar Z → base -X.
//
// Columns of R_BASE_LIDAR_* are lidar axes expressed in base_link.
// ---------------------------------------------------------------------------

struct Mat3 { double m[3][3]; };

constexpr Mat3 R_BASE_LIDAR_FRONT = {{
    {0,  0,  1},
    {-1, 0,  0},
    {0, -1,  0},
}};

constexpr Mat3 R_BASE_LIDAR_REAR = {{
    {0,  0, -1},
    {1,  0,  0},
    {0, -1,  0},
}};

void rotate(const Mat3& R, double vx, double vy, double vz, double& ox, double& oy, double& oz) {
    ox = R.m[0][0] * vx + R.m[0][1] * vy + R.m[0][2] * vz;
    oy = R.m[1][0] * vx + R.m[1][1] * vy + R.m[1][2] * vz;
    oz = R.m[2][0] * vx + R.m[2][1] * vy + R.m[2][2] * vz;
}

Mat3 matmul(const Mat3& A, const Mat3& B) {
    Mat3 C{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            C.m[i][j] = A.m[i][0]*B.m[0][j] + A.m[i][1]*B.m[1][j] + A.m[i][2]*B.m[2][j];
    return C;
}

double frobenius_distance_from_identity(const Mat3& R) {
    double s = 0.0;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            double d = R.m[i][j] - (i == j ? 1.0 : 0.0);
            s += d * d;
        }
    return std::sqrt(s);
}

// ---------------------------------------------------------------------------
// DIFOP C.17 parsing: 7 × uint32 big-endian at offset 1092, bit-cast to
// float32. Layout: q_x, q_y, q_z, q_w, t_x, t_y, t_z. Quaternion is
// IMU→lidar (inferred — not publicly documented); direction can be
// disambiguated via a quasi-static yaw test after calibration is applied.
// ---------------------------------------------------------------------------

struct DifopCalib {
    double qx, qy, qz, qw;  // normalized
    double tx, ty, tz;      // lever-arm, meters (unused in v1)
};

float bit_cast_u32_to_f32(uint32_t u) {
    float f;
    std::memcpy(&f, &u, sizeof(f));
    return f;
}

bool parse_difop_c17(const uint8_t* p, size_t n, DifopCalib& out) {
    if (n < DIFOP_PACKET_SIZE) return false;
    if (std::memcmp(p + 0, DIFOP_MAGIC, sizeof(DIFOP_MAGIC)) != 0) return false;
    if (std::memcmp(p + DIFOP_TAIL_OFFSET, DIFOP_TAIL, sizeof(DIFOP_TAIL)) != 0) return false;

    const uint8_t* q = p + DIFOP_C17_OFFSET;
    float f[7];
    for (int i = 0; i < 7; ++i) {
        f[i] = bit_cast_u32_to_f32(rd_be_u32(q + i * 4));
    }
    for (int i = 0; i < 7; ++i) {
        if (!std::isfinite(f[i])) return false;
    }

    double qx = f[0], qy = f[1], qz = f[2], qw = f[3];
    double norm_sq = qx*qx + qy*qy + qz*qz + qw*qw;
    if (norm_sq < 0.25) return false;  // clearly junk (all zeros etc.)
    double norm = std::sqrt(norm_sq);
    if (std::fabs(norm - 1.0) > 0.01) {
        // Reject rather than silently renormalize — a large norm deviation is
        // our signal that we're reading junk (wrong offset, byte order, etc.)
        // or that direction-convention inference is off. Safer to stay in
        // degraded mode than hot-swap a bogus rotation.
        std::fprintf(stderr, "[airy_imu_bridge] DIFOP C.17 quaternion norm %.4f off by >0.01 — "
                             "rejecting packet, keeping cal=PENDING\n", norm);
        return false;
    }
    out.qx = qx / norm;
    out.qy = qy / norm;
    out.qz = qz / norm;
    out.qw = qw / norm;
    out.tx = f[4];
    out.ty = f[5];
    out.tz = f[6];
    return true;
}

Mat3 quat_to_matrix(const DifopCalib& q) {
    const double x = q.qx, y = q.qy, z = q.qz, w = q.qw;
    const double xx = x*x, yy = y*y, zz = z*z;
    const double xy = x*y, xz = x*z, yz = y*z;
    const double wx = w*x, wy = w*y, wz = w*z;
    Mat3 R{};
    R.m[0][0] = 1 - 2*(yy + zz);
    R.m[0][1] =     2*(xy - wz);
    R.m[0][2] =     2*(xz + wy);
    R.m[1][0] =     2*(xy + wz);
    R.m[1][1] = 1 - 2*(xx + zz);
    R.m[1][2] =     2*(yz - wx);
    R.m[2][0] =     2*(xz - wy);
    R.m[2][1] =     2*(yz + wx);
    R.m[2][2] = 1 - 2*(xx + yy);
    return R;
}

void log_matrix(const char* label, const Mat3& R) {
    std::fprintf(stderr, "[airy_imu_bridge] %s =\n"
                         "  [% .5f % .5f % .5f]\n"
                         "  [% .5f % .5f % .5f]\n"
                         "  [% .5f % .5f % .5f]\n",
                 label,
                 R.m[0][0], R.m[0][1], R.m[0][2],
                 R.m[1][0], R.m[1][1], R.m[1][2],
                 R.m[2][0], R.m[2][1], R.m[2][2]);
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

// Hot-swappable R_imu_to_base. Initialized at startup to R_base_lidar (i.e.
// R_imu_to_lidar = identity → degraded mode), replaced when DIFOP C.17
// arrives. Pointer ownership: we allocate exactly two Mat3 over the
// process lifetime (initial + one DIFOP swap) and deliberately never free
// the stale pointer to avoid racing the IMU hot path's dereference.
std::atomic<Mat3*> g_R_imu_to_base{nullptr};
std::atomic<bool>  g_cal_ok{false};

// Background retry thread: polls DIFOP multicast every ~2s until a valid
// C.17 record arrives, then hot-swaps g_R_imu_to_base and sets g_cal_ok.
void difop_retry_loop(const char* mgroup, uint16_t difop_port, Mat3 R_base_lidar,
                      const std::string& which) {
    using namespace std::chrono_literals;
    while (g_running) {
        int sock = open_multicast(mgroup, difop_port);
        if (sock < 0) {
            std::fprintf(stderr, "[airy_imu_bridge] DIFOP socket bind failed on %s:%u: %s — "
                                 "retrying in 2s\n", mgroup, difop_port, std::strerror(errno));
            for (int i = 0; i < 20 && g_running; ++i) std::this_thread::sleep_for(100ms);
            continue;
        }

        uint8_t buf[2048];
        bool got_it = false;
        auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (g_running && std::chrono::steady_clock::now() < deadline) {
            ssize_t n = ::recv(sock, buf, sizeof(buf), 0);
            if (n < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
                break;
            }
            if (n != static_cast<ssize_t>(DIFOP_PACKET_SIZE)) continue;

            DifopCalib cal;
            if (!parse_difop_c17(buf, n, cal)) continue;

            Mat3 R_imu_to_lidar = quat_to_matrix(cal);
            Mat3 R_imu_to_base  = matmul(R_base_lidar, R_imu_to_lidar);
            double dist = frobenius_distance_from_identity(R_imu_to_lidar);

            std::fprintf(stderr,
                "[airy_imu_bridge] DIFOP C.17 received for %s Airy: "
                "q=(%.5f,%.5f,%.5f,%.5f) lever=(%.4f,%.4f,%.4f) "
                "|R_imu_to_lidar - I|_F=%.4f — cal=OK\n",
                which.c_str(), cal.qx, cal.qy, cal.qz, cal.qw,
                cal.tx, cal.ty, cal.tz, dist);
            log_matrix("R_imu_to_lidar (from C.17)", R_imu_to_lidar);
            log_matrix("R_imu_to_base (composed)",   R_imu_to_base);

            Mat3* fresh = new Mat3(R_imu_to_base);
            g_R_imu_to_base.store(fresh, std::memory_order_release);
            g_cal_ok.store(true, std::memory_order_release);
            got_it = true;
            break;
        }
        ::close(sock);
        if (got_it) return;

        for (int i = 0; i < 20 && g_running; ++i) std::this_thread::sleep_for(100ms);
    }
}

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
    // dependencies. Default "base_link" path applies R_imu_to_base composed
    // from R_base_lidar × R_imu_to_lidar (DIFOP C.17).
    const std::string out_frame = mod.arg("frame", "base_link");

    const char*  mgroup;
    uint16_t     mport;
    uint16_t     difop_port;
    Mat3         R_base_lidar;
    if (which == "front") {
        mgroup = MULTICAST_FRONT;
        mport  = PORT_FRONT;
        difop_port = DIFOP_PORT_FRONT;
        R_base_lidar = R_BASE_LIDAR_FRONT;
    } else if (which == "rear") {
        mgroup = MULTICAST_REAR;
        mport  = PORT_REAR;
        difop_port = DIFOP_PORT_REAR;
        R_base_lidar = R_BASE_LIDAR_REAR;
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

    // Seed g_R_imu_to_base with R_base_lidar (= R_base_lidar · I): assumes
    // IMU frame is perfectly aligned with lidar frame until DIFOP arrives.
    // Accepting degraded boot over blocking — factory pilot robustness.
    std::thread difop_thread;
    if (rotate_to_base) {
        Mat3* seed = new Mat3(R_base_lidar);
        g_R_imu_to_base.store(seed, std::memory_order_release);
        std::fprintf(stderr, "[airy_imu_bridge] DIFOP not yet received — running DEGRADED "
                             "(no factory IMU-lidar calibration applied). Drift may be large.\n");
        difop_thread = std::thread(difop_retry_loop, mgroup, difop_port, R_base_lidar, which);
    }

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
            // Signal the DIFOP retry thread so join() below doesn't hang.
            g_running.store(false, std::memory_order_release);
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


        // Rotate accel + gyro from IMU frame into base_link via the
        // hot-swappable R_imu_to_base (= R_base_lidar · R_imu_to_lidar).
        // R_imu_to_lidar comes from DIFOP C.17 factory calibration; until
        // DIFOP arrives, the atomic pointer holds R_base_lidar alone (i.e.
        // R_imu_to_lidar = identity → degraded). In `--frame sensor` mode
        // we pass through unchanged so downstream FAST-LIO2 sees IMU and
        // lidar cloud in the raw Airy sensor frame.
        //
        // NOTE: the translation lever-arm term ω × (ω × r) + α × r is NOT
        // subtracted here. For front Airy at ~(+0.32, 0, -0.013) m from
        // base, centripetal at |ω|=2 rad/s contributes ~1.3 m/s² worst-
        // axis — deferred to v2 (see FASTLIO2_LOG Finding #21).
        double ax, ay, az, gx, gy, gz;
        if (rotate_to_base) {
            const Mat3* R = g_R_imu_to_base.load(std::memory_order_acquire);
            rotate(*R, s.accel_ms2[0], s.accel_ms2[1], s.accel_ms2[2], ax, ay, az);
            rotate(*R, s.gyro_rads[0], s.gyro_rads[1], s.gyro_rads[2], gx, gy, gz);
        } else {
            ax = s.accel_ms2[0]; ay = s.accel_ms2[1]; az = s.accel_ms2[2];
            gx = s.gyro_rads[0]; gy = s.gyro_rads[1]; gz = s.gyro_rads[2];
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
            const char* cal_tag = rotate_to_base
                ? (g_cal_ok.load(std::memory_order_acquire) ? "cal=OK" : "cal=PENDING")
                : "cal=N/A";
            std::fprintf(stderr, "[airy_imu_bridge] %.1f Hz over 5s (pkts=%lu bad=%lu ptp_drop=%lu) "
                                 "last acc=(%.3f,%.3f,%.3f) gyro=(%.4f,%.4f,%.4f) "
                                 "pkt_vs_wall=%+.3fs %s\n",
                         rate,
                         static_cast<unsigned long>(n_pkts),
                         static_cast<unsigned long>(n_bad),
                         static_cast<unsigned long>(n_drop_ptp),
                         ax, ay, az, gx, gy, gz, pkt_vs_wall, cal_tag);
            n_pkts = 0;
            last_status = now;
        }
    }

    ::close(sock);
    if (difop_thread.joinable()) difop_thread.join();
    std::fprintf(stderr, "[airy_imu_bridge] Shutting down\n");
    return 0;
}
