// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Shared Livox SDK2 configuration utilities for dimos native modules.
// Used by both mid360_native and fastlio2_native.

#pragma once

#include <livox_lidar_api.h>
#include <livox_lidar_def.h>

#include <sys/mman.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <string>
#include <utility>

namespace livox_common {

// Gravity constant for converting accelerometer data from g to m/s^2
inline constexpr double GRAVITY_MS2 = 9.80665;

// Livox data_type values (not provided as named constants in SDK2 header)
inline constexpr uint8_t DATA_TYPE_IMU = 0x00;
inline constexpr uint8_t DATA_TYPE_CARTESIAN_HIGH = 0x01;
inline constexpr uint8_t DATA_TYPE_CARTESIAN_LOW = 0x02;

// SDK network port configuration for Livox Mid-360
struct SdkPorts {
    int cmd_data      = 56100;
    int push_msg      = 56200;
    int point_data    = 56300;
    int imu_data      = 56400;
    int log_data      = 56500;
    int host_cmd_data   = 56101;
    int host_push_msg   = 56201;
    int host_point_data = 56301;
    int host_imu_data   = 56401;
    int host_log_data   = 56501;
};

// Write Livox SDK JSON config to an in-memory (or ephemeral) file.
// Returns {fd, path} — caller must close(fd) after LivoxLidarSdkInit reads it.
//
// Linux: memfd_create gives us a pure anonymous in-memory file, reached via
//        /proc/self/fd/<fd>.
// macOS: no memfd_create and no procfs.  We fall back to mkstemp() in /tmp
//        and immediately unlink() the directory entry, so the inode lives
//        only as long as the fd is open.  The SDK reaches it via /dev/fd/<fd>
//        (Darwin's equivalent of /proc/self/fd).
inline std::pair<int, std::string> write_sdk_config(const std::string& host_ip,
                                                     const std::string& lidar_ip,
                                                     const SdkPorts& ports) {
#ifdef __linux__
    int fd = memfd_create("livox_sdk_config", 0);
    if (fd < 0) {
        perror("memfd_create");
        return {-1, ""};
    }
#elif defined(__APPLE__) && defined(__MACH__)
    // mkstemp replaces the 6 X's in place — e.g. livox_sdk_config.aB3xY9.
    // Honor $TMPDIR when set (sandboxed macOS apps and CI runners point
    // it at a per-process scratch dir); fall back to /tmp.
    const char* tmpdir = std::getenv("TMPDIR");
    if (tmpdir == nullptr || tmpdir[0] == '\0') {
        tmpdir = "/tmp";
    }
    char tmpl[256];
    snprintf(tmpl, sizeof(tmpl), "%s/livox_sdk_config.XXXXXX", tmpdir);
    int fd = mkstemp(tmpl);
    if (fd < 0) {
        perror("mkstemp");
        return {-1, ""};
    }
    // Drop the directory entry — the inode stays alive via the fd.
    unlink(tmpl);
#else
#error "livox_sdk_config: unsupported platform (need Linux memfd_create or Apple mkstemp)"
#endif

    FILE* fp = fdopen(fd, "w");
    if (!fp) {
        perror("fdopen");
        close(fd);
        return {-1, ""};
    }

    fprintf(fp,
        "{\n"
        "  \"MID360\": {\n"
        "    \"lidar_net_info\": {\n"
        "      \"cmd_data_port\": %d,\n"
        "      \"push_msg_port\": %d,\n"
        "      \"point_data_port\": %d,\n"
        "      \"imu_data_port\": %d,\n"
        "      \"log_data_port\": %d\n"
        "    },\n"
        "    \"host_net_info\": [\n"
        "      {\n"
        "        \"host_ip\": \"%s\",\n"
        "        \"multicast_ip\": \"224.1.1.5\",\n"
        "        \"cmd_data_port\": %d,\n"
        "        \"push_msg_port\": %d,\n"
        "        \"point_data_port\": %d,\n"
        "        \"imu_data_port\": %d,\n"
        "        \"log_data_port\": %d\n"
        "      }\n"
        "    ]\n"
        "  }\n"
        "}\n",
        ports.cmd_data, ports.push_msg, ports.point_data,
        ports.imu_data, ports.log_data,
        host_ip.c_str(),
        ports.host_cmd_data, ports.host_push_msg, ports.host_point_data,
        ports.host_imu_data, ports.host_log_data);
    fflush(fp);  // flush but don't fclose — that would close fd

    char path[64];
#ifdef __linux__
    snprintf(path, sizeof(path), "/proc/self/fd/%d", fd);
#elif defined(__APPLE__) && defined(__MACH__)
    // Darwin's /dev/fd/<fd> may share the underlying open file description,
    // so rewind before the SDK reads from the path.
    lseek(fd, 0, SEEK_SET);
    snprintf(path, sizeof(path), "/dev/fd/%d", fd);
#else
    #error "Unsupported platform: expected Linux or macOS"
#endif
    return {fd, path};
}

// Initialize Livox SDK from in-memory config.
// Returns true on success. Handles fd lifecycle internally.
inline bool init_livox_sdk(const std::string& host_ip,
                           const std::string& lidar_ip,
                           const SdkPorts& ports) {
    auto [fd, path] = write_sdk_config(host_ip, lidar_ip, ports);
    if (fd < 0) {
        fprintf(stderr, "Error: failed to write SDK config\n");
        return false;
    }

    bool ok = LivoxLidarSdkInit(path.c_str(), host_ip.c_str());
    close(fd);

    if (!ok) {
        fprintf(stderr, "Error: LivoxLidarSdkInit failed\n");
    }
    return ok;
}

}  // namespace livox_common
