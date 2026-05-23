// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// FAST-LIO2 + Livox Mid-360 native module for dimos NativeModule framework.
//
// Thin assembler: each subsystem parses its own CLI surface via a
// Config::from_args(mod) factory; main wires them together and runs the
// FAST-LIO processing loop.
//
// Usage:
//   ./fastlio2_native \
//       --lidar '/lidar#sensor_msgs.PointCloud2' \
//       --odometry '/odometry#nav_msgs.Odometry' \
//       --config_path /path/to/mid360.yaml \
//       --host_ip 192.168.1.5 --lidar_ip 192.168.1.155

#include <lcm/lcm-cpp.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <string>
#include <thread>

#include "cloud_filter.hpp"
#include "dimos_native_module.hpp"
#include "fast_lio.hpp"
#include "fast_lio_debug.hpp"
#include "fastlio_publisher.hpp"
#include "livox_driver_source.hpp"

// Signal-handler state must cross the C boundary, so it stays file-scope.
static std::atomic<bool> g_running{true};
static void signal_handler(int /*sig*/) { g_running.store(false); }

static void print_banner(bool debug,
                         const std::string& config_path,
                         const CloudFilterConfig& filter_cfg,
                         const LivoxDriverSource::Config& driver_cfg,
                         const FastlioPublisher::Config& pub_cfg) {
    if (!debug) return;
    std::printf("[fastlio2] Starting FAST-LIO2 + Livox Mid-360 native module\n");
    if (pub_cfg.init_pose.has_offset()) {
        const auto& ip = pub_cfg.init_pose;
        std::printf("[fastlio2] init_pose: xyz=(%.3f, %.3f, %.3f) quat=(%.4f, %.4f, %.4f, %.4f)\n",
                    ip.x, ip.y, ip.z, ip.qx, ip.qy, ip.qz, ip.qw);
    }
    std::printf("[fastlio2] lidar topic: %s\n",
                pub_cfg.lidar_topic.empty() ? "(disabled)" : pub_cfg.lidar_topic.c_str());
    std::printf("[fastlio2] odometry topic: %s\n",
                pub_cfg.odometry_topic.empty() ? "(disabled)" : pub_cfg.odometry_topic.c_str());
    std::printf("[fastlio2] global_map topic: %s\n",
                pub_cfg.map_topic.empty() ? "(disabled)" : pub_cfg.map_topic.c_str());
    std::printf("[fastlio2] config: %s\n", config_path.c_str());
    std::printf("[fastlio2] host_ip: %s  lidar_ip: %s  frequency: %.1f Hz\n",
                driver_cfg.host_ip.c_str(), driver_cfg.lidar_ip.c_str(),
                driver_cfg.frame_frequency_hz);
    std::printf("[fastlio2] pointcloud_freq: %.1f Hz  odom_freq: %.1f Hz\n",
                pub_cfg.pointcloud_freq, pub_cfg.odom_freq);
    std::printf("[fastlio2] voxel_size: %.3f  sor_mean_k: %d  sor_stddev: %.1f\n",
                filter_cfg.voxel_size, filter_cfg.sor_mean_k, filter_cfg.sor_stddev);
    if (pub_cfg.enable_map())
        std::printf("[fastlio2] map_voxel_size: %.3f  map_max_range: %.1f  map_freq: %.1f Hz\n",
                    pub_cfg.map_voxel_size, pub_cfg.map_max_range, pub_cfg.map_freq);
}


static void run_loop(LivoxDriverSource& source,
                     FastLio& fast_lio,
                     const CloudFilterConfig& filter_cfg,
                     FastlioPublisher& publisher,
                     lcm::LCM& lcm,
                     double main_freq) {
    const double process_period_ms = 1000.0 / main_freq;

    while (g_running.load()) {
        const auto loop_start = std::chrono::high_resolution_clock::now();
        const auto now = std::chrono::steady_clock::now();

        source.tick(now);
        fast_lio.process();

        auto pose = fast_lio.get_pose();
        if (!pose.empty() && (pose[0] != 0.0 || pose[1] != 0.0 || pose[2] != 0.0)) {
            const double ts = std::chrono::duration<double>(
                std::chrono::system_clock::now().time_since_epoch()).count();

            auto world_cloud = fast_lio.get_world_cloud();
            if (world_cloud && !world_cloud->empty()) {
                auto filtered = filter_cloud<PointType>(world_cloud, filter_cfg);
                publisher.publish_lidar_if_due(now, filtered, ts);
                publisher.publish_map_if_due(now, filtered, pose, ts);
            }
            publisher.publish_odom_if_due(now, fast_lio.get_odometry(), ts);
        }

        lcm.handleTimeout(0);

        const auto loop_end = std::chrono::high_resolution_clock::now();
        const auto elapsed_ms =
            std::chrono::duration<double, std::milli>(loop_end - loop_start).count();
        if (elapsed_ms < process_period_ms) {
            std::this_thread::sleep_for(std::chrono::microseconds(
                static_cast<int64_t>((process_period_ms - elapsed_ms) * 1000)));
        }
    }
}

int main(int argc, char** argv) {
    dimos::NativeModule mod(argc, argv);

    // FAST-LIO core knobs (small enough to keep inline)
    const std::string config_path = mod.arg("config_path", "");
    if (config_path.empty()) {
        std::fprintf(stderr, "Error: --config_path <path> is required\n");
        return 1;
    }

    // Per-scan filter — lives at the call site between source and publisher.
    const auto filter_cfg = CloudFilterConfig::from_args(mod);

    const bool debug = mod.arg_bool("debug", false);
    fastlio_debug = debug;  // FAST-LIO core's verbosity global

    const auto driver_cfg = LivoxDriverSource::Config::from_args(mod);
    const auto pub_cfg    = FastlioPublisher::Config::from_args(mod);  // throws if no output topic

    print_banner(debug, config_path, filter_cfg, driver_cfg, pub_cfg);

    std::signal(SIGTERM, signal_handler);
    std::signal(SIGINT,  signal_handler);

    lcm::LCM lcm;
    if (!lcm.good()) {
        std::fprintf(stderr, "Error: LCM init failed\n");
        return 1;
    }

    if (debug) std::printf("[fastlio2] Initializing FAST-LIO...\n");
    const double msr_freq  = mod.arg_float("msr_freq",  50.0f);
    const double main_freq = mod.arg_float("main_freq", 5000.0f);
    FastLio fast_lio(config_path, msr_freq, main_freq);
    if (debug) std::printf("[fastlio2] FAST-LIO initialized.\n");

    FastlioPublisher publisher(lcm, pub_cfg);
    LivoxDriverSource source(&fast_lio, driver_cfg, g_running);

    if (!source.start()) return 1;
    if (debug) std::printf("[fastlio2] SDK started, waiting for device...\n");

    // Main loop runs the FAST-LIO processing and publishes outputs 
    run_loop(source, fast_lio, filter_cfg, publisher, lcm, main_freq);

    if (debug) std::printf("[fastlio2] Shutting down...\n");
    source.stop();
    if (debug) std::printf("[fastlio2] Done.\n");

    return 0;
}
