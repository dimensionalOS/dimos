// Copyright 2026 Dimensional Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// Time cuVSLAM with nothing else in the loop.
///
/// The dimos module reaches the same tracker through sqlite, lz4 and an LCM round
/// trip of two 400 kB images per frame, and that path wedges partway through every
/// recording -- so timing it measures the transport. This reads pre-flattened mono8
/// planes from disk and calls Track() directly, which is the only way to say what
/// the algorithm itself costs.
///
/// Images are all read into RAM before the loop starts, so disk never appears in a
/// per-frame time.

#include <chrono>
#include <stdexcept>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "cuvslam/cuvslam2.h"

namespace {

constexpr double kNsPerSec = 1e9;

struct Frame {
    int index;
    std::int64_t left_ns;
    std::int64_t right_ns;
};

struct ImuSample {
    std::int64_t ts_ns;
    float gyro[3];
    float accel[3];
};

/// Minimal reader for the flat manifest the exporter writes; a JSON dependency
/// would be more machinery than a handful of numbers is worth.
double manifest_value(const std::string& text, const std::string& key) {
    const std::size_t at = text.find("\"" + key + "\"");
    if (at == std::string::npos) {
        return 0.0;
    }
    const std::size_t colon = text.find(':', at);
    return std::strtod(text.c_str() + colon + 1, nullptr);
}

// JSON booleans are not numbers, and strtod("true") is 0 -- reading a flag with
// manifest_value silently disables it.
bool manifest_flag(const std::string& text, const std::string& key) {
    const std::size_t at = text.find("\"" + key + "\"");
    if (at == std::string::npos) {
        return false;
    }
    const std::size_t colon = text.find(':', at);
    return text.find("true", colon) == text.find_first_not_of(" \t", colon + 1);
}

std::vector<std::uint8_t> read_plane(const std::string& path, std::size_t bytes) {
    std::vector<std::uint8_t> buffer(bytes);
    std::ifstream file(path, std::ios::binary);
    file.read(reinterpret_cast<char*>(buffer.data()), static_cast<std::streamsize>(bytes));
    if (!file) {
        buffer.clear();
    }
    return buffer;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: bench_cuvslam <export_dir> [--no-imu]\n");
        return 2;
    }
    const std::string dir = argv[1];
    bool use_imu = true;
    int imu_stride = 1;
    float imu_freq = 0.0f;  // 0 = derive from stride
    double imu_offset_s = 0.0;
    bool sync_sba = false;
    int start_frame = 0;
    bool debug_imu = false;
    std::string dump_dir;
    bool state_debug = false;
    int verbosity = 0;
    // Kalibr T_cam0_imu as-is. An earlier 180 deg X flip was wrong: with fusion
    // actually running it diverges (velocity ~99 m/s, gravity -Y), while this gives
    // gravity (0.06, 9.34, 3.00) and sane biases.
    float imu_quat[4] = {0.001197f, -0.003184f, -0.003371f, 0.999989f};
    for (int arg = 2; arg < argc; ++arg) {
        if (std::strcmp(argv[arg], "--no-imu") == 0) {
            use_imu = false;
        } else if (std::strcmp(argv[arg], "--imu-stride") == 0 && arg + 1 < argc) {
            imu_stride = std::atoi(argv[++arg]);
        } else if (std::strcmp(argv[arg], "--imu-freq") == 0 && arg + 1 < argc) {
            imu_freq = static_cast<float>(std::atof(argv[++arg]));
        } else if (std::strcmp(argv[arg], "--imu-offset") == 0 && arg + 1 < argc) {
            imu_offset_s = std::atof(argv[++arg]);
        } else if (std::strcmp(argv[arg], "--sync-sba") == 0) {
            sync_sba = true;
        } else if (std::strcmp(argv[arg], "--start-frame") == 0 && arg + 1 < argc) {
            start_frame = std::atoi(argv[++arg]);
        } else if (std::strcmp(argv[arg], "--imu-quat") == 0 && arg + 1 < argc) {
            std::sscanf(argv[++arg], "%f,%f,%f,%f", &imu_quat[0], &imu_quat[1], &imu_quat[2],
                        &imu_quat[3]);
        } else if (std::strcmp(argv[arg], "--verbosity") == 0 && arg + 1 < argc) {
            verbosity = std::atoi(argv[++arg]);
        } else if (std::strcmp(argv[arg], "--state-debug") == 0) {
            state_debug = true;
        } else if (std::strcmp(argv[arg], "--debug-imu") == 0) {
            debug_imu = true;
        } else if (std::strcmp(argv[arg], "--dump-edex") == 0 && arg + 1 < argc) {
            dump_dir = argv[++arg];
        }
    }

    cuvslam::SetVerbosity(verbosity);

    std::ifstream manifest_file(dir + "/manifest.json");
    std::stringstream manifest_buffer;
    manifest_buffer << manifest_file.rdbuf();
    const std::string manifest = manifest_buffer.str();
    const int width = static_cast<int>(manifest_value(manifest, "width"));
    const int height = static_cast<int>(manifest_value(manifest, "height"));
    const double baseline = manifest_value(manifest, "baseline_m");

    std::vector<Frame> frames;
    {
        std::ifstream file(dir + "/frames.csv");
        std::string line;
        std::getline(file, line);
        while (std::getline(file, line)) {
            Frame frame{};
            if (std::sscanf(line.c_str(), "%d,%ld,%ld", &frame.index, &frame.left_ns,
                            &frame.right_ns) == 3) {
                frames.push_back(frame);
            }
        }
    }

    std::vector<ImuSample> imu;
    if (use_imu) {
        std::ifstream file(dir + "/imu.csv");
        std::string line;
        std::getline(file, line);
        while (std::getline(file, line)) {
            ImuSample sample{};
            if (std::sscanf(line.c_str(), "%ld,%f,%f,%f,%f,%f,%f", &sample.ts_ns, &sample.gyro[0],
                            &sample.gyro[1], &sample.gyro[2], &sample.accel[0], &sample.accel[1],
                            &sample.accel[2]) == 7) {
                sample.ts_ns += static_cast<std::int64_t>(imu_offset_s * kNsPerSec);
                imu.push_back(sample);
            }
        }
    }
    std::printf("%zu frames, %zu imu samples, %dx%d, baseline %.2f mm\n", frames.size(), imu.size(),
                width, height, baseline * 1000.0);

    const std::size_t plane = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    std::vector<std::vector<std::uint8_t>> left_planes;
    std::vector<std::vector<std::uint8_t>> right_planes;
    left_planes.reserve(frames.size());
    right_planes.reserve(frames.size());
    for (const Frame& frame : frames) {
        char name[32];
        std::snprintf(name, sizeof(name), "/%06d.raw", frame.index);
        left_planes.push_back(read_plane(dir + "/left" + name, plane));
        right_planes.push_back(read_plane(dir + "/right" + name, plane));
    }
    std::printf("images resident: %.2f GB\n",
                static_cast<double>(2 * plane * frames.size()) / 1e9);

    cuvslam::Camera left{};
    left.size = {width, height};
    left.principal = {static_cast<float>(manifest_value(manifest, "cx")),
                      static_cast<float>(manifest_value(manifest, "cy"))};
    left.focal = {static_cast<float>(manifest_value(manifest, "fx")),
                  static_cast<float>(manifest_value(manifest, "fy"))};
    left.rig_from_camera = cuvslam::Pose{};
    left.distortion = cuvslam::Distortion{cuvslam::Distortion::Model::Pinhole};
    cuvslam::Camera right = left;
    right.rig_from_camera.translation = {static_cast<float>(baseline), 0.0f, 0.0f};

    cuvslam::Rig rig;
    rig.cameras = {left, right};
    if (use_imu) {
        cuvslam::ImuCalibration calibration{};
        // Kalibr T_cam0_imu for this unit: the IMU's pose in the left camera's
        // optical frame, which is cuVSLAM's rig frame. Identity here is a 3.5 cm
        // lever-arm error, and inertial fusion integrates that into the estimate.
        calibration.rig_from_imu.translation = {0.030829f, -0.004349f, -0.015420f};
        calibration.rig_from_imu.rotation = {imu_quat[0], imu_quat[1], imu_quat[2], imu_quat[3]};
        calibration.gyroscope_noise_density = 0.000244f;
        calibration.gyroscope_random_walk = 0.000019f;
        calibration.accelerometer_noise_density = 0.001862f;
        calibration.accelerometer_random_walk = 0.003f;
        // Must be the rate actually fed. cuVSLAM derives expected samples per frame
        // as frequency * frame_delta and treats the shortfall as lost IMU, so
        // over-declaring silently starves inertial alignment.
        calibration.frequency =
            imu_freq > 0.0f ? imu_freq : 400.0f / static_cast<float>(imu_stride);
        rig.imus = {calibration};
    }

    cuvslam::Odometry::Config config = cuvslam::Odometry::GetDefaultConfig();
    config.odometry_mode = use_imu ? cuvslam::Odometry::OdometryMode::Inertial
                                   : cuvslam::Odometry::OdometryMode::Multicamera;
    config.rectified_stereo_camera = manifest_flag(manifest, "rectified");
    if (sync_sba) {
        config.async_sba = false;
    }
    config.debug_imu_mode = debug_imu;
    if (state_debug) {
        config.enable_observations_export = true;
        config.enable_landmarks_export = true;
    }
    if (!dump_dir.empty()) {
        config.debug_dump_directory = dump_dir;
    }

    cuvslam::Odometry tracker(rig, config);

    // Nanoseconds as an integer: a unix timestamp in seconds needs more
    // significant digits than a default-formatted double carries.
    std::ofstream trajectory(dir + "/standalone_trajectory.tum");
    std::ofstream timings(dir + "/standalone_frame_times.csv");
    timings << "index,left_ns,track_ms,tracked\n";

    std::size_t imu_at = 0;
    std::size_t tracked = 0;
    std::size_t skipped_stamps = 0;
    std::int64_t last_ns = 0;
    double total_ms = 0.0;
    double worst_ms = 0.0;
    const auto wall_start = std::chrono::steady_clock::now();

    for (std::size_t i = static_cast<std::size_t>(start_frame); i < frames.size(); ++i) {
        if (left_planes[i].empty() || right_planes[i].empty()) {
            continue;
        }
        // The recorder emits the odd duplicate stamp -- two frames sharing a time,
        // followed by a double-length gap. Track() rejects anything not strictly
        // increasing, so the repeat is dropped and counted rather than fixed up.
        if (frames[i].left_ns <= last_ns) {
            ++skipped_stamps;
            continue;
        }
        last_ns = frames[i].left_ns;
        // Inertial mode needs the history leading up to the frame, not after it.
        std::size_t registered_here = 0;
        while (use_imu && imu_at < imu.size() && imu[imu_at].ts_ns <= frames[i].left_ns) {
            if (imu_stride > 1 && (imu_at % static_cast<std::size_t>(imu_stride)) != 0) {
                ++imu_at;
                continue;
            }
            cuvslam::ImuMeasurement measurement{};
            measurement.timestamp_ns = imu[imu_at].ts_ns;
            measurement.linear_accelerations = {imu[imu_at].accel[0], imu[imu_at].accel[1],
                                                imu[imu_at].accel[2]};
            measurement.angular_velocities = {imu[imu_at].gyro[0], imu[imu_at].gyro[1],
                                              imu[imu_at].gyro[2]};
            try {
                tracker.RegisterImuMeasurement(0, measurement);
            } catch (const std::exception& error) {
                std::printf(
                    "\nRegisterImuMeasurement threw at frame %zu (imu_at %zu, %zu registered for "
                    "this frame, %zu total tracked)\n  imu_ts %ld, frame_ts %ld, delta %.4f ms\n"
                    "  prev_imu_ts %ld (gap %.4f ms)\n  what(): %s\n",
                    i, imu_at, registered_here, tracked, measurement.timestamp_ns,
                    frames[i].left_ns,
                    static_cast<double>(measurement.timestamp_ns - frames[i].left_ns) / 1e6,
                    imu_at > 0 ? imu[imu_at - 1].ts_ns : -1,
                    imu_at > 0 ? static_cast<double>(measurement.timestamp_ns -
                                                     imu[imu_at - 1].ts_ns) / 1e6 : 0.0,
                    error.what());
                std::fflush(stdout);
                return 3;
            }
            ++registered_here;
            ++imu_at;
        }
        if (use_imu && i < 3) {
            std::printf("frame %zu: %zu imu registered before Track (imu_at=%zu)\n", i,
                        registered_here, imu_at);
        }

        cuvslam::Image l{};
        l.pixels = left_planes[i].data();
        l.width = width;
        l.height = height;
        l.pitch = width;
        l.encoding = cuvslam::ImageData::Encoding::MONO;
        l.data_type = cuvslam::ImageData::DataType::UINT8;
        l.is_gpu_mem = false;
        l.timestamp_ns = frames[i].left_ns;
        l.camera_index = 0;
        cuvslam::Image r = l;
        r.pixels = right_planes[i].data();
        r.timestamp_ns = frames[i].right_ns;
        r.camera_index = 1;

        const auto started = std::chrono::steady_clock::now();
        cuvslam::PoseEstimate estimate;
        try {
            estimate = tracker.Track({l, r}, {});
        } catch (const std::exception& error) {
            std::printf("\nTrack() threw at frame %zu (index %d, t=%.3f s, %zu tracked): %s\n",
                        i, frames[i].index,
                        static_cast<double>(frames[i].left_ns - frames.front().left_ns) / kNsPerSec,
                        tracked, error.what());
            std::fflush(stdout);
            break;
        }
        const double elapsed_ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started)
                .count();
        total_ms += elapsed_ms;
        worst_ms = std::max(worst_ms, elapsed_ms);

        const bool ok = estimate.world_from_rig.has_value();
        if (use_imu && (i % 500 == 0 || i + 1 == frames.size())) {
            const auto gravity = tracker.GetLastGravity();
            const auto imu_state = tracker.GetImuState();
            std::printf("  [imu] frame %zu gravity=", i);
            if (gravity) {
                std::printf("(%.3f, %.3f, %.3f)", (*gravity)[0], (*gravity)[1], (*gravity)[2]);
            } else {
                std::printf("<none>");
            }
            if (imu_state) {
                std::printf(" vel=(%.3f, %.3f, %.3f) gyro_bias=(%.5f, %.5f, %.5f) "
                            "acc_bias=(%.4f, %.4f, %.4f)",
                            imu_state->velocity[0], imu_state->velocity[1], imu_state->velocity[2],
                            imu_state->gyro_bias[0], imu_state->gyro_bias[1],
                            imu_state->gyro_bias[2], imu_state->acc_bias[0], imu_state->acc_bias[1],
                            imu_state->acc_bias[2]);
            } else {
                std::printf(" imu_state=<none>");
            }
            if (state_debug) {
                cuvslam::Odometry::State state;
                tracker.GetState(state);
                std::printf(" | state: warming_up=%d keyframe=%d frame_id=%llu obs=%zu state_gravity=%s",
                            state.warming_up ? 1 : 0, state.keyframe ? 1 : 0,
                            static_cast<unsigned long long>(state.frame_id),
                            state.observations.size(), state.gravity ? "yes" : "no");
            }
            std::printf("\n");
            std::fflush(stdout);
        }
        timings << frames[i].index << ',' << frames[i].left_ns << ',' << elapsed_ms << ','
                << (ok ? 1 : 0) << '\n';
        if (ok) {
            ++tracked;
            const auto& pose = estimate.world_from_rig->pose;
            trajectory << frames[i].left_ns << ' ' << pose.translation[0] << ' '
                       << pose.translation[1] << ' ' << pose.translation[2] << ' '
                       << pose.rotation[0] << ' ' << pose.rotation[1] << ' ' << pose.rotation[2]
                       << ' ' << pose.rotation[3] << '\n';
        }
        if (i % 500 == 0) {
            std::printf("  %zu/%zu, %zu tracked, %.1f ms mean\n", i, frames.size(), tracked,
                        total_ms / static_cast<double>(i + 1));
            std::fflush(stdout);
        }
    }

    const double wall_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count();
    const double span_s =
        static_cast<double>(frames.back().left_ns - frames.front().left_ns) / kNsPerSec;
    std::printf(
        "\ntracked %zu/%zu frames\n"
        "track mean %.2f ms, worst %.2f ms  -> %.1f fps sustained\n"
        "loop wall %.1f s for %.1f s of recording -> real-time factor %.3f\n",
        tracked, frames.size(), skipped_stamps,
        total_ms / static_cast<double>(frames.size()), worst_ms,
        1000.0 * static_cast<double>(frames.size()) / total_ms, wall_s, span_s, wall_s / span_s);
    return 0;
}
