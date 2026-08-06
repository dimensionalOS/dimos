// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// NVIDIA cuVSLAM stereo visual odometry as a dimos native module.
//
// in:  image_left/image_right (mono8, rectified), camera_info
// out: odometry, landmarks (cuVSLAM's tracked 3D points)
//
// cuVSLAM restarts its world frame after a tracking loss. The module rebases each
// restart onto the last published pose, so consumers see ONE continuous odometry
// path in ONE frame and never a jump. Resets are logged as debugging output; the
// motion across one is unmeasured, which is drift the stream cannot account for.
// baseline_m is config, not derived: these recordings have right P[3] == 0.

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "cuvslam/cuvslam2.h"
#include "dimos/native.hpp"
#include "nav_msgs/Odometry.hpp"
#include "sensor_msgs/CameraInfo.hpp"
#include "sensor_msgs/Image.hpp"
#include "sensor_msgs/Imu.hpp"
#include "sensor_msgs/PointCloud2.hpp"
#include "sensor_msgs/PointField.hpp"

using dimos::native::Builder;
using dimos::native::Config;
using dimos::native::Module;
using dimos::native::Output;
namespace logging = dimos::native::log;

namespace {

constexpr std::int64_t kNsPerSec = 1000000000LL;

std::int64_t stamp_to_ns(const std_msgs::Header& header) {
    return static_cast<std::int64_t>(header.stamp.sec) * kNsPerSec +
           static_cast<std::int64_t>(header.stamp.nsec);
}

/// cuVSLAM's Track() contract asks for stereo stamps within 1 ms.
constexpr std::int64_t kMaxPairSkewNs = 1000000LL;  // 1 ms

/// Rigid transform, rotation as xyzw to match cuvslam::Pose.
struct Transform {
    std::array<double, 4> rotation{0.0, 0.0, 0.0, 1.0};
    std::array<double, 3> translation{0.0, 0.0, 0.0};
};

std::array<double, 4> quat_multiply(const std::array<double, 4>& a,
                                    const std::array<double, 4>& b) {
    return {a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
            a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
            a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
            a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]};
}

std::array<double, 3> cross(const std::array<double, 3>& a, const std::array<double, 3>& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

/// v + 2u x (u x v + wv), the usual quaternion sandwich without building a matrix.
std::array<double, 3> quat_rotate(const std::array<double, 4>& q,
                                  const std::array<double, 3>& v) {
    const std::array<double, 3> u{q[0], q[1], q[2]};
    std::array<double, 3> inner = cross(u, v);
    for (int i = 0; i < 3; ++i) {
        inner[i] += q[3] * v[i];
    }
    const std::array<double, 3> outer = cross(u, inner);
    return {v[0] + 2.0 * outer[0], v[1] + 2.0 * outer[1], v[2] + 2.0 * outer[2]};
}

Transform compose(const Transform& a, const Transform& b) {
    const std::array<double, 3> rotated = quat_rotate(a.rotation, b.translation);
    return Transform{quat_multiply(a.rotation, b.rotation),
                     {a.translation[0] + rotated[0], a.translation[1] + rotated[1],
                      a.translation[2] + rotated[2]}};
}

Transform invert(const Transform& t) {
    const std::array<double, 4> conjugate{-t.rotation[0], -t.rotation[1], -t.rotation[2],
                                          t.rotation[3]};
    const std::array<double, 3> rotated = quat_rotate(conjugate, t.translation);
    return Transform{conjugate, {-rotated[0], -rotated[1], -rotated[2]}};
}

/// 3x3 median, then "how far above its own neighbourhood is this pixel", then a
/// dilation to cover each spot's halo. The Mid-360 paints bright dots on the IR
/// frame that move with the lidar's spin rather than with the world; measured at
/// a cuVSLAM world-frame restart there are ~46k such pixels against ~1.8k on an
/// average frame, so they are what breaks tracking on a lidar+camera rig.
void speckle_mask(const std::uint8_t* pixels, std::int32_t width, std::int32_t height,
                  std::int32_t pitch, int threshold, int grow, std::vector<std::uint8_t>& spike,
                  std::vector<std::uint8_t>& mask) {
    spike.assign(static_cast<std::size_t>(width) * height, 0);
    mask.assign(static_cast<std::size_t>(width) * height, 0);
    auto sort2 = [](std::uint8_t& a, std::uint8_t& b) {
        const std::uint8_t low = std::min(a, b);
        b = std::max(a, b);
        a = low;
    };
    for (std::int32_t y = 1; y < height - 1; ++y) {
        const std::uint8_t* above = pixels + (y - 1) * pitch;
        const std::uint8_t* row = pixels + y * pitch;
        const std::uint8_t* below = pixels + (y + 1) * pitch;
        for (std::int32_t x = 1; x < width - 1; ++x) {
            // 19-op median network; nth_element per pixel was the whole cost.
            std::uint8_t p0 = above[x - 1], p1 = above[x], p2 = above[x + 1];
            std::uint8_t p3 = row[x - 1], p4 = row[x], p5 = row[x + 1];
            std::uint8_t p6 = below[x - 1], p7 = below[x], p8 = below[x + 1];
            sort2(p1, p2); sort2(p4, p5); sort2(p7, p8);
            sort2(p0, p1); sort2(p3, p4); sort2(p6, p7);
            sort2(p1, p2); sort2(p4, p5); sort2(p7, p8);
            sort2(p0, p3); sort2(p5, p8); sort2(p4, p7);
            sort2(p3, p6); sort2(p1, p4); sort2(p2, p5);
            sort2(p4, p7); sort2(p4, p2); sort2(p6, p4);
            sort2(p4, p2);
            if (static_cast<int>(row[x]) - static_cast<int>(p4) > threshold) {
                spike[static_cast<std::size_t>(y) * width + x] = 255;
            }
        }
    }
    // Separable max filter: cheaper than a square structuring element and the
    // result is the same for a box.
    std::vector<std::uint8_t>& rows = mask;
    for (std::int32_t y = 0; y < height; ++y) {
        for (std::int32_t x = 0; x < width; ++x) {
            std::uint8_t best = 0;
            for (std::int32_t dx = -grow; dx <= grow && best == 0; ++dx) {
                const std::int32_t sx = x + dx;
                if (sx >= 0 && sx < width) {
                    best = spike[static_cast<std::size_t>(y) * width + sx];
                }
            }
            rows[static_cast<std::size_t>(y) * width + x] = best;
        }
    }
    spike = rows;
    for (std::int32_t y = 0; y < height; ++y) {
        for (std::int32_t x = 0; x < width; ++x) {
            std::uint8_t best = 0;
            for (std::int32_t dy = -grow; dy <= grow && best == 0; ++dy) {
                const std::int32_t sy = y + dy;
                if (sy >= 0 && sy < height) {
                    best = spike[static_cast<std::size_t>(sy) * width + x];
                }
            }
            mask[static_cast<std::size_t>(y) * width + x] = best;
        }
    }
}

Transform to_transform(const cuvslam::Pose& pose) {
    return Transform{{pose.rotation[0], pose.rotation[1], pose.rotation[2], pose.rotation[3]},
                     {pose.translation[0], pose.translation[1], pose.translation[2]}};
}

}  // namespace

struct CuvslamConfig {
    double baseline_m;   ///< metres
    bool rectified;      ///< D455 IR pair is rectified on-device (D=0, R=I)
    bool publish_landmarks;
    bool async_sba;  ///< off makes a replay reproducible, at some accuracy cost
    /// A step implying more than this is cuVSLAM changing world frames, not motion.
    double max_speed_mps;
    std::string odom_frame;
    std::string base_frame;
    /// Unused here. The module's python half publishes map->odom from these, and
    /// config for one module lives in one struct, so they cross the boundary too.
    std::string map_frame;
    bool publish_map_to_odom;
    /// cuvslam::Slam: pose graph + loop closure on top of the odometry. Without it
    /// there is nothing to pull a revisit back together and the map smears.
    bool enable_slam;
    /// Slam must finish each frame before Track() returns: GetPose() has no
    /// timestamp, so a pose from a thread running behind cannot be matched to the
    /// odometry pose it should be differenced against. Async measured 77 m ATE
    /// against 0.25 m for the odometry it is supposed to be correcting.
    bool slam_sync_mode;
    /// Poses kept in the graph. 300 is NVIDIA's real-time figure, 0 is unlimited.
    int slam_max_map_size;
    /// Floor on the interval between loop closures, milliseconds.
    int slam_throttling_ms;
    /// Fuse the IMU. NVIDIA's mode table calls Inertial "stereo VIO, adds
    /// robustness to brief visual failures", which is exactly what a world-frame
    /// restart is: vision briefly had nothing to hold on to.
    bool enable_imu;
    /// rig_from_imu, rig being the left camera. Kalibr's T_cam0_imu.
    double imu_tx, imu_ty, imu_tz;
    double imu_qx, imu_qy, imu_qz, imu_qw;
    double gyro_noise_density;
    double gyro_random_walk;
    double accel_noise_density;
    double accel_random_walk;
    double imu_frequency;
    /// Mask the lidar's dots out of the IR frames before tracking on them.
    bool mask_speckle;
    /// How far above its local median a pixel must sit to count as a lidar dot.
    int speckle_threshold;
    /// Dilation radius, to cover each dot's halo.
    int speckle_grow;
};

class CuvslamOdometry : public Module {
public:
    void build(Builder& builder, Config& config) override {
        const CuvslamConfig cfg = config.parse<CuvslamConfig>();
        baseline_m_ = cfg.baseline_m;
        rectified_ = cfg.rectified;
        publish_landmarks_ = cfg.publish_landmarks;
        async_sba_ = cfg.async_sba;
        max_speed_mps_ = cfg.max_speed_mps;
        odom_frame_ = cfg.odom_frame;
        base_frame_ = cfg.base_frame;
        map_frame_ = cfg.map_frame;
        enable_slam_ = cfg.enable_slam;
        slam_sync_mode_ = cfg.slam_sync_mode;
        slam_max_map_size_ = cfg.slam_max_map_size;
        slam_throttling_ms_ = cfg.slam_throttling_ms;
        cfg_imu_ = cfg;
        mask_speckle_ = cfg.mask_speckle;
        speckle_threshold_ = cfg.speckle_threshold;
        speckle_grow_ = cfg.speckle_grow;
        if (!(baseline_m_ > 0.0)) {
            throw std::runtime_error(
                "baseline_m must be a positive number of metres (D455 factory value is 0.09486)");
        }

        builder.input<sensor_msgs::CameraInfo>("camera_info", &CuvslamOdometry::on_camera_info,
                                               this);
        builder.input<sensor_msgs::Image>("image_left", &CuvslamOdometry::on_left, this);
        builder.input<sensor_msgs::Image>("image_right", &CuvslamOdometry::on_right, this);
        if (cfg.enable_imu) {
            builder.input<sensor_msgs::Imu>("imu", &CuvslamOdometry::on_imu, this);
        }

        odometry_ = builder.output<nav_msgs::Odometry>("odometry");
        landmarks_ = builder.output<sensor_msgs::PointCloud2>("landmarks");
        corrected_odometry_ = builder.output<nav_msgs::Odometry>("corrected_odometry");
        map_tf_ = builder.output<nav_msgs::Odometry>("map_tf");
    }

    void teardown() override {
        logging::info("cuvslam shutting down",
                      {logging::Field("frames", static_cast<std::int64_t>(frames_)),
                       logging::Field("tracked", static_cast<std::int64_t>(tracked_)),
                       logging::Field("resets", static_cast<std::int64_t>(segment_id_)),
                       logging::Field("loop_closures", static_cast<std::int64_t>(loop_closures_)),
                       logging::Field("imu_samples", static_cast<std::int64_t>(imu_samples_))});
    }

private:
    void on_camera_info(const sensor_msgs::CameraInfo& info) {
        if (tracker_) {
            return;  // rig is fixed once the tracker exists
        }
        width_ = info.width;
        height_ = info.height;
        fx_ = info.K[0];
        fy_ = info.K[4];
        cx_ = info.K[2];
        cy_ = info.K[5];
        have_info_ = true;
    }

    /// cuVSLAM buffers and orders these itself, so hand them over as they arrive.
    void on_imu(const sensor_msgs::Imu& msg) {
        if (!tracker_) {
            return;  // the rig, and so the tracker, does not exist yet
        }
        cuvslam::ImuMeasurement m{};
        m.timestamp_ns = stamp_to_ns(msg.header);
        m.linear_accelerations = {static_cast<float>(msg.linear_acceleration.x),
                                  static_cast<float>(msg.linear_acceleration.y),
                                  static_cast<float>(msg.linear_acceleration.z)};
        m.angular_velocities = {static_cast<float>(msg.angular_velocity.x),
                                static_cast<float>(msg.angular_velocity.y),
                                static_cast<float>(msg.angular_velocity.z)};
        tracker_->RegisterImuMeasurement(0, m);
        ++imu_samples_;
    }

    void on_left(const sensor_msgs::Image& img) {
        left_ = img;
        have_left_ = true;
        try_track();
    }

    void on_right(const sensor_msgs::Image& img) {
        right_ = img;
        have_right_ = true;
        try_track();
    }

    /// cuVSLAM uses OpenCV convention (x right, y down, z forward), so the right
    /// camera sits at +baseline on x. The rig frame is the left camera.
    void ensure_tracker() {
        if (tracker_ || !have_info_) {
            return;
        }
        cuvslam::Camera left{};
        left.size = {width_, height_};
        left.principal = {static_cast<float>(cx_), static_cast<float>(cy_)};
        left.focal = {static_cast<float>(fx_), static_cast<float>(fy_)};
        left.rig_from_camera = cuvslam::Pose{};  // identity: rig == left camera
        left.distortion = cuvslam::Distortion{cuvslam::Distortion::Model::Pinhole};

        cuvslam::Camera right = left;
        right.rig_from_camera.translation = {static_cast<float>(baseline_m_), 0.0f, 0.0f};

        cuvslam::Rig rig;
        rig.cameras = {left, right};
        if (cfg_imu_.enable_imu) {
            cuvslam::ImuCalibration imu{};
            imu.rig_from_imu.translation = {static_cast<float>(cfg_imu_.imu_tx),
                                            static_cast<float>(cfg_imu_.imu_ty),
                                            static_cast<float>(cfg_imu_.imu_tz)};
            imu.rig_from_imu.rotation = {
                static_cast<float>(cfg_imu_.imu_qx), static_cast<float>(cfg_imu_.imu_qy),
                static_cast<float>(cfg_imu_.imu_qz), static_cast<float>(cfg_imu_.imu_qw)};
            imu.gyroscope_noise_density = static_cast<float>(cfg_imu_.gyro_noise_density);
            imu.gyroscope_random_walk = static_cast<float>(cfg_imu_.gyro_random_walk);
            imu.accelerometer_noise_density = static_cast<float>(cfg_imu_.accel_noise_density);
            imu.accelerometer_random_walk = static_cast<float>(cfg_imu_.accel_random_walk);
            imu.frequency = static_cast<float>(cfg_imu_.imu_frequency);
            rig.imus = {imu};
        }

        cuvslam::Odometry::Config cfg = cuvslam::Odometry::GetDefaultConfig();
        cfg.odometry_mode = cfg_imu_.enable_imu ? cuvslam::Odometry::OdometryMode::Inertial
                                               : cuvslam::Odometry::OdometryMode::Multicamera;
        cfg.rectified_stereo_camera = rectified_;
        cfg.enable_landmarks_export = publish_landmarks_ || enable_slam_;
        // Slam reads the tracker's State, which GetState() only fills when the
        // export flags are on; without them it throws instead of returning empty.
        cfg.enable_observations_export = enable_slam_;
        cfg.async_sba = async_sba_;

        tracker_.emplace(rig, cfg);
        if (enable_slam_) {
            cuvslam::Slam::Config slam_cfg = cuvslam::Slam::GetDefaultConfig();
            slam_cfg.sync_mode = slam_sync_mode_;
            slam_cfg.max_map_size = static_cast<std::uint32_t>(slam_max_map_size_);
            slam_cfg.throttling_time_ms = static_cast<std::uint32_t>(slam_throttling_ms_);
            slam_.emplace(rig, tracker_->GetPrimaryCameras(), slam_cfg);
        }
        logging::info("cuvslam tracker created",
                      {logging::Field("width", static_cast<std::int64_t>(width_)),
                       logging::Field("height", static_cast<std::int64_t>(height_)),
                       logging::Field("baseline_mm", baseline_m_ * 1000.0)});
    }

    void try_track() {
        if (!have_left_ || !have_right_) {
            return;
        }
        ensure_tracker();
        if (!tracker_) {
            return;  // no camera_info yet
        }

        const std::int64_t t_left = stamp_to_ns(left_.header);
        const std::int64_t t_right = stamp_to_ns(right_.header);
        if (std::llabs(t_left - t_right) > kMaxPairSkewNs) {
            return;  // wait for the matching eye rather than pairing across motion
        }
        // cuVSLAM rejects a frame that is not strictly newer than the last one.
        if (last_ts_ns_ && t_left <= *last_ts_ns_) {
            have_left_ = have_right_ = false;
            return;
        }

        cuvslam::Image l{};
        l.pixels = left_.data.data();
        l.width = left_.width;
        l.height = left_.height;
        l.pitch = left_.step;
        l.encoding = cuvslam::ImageData::Encoding::MONO;
        l.data_type = cuvslam::ImageData::DataType::UINT8;
        l.is_gpu_mem = false;
        l.timestamp_ns = t_left;
        l.camera_index = 0;

        cuvslam::Image r = l;
        r.pixels = right_.data.data();
        r.pitch = right_.step;
        r.camera_index = 1;

        std::vector<cuvslam::Image> masks;
        if (mask_speckle_) {
            speckle_mask(left_.data.data(), left_.width, left_.height, left_.step,
                         speckle_threshold_, speckle_grow_, spike_buffer_, mask_left_);
            speckle_mask(right_.data.data(), right_.width, right_.height, right_.step,
                         speckle_threshold_, speckle_grow_, spike_buffer_, mask_right_);
            cuvslam::Image ml = l;
            ml.pixels = mask_left_.data();
            ml.pitch = left_.width;
            cuvslam::Image mr = r;
            mr.pixels = mask_right_.data();
            mr.pitch = right_.width;
            masks = {ml, mr};
        }
        const cuvslam::PoseEstimate est = tracker_->Track({l, r}, masks);
        ++frames_;
        last_ts_ns_ = t_left;
        have_left_ = have_right_ = false;

        if (!est.world_from_rig.has_value()) {
            if (was_tracking_) {
                ++segment_id_;
                was_tracking_ = false;
                pending_rebase_ = true;
                logging::warn("cuvslam tracking lost",
                              {logging::Field("segment", static_cast<std::int64_t>(segment_id_))});
            }
            return;
        }
        const Transform tracker_from_rig = to_transform(est.world_from_rig->pose);
        // cuVSLAM restarts its world frame after a loss, and measurement shows it
        // does so *without* ever returning an empty pose, so the restart has to be
        // caught here: a step no robot could have travelled is a frame change.
        // Odometry is allowed to drift but not to jump, so rebase onto the last
        // pose published and let the segment id say the motion is unmeasured.
        const Transform candidate = compose(world_from_tracker_, tracker_from_rig);
        if (last_pose_ns_) {
            const double dt = static_cast<double>(est.timestamp_ns - *last_pose_ns_) / kNsPerSec;
            double moved = 0.0;
            for (int axis = 0; axis < 3; ++axis) {
                const double d = candidate.translation[axis] - world_from_rig_.translation[axis];
                moved += d * d;
            }
            if (dt > 0.0 && std::sqrt(moved) / dt > max_speed_mps_) {
                ++segment_id_;
                pending_rebase_ = true;
                logging::warn("cuvslam world frame restarted",
                              {logging::Field("reset", static_cast<std::int64_t>(segment_id_)),
                               logging::Field("timestamp_ns", est.timestamp_ns),
                               logging::Field("implied_speed_mps", std::sqrt(moved) / dt)});
            }
        }
        if (pending_rebase_) {
            world_from_tracker_ = compose(world_from_rig_, invert(tracker_from_rig));
            pending_rebase_ = false;
        }
        world_from_rig_ = compose(world_from_tracker_, tracker_from_rig);
        last_pose_ns_ = est.timestamp_ns;
        was_tracking_ = true;
        ++tracked_;
        publish(est.timestamp_ns);
        if (publish_landmarks_) {
            publish_landmarks(est.timestamp_ns);
        }
        if (slam_) {
            run_slam(est.timestamp_ns);
        }
    }

    /// Pose graph on top of the odometry. Its pose jumps at a loop closure, which
    /// is exactly why the jump belongs on map->odom and not on the odometry.
    void run_slam(std::int64_t timestamp_ns) {
        cuvslam::Odometry::State state;
        tracker_->GetState(state);
        slam_->Track(state);

        // Slam's pose is identity until it has a keyframe; publishing that would
        // read as a correction the size of however far the robot has driven.
        slam_started_ = slam_started_ || state.keyframe;
        if (!slam_started_) {
            return;
        }
        // GetPose() carries no timestamp, so it can only be differenced against the
        // odometry pose of the frame just tracked. That holds in sync mode, where
        // Slam finishes the frame before Track() returns. See slam_sync_mode.
        const Transform map_from_rig = to_transform(slam_->GetPose());

        // No magnitude guard here. When the odometry restarts its world frame the
        // module rebases to keep odom continuous, and that offset has to land
        // somewhere: map->odom is exactly where it belongs. A correction of
        // several metres after a handful of restarts is the right answer, not
        // divergence -- an earlier absolute cap threw away 6161 good corrections
        // on one run and made cuVSLAM look like it had stopped.
        const Transform map_from_odom_raw = compose(map_from_rig, invert(world_from_rig_));

        nav_msgs::Odometry corrected{};
        fill_pose(corrected, map_from_rig, timestamp_ns, map_frame_, base_frame_);
        corrected_odometry_.publish(corrected);

        nav_msgs::Odometry correction{};
        fill_pose(correction, map_from_odom_raw, timestamp_ns, map_frame_, odom_frame_);
        map_tf_.publish(correction);

        cuvslam::Slam::Metrics metrics{};
        slam_->GetSlamMetrics(metrics);
        if (metrics.lc_status && timestamp_ns != last_closure_ns_) {
            ++loop_closures_;
            last_closure_ns_ = timestamp_ns;
            logging::info("cuvslam loop closure",
                          {logging::Field("count", static_cast<std::int64_t>(loop_closures_)),
                           logging::Field("timestamp_ns", timestamp_ns),
                           logging::Field("tracked_landmarks",
                                          static_cast<std::int64_t>(metrics.lc_tracked_landmarks_count))});
        }
    }

    static void fill_pose(nav_msgs::Odometry& msg, const Transform& pose, std::int64_t timestamp_ns,
                          const std::string& frame, const std::string& child) {
        msg.header.stamp.sec = static_cast<std::int32_t>(timestamp_ns / kNsPerSec);
        msg.header.stamp.nsec = static_cast<std::int32_t>(timestamp_ns % kNsPerSec);
        msg.header.frame_id = frame;
        msg.child_frame_id = child;
        msg.pose.pose.position.x = pose.translation[0];
        msg.pose.pose.position.y = pose.translation[1];
        msg.pose.pose.position.z = pose.translation[2];
        msg.pose.pose.orientation.x = pose.rotation[0];
        msg.pose.pose.orientation.y = pose.rotation[1];
        msg.pose.pose.orientation.z = pose.rotation[2];
        msg.pose.pose.orientation.w = pose.rotation[3];
    }

    /// One edge for the whole run: a robot only ever sees a single odom path.
    /// Resets are debugging output, on the log, not a break in this stream.
    void publish(std::int64_t timestamp_ns) {
        nav_msgs::Odometry msg{};
        fill_pose(msg, world_from_rig_, timestamp_ns, odom_frame_, base_frame_);
        odometry_.publish(msg);
    }

    /// cuVSLAM's landmarks are the map: 3D points it is currently tracking.
    /// GetLastLandmarks() returns them in the *camera* frame, so they have to be
    /// carried into the world frame or every frame's points pile up on the origin.
    void publish_landmarks(std::int64_t timestamp_ns) {
        const std::vector<cuvslam::Landmark> pts = tracker_->GetLastLandmarks();
        if (pts.empty()) {
            return;
        }
        sensor_msgs::PointCloud2 msg{};
        msg.header.stamp.sec = static_cast<std::int32_t>(timestamp_ns / kNsPerSec);
        msg.header.stamp.nsec = static_cast<std::int32_t>(timestamp_ns % kNsPerSec);
        msg.header.frame_id = odom_frame_;
        msg.height = 1;
        msg.width = static_cast<std::int32_t>(pts.size());
        msg.is_bigendian = 0;
        msg.is_dense = 1;
        msg.point_step = 3 * static_cast<std::int32_t>(sizeof(float));
        msg.row_step = msg.point_step * msg.width;

        const char* names[3] = {"x", "y", "z"};
        msg.fields.resize(3);
        for (int i = 0; i < 3; ++i) {
            msg.fields[i].name = names[i];
            msg.fields[i].offset = i * static_cast<std::int32_t>(sizeof(float));
            msg.fields[i].datatype = 7;  // FLOAT32
            msg.fields[i].count = 1;
        }
        msg.fields_length = static_cast<std::int32_t>(msg.fields.size());

        msg.data.resize(static_cast<std::size_t>(msg.row_step));
        auto* out = reinterpret_cast<float*>(msg.data.data());
        for (std::size_t i = 0; i < pts.size(); ++i) {
            const std::array<double, 3> local{pts[i].coords[0], pts[i].coords[1],
                                              pts[i].coords[2]};
            const std::array<double, 3> world = quat_rotate(world_from_rig_.rotation, local);
            for (int axis = 0; axis < 3; ++axis) {
                out[3 * i + axis] =
                    static_cast<float>(world[axis] + world_from_rig_.translation[axis]);
            }
        }
        msg.data_length = static_cast<std::int32_t>(msg.data.size());
        landmarks_.publish(msg);
    }

    // config
    double baseline_m_{0.0};
    bool rectified_{true};
    bool publish_landmarks_{true};
    bool async_sba_{true};
    double max_speed_mps_{10.0};
    std::string odom_frame_{"odom"};
    std::string base_frame_{"base_link"};
    std::string map_frame_{"map"};

    // slam
    std::optional<cuvslam::Slam> slam_;
    bool enable_slam_{true};
    bool slam_sync_mode_{false};
    int slam_max_map_size_{300};
    int slam_throttling_ms_{0};
    CuvslamConfig cfg_imu_{};
    std::uint64_t imu_samples_{0};
    bool mask_speckle_{true};
    int speckle_threshold_{6};
    int speckle_grow_{2};
    std::vector<std::uint8_t> spike_buffer_, mask_left_, mask_right_;
    bool slam_started_{false};
    std::uint64_t loop_closures_{0};
    std::int64_t last_closure_ns_{-1};

    // intrinsics
    bool have_info_{false};
    std::int32_t width_{0};
    std::int32_t height_{0};
    double fx_{0.0}, fy_{0.0}, cx_{0.0}, cy_{0.0};

    // stereo pairing
    sensor_msgs::Image left_{}, right_{};
    bool have_left_{false}, have_right_{false};
    std::optional<std::int64_t> last_ts_ns_;

    // tracking state
    std::optional<cuvslam::Odometry> tracker_;
    Transform world_from_tracker_;  ///< accumulated offset across tracking losses
    Transform world_from_rig_;      ///< last published pose, in the continuous frame
    std::optional<std::int64_t> last_pose_ns_;
    bool pending_rebase_{false};
    bool was_tracking_{false};
    std::uint64_t segment_id_{0};
    std::uint64_t frames_{0};
    std::uint64_t tracked_{0};

    Output<nav_msgs::Odometry> odometry_;
    Output<sensor_msgs::PointCloud2> landmarks_;
    Output<nav_msgs::Odometry> corrected_odometry_;
    Output<nav_msgs::Odometry> map_tf_;
};

int main() {
    dimos::native::run_with_transport<CuvslamOdometry>();
    return 0;
}
