// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// NVIDIA cuVSLAM stereo visual odometry as a dimos native module.
//
// in:  image_left/image_right (mono8, rectified), camera_info
// out: odometry, corrected_odometry, tf
//
// cuVSLAM restarts its world frame after a tracking loss. The module rebases each
// restart onto the last published pose, so consumers see ONE continuous odometry
// path in ONE frame and never a jump. Resets are logged as debugging output; the
// motion across one is unmeasured, which is drift the stream cannot account for.
// The baseline comes off the right camera_info's P[3], so any rectified stereo
// camera works without a per-model constant; stereo_baseline_meters overrides it.

#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "cuvslam/cuvslam2.h"
#include "dimos/native.hpp"
#include "nav_msgs/Odometry.hpp"
#include "sensor_msgs/CameraInfo.hpp"
#include "sensor_msgs/Image.hpp"
#include "sensor_msgs/Imu.hpp"

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

/// cuVSLAM's rig is the left camera, so every pose it returns is in the optical
/// convention: z forward, x right, y down. REP-103 wants a body frame -- x forward,
/// y left, z up -- and publishing the optical pose as `base_link` tilts the entire
/// tree ninety degrees, which reads in a viewer as the robot lying on its back.
///
/// This is the fixed body <- optical rotation, RPY(-pi/2, 0, -pi/2), and it matches
/// `OPTICAL_ROTATION` in dimos/hardware/sensors/camera/spec.py so the tracker and the
/// camera driver agree on what an optical frame is.
const Transform kBaseFromRig{{-0.5, 0.5, -0.5, 0.5}, {0.0, 0.0, 0.0}};

/// cuVSLAM's world is aligned with the rig's first pose, so it is optical-convention
/// too. Rotating both ends -- R * T * R^-1 -- moves the world axes and the body axes
/// together, which is what makes gravity fall along -z instead of +y.
Transform to_body_frame(const Transform& t) {
    return compose(compose(kBaseFromRig, t), invert(kBaseFromRig));
}

Transform to_transform(const cuvslam::Pose& pose) {
    return Transform{{pose.rotation[0], pose.rotation[1], pose.rotation[2], pose.rotation[3]},
                     {pose.translation[0], pose.translation[1], pose.translation[2]}};
}

}  // namespace

struct CuvslamConfig {
    /// "stereo" or "mono". Mono is accurate only up to an unknown scale, so its
    /// poses are not metres.
    std::string camera_mode;
    /// Stereo only. Metres, or 0 to read it off the right camera_info's P[3].
    double stereo_baseline_meters;
    /// The stereo pair arrives rectified: D all zero, R identity.
    bool rectified;
    bool async_sba;  ///< off makes a replay reproducible, at some accuracy cost
    /// A step implying more than this is cuVSLAM changing world frames, not motion.
    double implausible_speed_meters_per_second;
    std::string odom_frame;
    std::string base_frame;
    std::string map_frame;
    /// Unused here: the python half acts on it. Config for one module lives in one
    /// struct, so it crosses the boundary anyway -- parse<T>() requires every key.
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
    /// rig_from_imu, rig being the left camera. Flattened from the python side's
    /// ImuCalibration, which is per-serial and has no default -- all zero here
    /// means enable_imu is off and none of it is read.
    double imu_tx, imu_ty, imu_tz;
    double imu_qx, imu_qy, imu_qz, imu_qw;
    double imu_gyro_noise_density;
    double imu_gyro_random_walk;
    double imu_accel_noise_density;
    double imu_accel_random_walk;
    /// The rate actually fed. Declaring more than arrives makes cuVSLAM log a drop
    /// ratio and silently never initialise inertial alignment.
    double imu_frequency;
};

class CuvslamOdometry : public Module {
public:
    void build(Builder& builder, Config& config) override {
        cfg_ = config.parse<CuvslamConfig>();
        baseline_meters_ = cfg_.stereo_baseline_meters;
        // Only stereo triangulates, so only stereo needs a baseline before it can start.
        have_baseline_ = cfg_.camera_mode != "stereo";

        builder.input<sensor_msgs::CameraInfo>("camera_info", &CuvslamOdometry::on_camera_info,
                                               this);
        builder.input<sensor_msgs::Image>("image_left", &CuvslamOdometry::on_left, this);
        if (cfg_.camera_mode == "stereo") {
            builder.input<sensor_msgs::CameraInfo>("camera_info_right",
                                                   &CuvslamOdometry::on_camera_info_right, this);
            builder.input<sensor_msgs::Image>("image_right", &CuvslamOdometry::on_right, this);
        }
        if (cfg_.enable_imu) {
            builder.input<sensor_msgs::Imu>("imu", &CuvslamOdometry::on_imu, this);
        }

        odometry_ = builder.output<nav_msgs::Odometry>("odometry");
        corrected_odometry_ = builder.output<nav_msgs::Odometry>("corrected_odometry");
        cpp_tf_workaround_ = builder.output<nav_msgs::Odometry>("cpp_tf_workaround");
    }

    void teardown() override {
        logging::info("cuvslam shutting down",
                      {logging::Field("frames", static_cast<std::int64_t>(frames_)),
                       logging::Field("tracked", static_cast<std::int64_t>(tracked_)),
                       logging::Field("resets", static_cast<std::int64_t>(segment_id_)),
                       logging::Field("loop_closures", static_cast<std::int64_t>(loop_closures_)),
                       logging::Field("imu_samples", static_cast<std::int64_t>(imu_samples_)),
                       logging::Field("imu_dropped_before_start",
                                      static_cast<std::int64_t>(imu_dropped_)),
                       logging::Field("stereo_skew_rejects",
                                      static_cast<std::int64_t>(skew_rejects_))});
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

    /// The right imager carries the baseline: P = [fx 0 cx -fx*B; ...], so
    /// B = -P[3]/P[0]. Reading it here is what lets any stereo camera work without
    /// a per-model constant compiled in.
    void on_camera_info_right(const sensor_msgs::CameraInfo& info) {
        if (tracker_ || have_baseline_) {
            return;
        }
        if (cfg_.stereo_baseline_meters > 0.0) {
            baseline_meters_ = cfg_.stereo_baseline_meters;
            have_baseline_ = true;
            return;
        }
        if (info.P[0] == 0.0 || info.P[3] == 0.0) {
            if (!baseline_warned_) {
                baseline_warned_ = true;
                logging::warn(
                    "right camera_info carries no baseline (P[3] == 0), so there is no metric "
                    "scale. The camera driver is not publishing the stereo extrinsic; set "
                    "stereo_baseline_meters explicitly to override.");
            }
            return;
        }
        baseline_meters_ = std::abs(info.P[3] / info.P[0]);
        have_baseline_ = true;
        logging::info("cuvslam baseline from camera_info",
                      {logging::Field("baseline_mm", baseline_meters_ * 1000.0)});
    }

    /// cuVSLAM buffers and orders these itself, so hand them over as they arrive.
    void on_imu(const sensor_msgs::Imu& msg) {
        if (!tracker_) {
            // The rig, and so the tracker, does not exist yet. Counted rather than
            // dropped silently: this is exactly the window inertial init needs.
            ++imu_dropped_;
            return;
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

    cuvslam::Odometry::OdometryMode odometry_mode() const {
        using Mode = cuvslam::Odometry::OdometryMode;
        if (cfg_.camera_mode == "mono") {
            return Mode::Mono;
        }
        // Inertial is the stereo pair plus an IMU; there is no inertial mono.
        return cfg_.enable_imu ? Mode::Inertial : Mode::Multicamera;
    }

    /// cuVSLAM uses OpenCV convention (x right, y down, z forward), so the right
    /// camera sits at +baseline on x. The rig frame is the left camera.
    void ensure_tracker() {
        if (tracker_) {
            return;
        }
        if (!have_info_ || !have_baseline_) {
            if (!waiting_warned_ && ++frames_waiting_ % kWaitingWarnFrames == 0) {
                waiting_warned_ = true;
                logging::warn("cuvslam has consumed frames without starting",
                              {logging::Field("frames", static_cast<std::int64_t>(frames_waiting_)),
                               logging::Field("have_camera_info", have_info_),
                               logging::Field("have_baseline", have_baseline_)});
            }
            return;
        }
        cuvslam::Camera left{};
        left.size = {width_, height_};
        left.principal = {static_cast<float>(cx_), static_cast<float>(cy_)};
        left.focal = {static_cast<float>(fx_), static_cast<float>(fy_)};
        left.rig_from_camera = cuvslam::Pose{};  // identity: rig == left camera
        left.distortion = cuvslam::Distortion{cuvslam::Distortion::Model::Pinhole};

        cuvslam::Rig rig;
        if (cfg_.camera_mode == "stereo") {
            cuvslam::Camera right = left;
            right.rig_from_camera.translation = {static_cast<float>(baseline_meters_), 0.0f, 0.0f};
            rig.cameras = {left, right};
        } else {
            rig.cameras = {left};
        }
        if (cfg_.enable_imu) {
            cuvslam::ImuCalibration imu{};
            imu.rig_from_imu.translation = {static_cast<float>(cfg_.imu_tx),
                                            static_cast<float>(cfg_.imu_ty),
                                            static_cast<float>(cfg_.imu_tz)};
            imu.rig_from_imu.rotation = {
                static_cast<float>(cfg_.imu_qx), static_cast<float>(cfg_.imu_qy),
                static_cast<float>(cfg_.imu_qz), static_cast<float>(cfg_.imu_qw)};
            imu.gyroscope_noise_density = static_cast<float>(cfg_.imu_gyro_noise_density);
            imu.gyroscope_random_walk = static_cast<float>(cfg_.imu_gyro_random_walk);
            imu.accelerometer_noise_density = static_cast<float>(cfg_.imu_accel_noise_density);
            imu.accelerometer_random_walk = static_cast<float>(cfg_.imu_accel_random_walk);
            imu.frequency = static_cast<float>(cfg_.imu_frequency);
            rig.imus = {imu};
        }

        cuvslam::Odometry::Config odometry_cfg = cuvslam::Odometry::GetDefaultConfig();
        odometry_cfg.odometry_mode = odometry_mode();
        // cuVSLAM rejects this outright unless the rig has a stereo pair: "Rectified
        // stereo camera mode only works with 1+ stereo cameras".
        odometry_cfg.rectified_stereo_camera = cfg_.rectified && cfg_.camera_mode == "stereo";
        odometry_cfg.enable_landmarks_export = cfg_.enable_slam;
        // Slam reads the tracker's State, which GetState() only fills when the
        // export flags are on; without them it throws instead of returning empty.
        odometry_cfg.enable_observations_export = cfg_.enable_slam;
        odometry_cfg.async_sba = cfg_.async_sba;

        tracker_.emplace(rig, odometry_cfg);
        if (cfg_.enable_slam) {
            cuvslam::Slam::Config slam_cfg = cuvslam::Slam::GetDefaultConfig();
            slam_cfg.sync_mode = cfg_.slam_sync_mode;
            slam_cfg.max_map_size = static_cast<std::uint32_t>(cfg_.slam_max_map_size);
            slam_cfg.throttling_time_ms = static_cast<std::uint32_t>(cfg_.slam_throttling_ms);
            slam_.emplace(rig, tracker_->GetPrimaryCameras(), slam_cfg);
        }
        logging::info("cuvslam tracker created",
                      {logging::Field("width", static_cast<std::int64_t>(width_)),
                       logging::Field("height", static_cast<std::int64_t>(height_)),
                       logging::Field("baseline_mm", baseline_meters_ * 1000.0)});
    }

    void try_track() {
        if (!have_left_) {
            return;
        }
        if (cfg_.camera_mode == "stereo" && !have_right_) {
            return;
        }
        ensure_tracker();
        if (!tracker_) {
            return;  // no camera_info yet
        }

        const std::int64_t t_left = stamp_to_ns(left_.header);
        const std::int64_t t_right =
            cfg_.camera_mode == "stereo" ? stamp_to_ns(right_.header) : t_left;
        if (std::llabs(t_left - t_right) > kMaxPairSkewNs) {
            // Wait for the matching eye rather than pairing across motion. A fixed
            // offset between the two streams lands here every frame, so say so once
            // instead of tracking nothing in silence.
            if (++skew_rejects_ % kWaitingWarnFrames == 0) {
                logging::warn("cuvslam stereo pairs exceed the 1 ms skew limit",
                              {logging::Field("rejected", static_cast<std::int64_t>(skew_rejects_)),
                               logging::Field("skew_ms", (t_left - t_right) / 1.0e6)});
            }
            return;
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
        // A three-channel image has to be declared as such: fed as MONO, cuVSLAM
        // reads a third of each row and tracks nothing.
        l.encoding = left_.encoding == "mono8" ? cuvslam::ImageData::Encoding::MONO
                                               : cuvslam::ImageData::Encoding::RGB;
        l.data_type = cuvslam::ImageData::DataType::UINT8;
        l.is_gpu_mem = false;
        l.timestamp_ns = t_left;
        l.camera_index = 0;

        cuvslam::Odometry::ImageSet images{l};
        if (cfg_.camera_mode == "stereo") {
            cuvslam::Image r = l;
            r.pixels = right_.data.data();
            r.pitch = right_.step;
            r.camera_index = 1;
            images.push_back(r);
        }

        const cuvslam::PoseEstimate est = tracker_->Track(images);
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
            if (dt > 0.0 && std::sqrt(moved) / dt > cfg_.implausible_speed_meters_per_second) {
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
        fill_pose(corrected, to_body_frame(map_from_rig), timestamp_ns, cfg_.map_frame, cfg_.base_frame);
        corrected_odometry_.publish(corrected);

        nav_msgs::Odometry correction{};
        fill_pose(correction, to_body_frame(map_from_odom_raw), timestamp_ns, cfg_.map_frame,
                  cfg_.odom_frame);
        cpp_tf_workaround_.publish(correction);

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
        fill_pose(msg, to_body_frame(world_from_rig_), timestamp_ns, cfg_.odom_frame, cfg_.base_frame);
        odometry_.publish(msg);
    }

    // config. Every one of these is set from cfg_ in build(), so no initializer
    // here can ever apply; python owns the defaults.
    CuvslamConfig cfg_{};
    /// The only config value that is not simply cfg_: zero there means "read it off
    /// the right camera_info", and this is where the reading lands.
    double baseline_meters_{0.0};
    bool have_baseline_{false};
    bool baseline_warned_{false};

    // slam
    std::optional<cuvslam::Slam> slam_;
    std::uint64_t imu_samples_{0};
    std::uint64_t imu_dropped_{0};
    bool slam_started_{false};
    std::uint64_t loop_closures_{0};
    std::int64_t last_closure_ns_{-1};

    /// Frames consumed before the tracker could start. A blueprint that mis-wires
    /// camera_info otherwise runs forever publishing nothing.
    static constexpr std::uint64_t kWaitingWarnFrames = 100;
    std::uint64_t frames_waiting_{0};
    std::uint64_t skew_rejects_{0};
    bool waiting_warned_{false};

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
    Output<nav_msgs::Odometry> corrected_odometry_;
    Output<nav_msgs::Odometry> cpp_tf_workaround_;
};

int main() {
    dimos::native::run_with_transport<CuvslamOdometry>();
    return 0;
}
