// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// NVIDIA cuVSLAM visual odometry as a dimos native module.
//
// in:  image, camera_info (every camera on the one stream, told apart by frame_id),
//      rig_transforms, depth_image, imu
// out: odometry, corrected_odometry, tf
//
// cuVSLAM restarts its world frame after a tracking loss. The module rebases each
// restart onto the last published pose, so consumers see ONE continuous odometry
// path in ONE frame and never a jump. Resets are logged as debugging output; the
// motion across one is unmeasured, which is drift the stream cannot account for.
// The rig -- which cameras there are, in which order, and where each one sits --
// arrives on rig_transforms, resolved from tf by the python half.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "cuvslam/cuvslam2.h"
#include "dimos/native.hpp"
#include "nav_msgs/Odometry.hpp"
#include "sensor_msgs/CameraInfo.hpp"
#include "sensor_msgs/Image.hpp"
#include "sensor_msgs/Imu.hpp"
#include "tf2_msgs/TFMessage.hpp"

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

Transform to_transform(const cuvslam::Pose& pose) {
    return Transform{{pose.rotation[0], pose.rotation[1], pose.rotation[2], pose.rotation[3]},
                     {pose.translation[0], pose.translation[1], pose.translation[2]}};
}

Transform to_transform(const geometry_msgs::Transform& t) {
    return Transform{{t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w},
                     {t.translation.x, t.translation.y, t.translation.z}};
}

cuvslam::Pose to_pose(const Transform& t) {
    cuvslam::Pose pose{};
    pose.rotation = {static_cast<float>(t.rotation[0]), static_cast<float>(t.rotation[1]),
                     static_cast<float>(t.rotation[2]), static_cast<float>(t.rotation[3])};
    pose.translation = {static_cast<float>(t.translation[0]),
                        static_cast<float>(t.translation[1]),
                        static_cast<float>(t.translation[2])};
    return pose;
}

}  // namespace

struct CuvslamConfig {
    /// "stereo", "mono" or "rgbd". Mono is accurate only up to an unknown scale, so
    /// its poses are not metres.
    std::string camera_mode;
    /// The stereo pair arrives rectified: D all zero, R identity.
    bool rectified;
    bool async_sba;  ///< off makes a replay reproducible, at some accuracy cost
    /// A step implying more than this is cuVSLAM changing world frames, not motion.
    double implausible_speed_meters_per_second;
    std::string odom_frame;
    std::string base_frame;
    std::string map_frame;
    /// Only read when Slam is off: pure odometry has no global correction, so its
    /// map->odom can only be identity, and something else may already own that edge.
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
    /// Which transform on rig_transforms places the IMU rather than a camera.
    std::string imu_frame;
    /// Flattened from the python side's ImuCalibration, which is per-serial and has
    /// no default -- all zero here means enable_imu is off and none of it is read.
    double imu_gyro_noise_density;
    double imu_gyro_random_walk;
    double imu_accel_noise_density;
    double imu_accel_random_walk;
    /// The rate actually fed. Declaring more than arrives makes cuVSLAM log a drop
    /// ratio and silently never initialise inertial alignment.
    double imu_frequency;
    /// rgbd only: raw depth units per metre. 1000 for sixteen-bit millimetres.
    double depth_units_per_meter;

    /// Called by parse(). Without it an unrecognised mode would quietly behave as mono.
    void validate() const {
        if (camera_mode != "stereo" && camera_mode != "mono" && camera_mode != "rgbd") {
            throw std::runtime_error("camera_mode must be stereo, mono or rgbd, got '" +
                                     camera_mode + "'");
        }
    }
};

class CuvslamOdometry : public Module {
public:
    void build(Builder& builder, Config& config) override {
        cfg_ = config.parse<CuvslamConfig>();
        stereo_ = cfg_.camera_mode == "stereo";
        rgbd_ = cfg_.camera_mode == "rgbd";

        builder.input<tf2_msgs::TFMessage>("rig_transforms", &CuvslamOdometry::on_rig, this);
        builder.input<sensor_msgs::CameraInfo>("camera_info", &CuvslamOdometry::on_camera_info,
                                               this);
        builder.input<sensor_msgs::Image>("image", &CuvslamOdometry::on_image, this);
        if (rgbd_) {
            builder.input<sensor_msgs::Image>("depth_image", &CuvslamOdometry::on_depth, this);
        }
        if (cfg_.enable_imu) {
            builder.input<sensor_msgs::Imu>("imu", &CuvslamOdometry::on_imu, this);
        }

        odometry_ = builder.output<nav_msgs::Odometry>("odometry");
        corrected_odometry_ = builder.output<nav_msgs::Odometry>("corrected_odometry");
        tf_ = builder.output<tf2_msgs::TFMessage>("tf");
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
                       logging::Field("skew_rejects", static_cast<std::int64_t>(skew_rejects_)),
                       logging::Field("images_off_the_rig",
                                      static_cast<std::int64_t>(unplaced_images_))});
    }

private:
    void on_camera_info(const sensor_msgs::CameraInfo& info) {
        if (tracker_) {
            return;  // rig is fixed once the tracker exists
        }
        camera_info_[info.header.frame_id] = info;
    }

    /// The rig: which cameras there are, in cuVSLAM's index order, and where each one
    /// sits relative to the body. It is the whole of the extrinsic calibration -- a
    /// stereo baseline is the gap between two of these -- so nothing here is a
    /// per-model constant.
    void on_rig(const tf2_msgs::TFMessage& message) {
        if (tracker_) {
            return;
        }
        camera_frames_.clear();
        rig_from_camera_.clear();
        rig_from_imu_.reset();
        for (const geometry_msgs::TransformStamped& stamped : message.transforms) {
            if (!cfg_.imu_frame.empty() && stamped.child_frame_id == cfg_.imu_frame) {
                rig_from_imu_ = to_transform(stamped.transform);
                continue;
            }
            camera_frames_.push_back(stamped.child_frame_id);
            rig_from_camera_.push_back(to_transform(stamped.transform));
        }
        images_.assign(camera_frames_.size(), sensor_msgs::Image{});
        have_image_.assign(camera_frames_.size(), false);
    }

    /// Which camera in the rig a frame_id names, or -1 for one that is not on it.
    int camera_index(const std::string& frame_id) const {
        for (std::size_t i = 0; i < camera_frames_.size(); ++i) {
            if (camera_frames_[i] == frame_id) {
                return static_cast<int>(i);
            }
        }
        return -1;
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

    void on_image(const sensor_msgs::Image& img) {
        const int index = camera_index(img.header.frame_id);
        if (index < 0) {
            // Either the rig has not arrived yet or this camera is not on it. Both
            // are silent no-output modes, so say so -- once, since a rig that never
            // arrives would otherwise repeat this for the length of the run. The
            // teardown count is what says how long it went on for.
            if (++unplaced_images_ == kWaitingWarnFrames) {
                logging::warn("cuvslam is dropping images from a camera that is not on the rig",
                              {logging::Field("frame_id", img.header.frame_id),
                               logging::Field("dropped", static_cast<std::int64_t>(unplaced_images_)),
                               logging::Field("rig_cameras",
                                              static_cast<std::int64_t>(camera_frames_.size()))});
            }
            return;
        }
        images_[index] = img;
        have_image_[index] = true;
        try_track();
    }

    void on_depth(const sensor_msgs::Image& img) {
        depth_ = img;
        have_depth_ = true;
        try_track();
    }

    cuvslam::Odometry::OdometryMode odometry_mode() const {
        using Mode = cuvslam::Odometry::OdometryMode;
        if (rgbd_) {
            return Mode::RGBD;
        }
        if (!stereo_) {
            return Mode::Mono;
        }
        // Inertial is the stereo pair plus an IMU; there is no inertial mono or rgbd.
        return cfg_.enable_imu ? Mode::Inertial : Mode::Multicamera;
    }

    /// The rig frame is the body frame the transforms were resolved against, so every
    /// pose cuVSLAM returns is already the body's and needs no re-referencing.
    void ensure_tracker() {
        if (tracker_) {
            return;
        }
        const bool have_rig = !camera_frames_.empty();
        std::size_t with_info = 0;
        for (const std::string& frame : camera_frames_) {
            with_info += camera_info_.count(frame);
        }
        if (!have_rig || with_info != camera_frames_.size()) {
            if (!waiting_warned_ && ++frames_waiting_ % kWaitingWarnFrames == 0) {
                waiting_warned_ = true;
                logging::warn("cuvslam has consumed frames without starting",
                              {logging::Field("frames", static_cast<std::int64_t>(frames_waiting_)),
                               logging::Field("rig_cameras",
                                              static_cast<std::int64_t>(camera_frames_.size())),
                               logging::Field("with_camera_info",
                                              static_cast<std::int64_t>(with_info))});
            }
            return;
        }

        cuvslam::Rig rig;
        for (std::size_t i = 0; i < camera_frames_.size(); ++i) {
            const sensor_msgs::CameraInfo& info = camera_info_.at(camera_frames_[i]);
            cuvslam::Camera camera{};
            camera.size = {info.width, info.height};
            camera.principal = {static_cast<float>(info.K[2]), static_cast<float>(info.K[5])};
            camera.focal = {static_cast<float>(info.K[0]), static_cast<float>(info.K[4])};
            camera.rig_from_camera = to_pose(rig_from_camera_[i]);
            camera.distortion = cuvslam::Distortion{cuvslam::Distortion::Model::Pinhole};
            rig.cameras.push_back(camera);
        }
        if (cfg_.enable_imu) {
            if (!rig_from_imu_) {
                logging::warn("cuvslam: enable_imu is on but tf never placed the IMU",
                              {logging::Field("imu_frame", cfg_.imu_frame)});
                return;
            }
            cuvslam::ImuCalibration imu{};
            imu.rig_from_imu = to_pose(*rig_from_imu_);
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
        odometry_cfg.rectified_stereo_camera = cfg_.rectified && stereo_;
        odometry_cfg.enable_landmarks_export = cfg_.enable_slam;
        // Slam reads the tracker's State, which GetState() only fills when the
        // export flags are on; without them it throws instead of returning empty.
        odometry_cfg.enable_observations_export = cfg_.enable_slam;
        odometry_cfg.async_sba = cfg_.async_sba;
        odometry_cfg.rgbd_settings.depth_scale_factor =
            static_cast<float>(cfg_.depth_units_per_meter);
        // Which camera the depth is pixel-aligned with. It defaults to -1, meaning no
        // camera, and a depth image that belongs to nothing is silently ignored: the
        // tracker runs, consumes every frame and returns no pose at all. A depth stream
        // usually reports its own frame rather than the imager it is aligned to, and
        // cuVSLAM takes one RGB-D camera anyway, so an unrecognised frame means camera 0.
        odometry_cfg.rgbd_settings.depth_camera_id = std::max(camera_index(depth_.header.frame_id), 0);

        tracker_.emplace(rig, odometry_cfg);
        if (cfg_.enable_slam) {
            cuvslam::Slam::Config slam_cfg = cuvslam::Slam::GetDefaultConfig();
            slam_cfg.sync_mode = cfg_.slam_sync_mode;
            slam_cfg.max_map_size = static_cast<std::uint32_t>(cfg_.slam_max_map_size);
            slam_cfg.throttling_time_ms = static_cast<std::uint32_t>(cfg_.slam_throttling_ms);
            slam_.emplace(rig, tracker_->GetPrimaryCameras(), slam_cfg);
        }
        logging::info("cuvslam tracker created",
                      {logging::Field("cameras", static_cast<std::int64_t>(rig.cameras.size())),
                       logging::Field("width", static_cast<std::int64_t>(rig.cameras[0].size[0])),
                       logging::Field("height", static_cast<std::int64_t>(rig.cameras[0].size[1])),
                       logging::Field("rig_frames", join(camera_frames_))});
    }

    static cuvslam::Image to_cuvslam_image(const sensor_msgs::Image& img,
                                           std::int64_t timestamp_ns, std::uint32_t camera_index) {
        cuvslam::Image out{};
        out.pixels = img.data.data();
        out.width = img.width;
        out.height = img.height;
        out.pitch = img.step;
        // A three-channel image has to be declared as such: fed as MONO, cuVSLAM
        // reads a third of each row and tracks nothing.
        out.encoding = img.encoding == "mono8" ? cuvslam::ImageData::Encoding::MONO
                                               : cuvslam::ImageData::Encoding::RGB;
        out.data_type = cuvslam::ImageData::DataType::UINT8;
        out.is_gpu_mem = false;
        out.timestamp_ns = timestamp_ns;
        out.camera_index = camera_index;
        return out;
    }

    void clear_frame_set() {
        have_image_.assign(have_image_.size(), false);
        have_depth_ = false;
    }

    static std::string join(const std::vector<std::string>& parts) {
        std::string joined;
        for (const std::string& part : parts) {
            joined += joined.empty() ? part : ", " + part;
        }
        return joined;
    }

    void try_track() {
        for (const bool have : have_image_) {
            if (!have) {
                return;
            }
        }
        if (have_image_.empty() || (rgbd_ && !have_depth_)) {
            return;
        }
        ensure_tracker();
        if (!tracker_) {
            return;  // no camera_info yet
        }

        // One frame per camera, and they have to be the same instant: pairing across
        // motion is worse than waiting for the camera that is behind. A fixed offset
        // between two streams lands here every frame, so say so rather than tracking
        // nothing in silence.
        std::int64_t oldest = stamp_to_ns(images_[0].header);
        std::int64_t newest = oldest;
        for (const sensor_msgs::Image& image : images_) {
            const std::int64_t ts = stamp_to_ns(image.header);
            oldest = std::min(oldest, ts);
            newest = std::max(newest, ts);
        }
        if (newest - oldest > kMaxPairSkewNs) {
            if (++skew_rejects_ % kWaitingWarnFrames == 0) {
                logging::warn("cuvslam frame sets exceed the 1 ms skew limit",
                              {logging::Field("rejected", static_cast<std::int64_t>(skew_rejects_)),
                               logging::Field("skew_ms", (newest - oldest) / 1.0e6)});
            }
            return;
        }
        // cuVSLAM rejects a frame that is not strictly newer than the last one.
        if (last_ts_ns_ && newest <= *last_ts_ns_) {
            clear_frame_set();
            return;
        }

        cuvslam::Odometry::ImageSet images;
        for (std::size_t i = 0; i < images_.size(); ++i) {
            images.push_back(to_cuvslam_image(images_[i], newest, static_cast<std::uint32_t>(i)));
        }
        cuvslam::Odometry::ImageSet depths;
        if (rgbd_) {
            cuvslam::Image d = to_cuvslam_image(depth_, newest, images[0].camera_index);
            d.encoding = cuvslam::ImageData::Encoding::MONO;
            d.data_type = cuvslam::ImageData::DataType::UINT16;
            depths.push_back(d);
        }

        const cuvslam::PoseEstimate est = tracker_->Track(images, {}, depths);
        ++frames_;
        last_ts_ns_ = newest;
        clear_frame_set();

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
        fill_pose(corrected, map_from_rig, timestamp_ns, cfg_.map_frame, cfg_.base_frame);
        corrected_odometry_.publish(corrected);

        publish_tf(map_from_odom_raw, timestamp_ns, cfg_.map_frame, cfg_.odom_frame);

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

    void publish_tf(const Transform& pose, std::int64_t timestamp_ns, const std::string& frame,
                    const std::string& child) {
        geometry_msgs::TransformStamped stamped{};
        stamped.header.stamp.sec = static_cast<std::int32_t>(timestamp_ns / kNsPerSec);
        stamped.header.stamp.nsec = static_cast<std::int32_t>(timestamp_ns % kNsPerSec);
        stamped.header.frame_id = frame;
        stamped.child_frame_id = child;
        stamped.transform.translation.x = pose.translation[0];
        stamped.transform.translation.y = pose.translation[1];
        stamped.transform.translation.z = pose.translation[2];
        stamped.transform.rotation.x = pose.rotation[0];
        stamped.transform.rotation.y = pose.rotation[1];
        stamped.transform.rotation.z = pose.rotation[2];
        stamped.transform.rotation.w = pose.rotation[3];

        tf2_msgs::TFMessage message{};
        message.transforms = {stamped};
        message.transforms_length = 1;
        tf_.publish(message);
    }

    /// One edge for the whole run: a robot only ever sees a single odom path.
    /// Resets are debugging output, on the log, not a break in this stream.
    void publish(std::int64_t timestamp_ns) {
        nav_msgs::Odometry msg{};
        fill_pose(msg, world_from_rig_, timestamp_ns, cfg_.odom_frame, cfg_.base_frame);
        odometry_.publish(msg);
        publish_tf(world_from_rig_, timestamp_ns, cfg_.odom_frame, cfg_.base_frame);
        // With Slam running, map->odom is its correction and is published there. Only
        // one publisher may own an edge.
        if (!cfg_.enable_slam && cfg_.publish_map_to_odom) {
            publish_tf(Transform{}, timestamp_ns, cfg_.map_frame, cfg_.odom_frame);
        }
    }

    // config. Every one of these is set from cfg_ in build(), so no initializer
    // here can ever apply; python owns the defaults.
    CuvslamConfig cfg_{};
    /// camera_mode, parsed once. Everything downstream asks these rather than
    /// re-comparing the string, so an unknown mode cannot quietly mean "not stereo".
    bool stereo_{true};
    bool rgbd_{false};

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

    /// The rig, in cuVSLAM's camera order, and each camera's intrinsics by frame_id.
    std::vector<std::string> camera_frames_;
    std::vector<Transform> rig_from_camera_;
    std::optional<Transform> rig_from_imu_;
    std::unordered_map<std::string, sensor_msgs::CameraInfo> camera_info_;
    std::uint64_t unplaced_images_{0};

    // the frame set being assembled, one slot per rig camera
    std::vector<sensor_msgs::Image> images_;
    std::vector<bool> have_image_;
    sensor_msgs::Image depth_{};
    bool have_depth_{false};
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
    Output<tf2_msgs::TFMessage> tf_;
};

int main() {
    dimos::native::run_with_transport<CuvslamOdometry>();
    return 0;
}
