// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// NVIDIA cuVSLAM visual odometry as a dimos native module.
//
// in:  image, camera_info (every camera on the one stream, told apart by frame_id),
//      tf, depth_image, imu
// out: odometry, corrected_odometry, tf
//
// Nothing is published while tracking is lost. cuVSLAM keeps one world frame for the
// life of the tracker, so it resumes in the same frame. The rig comes from the tf tree.

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
#include "utils/depth_reproject.hpp"
#include "utils/msg_convert.hpp"
#include "utils/transform.hpp"

using dimos::native::Builder;
using dimos::native::Config;
using dimos::native::Module;
using dimos::native::Output;
namespace logging = dimos::native::log;
using namespace depth_reproject;
using namespace msg_convert;
using namespace transform_math;

namespace {

/// cuVSLAM's Track() contract asks for stereo stamps within 1 ms.
constexpr std::int64_t MAX_PAIR_SKEW_NS = 1000000LL;  // 1 ms

std::string join(const std::vector<std::string>& parts) {
    std::string joined;
    for (const std::string& part : parts) {
        joined += joined.empty() ? part : ", " + part;
    }
    return joined;
}

struct RigCamera {
    std::string frame;
    Transform rig_from_camera;
    sensor_msgs::CameraInfo info;
    sensor_msgs::Image image;
    bool have_image{false};
};


}  // namespace

struct CuvslamConfig {
    /// "stereo", "mono" or "rgbd". Mono is accurate only up to scale.
    std::string camera_mode;
    /// One tf frame per camera, in cuVSLAM's index order. Empty discovers them off
    /// camera_info.
    std::vector<std::string> camera_frames;
    /// Images arrive rectified: no distortion, rows aligned.
    bool rectified;
    /// Off runs the tracker on the CPU. Needs a libcuvslam built with ENFORCE_GPU=OFF
    /// (the jeff-hykin/cuVSLAM fork build); NVIDIA's stock SDK binaries are GPU-only.
    bool use_gpu;
    std::string odom_frame;
    std::string base_frame;
    /// Frame the cuVSLAM rig is expressed in. Empty means base_frame. Setting it to a camera's
    /// optical frame reproduces NVIDIA's examples, whose rig IS the left camera; the published
    /// odometry stays on base_frame either way, since the two differ by a fixed transform.
    std::string rig_frame;
    std::string map_frame;
    /// Only read when Slam is off, where map->odom can only be identity.
    bool publish_map_to_odom;
    /// Pose graph + loop closure.
    bool enable_slam;
    /// GetPose() carries no timestamp, so a Slam thread running behind cannot be
    /// matched to the odometry pose it corrects.
    bool slam_async;
    /// Poses kept in the graph; 0 is unlimited.
    int slam_max_poses;
    /// Floor on the interval between loop closures, milliseconds.
    int slam_throttling_ms;
    /// Rebase guard. A frame whose translation standard deviation (root of the largest
    /// translation term of cuVSLAM's covariance) exceeds this is unconstrained: its motion
    /// is dropped, the pose holds, and later frames are rebased onto the held pose so the
    /// published path never carries the teleport. Meters; 0 disables the guard and the raw
    /// integrator is published untouched.
    double covariance_gate_translation_std;
    /// Rebase guard on physically implausible frame-to-frame motion, sharing the
    /// covariance gate's hold-and-rebase machinery but trusting kinematics instead of the
    /// tracker's self-report: a teleport with confident covariance still cannot claim the
    /// rig moved faster than the platform can. Linear is metres/second, angular is
    /// radians/second, both measured on the raw pose against the previous tracked frame;
    /// 0 disables that limit.
    double speed_gate_max_linear;
    double speed_gate_max_angular;
    /// cuVSLAM's Inertial mode is stereo plus one IMU.
    bool enable_imu;
    /// Flattened from the python ImuCalibration. All zero means enable_imu is off.
    double imu_gyro_noise_density;
    double imu_gyro_random_walk;
    double imu_accel_noise_density;
    double imu_accel_random_walk;
    /// The rate actually fed. Declaring more than arrives never initialises alignment.
    double imu_frequency;
    /// rgbd only: raw depth units per metre. 1000 for sixteen-bit millimetres.
    double depth_units_per_meter;

    void validate() const {
        if (camera_mode != "stereo" && camera_mode != "mono" && camera_mode != "rgbd") {
            throw std::runtime_error("camera_mode must be stereo, mono or rgbd, got '" +
                                     camera_mode + "'");
        }
    }
};

class CuvslamOdometry : public Module {
public:
    enum class Mode { Stereo, Mono, Rgbd };

    void build(Builder& builder, Config& config) override {
        cfg_ = config.parse<CuvslamConfig>();
        mode_ = cfg_.camera_mode == "stereo" ? Mode::Stereo
                : cfg_.camera_mode == "rgbd"  ? Mode::Rgbd
                                              : Mode::Mono;

        builder.input<tf2_msgs::TFMessage>("tf", &CuvslamOdometry::on_tf, this);
        builder.input<sensor_msgs::CameraInfo>("camera_info", &CuvslamOdometry::on_camera_info,
                                               this);
        builder.input<sensor_msgs::Image>("image", &CuvslamOdometry::on_image, this);
        if (mode_ == Mode::Rgbd) {
            builder.input<sensor_msgs::Image>("depth_image", &CuvslamOdometry::on_depth, this);
            builder.input<sensor_msgs::CameraInfo>("depth_camera_info",
                                                   &CuvslamOdometry::on_depth_camera_info, this);
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
                       logging::Field("depth_reprojected",
                                      static_cast<std::int64_t>(depth_reprojected_)),
                       logging::Field("covariance_gated",
                                      static_cast<std::int64_t>(covariance_gated_)),
                       logging::Field("speed_gated",
                                      static_cast<std::int64_t>(speed_gated_)),
                       logging::Field("unmatched_images",
                                      static_cast<std::int64_t>(unplaced_images_))});
    }

private:
    void on_camera_info(const sensor_msgs::CameraInfo& info) {
        if (tracker_) {
            return;  // rig is fixed once the tracker exists
        }
        camera_info_[info.header.frame_id] = info;
        resolve_rig();
    }

    /// The mount tree, which is the whole of the extrinsic calibration.
    void on_tf(const tf2_msgs::TFMessage& message) {
        if (tracker_) {
            return;  // rig is fixed once the tracker exists
        }
        for (const geometry_msgs::TransformStamped& stamped : message.transforms) {
            // Skip what this module publishes itself: odom and map are poses, not mount.
            if (stamped.header.frame_id == cfg_.map_frame ||
                stamped.header.frame_id == cfg_.odom_frame) {
                continue;
            }
            tf_edges_[stamped.child_frame_id] = {stamped.header.frame_id,
                                                 to_transform(stamped.transform)};
        }
        resolve_rig();
    }

    /// parent_frame -> child_frame through their nearest common ancestor. Not
    /// time-aware; the mount tree is rigid.
    std::optional<Transform> tf_lookup(const std::string& parent_frame,
                                       const std::string& child_frame) const {
        std::unordered_map<std::string, Transform> from_child{{child_frame, Transform{}}};
        std::string frame = child_frame;
        Transform ancestor_from_child;
        for (std::size_t step = 0; step < MAX_TF_DEPTH; ++step) {
            auto edge = tf_edges_.find(frame);
            if (edge == tf_edges_.end()) {
                break;
            }
            ancestor_from_child = compose(edge->second.second, ancestor_from_child);
            frame = edge->second.first;
            from_child[frame] = ancestor_from_child;
        }

        frame = parent_frame;
        Transform ancestor_from_parent;
        for (std::size_t step = 0; step <= MAX_TF_DEPTH; ++step) {
            auto shared = from_child.find(frame);
            if (shared != from_child.end()) {
                return compose(invert(ancestor_from_parent), shared->second);
            }
            auto edge = tf_edges_.find(frame);
            if (edge == tf_edges_.end()) {
                return std::nullopt;  // the two are not connected
            }
            ancestor_from_parent = compose(edge->second.second, ancestor_from_parent);
            frame = edge->second.first;
        }
        return std::nullopt;
    }

    /// The frame cuVSLAM's rig is expressed in, which need not be the published frame.
    const std::string& rig_frame() const {
        return cfg_.rig_frame.empty() ? cfg_.base_frame : cfg_.rig_frame;
    }

    /// Place every camera against the rig frame, or nothing until they all resolve.
    void resolve_rig() {
        if (!cameras_.empty()) {
            return;
        }
        const bool discovered = cfg_.camera_frames.empty();
        std::vector<std::string> frames = cfg_.camera_frames;
        if (discovered) {
            for (const auto& [frame, info] : camera_info_) {
                frames.push_back(frame);
            }
            // camera_info_ has no order of its own, and the rig is indexed.
            std::sort(frames.begin(), frames.end());
            const std::size_t expected = mode_ == Mode::Stereo ? 2 : 1;
            if (frames.size() != expected) {
                if (frames.size() > expected) {
                    DIMOS_LOG_THROTTLED(
                        logging::Level::Error, logging::from_secs(10),
                        "cuvslam found more cameras on camera_info than camera_mode uses, "
                        "and they have no discoverable order. Set camera_frames.",
                        logging::Field("found", join(frames)),
                        logging::Field("mode", cfg_.camera_mode));
                }
                return;
            }
        }

        std::vector<RigCamera> cameras;
        for (const std::string& frame : frames) {
            const std::optional<Transform> rig_from_camera = tf_lookup(rig_frame(), frame);
            const auto info = camera_info_.find(frame);
            if (!rig_from_camera || info == camera_info_.end()) {
                return;
            }
            cameras.push_back(RigCamera{frame, *rig_from_camera, info->second});
        }
        // cuVSLAM wants camera 0 to be the left of a pair, which is the one whose
        // partner sits at +x (optical convention, x to the right).
        if (discovered && cameras.size() == 2 &&
            compose(invert(cameras[0].rig_from_camera), cameras[1].rig_from_camera)
                    .translation[0] < 0.0) {
            std::swap(cameras[0], cameras[1]);
        }
        cameras_ = std::move(cameras);
    }

    std::vector<std::string> rig_frames() const {
        std::vector<std::string> frames;
        for (const RigCamera& camera : cameras_) {
            frames.push_back(camera.frame);
        }
        return frames;
    }

    /// Which camera in the rig a frame_id names, or -1 for one that is not on it.
    int camera_index(const std::string& frame_id) const {
        for (std::size_t i = 0; i < cameras_.size(); ++i) {
            if (cameras_[i].frame == frame_id) {
                return static_cast<int>(i);
            }
        }
        return -1;
    }

    /// Buffered. cuVSLAM requires Track() and RegisterImuMeasurement() in
    /// non-decreasing timestamp order; the round-robin dispatcher lets images
    /// overtake a 400 Hz IMU.
    void on_imu(const sensor_msgs::Imu& msg) {
        imu_frame_ = msg.header.frame_id;
        if (!tracker_) {
            // No tracker yet; this is the window inertial init needs.
            ++imu_dropped_;
            return;
        }
        cuvslam::ImuMeasurement measurement{};
        measurement.timestamp_ns = stamp_to_ns(msg.header);
        // Track() has already consumed everything up to last_ts_ns_.
        measurement.linear_accelerations = {static_cast<float>(msg.linear_acceleration.x),
                                            static_cast<float>(msg.linear_acceleration.y),
                                            static_cast<float>(msg.linear_acceleration.z)};
        measurement.angular_velocities = {static_cast<float>(msg.angular_velocity.x),
                                          static_cast<float>(msg.angular_velocity.y),
                                          static_cast<float>(msg.angular_velocity.z)};
        pending_imu_.push_back(measurement);
        ++imu_samples_;
    }

    void on_image(const sensor_msgs::Image& img) {
        const int index = camera_index(img.header.frame_id);
        if (index < 0) {
            ++unplaced_images_;
            DIMOS_LOG_THROTTLED(logging::Level::Warn, logging::from_secs(10),
                                "cuvslam dropping image with a frame_id not on the rig",
                                logging::Field("frame_id", img.header.frame_id),
                                logging::Field("dropped",
                                               static_cast<std::int64_t>(unplaced_images_)),
                                logging::Field("rig_cameras",
                                               static_cast<std::int64_t>(cameras_.size())));
            return;
        }
        cameras_[index].image = img;
        cameras_[index].have_image = true;
        try_track();
    }

    void on_depth(const sensor_msgs::Image& img) {
        depth_ = img;
        have_depth_ = true;
        try_track();
    }

    /// Intrinsics of the depth sensor itself, only needed when depth has to be reprojected
    /// onto the rig camera. Kept off `camera_info` so it cannot join the rig discovery.
    void on_depth_camera_info(const sensor_msgs::CameraInfo& info) {
        if (!have_depth_info_) {
            depth_info_ = info;
            have_depth_info_ = true;
        }
    }

    /// Depth pixel-aligned with the rig camera, as cuVSLAM's RGBD contract requires.
    /// Passthrough when it was recorded against that camera; reprojected through the depth
    /// intrinsics and the tf between the two sensors when it was not (a D455 records depth
    /// against the left IR camera, not the color camera). Null while the pieces to
    /// reproject are still missing.
    const sensor_msgs::Image* align_depth() {
        const RigCamera& camera = cameras_[0];
        if (depth_.header.frame_id == camera.frame) {
            return &depth_;
        }
        if (!have_depth_info_) {
            DIMOS_LOG_THROTTLED(logging::Level::Warn, logging::from_secs(10),
                                "cuvslam: depth is in another frame than the camera and needs "
                                "depth_camera_info to reproject",
                                logging::Field("depth_frame", depth_.header.frame_id),
                                logging::Field("camera_frame", camera.frame));
            return nullptr;
        }
        if (!camera_from_depth_) {
            camera_from_depth_ = tf_lookup(camera.frame, depth_.header.frame_id);
            if (!camera_from_depth_) {
                DIMOS_LOG_THROTTLED(logging::Level::Warn, logging::from_secs(10),
                                    "cuvslam: tf does not connect the depth frame to the camera",
                                    logging::Field("depth_frame", depth_.header.frame_id),
                                    logging::Field("camera_frame", camera.frame));
                return nullptr;
            }
            logging::info("cuvslam reprojecting depth onto the rig camera",
                          {logging::Field("depth_frame", depth_.header.frame_id),
                           logging::Field("camera_frame", camera.frame)});
        }
        reproject_depth(depth_, depth_info_, camera.info, *camera_from_depth_,
                        cfg_.depth_units_per_meter, aligned_depth_);
        ++depth_reprojected_;
        return &aligned_depth_;
    }

    cuvslam::Odometry::OdometryMode odometry_mode() const {
        using OdometryMode = cuvslam::Odometry::OdometryMode;
        if (mode_ == Mode::Rgbd) {
            return OdometryMode::RGBD;
        }
        if (mode_ == Mode::Mono) {
            return OdometryMode::Mono;
        }
        // Inertial is the stereo pair plus an IMU; there is no inertial mono or rgbd.
        return cfg_.enable_imu ? OdometryMode::Inertial : OdometryMode::Multicamera;
    }

    /// Builds the tracker against rig_frame(). Poses are converted back onto base_frame on
    /// publish, so rig_frame is an internal choice with no effect on the output contract.
    void ensure_tracker() {
        if (tracker_) {
            return;
        }
        const std::optional<Transform> base_from_rig = tf_lookup(cfg_.base_frame, rig_frame());
        if (!base_from_rig) {
            DIMOS_LOG_THROTTLED(logging::Level::Warn, logging::from_secs(10),
                                "cuvslam: tf does not place the rig frame against base_frame",
                                logging::Field("rig_frame", rig_frame()),
                                logging::Field("base_frame", cfg_.base_frame));
            return;
        }
        base_from_rig_ = *base_from_rig;
        rig_from_base_ = invert(*base_from_rig);
        cuvslam::Rig rig;
        for (const RigCamera& rig_camera : cameras_) {
            const sensor_msgs::CameraInfo& info = rig_camera.info;
            cuvslam::Camera camera{};
            camera.size = {info.width, info.height};
            camera.principal = {static_cast<float>(info.K[2]), static_cast<float>(info.K[5])};
            camera.focal = {static_cast<float>(info.K[0]), static_cast<float>(info.K[4])};
            camera.rig_from_camera = to_pose(rig_camera.rig_from_camera);
            camera.distortion = to_distortion(info);
            rig.cameras.push_back(camera);
        }
        if (cfg_.enable_imu) {
            const std::optional<Transform> rig_from_imu =
                imu_frame_.empty() ? std::nullopt : tf_lookup(rig_frame(), imu_frame_);
            if (!rig_from_imu) {
                DIMOS_LOG_THROTTLED(logging::Level::Warn, logging::from_secs(10),
                                    "cuvslam: enable_imu is on but tf does not place the IMU",
                                    logging::Field("imu_frame", imu_frame_));
                return;
            }
            cuvslam::ImuCalibration imu{};
            imu.rig_from_imu = to_pose(*rig_from_imu);
            imu.gyroscope_noise_density = static_cast<float>(cfg_.imu_gyro_noise_density);
            imu.gyroscope_random_walk = static_cast<float>(cfg_.imu_gyro_random_walk);
            imu.accelerometer_noise_density = static_cast<float>(cfg_.imu_accel_noise_density);
            imu.accelerometer_random_walk = static_cast<float>(cfg_.imu_accel_random_walk);
            imu.frequency = static_cast<float>(cfg_.imu_frequency);
            rig.imus = {imu};
        }

        cuvslam::Odometry::Config odometry_cfg = cuvslam::Odometry::GetDefaultConfig();
        odometry_cfg.odometry_mode = odometry_mode();
        odometry_cfg.use_gpu = cfg_.use_gpu;
        // cuVSLAM rejects this outright unless the rig has a stereo pair: "Rectified
        // stereo camera mode only works with 1+ stereo cameras".
        odometry_cfg.rectified_stereo_camera = cfg_.rectified && mode_ == Mode::Stereo;
        odometry_cfg.enable_landmarks_export = cfg_.enable_slam;
        // Slam reads the tracker's State, which GetState() only fills when the
        // export flags are on; without them it throws instead of returning empty.
        odometry_cfg.enable_observations_export = cfg_.enable_slam;
        // On, cuVSLAM's bundle adjustment throws from its own thread, where no
        // caller-side handler can catch it, and the process aborts.
        odometry_cfg.async_sba = false;
        if (mode_ == Mode::Rgbd) {
            odometry_cfg.rgbd_settings.depth_scale_factor =
                static_cast<float>(cfg_.depth_units_per_meter);
            // The default of -1 means no camera, and depth belonging to nothing is silently
            // ignored. A depth stream usually reports its own frame, so unrecognised means 0.
            odometry_cfg.rgbd_settings.depth_camera_id =
                std::max(camera_index(depth_.header.frame_id), 0);
        }

        tracker_.emplace(rig, odometry_cfg);
        if (cfg_.enable_slam) {
            cuvslam::Slam::Config slam_cfg = cuvslam::Slam::GetDefaultConfig();
            slam_cfg.sync_mode = !cfg_.slam_async;
            slam_cfg.use_gpu = cfg_.use_gpu;
            slam_cfg.max_map_size = static_cast<std::uint32_t>(cfg_.slam_max_poses);
            slam_cfg.throttling_time_ms = static_cast<std::uint32_t>(cfg_.slam_throttling_ms);
            slam_.emplace(rig, tracker_->GetPrimaryCameras(), slam_cfg);
        }
        logging::info("cuvslam tracker created",
                      {logging::Field("cameras", static_cast<std::int64_t>(rig.cameras.size())),
                       logging::Field("width", static_cast<std::int64_t>(rig.cameras[0].size[0])),
                       logging::Field("height", static_cast<std::int64_t>(rig.cameras[0].size[1])),
                       logging::Field("rig_frames", join(rig_frames()))});
    }

    /// Hand cuVSLAM every sample that precedes the frame about to be tracked.
    void register_imu_through(std::int64_t timestamp_ns) {
        auto after = std::find_if(pending_imu_.begin(), pending_imu_.end(),
                                  [timestamp_ns](const cuvslam::ImuMeasurement& measurement) {
                                      return measurement.timestamp_ns > timestamp_ns;
                                  });
        for (auto it = pending_imu_.begin(); it != after; ++it) {
            tracker_->RegisterImuMeasurement(0, *it);
        }
        pending_imu_.erase(pending_imu_.begin(), after);
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
        for (RigCamera& camera : cameras_) {
            camera.have_image = false;
        }
        have_depth_ = false;
    }

    void try_track() {
        if (cameras_.empty() || (mode_ == Mode::Rgbd && !have_depth_)) {
            return;
        }
        for (const RigCamera& camera : cameras_) {
            if (!camera.have_image) {
                return;
            }
        }
        ensure_tracker();
        if (!tracker_) {
            return;  // no camera_info yet
        }

        std::int64_t oldest = stamp_to_ns(cameras_[0].image.header);
        std::int64_t newest = oldest;
        for (const RigCamera& camera : cameras_) {
            const std::int64_t ts = stamp_to_ns(camera.image.header);
            oldest = std::min(oldest, ts);
            newest = std::max(newest, ts);
        }
        if (newest - oldest > MAX_PAIR_SKEW_NS) {
            ++skew_rejects_;
            DIMOS_LOG_THROTTLED(logging::Level::Warn, logging::from_secs(10),
                                "cuvslam frame sets exceed the 1 ms skew limit",
                                logging::Field("rejected", static_cast<std::int64_t>(skew_rejects_)),
                                logging::Field("skew_ms", (newest - oldest) / 1.0e6));
            return;
        }
        // cuVSLAM rejects a frame that is not strictly newer than the last one.
        if (last_ts_ns_ && newest <= *last_ts_ns_) {
            clear_frame_set();
            return;
        }

        cuvslam::Odometry::ImageSet images;
        for (std::size_t i = 0; i < cameras_.size(); ++i) {
            images.push_back(
                to_cuvslam_image(cameras_[i].image, newest, static_cast<std::uint32_t>(i)));
        }
        cuvslam::Odometry::ImageSet depths;
        if (mode_ == Mode::Rgbd) {
            const sensor_msgs::Image* aligned = align_depth();
            if (aligned == nullptr) {
                return;  // reprojection inputs still missing; retry on the next message
            }
            cuvslam::Image depth = to_cuvslam_image(*aligned, newest, images[0].camera_index);
            depth.encoding = cuvslam::ImageData::Encoding::MONO;
            depth.data_type = cuvslam::ImageData::DataType::UINT16;
            depths.push_back(depth);
        }

        register_imu_through(newest);
        const cuvslam::PoseEstimate est = tracker_->Track(images, {}, depths);
        ++frames_;
        last_ts_ns_ = newest;
        clear_frame_set();

        if (!est.world_from_rig.has_value()) {
            if (was_tracking_) {
                ++segment_id_;
                was_tracking_ = false;
                // The next tracked frame restarts across an unmeasured gap; its speed
                // against the pre-loss pose would be meaningless.
                previous_raw_.reset();
                logging::warn("cuvslam tracking lost",
                              {logging::Field("segment", static_cast<std::int64_t>(segment_id_))});
            }
            return;
        }
        // cuVSLAM tracks rig_frame(); the contract is base_frame starting at identity. Both
        // collapse to the raw pose when the two frames are the same.
        const Transform raw_pose = compose(
            base_from_rig_, compose(to_transform(est.world_from_rig->pose), rig_from_base_));
        covariance_ = est.world_from_rig->covariance_xyz_rpy;
        const double translation_std = std::sqrt(static_cast<double>(
            std::max({covariance_[0], covariance_[7], covariance_[14]})));
        // NaN covariance is the tracker's own way of saying unconstrained; a NaN never
        // exceeds a threshold, so it has to be gated explicitly.
        bool gate_frame = false;
        if (cfg_.covariance_gate_translation_std > 0.0 && was_tracking_ &&
            (!std::isfinite(translation_std) ||
             translation_std > cfg_.covariance_gate_translation_std)) {
            gate_frame = true;
            ++covariance_gated_;
            DIMOS_LOG_THROTTLED(logging::Level::Warn, logging::from_secs(5),
                                "cuvslam covariance gate holding pose",
                                logging::Field("translation_std", translation_std),
                                logging::Field("gated",
                                               static_cast<std::int64_t>(covariance_gated_)));
        }
        // Speed is judged against the previous raw pose even when that frame was gated:
        // after a teleport the tracker keeps integrating from the far side, so later
        // frames are near the teleported pose and only the jump frame itself trips.
        if ((cfg_.speed_gate_max_linear > 0.0 || cfg_.speed_gate_max_angular > 0.0) &&
            previous_raw_.has_value()) {
            // Track() rejects non-increasing stamps, so dt is strictly positive here.
            const double dt = static_cast<double>(est.timestamp_ns - previous_raw_ns_) / 1.0e9;
            const double linear_speed = translation_between(*previous_raw_, raw_pose) / dt;
            const double angular_speed =
                angle_between(previous_raw_->rotation, raw_pose.rotation) / dt;
            if ((cfg_.speed_gate_max_linear > 0.0 &&
                 linear_speed > cfg_.speed_gate_max_linear) ||
                (cfg_.speed_gate_max_angular > 0.0 &&
                 angular_speed > cfg_.speed_gate_max_angular)) {
                gate_frame = true;
                ++speed_gated_;
                DIMOS_LOG_THROTTLED(logging::Level::Warn, logging::from_secs(5),
                                    "cuvslam speed gate holding pose",
                                    logging::Field("linear_mps", linear_speed),
                                    logging::Field("angular_rps", angular_speed),
                                    logging::Field("gated",
                                                   static_cast<std::int64_t>(speed_gated_)));
            }
        }
        previous_raw_ = raw_pose;
        previous_raw_ns_ = est.timestamp_ns;
        if (gate_frame) {
            // Implausible frame (blank wall, repeated texture, teleport): drop its motion
            // and keep rebasing onto the held pose, so recovery continues from here with
            // only the delta measured after tracking became sane again.
            rebase_ = compose(world_from_rig_, invert(raw_pose));
            frame_gated_ = true;
        } else {
            world_from_rig_ = compose(rebase_, raw_pose);
            frame_gated_ = false;
        }
        was_tracking_ = true;
        ++tracked_;
        publish(est.timestamp_ns);
        if (slam_) {
            run_slam(est.timestamp_ns);
        }
    }

    void run_slam(std::int64_t timestamp_ns) {
        cuvslam::Odometry::State state;
        tracker_->GetState(state);
        slam_->Track(state);

        // Identity until Slam has a keyframe, which would read as a huge correction.
        slam_started_ = slam_started_ || state.keyframe;
        if (!slam_started_) {
            return;
        }
        // Slam tracks the same rig, so its pose needs the same move onto base_frame as the
        // odometry one before it can be compared with world_from_rig_ or published.
        const Transform slam_pose = compose(to_transform(slam_->GetPose()), rig_from_base_);
        // The gate's decision carries over: an unconstrained frame poisons the slam pose
        // through the same estimate, so the loop-closed stream holds and rebases with the
        // VO stream. A real loop-closure snap has ordinary covariance and passes through.
        if (frame_gated_) {
            corrected_rebase_ = compose(map_from_base_, invert(slam_pose));
        } else {
            map_from_base_ = compose(corrected_rebase_, slam_pose);
        }
        const Transform& map_from_base = map_from_base_;

        const Transform map_from_odom_raw = compose(map_from_base, invert(world_from_rig_));

        nav_msgs::Odometry corrected{};
        fill_pose(corrected, map_from_base, timestamp_ns, cfg_.map_frame, cfg_.base_frame);
        corrected_odometry_.publish(corrected);

        publish_tf(map_from_odom_raw, timestamp_ns, cfg_.map_frame, cfg_.odom_frame);

        cuvslam::Slam::Metrics metrics{};
        slam_->GetSlamMetrics(metrics);
        // The metrics are retained between Slam steps, which happen per keyframe, so a
        // closure reads back on every frame until the next one. Key on the step's own stamp.
        if (metrics.lc_status && metrics.timestamp_ns != last_closure_ns_) {
            ++loop_closures_;
            last_closure_ns_ = metrics.timestamp_ns;
            logging::info("cuvslam loop closure",
                          {logging::Field("count", static_cast<std::int64_t>(loop_closures_)),
                           logging::Field("timestamp_ns", metrics.timestamp_ns),
                           logging::Field("tracked_landmarks",
                                          static_cast<std::int64_t>(metrics.lc_tracked_landmarks_count))});
        }
    }

    static void fill_pose(nav_msgs::Odometry& msg, const Transform& pose, std::int64_t timestamp_ns,
                          const std::string& frame, const std::string& child) {
        msg.header.stamp = to_stamp(timestamp_ns);
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
        stamped.header.stamp = to_stamp(timestamp_ns);
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

    void publish(std::int64_t timestamp_ns) {
        nav_msgs::Odometry msg{};
        fill_pose(msg, world_from_rig_, timestamp_ns, cfg_.odom_frame, cfg_.base_frame);
        // cuVSLAM's own 6x6, row-major xyz-rpy, the order ROS uses. Reported against the
        // rig frame, which is base_frame unless rig_frame overrides it.
        for (std::size_t i = 0; i < covariance_.size(); ++i) {
            msg.pose.covariance[i] = covariance_[i];
        }
        odometry_.publish(msg);
        publish_tf(world_from_rig_, timestamp_ns, cfg_.odom_frame, cfg_.base_frame);
        // With Slam running, map->odom is its correction; only one publisher per edge.
        if (!cfg_.enable_slam && cfg_.publish_map_to_odom) {
            publish_tf(Transform{}, timestamp_ns, cfg_.map_frame, cfg_.odom_frame);
        }
    }

    CuvslamConfig cfg_{};
    /// camera_mode, parsed once.
    Mode mode_{Mode::Stereo};

    // slam
    std::optional<cuvslam::Slam> slam_;
    std::vector<cuvslam::ImuMeasurement> pending_imu_;
    std::uint64_t imu_samples_{0};
    std::uint64_t imu_dropped_{0};
    bool slam_started_{false};
    std::uint64_t loop_closures_{0};
    std::int64_t last_closure_ns_{-1};

    std::uint64_t skew_rejects_{0};

    /// The rig, in cuVSLAM's camera order.
    std::vector<RigCamera> cameras_;
    std::unordered_map<std::string, sensor_msgs::CameraInfo> camera_info_;
    std::uint64_t unplaced_images_{0};
    std::string imu_frame_;

    /// child frame -> (its parent, parent_from_child). The bound is a cycle guard.
    static constexpr std::size_t MAX_TF_DEPTH = 32;
    std::unordered_map<std::string, std::pair<std::string, Transform>> tf_edges_;

    /// covariance gate
    cuvslam::PoseCovariance covariance_{};
    Transform rebase_;  ///< identity until the gate first fires
    bool frame_gated_{false};
    Transform corrected_rebase_;
    Transform map_from_base_;  ///< last published loop-closed pose
    std::uint64_t covariance_gated_{0};
    std::optional<Transform> previous_raw_;  ///< raw pose of the last tracked frame
    std::int64_t previous_raw_ns_{0};
    std::uint64_t speed_gated_{0};

    sensor_msgs::Image depth_{};
    bool have_depth_{false};
    sensor_msgs::CameraInfo depth_info_{};
    bool have_depth_info_{false};
    sensor_msgs::Image aligned_depth_{};
    std::optional<Transform> camera_from_depth_;
    std::uint64_t depth_reprojected_{0};
    std::optional<std::int64_t> last_ts_ns_;

    // tracking state
    std::optional<cuvslam::Odometry> tracker_;
    Transform world_from_rig_;  ///< last published pose, on base_frame
    Transform base_from_rig_;   ///< fixed; identity when rig_frame is base_frame
    Transform rig_from_base_;
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
