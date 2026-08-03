// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// NVIDIA cuVSLAM stereo visual odometry as a dimos native module.
//
// in:  image_left/image_right (mono8, rectified), camera_info
// out: odometry, landmarks (cuVSLAM's tracked 3D points)
//
// cuVSLAM restarts its world frame after a tracking loss, so poses must never be
// differenced across a segment change; segment_id rides along in child_frame_id.
// baseline_m is config, not derived: these recordings have right P[3] == 0.

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

}  // namespace

struct CuvslamConfig {
    double baseline_m;   ///< metres
    bool rectified;      ///< D455 IR pair is rectified on-device (D=0, R=I)
    bool publish_landmarks;
    bool async_sba;  ///< off makes a replay reproducible, at some accuracy cost
};

class CuvslamOdometry : public Module {
public:
    void build(Builder& builder, Config& config) override {
        const CuvslamConfig cfg = config.parse<CuvslamConfig>();
        baseline_m_ = cfg.baseline_m;
        rectified_ = cfg.rectified;
        publish_landmarks_ = cfg.publish_landmarks;
        async_sba_ = cfg.async_sba;
        if (!(baseline_m_ > 0.0)) {
            throw std::runtime_error(
                "baseline_m must be a positive number of metres (D455 factory value is 0.09486)");
        }

        builder.input<sensor_msgs::CameraInfo>("camera_info", &CuvslamOdometry::on_camera_info,
                                               this);
        builder.input<sensor_msgs::Image>("image_left", &CuvslamOdometry::on_left, this);
        builder.input<sensor_msgs::Image>("image_right", &CuvslamOdometry::on_right, this);

        odometry_ = builder.output<nav_msgs::Odometry>("odometry");
        landmarks_ = builder.output<sensor_msgs::PointCloud2>("landmarks");
    }

    void teardown() override {
        logging::info("cuvslam shutting down",
                      {logging::Field("frames", static_cast<std::int64_t>(frames_)),
                       logging::Field("tracked", static_cast<std::int64_t>(tracked_)),
                       logging::Field("segments", static_cast<std::int64_t>(segment_id_))});
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

        cuvslam::Odometry::Config cfg = cuvslam::Odometry::GetDefaultConfig();
        cfg.odometry_mode = cuvslam::Odometry::OdometryMode::Multicamera;
        cfg.rectified_stereo_camera = rectified_;
        cfg.enable_landmarks_export = publish_landmarks_;
        cfg.async_sba = async_sba_;

        tracker_.emplace(rig, cfg);
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

        const cuvslam::PoseEstimate est = tracker_->Track({l, r});
        ++frames_;
        last_ts_ns_ = t_left;
        have_left_ = have_right_ = false;

        if (!est.world_from_rig.has_value()) {
            // Next valid pose may be in a new world frame.
            if (was_tracking_) {
                ++segment_id_;
                was_tracking_ = false;
                logging::warn("cuvslam tracking lost",
                              {logging::Field("segment", static_cast<std::int64_t>(segment_id_))});
            }
            return;
        }
        was_tracking_ = true;
        ++tracked_;
        publish(est);
        if (publish_landmarks_) {
            publish_landmarks(est.timestamp_ns);
        }
    }

    void publish(const cuvslam::PoseEstimate& est) {
        const cuvslam::Pose& p = est.world_from_rig->pose;
        nav_msgs::Odometry msg{};
        msg.header.stamp.sec = static_cast<std::int32_t>(est.timestamp_ns / kNsPerSec);
        msg.header.stamp.nsec = static_cast<std::int32_t>(est.timestamp_ns % kNsPerSec);
        msg.header.frame_id = "world";
        msg.child_frame_id = "cuvslam_rig/segment_" + std::to_string(segment_id_);
        msg.pose.pose.position.x = p.translation[0];
        msg.pose.pose.position.y = p.translation[1];
        msg.pose.pose.position.z = p.translation[2];
        msg.pose.pose.orientation.x = p.rotation[0];
        msg.pose.pose.orientation.y = p.rotation[1];
        msg.pose.pose.orientation.z = p.rotation[2];
        msg.pose.pose.orientation.w = p.rotation[3];
        odometry_.publish(msg);
    }

    /// cuVSLAM's landmarks are the map: 3D points it is currently tracking.
    void publish_landmarks(std::int64_t timestamp_ns) {
        const std::vector<cuvslam::Landmark> pts = tracker_->GetLastLandmarks();
        if (pts.empty()) {
            return;
        }
        sensor_msgs::PointCloud2 msg{};
        msg.header.stamp.sec = static_cast<std::int32_t>(timestamp_ns / kNsPerSec);
        msg.header.stamp.nsec = static_cast<std::int32_t>(timestamp_ns % kNsPerSec);
        msg.header.frame_id = "world";
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
            out[3 * i + 0] = pts[i].coords[0];
            out[3 * i + 1] = pts[i].coords[1];
            out[3 * i + 2] = pts[i].coords[2];
        }
        msg.data_length = static_cast<std::int32_t>(msg.data.size());
        landmarks_.publish(msg);
    }

    // config
    double baseline_m_{0.0};
    bool rectified_{true};
    bool publish_landmarks_{true};
    bool async_sba_{true};

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
    bool was_tracking_{false};
    std::uint64_t segment_id_{0};
    std::uint64_t frames_{0};
    std::uint64_t tracked_{0};

    Output<nav_msgs::Odometry> odometry_;
    Output<sensor_msgs::PointCloud2> landmarks_;
};

int main() {
    dimos::native::run_with_transport<CuvslamOdometry>();
    return 0;
}
