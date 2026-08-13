// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Boilerplate converters between dimos-lcm messages, cuVSLAM SDK types and
// Eigen: stamps, poses, distortion models.

#pragma once

#include <algorithm>
#include <cstdint>
#include <vector>

#include <Eigen/Geometry>

#include "cuvslam/cuvslam2.h"
#include "geometry_msgs/Transform.hpp"
#include "sensor_msgs/CameraInfo.hpp"
#include "std_msgs/Header.hpp"
#include "std_msgs/Time.hpp"
#include "tf.hpp"

namespace msg_convert {

constexpr std::int64_t NS_PER_SEC = 1000000000LL;

inline std::int64_t stamp_to_ns(const std_msgs::Header& header) {
    return static_cast<std::int64_t>(header.stamp.sec) * NS_PER_SEC +
           static_cast<std::int64_t>(header.stamp.nsec);
}

inline std_msgs::Time to_stamp(std::int64_t timestamp_ns) {
    std_msgs::Time stamp{};
    stamp.sec = static_cast<std::int32_t>(timestamp_ns / NS_PER_SEC);
    stamp.nsec = static_cast<std::int32_t>(timestamp_ns % NS_PER_SEC);
    return stamp;
}

inline Eigen::Isometry3d to_isometry(const cuvslam::Pose& pose) {
    Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
    // cuvslam::Pose rotation is xyzw; Eigen's quaternion constructor takes w first.
    iso.linear() = Eigen::Quaterniond(pose.rotation[3], pose.rotation[0], pose.rotation[1],
                                      pose.rotation[2])
                       .toRotationMatrix();
    iso.translation() =
        Eigen::Vector3d(pose.translation[0], pose.translation[1], pose.translation[2]);
    return iso;
}

inline Eigen::Isometry3d to_isometry(const geometry_msgs::Transform& message) {
    Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
    iso.linear() = Eigen::Quaterniond(message.rotation.w, message.rotation.x,
                                      message.rotation.y, message.rotation.z)
                       .toRotationMatrix();
    iso.translation() = Eigen::Vector3d(message.translation.x, message.translation.y,
                                        message.translation.z);
    return iso;
}

inline Eigen::Isometry3d to_isometry(const tf_client::Rigid& rigid) {
    Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
    iso.linear() = Eigen::Quaterniond(rigid.rotation[3], rigid.rotation[0], rigid.rotation[1],
                                      rigid.rotation[2])
                       .toRotationMatrix();
    iso.translation() = Eigen::Vector3d(rigid.translation[0], rigid.translation[1],
                                        rigid.translation[2]);
    return iso;
}

inline tf_client::Rigid to_rigid(const Eigen::Isometry3d& iso) {
    const Eigen::Quaterniond rotation(iso.linear());
    return tf_client::Rigid{
        {rotation.x(), rotation.y(), rotation.z(), rotation.w()},
        {iso.translation().x(), iso.translation().y(), iso.translation().z()}};
}

inline cuvslam::Pose to_pose(const Eigen::Isometry3d& iso) {
    const Eigen::Quaterniond rotation(iso.linear());
    const Eigen::Vector3d& translation = iso.translation();
    cuvslam::Pose pose{};
    pose.rotation = {static_cast<float>(rotation.x()), static_cast<float>(rotation.y()),
                     static_cast<float>(rotation.z()), static_cast<float>(rotation.w())};
    pose.translation = {static_cast<float>(translation.x()), static_cast<float>(translation.y()),
                        static_cast<float>(translation.z())};
    return pose;
}

/// Pinhole for a rectified camera, Brown for one whose camera_info carries real plumb_bob
/// coefficients (a D455's color stream is raw; its IR streams are rectified with D all zero).
/// ROS orders plumb_bob (k1, k2, p1, p2, k3); cuVSLAM's Brown wants (k1, k2, k3, p1, p2).
inline cuvslam::Distortion to_distortion(const sensor_msgs::CameraInfo& info) {
    const std::vector<double>& d = info.D;
    const bool distorted =
        d.size() >= 5 && std::any_of(d.begin(), d.end(), [](double c) { return c != 0.0; });
    if (!distorted) {
        return cuvslam::Distortion{cuvslam::Distortion::Model::Pinhole};
    }
    cuvslam::Distortion distortion{cuvslam::Distortion::Model::Brown};
    distortion.parameters = {static_cast<float>(d[0]), static_cast<float>(d[1]),
                             static_cast<float>(d[4]), static_cast<float>(d[2]),
                             static_cast<float>(d[3])};
    return distortion;
}

}  // namespace msg_convert
