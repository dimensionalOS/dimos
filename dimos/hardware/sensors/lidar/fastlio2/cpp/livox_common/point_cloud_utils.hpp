// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Shared helpers for livox-based lidar drivers.

#pragma once

#include <cmath>
#include <cstdint>
#include <string>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/header.hpp>
#include <builtin_interfaces/msg/time.hpp>

namespace dimos {

inline builtin_interfaces::msg::Time time_from_seconds(double t) {
    builtin_interfaces::msg::Time ts;
    ts.sec = static_cast<int32_t>(std::floor(t));
    ts.nanosec = static_cast<uint32_t>((t - ts.sec) * 1e9);
    return ts;
}

// ROS Header retains the source timestamp and frame.
inline std_msgs::msg::Header make_header(const std::string& frame_id, double ts) {
    std_msgs::msg::Header h;
    h.stamp = time_from_seconds(ts);
    h.frame_id = frame_id;
    return h;
}

// x, y, z, intensity, each a float32.
inline constexpr int32_t kXyziFieldCount = 4;
inline constexpr int32_t kXyziPointStep = kXyziFieldCount * sizeof(float);

// Empty XYZI PointCloud2 sized for num_points. The caller fills each point
// through xyzi_point() below.
inline sensor_msgs::msg::PointCloud2 make_xyzi_cloud(const std::string& frame_id, double ts,
                                                int num_points) {
    sensor_msgs::msg::PointCloud2 pc;
    pc.header = make_header(frame_id, ts);
    pc.height = 1;
    pc.width = num_points;
    pc.is_bigendian = 0;
    pc.is_dense = 1;

    pc.fields.resize(kXyziFieldCount);
    auto make_field = [](const std::string& name, int32_t index) {
        sensor_msgs::msg::PointField f;
        f.name = name;
        f.offset = index * static_cast<int32_t>(sizeof(float));
        f.datatype = sensor_msgs::msg::PointField::FLOAT32;
        f.count = 1;
        return f;
    };
    pc.fields[0] = make_field("x", 0);
    pc.fields[1] = make_field("y", 1);
    pc.fields[2] = make_field("z", 2);
    pc.fields[3] = make_field("intensity", 3);

    pc.point_step = kXyziPointStep;
    pc.row_step = pc.point_step * num_points;
    pc.data.resize(pc.row_step);
    return pc;
}

// The x/y/z/intensity slot of point i in a cloud from make_xyzi_cloud.
inline float* xyzi_point(sensor_msgs::msg::PointCloud2& pc, int i) {
    return reinterpret_cast<float*>(pc.data.data() + i * kXyziPointStep);
}

}  // namespace dimos
