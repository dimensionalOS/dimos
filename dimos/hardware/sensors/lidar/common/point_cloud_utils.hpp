// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Shared message helpers for the Livox-based lidar drivers (fastlio2, pointlio,
// livox). Each driver sources points differently, so the PointCloud2 layout and
// header are built here and the caller fills the packed xyzi buffer.

#pragma once

#include <atomic>
#include <cstdint>
#include <string>

#include "sensor_msgs/PointCloud2.hpp"
#include "sensor_msgs/PointField.hpp"
#include "std_msgs/Header.hpp"
#include "std_msgs/Time.hpp"

namespace dimos {

inline std_msgs::Time time_from_seconds(double t) {
    std_msgs::Time ts;
    ts.sec = static_cast<int32_t>(t);
    ts.nsec = static_cast<int32_t>((t - ts.sec) * 1e9);
    return ts;
}

// Stamped Header with a process-wide auto-incrementing sequence number.
inline std_msgs::Header make_header(const std::string& frame_id, double ts) {
    static std::atomic<int32_t> seq{0};
    std_msgs::Header h;
    h.seq = seq.fetch_add(1, std::memory_order_relaxed);
    h.stamp = time_from_seconds(ts);
    h.frame_id = frame_id;
    return h;
}

// Empty XYZI PointCloud2 (x/y/z/intensity float32, point_step 16) sized for
// num_points. The caller writes 4 floats per point into
// reinterpret_cast<float*>(pc.data.data()).
inline sensor_msgs::PointCloud2 make_xyzi_cloud(const std::string& frame_id, double ts,
                                                int num_points) {
    sensor_msgs::PointCloud2 pc;
    pc.header = make_header(frame_id, ts);
    pc.height = 1;
    pc.width = num_points;
    pc.is_bigendian = 0;
    pc.is_dense = 1;

    pc.fields_length = 4;
    pc.fields.resize(4);
    auto make_field = [](const std::string& name, int32_t offset) {
        sensor_msgs::PointField f;
        f.name = name;
        f.offset = offset;
        f.datatype = sensor_msgs::PointField::FLOAT32;
        f.count = 1;
        return f;
    };
    pc.fields[0] = make_field("x", 0);
    pc.fields[1] = make_field("y", 4);
    pc.fields[2] = make_field("z", 8);
    pc.fields[3] = make_field("intensity", 12);

    pc.point_step = 16;
    pc.row_step = pc.point_step * num_points;
    pc.data_length = pc.row_step;
    pc.data.resize(pc.data_length);
    return pc;
}

}  // namespace dimos
