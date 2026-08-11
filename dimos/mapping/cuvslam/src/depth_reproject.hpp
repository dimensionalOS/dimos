// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Depth-to-camera reprojection for the cuVSLAM module's rgbd mode.

#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "sensor_msgs/CameraInfo.hpp"
#include "sensor_msgs/Image.hpp"
#include "transform.hpp"

namespace depth_reproject {

/// Depth recorded against one camera, reprojected onto another: deproject through the depth
/// intrinsics, move through camera_from_depth, project through the target intrinsics. Where
/// several depth pixels land on one target pixel the nearest surface wins, matching what the
/// target camera would have seen. Unhit pixels stay 0, which cuVSLAM reads as no depth.
inline void reproject_depth(const sensor_msgs::Image& depth,
                            const sensor_msgs::CameraInfo& depth_info,
                            const sensor_msgs::CameraInfo& camera_info,
                            const transform_math::Transform& camera_from_depth,
                            double units_per_meter, sensor_msgs::Image& out) {
    using transform_math::quat_rotate;

    out.header = depth.header;
    out.header.frame_id = camera_info.header.frame_id;
    out.width = camera_info.width;
    out.height = camera_info.height;
    out.encoding = depth.encoding;
    out.is_bigendian = depth.is_bigendian;
    out.step = out.width * static_cast<std::int32_t>(sizeof(std::uint16_t));
    out.data.assign(static_cast<std::size_t>(out.step) * out.height, 0);
    out.data_length = static_cast<std::int32_t>(out.data.size());

    // The rotation as columns, so each point costs multiplies rather than quaternion algebra.
    const std::array<double, 3> col_x = quat_rotate(camera_from_depth.rotation, {1.0, 0.0, 0.0});
    const std::array<double, 3> col_y = quat_rotate(camera_from_depth.rotation, {0.0, 1.0, 0.0});
    const std::array<double, 3> col_z = quat_rotate(camera_from_depth.rotation, {0.0, 0.0, 1.0});
    const std::array<double, 3>& origin = camera_from_depth.translation;
    const double fx_d = depth_info.K[0], fy_d = depth_info.K[4];
    const double cx_d = depth_info.K[2], cy_d = depth_info.K[5];
    const double fx_c = camera_info.K[0], fy_c = camera_info.K[4];
    const double cx_c = camera_info.K[2], cy_c = camera_info.K[5];

    auto* aligned = reinterpret_cast<std::uint16_t*>(out.data.data());
    for (std::int32_t v = 0; v < depth.height; ++v) {
        const auto* row = reinterpret_cast<const std::uint16_t*>(
            depth.data.data() + static_cast<std::size_t>(v) * depth.step);
        for (std::int32_t u = 0; u < depth.width; ++u) {
            const std::uint16_t raw = row[u];
            if (raw == 0) {
                continue;
            }
            const double z = raw / units_per_meter;
            const double x = (u - cx_d) / fx_d * z;
            const double y = (v - cy_d) / fy_d * z;
            const double z_c = col_x[2] * x + col_y[2] * y + col_z[2] * z + origin[2];
            if (z_c <= 0.0) {
                continue;
            }
            const double x_c = col_x[0] * x + col_y[0] * y + col_z[0] * z + origin[0];
            const double y_c = col_x[1] * x + col_y[1] * y + col_z[1] * z + origin[1];
            const auto u_c = static_cast<std::int32_t>(std::lround(fx_c * x_c / z_c + cx_c));
            const auto v_c = static_cast<std::int32_t>(std::lround(fy_c * y_c / z_c + cy_c));
            if (u_c < 0 || u_c >= out.width || v_c < 0 || v_c >= out.height) {
                continue;
            }
            const auto value = static_cast<std::uint16_t>(
                std::min(std::lround(z_c * units_per_meter), 65535L));
            std::uint16_t& slot = aligned[static_cast<std::size_t>(v_c) * out.width + u_c];
            if (slot == 0 || value < slot) {
                slot = value;
            }
        }
    }
}

}  // namespace depth_reproject
