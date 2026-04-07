// Point cloud utility functions for SmartNav native modules.
// Provides PointCloud2 building/parsing helpers that work with dimos-lcm types.
// When USE_PCL is defined, also provides PCL interop utilities.

#pragma once

#include <cmath>
#include <cstring>
#include <vector>

#include "sensor_msgs/PointCloud2.hpp"
#include "sensor_msgs/PointField.hpp"
#include "std_msgs/Header.hpp"

#include "dimos_native_module.hpp"

#ifdef USE_PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#endif

namespace smart_nav {

// Simple XYZI point structure (no PCL dependency)
struct PointXYZI {
    float x, y, z, intensity;
};

// Build PointCloud2 from vector of XYZI points
inline sensor_msgs::PointCloud2 build_pointcloud2(
    const std::vector<PointXYZI>& points,
    const std::string& frame_id,
    double timestamp
) {
    sensor_msgs::PointCloud2 pc;
    pc.header = dimos::make_header(frame_id, timestamp);
    pc.height = 1;
    pc.width = static_cast<int32_t>(points.size());
    pc.is_bigendian = 0;
    pc.is_dense = 1;

    // Fields: x, y, z, intensity (all float32)
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
    pc.row_step = pc.point_step * pc.width;
    pc.data_length = pc.row_step;
    pc.data.resize(pc.data_length);

    for (size_t i = 0; i < points.size(); ++i) {
        float* dst = reinterpret_cast<float*>(pc.data.data() + i * 16);
        dst[0] = points[i].x;
        dst[1] = points[i].y;
        dst[2] = points[i].z;
        dst[3] = points[i].intensity;
    }

    return pc;
}

// Parse PointCloud2 into vector of XYZI points
inline std::vector<PointXYZI> parse_pointcloud2(const sensor_msgs::PointCloud2& pc) {
    std::vector<PointXYZI> points;
    if (pc.width == 0 || pc.height == 0) return points;

    int num_points = pc.width * pc.height;
    points.reserve(num_points);

    // Find field offsets
    int x_off = -1, y_off = -1, z_off = -1, i_off = -1;
    for (const auto& f : pc.fields) {
        if (f.name == "x") x_off = f.offset;
        else if (f.name == "y") y_off = f.offset;
        else if (f.name == "z") z_off = f.offset;
        else if (f.name == "intensity") i_off = f.offset;
    }

    if (x_off < 0 || y_off < 0 || z_off < 0) return points;

    for (int n = 0; n < num_points; ++n) {
        if (static_cast<size_t>((n + 1) * pc.point_step) > pc.data.size()) break;
        const uint8_t* base = pc.data.data() + n * pc.point_step;
        PointXYZI p;
        std::memcpy(&p.x, base + x_off, sizeof(float));
        std::memcpy(&p.y, base + y_off, sizeof(float));
        std::memcpy(&p.z, base + z_off, sizeof(float));
        if (i_off >= 0) std::memcpy(&p.intensity, base + i_off, sizeof(float));
        else p.intensity = 0.0f;
        points.push_back(p);
    }

    return points;
}

// Get timestamp from PointCloud2 header
inline double get_timestamp(const sensor_msgs::PointCloud2& pc) {
    return pc.header.stamp.sec + pc.header.stamp.nsec / 1e9;
}

#ifdef USE_PCL
// Convert dimos-lcm PointCloud2 to PCL point cloud
inline void to_pcl(const sensor_msgs::PointCloud2& pc,
                   pcl::PointCloud<pcl::PointXYZI>& cloud) {
    auto points = parse_pointcloud2(pc);
    cloud.clear();
    cloud.reserve(points.size());
    for (const auto& p : points) {
        pcl::PointXYZI pt;
        pt.x = p.x;
        pt.y = p.y;
        pt.z = p.z;
        pt.intensity = p.intensity;
        cloud.push_back(pt);
    }
    cloud.width = cloud.size();
    cloud.height = 1;
    cloud.is_dense = true;
}

// Convert PCL point cloud to dimos-lcm PointCloud2
inline sensor_msgs::PointCloud2 from_pcl(
    const pcl::PointCloud<pcl::PointXYZI>& cloud,
    const std::string& frame_id,
    double timestamp
) {
    std::vector<PointXYZI> points;
    points.reserve(cloud.size());
    for (const auto& pt : cloud) {
        points.push_back({pt.x, pt.y, pt.z, pt.intensity});
    }
    return build_pointcloud2(points, frame_id, timestamp);
}
#endif

// Quaternion to RPY conversion
inline void quat_to_rpy(double qx, double qy, double qz, double qw,
                         double& roll, double& pitch, double& yaw) {
    // Roll (x-axis rotation)
    double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2.0 * (qw * qy - qz * qx);
    if (std::abs(sinp) >= 1.0)
        pitch = std::copysign(M_PI / 2, sinp);
    else
        pitch = std::asin(sinp);

    // Yaw (z-axis rotation)
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace smart_nav
