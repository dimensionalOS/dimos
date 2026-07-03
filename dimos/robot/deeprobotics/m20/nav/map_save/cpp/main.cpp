// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Native DimOS point-cloud map accumulator for M20.
//
// Subscribes to PointCloud2 + Odometry over LCM. When the odometry pose moves
// beyond the configured translation or rotation threshold from the last accepted
// keyframe, the latest cloud is transformed by that odometry pose and accumulated
// into a global PointCloud2 map.

#include <lcm/lcm-cpp.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "dimos_native_module.hpp"

#include "nav_msgs/Odometry.hpp"
#include "sensor_msgs/PointCloud2.hpp"
#include "sensor_msgs/PointField.hpp"

namespace {

struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct Quat {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 1.0;
};

struct Pose {
    Vec3 p;
    Quat q;
    double ts = 0.0;
    std::string frame_id = "world";
    bool valid = false;
};

using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;

static std::atomic<bool> g_running{true};
static lcm::LCM* g_lcm = nullptr;

static std::string g_lidar_topic;
static std::string g_odometry_topic;
static std::string g_global_map_topic;
static std::string g_world_frame_id = "world";
static std::string g_save_path;

static float g_translation_threshold_m = 0.5f;
static float g_rotation_threshold_rad = static_cast<float>(M_PI / 12.0);
static float g_voxel_size = 0.0f;

static std::mutex g_odom_mutex;
static Pose g_latest_odom;

static std::mutex g_map_mutex;
static Pose g_last_keyframe;
static CloudT::Ptr g_accumulated(new CloudT);

using dimos::make_header;

static void signal_handler(int /*sig*/) {
    g_running.store(false);
}

static double stamp_to_seconds(const std_msgs::Time& stamp) {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nsec) * 1e-9;
}

static float read_f32_le(const std::vector<uint8_t>& data, size_t off) {
    float out = 0.0f;
    std::memcpy(&out, data.data() + off, sizeof(float));
    return out;
}

static bool find_float32_field(
    const sensor_msgs::PointCloud2& msg, const std::string& name, int32_t* offset) {
    for (int32_t i = 0; i < msg.fields_length; ++i) {
        const auto& f = msg.fields[i];
        if (f.name == name && f.datatype == sensor_msgs::PointField::FLOAT32) {
            *offset = f.offset;
            return true;
        }
    }
    return false;
}

static CloudT::Ptr extract_points(const sensor_msgs::PointCloud2& msg) {
    CloudT::Ptr out(new CloudT);
    if (msg.is_bigendian || msg.point_step <= 0) {
        return out;
    }

    int32_t x_off = 0;
    int32_t y_off = 0;
    int32_t z_off = 0;
    int32_t intensity_off = 0;
    if (!find_float32_field(msg, "x", &x_off) ||
        !find_float32_field(msg, "y", &y_off) ||
        !find_float32_field(msg, "z", &z_off)) {
        return out;
    }
    const bool has_intensity = find_float32_field(msg, "intensity", &intensity_off);

    const size_t n = static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height);
    const size_t step = static_cast<size_t>(msg.point_step);
    const size_t required = n * step;
    if (msg.data.size() < required) {
        return out;
    }
    if (x_off < 0 || y_off < 0 || z_off < 0 ||
        static_cast<size_t>(std::max({x_off, y_off, z_off}) + 4) > step) {
        return out;
    }
    if (has_intensity && (intensity_off < 0 || static_cast<size_t>(intensity_off + 4) > step)) {
        return out;
    }

    out->reserve(n);
    for (size_t i = 0; i < n; ++i) {
        const size_t base = i * step;
        PointT p;
        p.x = read_f32_le(msg.data, base + static_cast<size_t>(x_off));
        p.y = read_f32_le(msg.data, base + static_cast<size_t>(y_off));
        p.z = read_f32_le(msg.data, base + static_cast<size_t>(z_off));
        p.intensity = has_intensity
            ? read_f32_le(msg.data, base + static_cast<size_t>(intensity_off))
            : 0.0f;
        if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
            out->push_back(p);
        }
    }
    out->width = static_cast<uint32_t>(out->size());
    out->height = 1;
    out->is_dense = true;
    return out;
}

static Quat normalized(Quat q) {
    const double n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (n <= std::numeric_limits<double>::epsilon()) {
        return {};
    }
    q.x /= n;
    q.y /= n;
    q.z /= n;
    q.w /= n;
    return q;
}

static Eigen::Affine3f pose_to_affine(const Pose& pose) {
    const Quat qn = normalized(pose.q);
    Eigen::Quaternionf q(
        static_cast<float>(qn.w),
        static_cast<float>(qn.x),
        static_cast<float>(qn.y),
        static_cast<float>(qn.z));
    Eigen::Affine3f tf = Eigen::Affine3f::Identity();
    tf.translation() = Eigen::Vector3f(pose.p.x, pose.p.y, pose.p.z);
    tf.linear() = q.normalized().toRotationMatrix();
    return tf;
}

static float translation_delta(const Pose& a, const Pose& b) {
    const float dx = a.p.x - b.p.x;
    const float dy = a.p.y - b.p.y;
    const float dz = a.p.z - b.p.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

static float rotation_delta(const Pose& a, const Pose& b) {
    const Quat qa = normalized(a.q);
    const Quat qb = normalized(b.q);
    double dot = std::abs(qa.x * qb.x + qa.y * qb.y + qa.z * qb.z + qa.w * qb.w);
    dot = std::max(-1.0, std::min(1.0, dot));
    return static_cast<float>(2.0 * std::acos(dot));
}

static bool passes_threshold(const Pose& pose) {
    if (!g_last_keyframe.valid) {
        return true;
    }
    return translation_delta(pose, g_last_keyframe) >= g_translation_threshold_m ||
           rotation_delta(pose, g_last_keyframe) >= g_rotation_threshold_rad;
}

static void voxel_filter_in_place(CloudT::Ptr cloud) {
    if (g_voxel_size <= 0.0f || cloud->empty()) {
        return;
    }
    pcl::VoxelGrid<PointT> filter;
    filter.setInputCloud(cloud);
    filter.setLeafSize(g_voxel_size, g_voxel_size, g_voxel_size);
    CloudT filtered;
    filter.filter(filtered);
    *cloud = filtered;
}

static sensor_msgs::PointCloud2 make_cloud(const CloudT& points, double ts) {
    sensor_msgs::PointCloud2 pc;
    pc.header = make_header(g_world_frame_id, ts);
    pc.height = 1;
    pc.width = static_cast<int32_t>(points.size());
    pc.is_bigendian = 0;
    pc.is_dense = 1;

    auto make_field = [](const std::string& name, int32_t offset) {
        sensor_msgs::PointField f;
        f.name = name;
        f.offset = offset;
        f.datatype = sensor_msgs::PointField::FLOAT32;
        f.count = 1;
        return f;
    };

    pc.fields_length = 4;
    pc.fields.resize(4);
    pc.fields[0] = make_field("x", 0);
    pc.fields[1] = make_field("y", 4);
    pc.fields[2] = make_field("z", 8);
    pc.fields[3] = make_field("intensity", 12);
    pc.point_step = 16;
    pc.row_step = pc.point_step * pc.width;
    pc.data_length = pc.row_step;
    pc.data.resize(static_cast<size_t>(pc.data_length));

    for (size_t i = 0; i < points.size(); ++i) {
        float values[4] = {points[i].x, points[i].y, points[i].z, points[i].intensity};
        std::memcpy(pc.data.data() + i * 16, values, sizeof(values));
    }
    return pc;
}

static void publish_global_map(double ts) {
    CloudT::Ptr snapshot(new CloudT);
    {
        std::lock_guard<std::mutex> lock(g_map_mutex);
        *snapshot = *g_accumulated;
    }
    if (snapshot->empty() || g_global_map_topic.empty() || !g_lcm) {
        return;
    }

    sensor_msgs::PointCloud2 pc = make_cloud(*snapshot, ts);
    g_lcm->publish(g_global_map_topic, &pc);
}

static void save_pcd() {
    CloudT::Ptr snapshot(new CloudT);
    {
        std::lock_guard<std::mutex> lock(g_map_mutex);
        *snapshot = *g_accumulated;
    }
    if (g_save_path.empty() || snapshot->empty()) {
        return;
    }

    if (pcl::io::savePCDFileBinary(g_save_path, *snapshot) != 0) {
        std::fprintf(stderr, "[map_save] failed to save PCD: %s\n", g_save_path.c_str());
        return;
    }
    std::fprintf(stderr, "[map_save] saved %zu points to %s\n", snapshot->size(), g_save_path.c_str());
}

class Accumulator {
public:
    void on_odometry(
        const lcm::ReceiveBuffer* /*rbuf*/, const std::string& /*chan*/,
        const nav_msgs::Odometry* msg) {
        Pose pose;
        pose.p.x = static_cast<float>(msg->pose.pose.position.x);
        pose.p.y = static_cast<float>(msg->pose.pose.position.y);
        pose.p.z = static_cast<float>(msg->pose.pose.position.z);
        pose.q.x = msg->pose.pose.orientation.x;
        pose.q.y = msg->pose.pose.orientation.y;
        pose.q.z = msg->pose.pose.orientation.z;
        pose.q.w = msg->pose.pose.orientation.w;
        pose.ts = stamp_to_seconds(msg->header.stamp);
        pose.frame_id = msg->header.frame_id.empty() ? g_world_frame_id : msg->header.frame_id;
        pose.valid = true;
        std::cout<<"recevie lcm odom"<<std::endl;
        std::lock_guard<std::mutex> lock(g_odom_mutex);
        g_latest_odom = pose;
    }

    void on_lidar(
        const lcm::ReceiveBuffer* /*rbuf*/, const std::string& /*chan*/,
        const sensor_msgs::PointCloud2* msg) {
        Pose pose;
        {
            std::lock_guard<std::mutex> lock(g_odom_mutex);
            pose = g_latest_odom;
        }
        if (!pose.valid) {
            return;
        }

        CloudT::Ptr points = extract_points(*msg);
        if (points->empty()) {
            return;
        }

        {
            std::lock_guard<std::mutex> lock(g_map_mutex);
            if (!passes_threshold(pose)) {
                return;
            }
            CloudT transformed;
            pcl::transformPointCloud(*points, transformed, pose_to_affine(pose));
            *g_accumulated += transformed;
            voxel_filter_in_place(g_accumulated);
            g_last_keyframe = pose;
        }

        publish_global_map(pose.ts);
    }
};

}  // namespace

int main(int argc, char** argv) {
    dimos::NativeModule mod(argc, argv);

    g_lidar_topic = mod.has("lidar") ? mod.topic("lidar") : "";
    g_odometry_topic = mod.has("odometry") ? mod.topic("odometry") : "";
    g_global_map_topic = mod.has("global_map") ? mod.topic("global_map") : "";
    if (g_lidar_topic.empty() || g_odometry_topic.empty() || g_global_map_topic.empty()) {
        std::fprintf(stderr, "Usage: %s --lidar <topic> --odometry <topic> --global_map <topic>\n", argv[0]);
        return 1;
    }

    g_translation_threshold_m = mod.arg_float("translation_threshold_m", g_translation_threshold_m);
    g_rotation_threshold_rad = mod.arg_float("rotation_threshold_rad", g_rotation_threshold_rad);
    g_voxel_size = mod.arg_float("voxel_size", g_voxel_size);
    g_world_frame_id = mod.arg("world_frame_id", g_world_frame_id);
    g_save_path = mod.arg("save_path", "");

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    lcm::LCM lcm;
    if (!lcm.good()) {
        std::fprintf(stderr, "[map_save] failed to initialize LCM\n");
        return 1;
    }
    g_lcm = &lcm;

    Accumulator accumulator;
    lcm.subscribe(g_odometry_topic, &Accumulator::on_odometry, &accumulator);
    lcm.subscribe(g_lidar_topic, &Accumulator::on_lidar, &accumulator);

    std::fprintf(
        stderr,
        "[map_save] lidar=%s odometry=%s global_map=%s translation_threshold=%.3f "
        "rotation_threshold=%.3f voxel_size=%.3f\n",
        g_lidar_topic.c_str(),
        g_odometry_topic.c_str(),
        g_global_map_topic.c_str(),
        g_translation_threshold_m,
        g_rotation_threshold_rad,
        g_voxel_size);

    while (g_running.load()) {
        lcm.handleTimeout(100);
    }

    publish_global_map(g_last_keyframe.valid ? g_last_keyframe.ts : 0.0);
    save_pcd();
    return 0;
}
