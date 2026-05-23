// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// FastlioPublisher — owns all LCM output for fastlio2_native:
//   * per-scan registered point cloud at `pointcloud_freq`
//   * odometry at `odom_freq`
//   * global voxel map at `map_freq` (optional)
// and applies the init-pose offset (slam-origin → user world frame) on the
// way out. The input source (LivoxDriverSource today, an LcmStreamSource
// later) is unaware of this transform.

#ifndef FASTLIO_PUBLISHER_HPP_
#define FASTLIO_PUBLISHER_HPP_

#include <lcm/lcm-cpp.hpp>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "dimos_native_module.hpp"
#include "fast_lio.hpp"
#include "pose_transform.hpp"
#include "voxel_map.hpp"

#include "nav_msgs/Odometry.hpp"
#include "sensor_msgs/PointCloud2.hpp"
#include "sensor_msgs/PointField.hpp"

class FastlioPublisher {
public:
    using Clock = std::chrono::steady_clock;

    // Everything the publisher needs from the CLI / outer config. Hz values
    // mirror the binary's flag names; periods are derived in the ctor.
    struct Config {
        std::string lidar_topic;
        std::string odometry_topic;
        std::string map_topic;
        std::string frame_id;
        std::string child_frame_id;
        InitPose init_pose;
        float pointcloud_freq = 5.0f;
        float odom_freq       = 50.0f;
        float map_freq        = 0.0f;
        float map_voxel_size  = 0.1f;
        float map_max_range   = 100.0f;

        bool enable_map() const {
            return !map_topic.empty() && map_freq > 0.0f;
        }

        static Config from_args(const dimos::NativeModule& mod) {
            Config c;
            c.lidar_topic     = mod.has("lidar")      ? mod.topic("lidar")      : "";
            c.odometry_topic  = mod.has("odometry")   ? mod.topic("odometry")   : "";
            c.map_topic       = mod.has("global_map") ? mod.topic("global_map") : "";
            c.frame_id        = mod.arg_required("frame_id");
            c.child_frame_id  = mod.arg_required("child_frame_id");
            c.init_pose.parse(mod.arg("init_pose", ""));
            c.pointcloud_freq = mod.arg_float("pointcloud_freq", c.pointcloud_freq);
            c.odom_freq       = mod.arg_float("odom_freq",       c.odom_freq);
            c.map_freq        = mod.arg_float("map_freq",        c.map_freq);
            c.map_voxel_size  = mod.arg_float("map_voxel_size",  c.map_voxel_size);
            c.map_max_range   = mod.arg_float("map_max_range",   c.map_max_range);
            if (c.lidar_topic.empty() && c.odometry_topic.empty()) {
                throw std::runtime_error(
                    "FastlioPublisher: at least one of --lidar or --odometry is required");
            }
            return c;
        }
    };

    FastlioPublisher(lcm::LCM& lcm, Config cfg)
        : lcm_(lcm),
          cfg_(std::move(cfg)),
          pc_period_(period_us(cfg_.pointcloud_freq)),
          odom_period_(period_us(cfg_.odom_freq)),
          map_period_(cfg_.enable_map() ? period_us(cfg_.map_freq)
                                        : std::chrono::microseconds(0)) {
        const auto now = Clock::now();
        last_pc_publish_   = now;
        last_odom_publish_ = now;
        last_map_publish_  = now;
        if (cfg_.enable_map()) {
            global_map_ = std::make_unique<VoxelMap>(cfg_.map_voxel_size, cfg_.map_max_range);
        }
    }

    FastlioPublisher(const FastlioPublisher&) = delete;
    FastlioPublisher& operator=(const FastlioPublisher&) = delete;

    void publish_lidar_if_due(Clock::time_point now,
                              const PointCloudXYZI::Ptr& cloud,
                              double ts) {
        if (cfg_.lidar_topic.empty()) return;
        if (now - last_pc_publish_ < pc_period_) return;
        if (!cloud || cloud->empty()) return;
        publish_cloud(cfg_.lidar_topic, cloud, ts);
        last_pc_publish_ = now;
    }

    // Inserts into the optional global voxel map every call (so the map is
    // built continuously). At `map_period` cadence, prunes around the
    // current sensor position and publishes the map cloud.
    void publish_map_if_due(Clock::time_point now,
                            const PointCloudXYZI::Ptr& cloud,
                            const std::vector<double>& pose_xyz,
                            double ts) {
        if (!global_map_) return;
        if (!cloud || cloud->empty()) return;

        global_map_->insert<PointType>(cloud);

        if (now - last_map_publish_ < map_period_) return;
        if (pose_xyz.size() >= 3) {
            global_map_->prune(static_cast<float>(pose_xyz[0]),
                               static_cast<float>(pose_xyz[1]),
                               static_cast<float>(pose_xyz[2]));
        }
        auto map_cloud = global_map_->to_cloud<PointType>();
        publish_cloud(cfg_.map_topic, map_cloud, ts);
        last_map_publish_ = now;
    }

    void publish_odom_if_due(Clock::time_point now,
                             const custom_messages::Odometry& odom,
                             double ts) {
        if (cfg_.odometry_topic.empty()) return;
        if (now - last_odom_publish_ < odom_period_) return;

        nav_msgs::Odometry msg;
        msg.header = dimos::make_header(cfg_.frame_id, ts);
        msg.child_frame_id = cfg_.child_frame_id;

        if (cfg_.init_pose.has_offset()) {
            double rx, ry, rz;
            quat_rotate(cfg_.init_pose.qx, cfg_.init_pose.qy, cfg_.init_pose.qz, cfg_.init_pose.qw,
                        odom.pose.pose.position.x,
                        odom.pose.pose.position.y,
                        odom.pose.pose.position.z,
                        rx, ry, rz);
            msg.pose.pose.position.x = rx + cfg_.init_pose.x;
            msg.pose.pose.position.y = ry + cfg_.init_pose.y;
            msg.pose.pose.position.z = rz + cfg_.init_pose.z;

            double ox, oy, oz, ow;
            quat_mul(cfg_.init_pose.qx, cfg_.init_pose.qy, cfg_.init_pose.qz, cfg_.init_pose.qw,
                     odom.pose.pose.orientation.x,
                     odom.pose.pose.orientation.y,
                     odom.pose.pose.orientation.z,
                     odom.pose.pose.orientation.w,
                     ox, oy, oz, ow);
            msg.pose.pose.orientation.x = ox;
            msg.pose.pose.orientation.y = oy;
            msg.pose.pose.orientation.z = oz;
            msg.pose.pose.orientation.w = ow;
        } else {
            msg.pose.pose.position.x = odom.pose.pose.position.x;
            msg.pose.pose.position.y = odom.pose.pose.position.y;
            msg.pose.pose.position.z = odom.pose.pose.position.z;
            msg.pose.pose.orientation.x = odom.pose.pose.orientation.x;
            msg.pose.pose.orientation.y = odom.pose.pose.orientation.y;
            msg.pose.pose.orientation.z = odom.pose.pose.orientation.z;
            msg.pose.pose.orientation.w = odom.pose.pose.orientation.w;
        }

        for (int i = 0; i < 36; ++i) {
            msg.pose.covariance[i] = odom.pose.covariance[i];
        }

        msg.twist.twist.linear.x  = 0;
        msg.twist.twist.linear.y  = 0;
        msg.twist.twist.linear.z  = 0;
        msg.twist.twist.angular.x = 0;
        msg.twist.twist.angular.y = 0;
        msg.twist.twist.angular.z = 0;
        std::memset(msg.twist.covariance, 0, sizeof(msg.twist.covariance));

        lcm_.publish(cfg_.odometry_topic, &msg);
        last_odom_publish_ = now;
    }

private:
    void publish_cloud(const std::string& topic,
                       const PointCloudXYZI::Ptr& cloud,
                       double ts) {
        if (topic.empty() || !cloud || cloud->empty()) return;

        const int num_points = static_cast<int>(cloud->size());

        sensor_msgs::PointCloud2 pc;
        pc.header = dimos::make_header(cfg_.frame_id, ts);
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

        const bool apply = cfg_.init_pose.has_offset();
        for (int i = 0; i < num_points; ++i) {
            float* dst = reinterpret_cast<float*>(pc.data.data() + i * 16);
            if (apply) {
                double rx, ry, rz;
                quat_rotate(cfg_.init_pose.qx, cfg_.init_pose.qy, cfg_.init_pose.qz, cfg_.init_pose.qw,
                            cloud->points[i].x, cloud->points[i].y, cloud->points[i].z,
                            rx, ry, rz);
                dst[0] = static_cast<float>(rx + cfg_.init_pose.x);
                dst[1] = static_cast<float>(ry + cfg_.init_pose.y);
                dst[2] = static_cast<float>(rz + cfg_.init_pose.z);
            } else {
                dst[0] = cloud->points[i].x;
                dst[1] = cloud->points[i].y;
                dst[2] = cloud->points[i].z;
            }
            dst[3] = cloud->points[i].intensity;
        }

        lcm_.publish(topic, &pc);
    }

    static std::chrono::microseconds period_us(float hz) {
        return std::chrono::microseconds(static_cast<int64_t>(1e6 / hz));
    }

    lcm::LCM& lcm_;
    Config cfg_;

    std::chrono::microseconds pc_period_;
    std::chrono::microseconds odom_period_;
    std::chrono::microseconds map_period_;

    std::unique_ptr<VoxelMap> global_map_;

    Clock::time_point last_pc_publish_;
    Clock::time_point last_odom_publish_;
    Clock::time_point last_map_publish_;
};

#endif  // FASTLIO_PUBLISHER_HPP_
