// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// arise_slam native module for dimos NativeModule framework.
//
// LiDAR-Inertial SLAM combining feature extraction, laser mapping (ICP/Ceres),
// and IMU preintegration (GTSAM iSAM2) into a single LCM-based process.
//
// Upstream: github.com/jizhang-cmu/autonomy_stack_mecanum_wheel_platform
//           src/slam/arise_slam_mid360/
// Local upstream copy: ~/repos/arise_slam/arise_slam_mid360/
//
// Upstream has 3 separate ROS2 LifecycleNode processes:
//   1. featureExtraction_node.cpp → featureExtraction.cpp (on_configure at line 28)
//   2. laserMapping_node.cpp     → laserMapping.cpp      (on_configure at line 19)
//   3. imuPreintegration_node.cpp→ imuPreintegration.cpp (on_configure at line 16)
// This file combines all 3 into a single LCM-based process.
//
// Usage: ./arise_slam --lidar <topic> --imu <topic> --odometry <topic>
//        --registered_scan <topic> [config args...]

#include "arise_slam_mid360/FeatureExtraction/featureExtraction.h"
#include "arise_slam_mid360/LaserMapping/laserMapping.h"
#include "arise_slam_mid360/ImuPreintegration/imuPreintegration.h"
#include "arise_slam_mid360/config/parameter.h"
#include "arise_slam_mid360/sensor_data/pointcloud/point_os.h"
#include "arise_slam_mid360/common.h"

#include <lcm/lcm-cpp.hpp>
#include "dimos_native_module.hpp"
#include "nav_msgs/Odometry.hpp"
#include "sensor_msgs/Imu.hpp"
#include "sensor_msgs/PointCloud2.hpp"
#include "sensor_msgs/PointField.hpp"
#include "geometry_msgs/Quaternion.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <mutex>
#include <string>
#include <vector>

using namespace std;
using dimos::time_from_seconds;
using dimos::make_header;

// ============================================================================
// Global state
// ============================================================================
static atomic<bool> g_running{true};
static lcm::LCM* g_lcm = nullptr;

// LCM topics
static string g_lidar_topic, g_imu_topic;
static string g_odom_topic, g_scan_topic;

// Diagnostic LCM topics — upstream: laserMapping.cpp publishTopic() lines 927-1111
static string g_surround_map_topic;
static string g_global_map_topic;
static string g_laser_odom_topic;
static string g_incremental_odom_topic;
static string g_stats_topic;

// Diagnostic publish flags (default off to save CPU/bandwidth)
static bool g_publish_surround_map = false;
static bool g_publish_global_map = false;
static bool g_publish_laser_odometry = false;
static bool g_publish_incremental_odometry = false;
static bool g_publish_slam_stats = false;

// Algorithm components — upstream each runs in its own ROS2 node process:
//   featureExtraction: upstream featureExtraction_node.cpp:13 (MultiThreadedExecutor)
//   laserMapping:      upstream laserMapping_node.cpp:13
//   imuPreintegration: upstream imuPreintegration_node.cpp:13
static arise_slam::featureExtraction g_feature_extraction;
static arise_slam::laserMapping g_laser_mapping;
static arise_slam::imuPreintegration g_imu_preintegration;

// Processing state
static mutex g_process_mtx;
static bool g_has_imu_preintegration = true;  // can be disabled via config

// Frame counter for logging
static int g_frame_count = 0;

// ============================================================================
// Signal handling
// ============================================================================
static void signal_handler(int /*sig*/) {
    g_running.store(false);
}

// ============================================================================
// PointCloud2 → PCL conversion
// Upstream: featureExtraction.cpp subscribes to sensor_msgs::PointCloud2 (line 76)
// or livox_ros_driver2::msg::CustomMsg (line 80) and converts via laserCloudHandler
// or livoxHandler callbacks. We receive LCM PointCloud2 and convert to PCL directly.
// ============================================================================
static pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr
convertPointCloud2ToPCL(const sensor_msgs::PointCloud2* msg, double timestamp) {
    auto cloud = pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr(
        new pcl::PointCloud<point_os::PointcloudXYZITR>());

    int num_points = msg->width * msg->height;
    if (num_points == 0) {
        return cloud;
    }

    // Parse field offsets
    int x_off = -1, y_off = -1, z_off = -1, int_off = -1;
    int time_off = -1, ring_off = -1;
    uint8_t time_datatype = 0;
    for (const auto& f : msg->fields) {
        if (f.name == "x") { x_off = f.offset; }
        else if (f.name == "y") { y_off = f.offset; }
        else if (f.name == "z") { z_off = f.offset; }
        else if (f.name == "intensity") { int_off = f.offset; }
        else if (f.name == "time" || f.name == "offset_time" || f.name == "t") {
            time_off = f.offset;
            time_datatype = f.datatype;
        }
        else if (f.name == "ring") { ring_off = f.offset; }
    }

    if (x_off < 0 || y_off < 0 || z_off < 0) {
        fprintf(stderr, "[arise_slam] PointCloud2 missing x/y/z fields\n");
        return cloud;
    }

    cloud->resize(num_points);
    for (int i = 0; i < num_points; ++i) {
        const uint8_t* ptr = msg->data.data() + i * msg->point_step;
        auto& pt = cloud->points[i];

        pt.x = *reinterpret_cast<const float*>(ptr + x_off);
        pt.y = *reinterpret_cast<const float*>(ptr + y_off);
        pt.z = *reinterpret_cast<const float*>(ptr + z_off);
        pt.intensity = int_off >= 0 ? *reinterpret_cast<const float*>(ptr + int_off) : 0.0f;

        // Per-point timestamp
        if (time_off >= 0) {
            if (time_datatype == sensor_msgs::PointField::UINT32) {
                // offset_time / t: nanoseconds as uint32
                uint32_t t_ns = *reinterpret_cast<const uint32_t*>(ptr + time_off);
                pt.time = static_cast<float>(t_ns) * 1e-9f;  // ns → seconds
            } else {
                // time: float seconds (or ms depending on sensor)
                pt.time = *reinterpret_cast<const float*>(ptr + time_off);
            }
        } else {
            pt.time = 0.0f;
        }

        // Ring/scan line
        if (ring_off >= 0) {
            pt.ring = *reinterpret_cast<const uint16_t*>(ptr + ring_off);
        } else {
            pt.ring = 0;
        }
    }

    cloud->width = num_points;
    cloud->height = 1;
    cloud->is_dense = false;

    return cloud;
}

// ============================================================================
// LCM publish: odometry
// Upstream: imuPreintegration.cpp publishes to ProjectName+"/state_estimation"
// (line 63-64) via pubImuOdometry2. Also publishes health status (line 65-66)
// and IMU path (line 67-68). We combine into a single odometry message.
// ============================================================================
static void publish_odometry(const arise_slam::ImuState& state) {
    if (!g_lcm || g_odom_topic.empty()) {
        return;
    }

    nav_msgs::Odometry odom;
    odom.header = make_header("map", state.timestamp);
    odom.child_frame_id = "body";

    odom.pose.pose.position.x = state.position.x();
    odom.pose.pose.position.y = state.position.y();
    odom.pose.pose.position.z = state.position.z();
    odom.pose.pose.orientation.x = state.orientation.x();
    odom.pose.pose.orientation.y = state.orientation.y();
    odom.pose.pose.orientation.z = state.orientation.z();
    odom.pose.pose.orientation.w = state.orientation.w();

    odom.twist.twist.linear.x = state.velocity.x();
    odom.twist.twist.linear.y = state.velocity.y();
    odom.twist.twist.linear.z = state.velocity.z();
    odom.twist.twist.angular.x = state.angular_velocity.x();
    odom.twist.twist.angular.y = state.angular_velocity.y();
    odom.twist.twist.angular.z = state.angular_velocity.z();

    // Store health/result info in covariance
    odom.pose.covariance[0] = state.health_status ? 1.0 : 0.0;
    odom.pose.covariance[1] = static_cast<double>(state.result);
    odom.pose.covariance[2] = state.gravity;

    g_lcm->publish(g_odom_topic, &odom);
}

// Fallback: publish odometry from laser mapping output (no IMU preintegration)
// Upstream: laserMapping.cpp publishes to ProjectName+"/laser_odometry" (line 89-90)
// via pubOdomAftMapped. This fallback is used when IMU preintegration is disabled.
static void publish_odometry_from_mapping(const arise_slam::LaserMappingOutput& output) {
    if (!g_lcm || g_odom_topic.empty()) {
        return;
    }

    nav_msgs::Odometry odom;
    odom.header = make_header("map", output.timestamp);
    odom.child_frame_id = "body";

    odom.pose.pose.position.x = output.position.x();
    odom.pose.pose.position.y = output.position.y();
    odom.pose.pose.position.z = output.position.z();
    odom.pose.pose.orientation.x = output.orientation.x();
    odom.pose.pose.orientation.y = output.orientation.y();
    odom.pose.pose.orientation.z = output.orientation.z();
    odom.pose.pose.orientation.w = output.orientation.w();

    odom.twist.twist.linear.x = output.velocity.x();
    odom.twist.twist.linear.y = output.velocity.y();
    odom.twist.twist.linear.z = output.velocity.z();

    // Flag degeneracy in covariance
    odom.pose.covariance[0] = output.is_degenerate ? 0.0 : 1.0;

    g_lcm->publish(g_odom_topic, &odom);
}

// ============================================================================
// LCM publish: point cloud (generic helper for registered_scan/surround/global map)
// ============================================================================
static void publish_pointcloud(const string& topic,
                                const pcl::PointCloud<PointType>::Ptr& cloud,
                                double timestamp) {
    if (!g_lcm || topic.empty() || !cloud || cloud->empty()) {
        return;
    }

    int num_points = cloud->size();

    sensor_msgs::PointCloud2 pc;
    pc.header = make_header("map", timestamp);
    pc.height = 1;
    pc.width = num_points;
    pc.is_bigendian = 0;
    pc.is_dense = 1;

    pc.fields_length = 4;
    pc.fields.resize(4);
    auto make_field = [](const string& name, int32_t offset) {
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

    for (int i = 0; i < num_points; ++i) {
        float* dst = reinterpret_cast<float*>(pc.data.data() + i * 16);
        dst[0] = cloud->points[i].x;
        dst[1] = cloud->points[i].y;
        dst[2] = cloud->points[i].z;
        dst[3] = cloud->points[i].intensity;
    }

    g_lcm->publish(topic, &pc);
}

// publish_registered_scan: use generic publish_pointcloud helper
// Upstream: laserMapping.cpp publishes to ProjectName+"/registered_scan" (line 78-79)
static inline void publish_registered_scan(const pcl::PointCloud<PointType>::Ptr& cloud,
                                            double timestamp) {
    publish_pointcloud(g_scan_topic, cloud, timestamp);
}

// ============================================================================
// LCM publish: laser odometry (mapped pose, upstream line 1020-1086)
// ============================================================================
static void publish_laser_odometry(const arise_slam::LaserMappingOutput& output) {
    if (!g_lcm || g_laser_odom_topic.empty()) {
        return;
    }

    nav_msgs::Odometry odom;
    odom.header = make_header("map", output.timestamp);
    odom.child_frame_id = "body";

    odom.pose.pose.position.x = output.position.x();
    odom.pose.pose.position.y = output.position.y();
    odom.pose.pose.position.z = output.position.z();
    odom.pose.pose.orientation.x = output.orientation.x();
    odom.pose.pose.orientation.y = output.orientation.y();
    odom.pose.pose.orientation.z = output.orientation.z();
    odom.pose.pose.orientation.w = output.orientation.w();

    // Degeneracy flag in covariance[0] — upstream line 1078-1082
    // Convention: 1.0 = healthy, 0.0 = degenerate (consistent with publish_odometry_from_mapping)
    odom.pose.covariance[0] = output.is_degenerate ? 0.0 : 1.0;

    g_lcm->publish(g_laser_odom_topic, &odom);
}

// ============================================================================
// LCM publish: incremental odometry (upstream line 1042-1075)
// ============================================================================
static void publish_incremental_odometry(const arise_slam::LaserMappingOutput& output) {
    if (!g_lcm || g_incremental_odom_topic.empty()) {
        return;
    }

    nav_msgs::Odometry odom;
    odom.header = make_header("map", output.timestamp);
    odom.child_frame_id = "body";

    odom.pose.pose.position.x = output.incremental_position.x();
    odom.pose.pose.position.y = output.incremental_position.y();
    odom.pose.pose.position.z = output.incremental_position.z();
    odom.pose.pose.orientation.x = output.incremental_orientation.x();
    odom.pose.pose.orientation.y = output.incremental_orientation.y();
    odom.pose.pose.orientation.z = output.incremental_orientation.z();
    odom.pose.pose.orientation.w = output.incremental_orientation.w();

    g_lcm->publish(g_incremental_odom_topic, &odom);
}

// ============================================================================
// LCM publish: SLAM optimization stats (upstream line 1097-1111)
// Packed into an Odometry message's covariance array since we don't have the
// custom arise_slam_mid360_msgs::OptimizationStats message type.
//
// Covariance layout:
//   [0]  edge_features_from_map     [1]  plane_features_from_map
//   [2]  edge_features_from_scan    [3]  plane_features_from_scan
//   [4]  translation_norm           [5]  rotation_norm
//   [6]  uncertainty_x              [7]  uncertainty_y
//   [8]  uncertainty_z              [9]  uncertainty_roll
//   [10] uncertainty_pitch          [11] uncertainty_yaw
//   [12] average_distance           [13] is_degenerate (1.0=degenerate, 0.0=healthy)
//   [14] prediction_source
// ============================================================================
static void publish_slam_stats(const arise_slam::LaserMappingOutput& output) {
    if (!g_lcm || g_stats_topic.empty()) {
        return;
    }

    nav_msgs::Odometry msg;
    msg.header = make_header("map", output.timestamp);
    msg.child_frame_id = "slam_stats";

    const auto& s = output.stats;
    msg.pose.covariance[0]  = static_cast<double>(s.edge_features_from_map);
    msg.pose.covariance[1]  = static_cast<double>(s.plane_features_from_map);
    msg.pose.covariance[2]  = static_cast<double>(s.edge_features_from_scan);
    msg.pose.covariance[3]  = static_cast<double>(s.plane_features_from_scan);
    msg.pose.covariance[4]  = s.translation_norm;
    msg.pose.covariance[5]  = s.rotation_norm;
    msg.pose.covariance[6]  = s.uncertainty_x;
    msg.pose.covariance[7]  = s.uncertainty_y;
    msg.pose.covariance[8]  = s.uncertainty_z;
    msg.pose.covariance[9]  = s.uncertainty_roll;
    msg.pose.covariance[10] = s.uncertainty_pitch;
    msg.pose.covariance[11] = s.uncertainty_yaw;
    msg.pose.covariance[12] = s.average_distance;
    msg.pose.covariance[13] = output.is_degenerate ? 1.0 : 0.0;
    msg.pose.covariance[14] = static_cast<double>(output.prediction_source);

    g_lcm->publish(g_stats_topic, &msg);
}

// ============================================================================
// Processing pipeline (triggered by lidar callback)
//
// Upstream data flow across 3 ROS2 nodes:
//   featureExtraction publishes LaserFeature msg (line 108) →
//   laserMapping subscribes via laserFeatureInfoHandler (line 54-57) →
//   laserMapping publishes laser_odometry (line 89-90) →
//   imuPreintegration subscribes via laserodometryHandler (line 52-55) →
//   imuPreintegration publishes state_estimation (line 63-64)
//
// Here we call the same functions directly instead of going through ROS topics.
// ============================================================================
static void run_pipeline() {
    // Step 1: Run feature extraction undistortion + scan registration
    // Upstream: featureExtraction.cpp::undistortionAndscanregistration() is called
    // from laserCloudHandler (Velodyne/Ouster) or livoxHandler (Livox) callbacks
    g_feature_extraction.undistortionAndscanregistration();

    // Step 2: Get latest feature extraction result
    // Upstream: published as arise_slam_mid360_msgs::LaserFeature (line 108)
    // containing edge points, planner points, depth points, and orientation
    arise_slam::FeatureExtractionResult fe_result = g_feature_extraction.getLatestResult();
    if (!fe_result.valid) {
        return;
    }

    g_frame_count++;

    // Step 3: Feed features to laser mapping
    // Upstream: laserMapping subscribes to LaserFeature (line 54-57)
    // and calls laserFeatureInfoHandler which unpacks the message:
    //   cornerLastBuf ← cloud_corner (line 536)
    //   surfLastBuf   ← cloud_surface (line 537)
    //   realsenseBuf  ← cloud_realsense (line 538)
    //   fullResBuf    ← cloud_nodistortion (line 539) — used for registered_scan
    //   IMUPredictionBuf ← initial_quaternion (line 540-543)

    // Convert undistorted cloud (PointcloudXYZITR) to PointType (PointXYZI)
    // for the full-resolution cloud that laserMapping uses for registered_scan.
    // Upstream: cloud_nodistortion (LaserFeature.msg:28) → fullResBuf → laserCloudFullRes
    pcl::PointCloud<PointType>::Ptr fullCloud(new pcl::PointCloud<PointType>());
    if (fe_result.undistortedCloud && !fe_result.undistortedCloud->empty()) {
        fullCloud->resize(fe_result.undistortedCloud->size());
        for (size_t i = 0; i < fe_result.undistortedCloud->size(); ++i) {
            auto& src = fe_result.undistortedCloud->points[i];
            auto& dst = fullCloud->points[i];
            dst.x = src.x;
            dst.y = src.y;
            dst.z = src.z;
            dst.intensity = src.intensity;
        }
    }

    pcl::PointCloud<PointType>::Ptr realsense_cloud(new pcl::PointCloud<PointType>());
    g_laser_mapping.addLaserFeature(
        fe_result.edgePoints,
        fe_result.plannerPoints,
        fullCloud,              // upstream: cloud_nodistortion → fullResBuf
        realsense_cloud,
        fe_result.q_w_original_l,
        fe_result.timestamp
    );

    // Step 4: Run laser mapping optimization
    // Upstream: laserMapping::process() runs Ceres ICP (same function)
    g_laser_mapping.process();

    // Step 5: Get mapping output
    // Upstream: laserMapping publishes multiple topics after process():
    //   registered_scan (line 78), laser_odometry (line 89), stats (line 106)
    arise_slam::LaserMappingOutput mapping_output = g_laser_mapping.getLatestOutput();

    // Step 6: Feed laser odometry to IMU preintegration
    // Upstream: imuPreintegration subscribes to laser_odometry (line 52-55)
    // via laserodometryHandler callback
    if (g_has_imu_preintegration) {
        double degenerate_flag = mapping_output.is_degenerate ? 1.0 : 0.0;
        g_imu_preintegration.addLaserOdometry(
            mapping_output.timestamp,
            mapping_output.position,
            mapping_output.orientation,
            degenerate_flag
        );

        // Step 7: Get IMU-fused state and publish
        // Upstream: imuPreintegration publishes state_estimation (line 63-64)
        arise_slam::ImuState imu_state = g_imu_preintegration.getLatestState();
        publish_odometry(imu_state);
    } else {
        // Fallback: publish laser mapping odometry directly
        publish_odometry_from_mapping(mapping_output);
    }

    // Step 8: Publish registered scan
    // Upstream: laserMapping publishes registered_scan (line 78-79)
    if (mapping_output.registered_scan && !mapping_output.registered_scan->empty()) {
        publish_registered_scan(mapping_output.registered_scan, mapping_output.timestamp);
    }

    // Step 9: Publish diagnostic topics (each gated by config flag)
    // Upstream: laserMapping.cpp publishTopic() lines 927-1111

    // Surround map — upstream line 944-952 (every 5 frames)
    if (g_publish_surround_map && mapping_output.surround_map &&
        !mapping_output.surround_map->empty()) {
        publish_pointcloud(g_surround_map_topic, mapping_output.surround_map,
                           mapping_output.timestamp);
    }

    // Global map — upstream line 955-962 (every 20 frames)
    if (g_publish_global_map && mapping_output.global_map &&
        !mapping_output.global_map->empty()) {
        publish_pointcloud(g_global_map_topic, mapping_output.global_map,
                           mapping_output.timestamp);
    }

    // Laser odometry (mapped pose) — upstream line 1020-1086
    if (g_publish_laser_odometry) {
        publish_laser_odometry(mapping_output);
    }

    // Incremental odometry — upstream line 1042-1075
    if (g_publish_incremental_odometry) {
        publish_incremental_odometry(mapping_output);
    }

    // SLAM stats — upstream line 1097-1111
    if (g_publish_slam_stats) {
        publish_slam_stats(mapping_output);
    }

    if (g_frame_count <= 3 || g_frame_count % 50 == 0) {
        fprintf(stderr, "[arise_slam] frame #%d: "
                "fe: edge=%zu plane=%zu, "
                "map: pos=(%.2f,%.2f,%.2f) degen=%d, "
                "reg_scan=%zu pts\n",
                g_frame_count,
                fe_result.edgePoints ? fe_result.edgePoints->size() : 0,
                fe_result.plannerPoints ? fe_result.plannerPoints->size() : 0,
                mapping_output.position.x(),
                mapping_output.position.y(),
                mapping_output.position.z(),
                mapping_output.is_degenerate ? 1 : 0,
                (mapping_output.registered_scan ? mapping_output.registered_scan->size() : 0));
    }
}

// ============================================================================
// LCM callbacks
// ============================================================================
static int g_lidar_debug_count = 0;
static int g_lidar_warmup_count = 0;
// Upstream: featureExtraction.cpp:1490 skips frames until IMU buffer is populated
// and discards first 5 frames: `if (imuBuf.empty() || delay_count_++ <= 5) return;`
static constexpr int IMU_WARMUP_FRAMES = 5;

static void on_lidar(const lcm::ReceiveBuffer* /*rbuf*/, const string& /*channel*/,
                     const sensor_msgs::PointCloud2* msg) {
    if (!g_running.load()) {
        return;
    }

    double timestamp = msg->header.stamp.sec + msg->header.stamp.nsec * 1e-9;

    // Convert LCM PointCloud2 → PCL PointcloudXYZITR
    auto cloud = convertPointCloud2ToPCL(msg, timestamp);
    if (cloud->empty()) {
        fprintf(stderr, "[arise_slam] on_lidar: empty cloud after conversion\n");
        return;
    }

    g_lidar_debug_count++;
    if (g_lidar_debug_count <= 3 || g_lidar_debug_count % 50 == 0) {
        // Compute bounds
        float xmin = 1e9, xmax = -1e9, ymin = 1e9, ymax = -1e9, zmin = 1e9, zmax = -1e9;
        int has_time = 0, has_ring = 0;
        for (size_t i = 0; i < cloud->size(); ++i) {
            auto& p = cloud->points[i];
            if (p.x < xmin) xmin = p.x; if (p.x > xmax) xmax = p.x;
            if (p.y < ymin) ymin = p.y; if (p.y > ymax) ymax = p.y;
            if (p.z < zmin) zmin = p.z; if (p.z > zmax) zmax = p.z;
            if (p.time != 0.0f) has_time++;
            if (p.ring != 0) has_ring++;
        }
        fprintf(stderr, "[arise_slam] lidar #%d: %zu pts, "
                "x=[%.2f,%.2f] y=[%.2f,%.2f] z=[%.2f,%.2f], "
                "pts_with_time=%d, pts_with_ring=%d, "
                "fields=%d, point_step=%d\n",
                g_lidar_debug_count, cloud->size(),
                xmin, xmax, ymin, ymax, zmin, zmax,
                has_time, has_ring,
                msg->fields_length, msg->point_step);
    }

    // Upstream: featureExtraction.cpp:1490 skips until IMU data available
    // and discards first 5 frames for IMU warmup
    g_lidar_warmup_count++;
    if (g_lidar_warmup_count <= IMU_WARMUP_FRAMES) {
        if (g_lidar_warmup_count == IMU_WARMUP_FRAMES) {
            fprintf(stderr, "[arise_slam] IMU warmup complete (%d frames skipped)\n",
                    IMU_WARMUP_FRAMES);
        }
        return;
    }

    // Feed point cloud to feature extraction
    // Upstream: featureExtraction.cpp subscribes to PointCloud2 (line 76) or
    // CustomMsg (line 80). laserCloudHandler/livoxHandler push to lidarBuf (line 33)
    // then call undistortionAndscanregistration
    g_feature_extraction.addPointCloud(timestamp, cloud);

    // Run the full processing pipeline
    // Upstream: triggered by the lidar callback in featureExtraction,
    // then features flow through ROS topics to laserMapping and imuPreintegration
    lock_guard<mutex> lock(g_process_mtx);
    run_pipeline();
}

// Upstream: featureExtraction.cpp subscribes to IMU (line 85-88) via imu_Handler
// which pushes to imuBuf. imuPreintegration.cpp also subscribes (line 48-51) via
// imuHandler which pushes to its own imuBuf.
// Here we feed both components from a single LCM callback.
static void on_imu(const lcm::ReceiveBuffer* /*rbuf*/, const string& /*channel*/,
                   const sensor_msgs::Imu* msg) {
    if (!g_running.load()) {
        return;
    }

    double timestamp = msg->header.stamp.sec + msg->header.stamp.nsec * 1e-9;

    Eigen::Vector3d acc(
        msg->linear_acceleration.x,
        msg->linear_acceleration.y,
        msg->linear_acceleration.z
    );
    Eigen::Vector3d gyr(
        msg->angular_velocity.x,
        msg->angular_velocity.y,
        msg->angular_velocity.z
    );
    Eigen::Quaterniond orientation(
        msg->orientation.w,
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z
    );

    // Check if orientation data is valid (non-zero quaternion)
    bool has_orientation = (orientation.squaredNorm() > 0.5);

    // Feed IMU to feature extraction (for deskewing) and preintegration
    // Lock to prevent data race with on_lidar pipeline
    lock_guard<mutex> lock(g_process_mtx);
    g_feature_extraction.addImuData(timestamp, acc, gyr, orientation, has_orientation);

    // Feed IMU to preintegration (for factor graph)
    if (g_has_imu_preintegration) {
        g_imu_preintegration.addImuMeasurement(timestamp, acc, gyr, orientation);
    }
}

// Upstream: laserMapping.cpp:64-67 subscribes to visual odometry
// ("vins_estimator/imu_propagate") for degeneracy recovery. Any odometry
// source works — not VINS-specific. Used when LiDAR scan matching degenerates
// (featureless corridors, etc.) as a fallback initial guess for ICP.
static void on_fallback_odom(const lcm::ReceiveBuffer* /*rbuf*/, const string& /*channel*/,
                              const nav_msgs::Odometry* msg) {
    if (!g_running.load()) {
        return;
    }

    double timestamp = msg->header.stamp.sec + msg->header.stamp.nsec * 1e-9;

    Eigen::Vector3d position(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z
    );
    Eigen::Quaterniond orientation(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z
    );

    // Pack covariance for init/failure status checks
    // Upstream: laserMapping.cpp:856-862 checks covariance[0] (failure) and
    // covariance[1] (init status). Set healthy defaults.
    double covariance[36] = {};
    covariance[0] = msg->pose.covariance[0];  // failure status
    covariance[1] = msg->pose.covariance[1];  // init status

    lock_guard<mutex> lock(g_process_mtx);
    // Feed as visual odometry (for degeneracy recovery, upstream line 824-910)
    g_laser_mapping.addVisualOdometry(timestamp, position, orientation, covariance);
    // When IMU preintegration is disabled, also feed as IMU odometry so
    // extractVisualIMUOdometryAndCheck has data for the ICP initial guess.
    // When IMU preint is ON, it provides its own addIMUOdometry via setOutputCallback.
    if (!g_has_imu_preintegration) {
        g_laser_mapping.addIMUOdometry(timestamp, position, orientation);
    }
}

// ============================================================================
// Parse sensor type from string
// ============================================================================
static SensorType parseSensorType(const string& s) {
    if (s == "velodyne") { return SensorType::VELODYNE; }
    if (s == "ouster") { return SensorType::OUSTER; }
    if (s == "livox") { return SensorType::LIVOX; }
    // Default to livox
    fprintf(stderr, "[arise_slam] Unknown sensor type '%s', defaulting to livox\n", s.c_str());
    return SensorType::LIVOX;
}

// ============================================================================
// Main
// ============================================================================
int main(int argc, char** argv) {
    dimos::NativeModule mod(argc, argv);

    // ---- Required topics ----
    g_lidar_topic = mod.has("lidar") ? mod.topic("lidar") : "";
    g_imu_topic = mod.has("imu") ? mod.topic("imu") : "";
    g_odom_topic = mod.has("odometry") ? mod.topic("odometry") : "";
    g_scan_topic = mod.has("registered_scan") ? mod.topic("registered_scan") : "";
    // Optional fallback odometry for degeneracy recovery
    // Upstream: laserMapping.cpp:64-67 subscribes to "vins_estimator/imu_propagate"
    // for visual odometry. Any odometry source works here (not VINS-specific).
    string g_fallback_odom_topic = mod.has("fallback_odometry") ? mod.topic("fallback_odometry") : "";

    // ---- Diagnostic output topics — upstream publishTopic() lines 927-1111 ----
    g_surround_map_topic = mod.has("surround_map") ? mod.topic("surround_map") : "";
    g_global_map_topic = mod.has("global_map") ? mod.topic("global_map") : "";
    g_laser_odom_topic = mod.has("laser_odometry") ? mod.topic("laser_odometry") : "";
    g_incremental_odom_topic = mod.has("incremental_odometry") ? mod.topic("incremental_odometry") : "";
    g_stats_topic = mod.has("slam_stats") ? mod.topic("slam_stats") : "";

    // ---- Diagnostic publish flags (default off to save CPU/bandwidth) ----
    g_publish_surround_map = mod.arg("publish_surround_map", "false") == "true";
    g_publish_global_map = mod.arg("publish_global_map", "false") == "true";
    g_publish_laser_odometry = mod.arg("publish_laser_odometry", "false") == "true";
    g_publish_incremental_odometry = mod.arg("publish_incremental_odometry", "false") == "true";
    g_publish_slam_stats = mod.arg("publish_slam_stats", "false") == "true";

    if (g_lidar_topic.empty() || g_imu_topic.empty()) {
        fprintf(stderr, "Error: --lidar and --imu topics are required\n");
        return 1;
    }

    // ---- Parse shared config ----
    string sensor_str = mod.arg("sensor", "livox");
    SensorType sensor_type = parseSensorType(sensor_str);
    bool use_imu_roll_pitch = mod.arg("use_imu_roll_pitch", "false") == "true";
    bool lidar_flip = mod.arg("lidar_flip", "false") == "true";

    // IMU offset/limit params (shared by feature extraction and IMU preintegration)
    // Upstream YAML: feature_extraction_node has x=0.04, y=0.0, z=0.0, limits=1.0
    // imu_preintegration_node has x=0.04, y=0.0, z=0.0, limits=0.3/0.2/0.1
    // We use feature_extraction defaults since both nodes share these.
    double imu_acc_x_offset = mod.arg_float("imu_acc_x_offset", 0.04);
    double imu_acc_y_offset = mod.arg_float("imu_acc_y_offset", 0.0);
    double imu_acc_z_offset = mod.arg_float("imu_acc_z_offset", 0.0);
    double imu_acc_x_limit = mod.arg_float("imu_acc_x_limit", 1.0);
    double imu_acc_y_limit = mod.arg_float("imu_acc_y_limit", 1.0);
    double imu_acc_z_limit = mod.arg_float("imu_acc_z_limit", 1.0);

    // ---- Feature extraction config ----
    // Upstream: featureExtraction.cpp::readParameters() reads from ROS params
    // Config YAML: arise_slam_mid360/config/livox_mid360.yaml
    arise_slam::feature_extraction_config fe_config;
    fe_config.box_size.blindFront = mod.arg_float("blind_front", 0.2);
    fe_config.box_size.blindBack = mod.arg_float("blind_back", -0.2);
    fe_config.box_size.blindRight = mod.arg_float("blind_right", -0.3);
    fe_config.box_size.blindLeft = mod.arg_float("blind_left", 0.3);
    fe_config.box_size.blindDiskLow = mod.arg_float("blind_disk_low", -0.05);
    fe_config.box_size.blindDiskHigh = mod.arg_float("blind_disk_high", 0.05);
    fe_config.box_size.blindDiskRadius = mod.arg_float("blind_disk_radius", 0.5);
    fe_config.skipFrame = mod.arg_int("skip_frame", 1);
    fe_config.lidar_flip = lidar_flip;
    fe_config.N_SCANS = mod.arg_int("scan_line", 4);
    fe_config.provide_point_time = mod.arg_int("provide_point_time", 1);
    fe_config.point_filter_num = mod.arg_int("point_filter_num", 1);
    fe_config.use_dynamic_mask = mod.arg("use_dynamic_mask", "false") == "true";
    fe_config.use_imu_roll_pitch = use_imu_roll_pitch;
    fe_config.use_up_realsense_points = mod.arg("use_up_realsense_points", "false") == "true";
    fe_config.use_down_realsense_points = mod.arg("use_down_realsense_points", "false") == "true";
    fe_config.debug_view_enabled = mod.arg("debug_view_enabled", "false") == "true";
    fe_config.min_range = mod.arg_float("min_range", 0.1);
    fe_config.max_range = mod.arg_float("max_range", 130.0);
    fe_config.skip_realsense_points = mod.arg_int("skip_realsense_points", 3);
    fe_config.sensor = sensor_type;
    fe_config.livox_pitch = mod.arg_float("livox_pitch", 0.0);
    fe_config.imu_acc_x_offset = imu_acc_x_offset;
    fe_config.imu_acc_y_offset = imu_acc_y_offset;
    fe_config.imu_acc_z_offset = imu_acc_z_offset;
    fe_config.imu_acc_x_limit = imu_acc_x_limit;
    fe_config.imu_acc_y_limit = imu_acc_y_limit;
    fe_config.imu_acc_z_limit = imu_acc_z_limit;

    // ---- Laser mapping config ----
    // Upstream: laserMapping.cpp::readParameters() reads from ROS params
    // laserMapping.cpp on_configure (line 19) also sets slam.localMap fields (line 115-120)
    arise_slam::laser_mapping_config lm_config;
    lm_config.period = mod.arg_float("mapping_period", 0.1);
    lm_config.lineRes = mod.arg_float("mapping_line_resolution", 0.1);
    lm_config.planeRes = mod.arg_float("mapping_plane_resolution", 0.2);
    lm_config.max_iterations = mod.arg_int("max_iterations", 5);
    lm_config.debug_view_enabled = mod.arg("debug_view_enabled", "false") == "true";
    lm_config.enable_ouster_data = (sensor_type == SensorType::OUSTER);
    lm_config.publish_only_feature_points = mod.arg("publish_only_feature_points", "false") == "true";
    lm_config.use_imu_roll_pitch = use_imu_roll_pitch;
    lm_config.max_surface_features = mod.arg_int("max_surface_features", 2000);
    lm_config.velocity_failure_threshold = mod.arg_float("velocity_failure_threshold", 30.0);
    lm_config.auto_voxel_size = mod.arg("auto_voxel_size", "true") == "true";
    lm_config.forget_far_chunks = mod.arg("forget_far_chunks", "false") == "true";
    lm_config.visual_confidence_factor = mod.arg_float("visual_confidence_factor", 1.0);
    lm_config.pos_degeneracy_threshold = mod.arg_float("pos_degeneracy_threshold", 1.0);
    lm_config.ori_degeneracy_threshold = mod.arg_float("ori_degeneracy_threshold", 1.0);
    lm_config.shift_avg_ratio = mod.arg_float("shift_avg_ratio", 0.2);
    lm_config.shift_undistortion = mod.arg("shift_undistortion", "true") == "true";
    // Upstream calibration YAML: yaw_ratio: 0.0
    lm_config.yaw_ratio = mod.arg_float("yaw_ratio", 0.0);
    lm_config.relocalization_map_path = mod.arg("relocalization_map_path", "");
    lm_config.local_mode = mod.arg("local_mode", "false") == "true";
    lm_config.init_x = mod.arg_float("init_x", 0.0);
    lm_config.init_y = mod.arg_float("init_y", 0.0);
    lm_config.init_z = mod.arg_float("init_z", 0.0);
    lm_config.init_roll = mod.arg_float("init_roll", 0.0);
    lm_config.init_pitch = mod.arg_float("init_pitch", 0.0);
    lm_config.init_yaw = mod.arg_float("init_yaw", 0.0);
    lm_config.read_pose_file = mod.arg("read_pose_file", "false") == "true";
    lm_config.trust_fallback_odom = mod.arg("trust_fallback_odom", "false") == "true";

    // ---- IMU preintegration config ----
    // Upstream: imuPreintegration.cpp::readParameters() reads from ROS params
    // GTSAM preintegration params set in on_configure (line 76-119)
    g_has_imu_preintegration = mod.arg("use_imu_preintegration", "true") == "true";

    arise_slam::imuPreintegration_config imu_config;
    imu_config.imuAccNoise = mod.arg_float("acc_n", 0.3994);
    imu_config.imuAccBiasN = mod.arg_float("acc_w", 0.006436);
    imu_config.imuGyrNoise = mod.arg_float("gyr_n", 0.001564);
    imu_config.imuGyrBiasN = mod.arg_float("gyr_w", 0.0000356);
    imu_config.imuGravity = mod.arg_float("g_norm", 9.80511);
    imu_config.lidar_correction_noise = mod.arg_float("lidar_correction_noise", 0.01);
    imu_config.smooth_factor = mod.arg_float("smooth_factor", 0.9);
    imu_config.use_imu_roll_pitch = use_imu_roll_pitch;
    imu_config.lidar_flip = lidar_flip;
    imu_config.sensor = sensor_type;
    imu_config.imu_acc_x_offset = imu_acc_x_offset;
    imu_config.imu_acc_y_offset = imu_acc_y_offset;
    imu_config.imu_acc_z_offset = imu_acc_z_offset;
    // Upstream YAML: imu_preintegration_node has tighter limits (0.3/0.2/0.1)
    // than feature_extraction_node (1.0/1.0/1.0). Use separate CLI args.
    imu_config.imu_acc_x_limit = mod.arg_float("imu_preint_acc_x_limit", 0.3);
    imu_config.imu_acc_y_limit = mod.arg_float("imu_preint_acc_y_limit", 0.2);
    imu_config.imu_acc_z_limit = mod.arg_float("imu_preint_acc_z_limit", 0.1);

    // ---- Set global calibration parameters (from parameter.h) ----
    // These globals are used internally by the arise_slam library
    sensor = sensor_type;
    readGlobalparam();
    readCalibration();

    // ---- Print config summary ----
    printf("[arise_slam] Starting LiDAR-Inertial SLAM module\n");
    printf("[arise_slam] lidar: %s | imu: %s\n",
           g_lidar_topic.c_str(), g_imu_topic.c_str());
    printf("[arise_slam] odom: %s | scan: %s\n",
           g_odom_topic.c_str(), g_scan_topic.c_str());
    printf("[arise_slam] sensor=%s scan_line=%d use_imu_roll_pitch=%s\n",
           sensor_str.c_str(), fe_config.N_SCANS,
           use_imu_roll_pitch ? "true" : "false");
    printf("[arise_slam] mapping: line_res=%.2f plane_res=%.2f max_iter=%d\n",
           lm_config.lineRes, lm_config.planeRes, lm_config.max_iterations);
    printf("[arise_slam] imu_preintegration=%s acc_n=%.4f gyr_n=%.5f\n",
           g_has_imu_preintegration ? "on" : "off",
           imu_config.imuAccNoise, imu_config.imuGyrNoise);
    if (g_publish_surround_map || g_publish_global_map || g_publish_laser_odometry ||
        g_publish_incremental_odometry || g_publish_slam_stats) {
        printf("[arise_slam] diagnostics: surround_map=%s global_map=%s laser_odom=%s "
               "incremental_odom=%s stats=%s\n",
               g_publish_surround_map ? "on" : "off",
               g_publish_global_map ? "on" : "off",
               g_publish_laser_odometry ? "on" : "off",
               g_publish_incremental_odometry ? "on" : "off",
               g_publish_slam_stats ? "on" : "off");
    }

    // ---- Signal handlers ----
    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);

    // ---- Init LCM ----
    lcm::LCM lcm;
    if (!lcm.good()) {
        fprintf(stderr, "[arise_slam] Error: LCM init failed\n");
        return 1;
    }
    g_lcm = &lcm;

    // ---- Initialize algorithm components ----
    // Upstream: each node's on_configure callback initializes its component.
    // featureExtraction on_configure (line 28): readGlobalparam, readParameters, readCalibration
    // laserMapping on_configure (line 19): same + initROSInterface, slam setup
    // imuPreintegration on_configure (line 16): same + GTSAM factor graph setup
    printf("[arise_slam] Initializing feature extraction...\n");
    if (!g_feature_extraction.init(fe_config)) {
        fprintf(stderr, "[arise_slam] Error: feature extraction init failed\n");
        return 1;
    }

    printf("[arise_slam] Initializing laser mapping...\n");
    g_laser_mapping.init(lm_config);

    if (g_has_imu_preintegration) {
        printf("[arise_slam] Initializing IMU preintegration...\n");
        g_imu_preintegration.init(imu_config);
    }

    // ---- Set up output callbacks for IMU preintegration ----
    // Upstream: imuPreintegration publishes at IMU rate to two topics:
    //   1. state_estimation (line 63-64) — consumed by downstream
    //   2. integrated_to_init5 — consumed by laserMapping (line 59-62)
    //      for IMU odometry initial guess in extractVisualIMUOdometryAndCheck()
    // Here we use a callback to both publish via LCM and feed back to laser mapping.
    if (g_has_imu_preintegration) {
        g_imu_preintegration.setOutputCallback([](const arise_slam::ImuState& state) {
            publish_odometry(state);
            // Feed IMU-fused odometry back to laser mapping as ICP initial guess
            // Upstream: laserMapping subscribes to integrated_to_init5 (line 59-62)
            // via IMUOdometryHandler which pushes to imu_odom_buf
            g_laser_mapping.addIMUOdometry(
                state.timestamp,
                state.position,
                state.orientation
            );
        });
    }

    // ---- Subscribe to LCM inputs ----
    // Upstream subscriptions spread across 3 nodes:
    //   featureExtraction: lidar (line 76/80), IMU (line 85), odom (line 90)
    //   laserMapping:      features (line 54), IMU odom (line 59), visual odom (line 64)
    //   imuPreintegration: IMU (line 48), laser odom (line 52)
    // Here we need lidar + IMU + optional fallback odometry.
    // Features and laser odom flow internally between modules.
    lcm::LCM::HandlerFunction<sensor_msgs::PointCloud2> lidar_handler = on_lidar;
    lcm::LCM::HandlerFunction<sensor_msgs::Imu> imu_handler = on_imu;
    lcm.subscribe(g_lidar_topic, lidar_handler);
    lcm.subscribe(g_imu_topic, imu_handler);

    // Optional fallback odometry for degeneracy recovery
    // Upstream: laserMapping subscribes to visual odom (line 64)
    if (!g_fallback_odom_topic.empty()) {
        lcm::LCM::HandlerFunction<nav_msgs::Odometry> fallback_odom_handler = on_fallback_odom;
        lcm.subscribe(g_fallback_odom_topic, fallback_odom_handler);
        printf("[arise_slam] Fallback odometry: %s\n", g_fallback_odom_topic.c_str());
    }

    printf("[arise_slam] Initialized. Waiting for data...\n");

    // ---- Main loop ----
    // Upstream: each node uses rclcpp::executors::MultiThreadedExecutor::spin()
    // Here we use a single LCM event loop that dispatches lidar and IMU callbacks.
    while (g_running.load()) {
        lcm.handleTimeout(10);
    }

    // ---- Cleanup ----
    printf("[arise_slam] Shutting down...\n");
    g_lcm = nullptr;
    printf("[arise_slam] Done.\n");
    return 0;
}
