// PGO (Pose Graph Optimization) — dimos NativeModule
// Ported from ROS2: src/slam/FASTLIO2_ROS2/pgo/src/pgos/simple_pgo.cpp
//
// Performs keyframe-based pose graph optimization with loop closure detection.
// Subscribes to registered_scan + odometry, publishes corrected_odometry + global_map.
//
// Loop closure pipeline:
//   1. Keyframe detection (translation/rotation thresholds)
//   2. KD-tree radius search on past keyframe positions
//   3. ICP verification between current and candidate submaps
//   4. GTSAM iSAM2 pose graph optimization
//   5. Global map assembly from corrected keyframes

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <lcm/lcm-cpp.hpp>

#include "dimos_native_module.hpp"
#include "point_cloud_utils.hpp"

#include "sensor_msgs/PointCloud2.hpp"
#include "nav_msgs/Odometry.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

using PointType = pcl::PointXYZI;
using CloudType = pcl::PointCloud<PointType>;
using M3D = Eigen::Matrix3d;
using V3D = Eigen::Vector3d;
using M4F = Eigen::Matrix4f;

// ─── Configuration ───────────────────────────────────────────────────────────

struct PGOConfig {
    double key_pose_delta_trans = 0.5;
    double key_pose_delta_deg = 10.0;
    double loop_search_radius = 15.0;
    double loop_time_thresh = 60.0;
    double loop_score_thresh = 0.3;
    int    loop_submap_half_range = 5;
    double submap_resolution = 0.1;
    double min_loop_detect_duration = 5.0;
    double global_map_publish_rate = 0.5;
    double global_map_voxel_size = 0.15;
    int    max_icp_iterations = 50;
    double max_icp_correspondence_dist = 10.0;
};

// ─── Keyframe storage ────────────────────────────────────────────────────────

struct KeyPoseWithCloud {
    M3D r_local;
    V3D t_local;
    M3D r_global;
    V3D t_global;
    double time;
    CloudType::Ptr body_cloud;
};

struct LoopPair {
    size_t source_id;
    size_t target_id;
    M3D r_offset;
    V3D t_offset;
    double score;
};

// ─── SimplePGO core algorithm ────────────────────────────────────────────────

class SimplePGO {
public:
    SimplePGO(const PGOConfig& config) : m_config(config) {
        gtsam::ISAM2Params isam2_params;
        isam2_params.relinearizeThreshold = 0.01;
        isam2_params.relinearizeSkip = 1;
        m_isam2 = std::make_shared<gtsam::ISAM2>(isam2_params);
        m_initial_values.clear();
        m_graph.resize(0);
        m_r_offset.setIdentity();
        m_t_offset.setZero();

        m_icp.setMaximumIterations(config.max_icp_iterations);
        m_icp.setMaxCorrespondenceDistance(config.max_icp_correspondence_dist);
        m_icp.setTransformationEpsilon(1e-6);
        m_icp.setEuclideanFitnessEpsilon(1e-6);
        m_icp.setRANSACIterations(0);
    }

    bool isKeyPose(const M3D& r, const V3D& t) {
        if (m_key_poses.empty()) return true;
        const auto& last = m_key_poses.back();
        double delta_trans = (t - last.t_local).norm();
        double delta_deg = Eigen::Quaterniond(r).angularDistance(
            Eigen::Quaterniond(last.r_local)) * 180.0 / M_PI;
        return (delta_trans > m_config.key_pose_delta_trans ||
                delta_deg > m_config.key_pose_delta_deg);
    }

    bool addKeyPose(const M3D& r_local, const V3D& t_local,
                    double timestamp, CloudType::Ptr body_cloud) {
        if (!isKeyPose(r_local, t_local)) return false;

        size_t idx = m_key_poses.size();
        M3D init_r = m_r_offset * r_local;
        V3D init_t = m_r_offset * t_local + m_t_offset;

        // Add initial value
        m_initial_values.insert(idx, gtsam::Pose3(gtsam::Rot3(init_r), gtsam::Point3(init_t)));

        if (idx == 0) {
            // Prior factor on first pose
            auto noise = gtsam::noiseModel::Diagonal::Variances(
                gtsam::Vector6::Ones() * 1e-12);
            m_graph.add(gtsam::PriorFactor<gtsam::Pose3>(
                idx, gtsam::Pose3(gtsam::Rot3(init_r), gtsam::Point3(init_t)), noise));
        } else {
            // Odometry factor
            const auto& last = m_key_poses.back();
            M3D r_between = last.r_local.transpose() * r_local;
            V3D t_between = last.r_local.transpose() * (t_local - last.t_local);
            auto noise = gtsam::noiseModel::Diagonal::Variances(
                (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-6).finished());
            m_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                idx - 1, idx,
                gtsam::Pose3(gtsam::Rot3(r_between), gtsam::Point3(t_between)),
                noise));
        }

        KeyPoseWithCloud item;
        item.time = timestamp;
        item.r_local = r_local;
        item.t_local = t_local;
        item.body_cloud = body_cloud;
        item.r_global = init_r;
        item.t_global = init_t;
        m_key_poses.push_back(item);
        return true;
    }

    CloudType::Ptr getSubMap(int idx, int half_range, double resolution) {
        int min_idx = std::max(0, idx - half_range);
        int max_idx = std::min(static_cast<int>(m_key_poses.size()) - 1, idx + half_range);

        CloudType::Ptr ret(new CloudType);
        for (int i = min_idx; i <= max_idx; i++) {
            CloudType::Ptr global_cloud(new CloudType);
            pcl::transformPointCloud(*m_key_poses[i].body_cloud, *global_cloud,
                m_key_poses[i].t_global.cast<float>(),
                Eigen::Quaternionf(m_key_poses[i].r_global.cast<float>()));
            *ret += *global_cloud;
        }
        if (resolution > 0 && ret->size() > 0) {
            pcl::VoxelGrid<PointType> voxel_grid;
            voxel_grid.setLeafSize(resolution, resolution, resolution);
            voxel_grid.setInputCloud(ret);
            voxel_grid.filter(*ret);
        }
        return ret;
    }

    void searchForLoopPairs() {
        if (m_key_poses.size() < 10) return;

        // Rate-limit loop detection
        if (m_config.min_loop_detect_duration > 0.0 && !m_history_pairs.empty()) {
            double current_time = m_key_poses.back().time;
            double last_time = m_key_poses[m_history_pairs.back().second].time;
            if (current_time - last_time < m_config.min_loop_detect_duration) return;
        }

        size_t cur_idx = m_key_poses.size() - 1;
        const auto& last_item = m_key_poses.back();

        // Build KD-tree of all previous keyframe positions
        pcl::PointCloud<pcl::PointXYZ>::Ptr key_poses_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (size_t i = 0; i < m_key_poses.size() - 1; i++) {
            pcl::PointXYZ pt;
            pt.x = m_key_poses[i].t_global(0);
            pt.y = m_key_poses[i].t_global(1);
            pt.z = m_key_poses[i].t_global(2);
            key_poses_cloud->push_back(pt);
        }

        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(key_poses_cloud);

        pcl::PointXYZ search_pt;
        search_pt.x = last_item.t_global(0);
        search_pt.y = last_item.t_global(1);
        search_pt.z = last_item.t_global(2);

        std::vector<int> ids;
        std::vector<float> sqdists;
        int neighbors = kdtree.radiusSearch(search_pt, m_config.loop_search_radius, ids, sqdists);
        if (neighbors == 0) return;

        // Find candidate far enough in time
        int loop_idx = -1;
        for (size_t i = 0; i < ids.size(); i++) {
            int idx = ids[i];
            if (std::abs(last_item.time - m_key_poses[idx].time) > m_config.loop_time_thresh) {
                loop_idx = idx;
                break;
            }
        }
        if (loop_idx == -1) return;

        // ICP verification
        CloudType::Ptr target_cloud = getSubMap(loop_idx, m_config.loop_submap_half_range,
                                                 m_config.submap_resolution);
        CloudType::Ptr source_cloud = getSubMap(m_key_poses.size() - 1, 0,
                                                 m_config.submap_resolution);
        CloudType::Ptr align_cloud(new CloudType);

        m_icp.setInputSource(source_cloud);
        m_icp.setInputTarget(target_cloud);
        m_icp.align(*align_cloud);

        if (!m_icp.hasConverged() || m_icp.getFitnessScore() > m_config.loop_score_thresh)
            return;

        M4F loop_transform = m_icp.getFinalTransformation();

        LoopPair pair;
        pair.source_id = cur_idx;
        pair.target_id = loop_idx;
        pair.score = m_icp.getFitnessScore();
        M3D r_refined = loop_transform.block<3,3>(0,0).cast<double>() * m_key_poses[cur_idx].r_global;
        V3D t_refined = loop_transform.block<3,3>(0,0).cast<double>() * m_key_poses[cur_idx].t_global +
                         loop_transform.block<3,1>(0,3).cast<double>();
        pair.r_offset = m_key_poses[loop_idx].r_global.transpose() * r_refined;
        pair.t_offset = m_key_poses[loop_idx].r_global.transpose() * (t_refined - m_key_poses[loop_idx].t_global);
        m_cache_pairs.push_back(pair);
        m_history_pairs.emplace_back(pair.target_id, pair.source_id);

        printf("[PGO] Loop closure detected: %zu <-> %zu (score=%.4f)\n",
               pair.target_id, pair.source_id, pair.score);
    }

    void smoothAndUpdate() {
        bool has_loop = !m_cache_pairs.empty();

        // Add loop closure factors
        if (has_loop) {
            for (auto& pair : m_cache_pairs) {
                m_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                    pair.target_id, pair.source_id,
                    gtsam::Pose3(gtsam::Rot3(pair.r_offset), gtsam::Point3(pair.t_offset)),
                    gtsam::noiseModel::Diagonal::Variances(
                        gtsam::Vector6::Ones() * pair.score)));
            }
            m_cache_pairs.clear();
        }

        // iSAM2 update
        m_isam2->update(m_graph, m_initial_values);
        m_isam2->update();
        if (has_loop) {
            // Extra iterations for convergence after loop closure
            m_isam2->update();
            m_isam2->update();
            m_isam2->update();
            m_isam2->update();
        }
        m_graph.resize(0);
        m_initial_values.clear();

        // Update keyframe poses from optimized values
        gtsam::Values estimates = m_isam2->calculateBestEstimate();
        for (size_t i = 0; i < m_key_poses.size(); i++) {
            gtsam::Pose3 pose = estimates.at<gtsam::Pose3>(i);
            m_key_poses[i].r_global = pose.rotation().matrix();
            m_key_poses[i].t_global = pose.translation();
        }

        // Update offset for incoming poses
        const auto& last = m_key_poses.back();
        m_r_offset = last.r_global * last.r_local.transpose();
        m_t_offset = last.t_global - m_r_offset * last.t_local;
    }

    // Build global map from all corrected keyframes
    CloudType::Ptr buildGlobalMap(double voxel_size) {
        CloudType::Ptr global_map(new CloudType);
        for (auto& kp : m_key_poses) {
            CloudType::Ptr world_cloud(new CloudType);
            pcl::transformPointCloud(*kp.body_cloud, *world_cloud,
                kp.t_global.cast<float>(),
                Eigen::Quaternionf(kp.r_global.cast<float>()));
            *global_map += *world_cloud;
        }
        if (voxel_size > 0 && global_map->size() > 0) {
            pcl::VoxelGrid<PointType> voxel;
            voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
            voxel.setInputCloud(global_map);
            voxel.filter(*global_map);
        }
        return global_map;
    }

    // Accessors
    const std::vector<KeyPoseWithCloud>& keyPoses() const { return m_key_poses; }
    size_t numKeyPoses() const { return m_key_poses.size(); }
    M3D offsetR() const { return m_r_offset; }
    V3D offsetT() const { return m_t_offset; }

    // Get corrected pose for current local pose
    void getCorrectedPose(const M3D& r_local, const V3D& t_local,
                          M3D& r_corrected, V3D& t_corrected) const {
        r_corrected = m_r_offset * r_local;
        t_corrected = m_r_offset * t_local + m_t_offset;
    }

private:
    PGOConfig m_config;
    std::vector<KeyPoseWithCloud> m_key_poses;
    std::vector<std::pair<size_t, size_t>> m_history_pairs;
    std::vector<LoopPair> m_cache_pairs;
    M3D m_r_offset;
    V3D m_t_offset;
    std::shared_ptr<gtsam::ISAM2> m_isam2;
    gtsam::Values m_initial_values;
    gtsam::NonlinearFactorGraph m_graph;
    pcl::IterativeClosestPoint<PointType, PointType> m_icp;
};

// ─── LCM Handler ─────────────────────────────────────────────────────────────

static std::atomic<bool> g_running{true};
void signal_handler(int) { g_running = false; }

struct PGOHandler {
    lcm::LCM* lcm;
    SimplePGO* pgo;
    std::string topic_corrected_odom;
    std::string topic_global_map;
    PGOConfig config;

    std::mutex mtx;
    M3D latest_r = M3D::Identity();
    V3D latest_t = V3D::Zero();
    double latest_time = 0.0;
    bool has_odom = false;

    // Global map publishing state
    double last_global_map_time = 0.0;

    void onOdometry(const lcm::ReceiveBuffer*, const std::string&,
                    const nav_msgs::Odometry* msg) {
        std::lock_guard<std::mutex> lock(mtx);
        latest_t = V3D(msg->pose.pose.position.x,
                       msg->pose.pose.position.y,
                       msg->pose.pose.position.z);
        Eigen::Quaterniond q(msg->pose.pose.orientation.w,
                             msg->pose.pose.orientation.x,
                             msg->pose.pose.orientation.y,
                             msg->pose.pose.orientation.z);
        latest_r = q.toRotationMatrix();
        latest_time = msg->header.stamp.sec + msg->header.stamp.nsec / 1e9;
        has_odom = true;
    }

    void onRegisteredScan(const lcm::ReceiveBuffer*, const std::string&,
                          const sensor_msgs::PointCloud2* msg) {
        std::lock_guard<std::mutex> lock(mtx);
        if (!has_odom) return;

        double scan_time = msg->header.stamp.sec + msg->header.stamp.nsec / 1e9;

        // Convert PointCloud2 to PCL (body frame)
        CloudType::Ptr body_cloud(new CloudType);
        smart_nav::to_pcl(*msg, *body_cloud);

        if (body_cloud->empty()) return;

        // Downsample body cloud for storage
        if (config.submap_resolution > 0) {
            pcl::VoxelGrid<PointType> voxel;
            voxel.setLeafSize(config.submap_resolution, config.submap_resolution,
                              config.submap_resolution);
            voxel.setInputCloud(body_cloud);
            voxel.filter(*body_cloud);
        }

        // Try to add as keyframe
        bool added = pgo->addKeyPose(latest_r, latest_t, latest_time, body_cloud);

        if (added) {
            pgo->searchForLoopPairs();
            pgo->smoothAndUpdate();
            printf("[PGO] Keyframe %zu added (%.1f, %.1f, %.1f)\n",
                   pgo->numKeyPoses(), latest_t(0), latest_t(1), latest_t(2));
        }

        // Publish corrected odometry
        publishCorrectedOdometry(scan_time);

        // Publish global map at configured rate
        double now = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        double interval = (config.global_map_publish_rate > 0) ?
            1.0 / config.global_map_publish_rate : 2.0;
        if (now - last_global_map_time > interval) {
            publishGlobalMap(scan_time);
            last_global_map_time = now;
        }
    }

    void publishCorrectedOdometry(double timestamp) {
        M3D r_corrected;
        V3D t_corrected;
        pgo->getCorrectedPose(latest_r, latest_t, r_corrected, t_corrected);

        Eigen::Quaterniond q(r_corrected);

        nav_msgs::Odometry odom;
        odom.header = dimos::make_header("map", timestamp);
        odom.child_frame_id = "sensor";
        odom.pose.pose.position.x = t_corrected(0);
        odom.pose.pose.position.y = t_corrected(1);
        odom.pose.pose.position.z = t_corrected(2);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        lcm->publish(topic_corrected_odom, &odom);
    }

    void publishGlobalMap(double timestamp) {
        if (pgo->numKeyPoses() == 0) return;

        CloudType::Ptr global_map = pgo->buildGlobalMap(config.global_map_voxel_size);

        sensor_msgs::PointCloud2 pc = smart_nav::from_pcl(*global_map, "map", timestamp);
        lcm->publish(topic_global_map, &pc);

        printf("[PGO] Global map published: %zu points, %zu keyframes\n",
               global_map->size(), pgo->numKeyPoses());
    }
};

// ─── Main ────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    dimos::NativeModule mod(argc, argv);

    // Read config from CLI args
    PGOConfig config;
    config.key_pose_delta_trans       = mod.arg_float("keyPoseDeltaTrans", 0.5f);
    config.key_pose_delta_deg         = mod.arg_float("keyPoseDeltaDeg", 10.0f);
    config.loop_search_radius         = mod.arg_float("loopSearchRadius", 15.0f);
    config.loop_time_thresh           = mod.arg_float("loopTimeThresh", 60.0f);
    config.loop_score_thresh          = mod.arg_float("loopScoreThresh", 0.3f);
    config.loop_submap_half_range     = mod.arg_int("loopSubmapHalfRange", 5);
    config.submap_resolution          = mod.arg_float("submapResolution", 0.1f);
    config.min_loop_detect_duration   = mod.arg_float("minLoopDetectDuration", 5.0f);
    config.global_map_publish_rate    = mod.arg_float("globalMapPublishRate", 0.5f);
    config.global_map_voxel_size      = mod.arg_float("globalMapVoxelSize", 0.15f);
    config.max_icp_iterations         = mod.arg_int("maxIcpIterations", 50);
    config.max_icp_correspondence_dist = mod.arg_float("maxIcpCorrespondenceDist", 10.0f);

    printf("[PGO] Config: keyPoseDeltaTrans=%.2f keyPoseDeltaDeg=%.1f "
           "loopSearchRadius=%.1f loopTimeThresh=%.1f loopScoreThresh=%.2f "
           "globalMapVoxelSize=%.2f\n",
           config.key_pose_delta_trans, config.key_pose_delta_deg,
           config.loop_search_radius, config.loop_time_thresh,
           config.loop_score_thresh, config.global_map_voxel_size);

    // Create PGO instance
    SimplePGO pgo(config);

    // LCM setup
    lcm::LCM lcm;
    if (!lcm.good()) {
        fprintf(stderr, "[PGO] LCM initialization failed\n");
        return 1;
    }

    PGOHandler handler;
    handler.lcm = &lcm;
    handler.pgo = &pgo;
    handler.topic_corrected_odom = mod.topic("corrected_odometry");
    handler.topic_global_map = mod.topic("global_map");
    handler.config = config;

    std::string topic_scan = mod.topic("registered_scan");
    std::string topic_odom = mod.topic("odometry");

    lcm.subscribe(topic_odom, &PGOHandler::onOdometry, &handler);
    lcm.subscribe(topic_scan, &PGOHandler::onRegisteredScan, &handler);

    printf("[PGO] Listening on: registered_scan=%s odometry=%s\n",
           topic_scan.c_str(), topic_odom.c_str());
    printf("[PGO] Publishing:   corrected_odometry=%s global_map=%s\n",
           handler.topic_corrected_odom.c_str(), handler.topic_global_map.c_str());

    while (g_running) {
        lcm.handleTimeout(100);
    }

    printf("[PGO] Shutting down. Total keyframes: %zu\n", pgo.numKeyPoses());
    return 0;
}
