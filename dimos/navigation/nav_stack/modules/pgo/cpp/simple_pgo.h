#pragma once
#include "commons.h"
#include "scan_context.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

struct KeyPoseWithCloud
{
    M3D r_local;
    V3D t_local;
    M3D r_global;
    V3D t_global;
    double time;
    CloudType::Ptr body_cloud;
};
struct LoopPair
{
    size_t source_id;
    size_t target_id;
    M3D r_offset;
    V3D t_offset;
    double score;
};

struct Config
{
    double key_pose_delta_deg = 10;
    double key_pose_delta_trans = 1.0;
    double loop_search_radius = 1.0;
    double loop_time_thresh = 60.0;
    double loop_score_thresh = 0.15;
    int loop_submap_half_range = 5;
    double submap_resolution = 0.1;
    double min_loop_detect_duration = 10.0;
    // Sanity gate: skip ICP if candidate keyframe is farther than this
    // from current keyframe in global pose. 0 disables the check.
    double loop_candidate_max_distance_m = 30.0;

    // Scan Context settings
    bool use_scan_context = true;
    int scan_context_num_rings = 20;
    int scan_context_num_sectors = 60;
    double scan_context_max_range_m = 80.0;
    int scan_context_top_k = 10;
    double scan_context_match_threshold = 0.4;
    double scan_context_lidar_height_m = 2.0;
};

class SimplePGO
{
public:
    SimplePGO(const Config &config);

    bool isKeyPose(const PoseWithTime &pose);

    bool addKeyPose(const CloudWithPose &cloud_with_pose);

    bool hasLoop(){return m_cache_pairs.size() > 0;}

    void searchForLoopPairs();

    void smoothAndUpdate();

    CloudType::Ptr getSubMap(int idx, int half_range, double resolution);
    std::vector<std::pair<size_t, size_t>> &historyPairs() { return m_history_pairs; }
    std::vector<KeyPoseWithCloud> &keyPoses() { return m_key_poses; }

    M3D offsetR() { return m_r_offset; }
    V3D offsetT() { return m_t_offset; }

    // Place recognition exposed for diagnostics / persistence.
    const std::vector<scan_context::Descriptor>& descriptors() const { return m_scan_context_descriptors; }
    const std::vector<scan_context::RingKey>& ringKeys() const { return m_scan_context_ring_keys; }

private:
    // Scan-context-based candidate search; returns -1 if no acceptable match.
    int searchByScanContext(int& out_sector_shift) const;
    // Original position-based fallback (radius search on past key-pose
    // positions). Kept for ablation + when scan context is disabled.
    int searchByPosition() const;

    Config m_config;
    scan_context::Config m_scan_context_config;
    std::vector<KeyPoseWithCloud> m_key_poses;
    std::vector<std::pair<size_t, size_t>> m_history_pairs;
    std::vector<LoopPair> m_cache_pairs;
    std::vector<scan_context::Descriptor> m_scan_context_descriptors;
    std::vector<scan_context::RingKey> m_scan_context_ring_keys;
    M3D m_r_offset;
    V3D m_t_offset;
    std::shared_ptr<gtsam::ISAM2> m_isam2;
    gtsam::Values m_initial_values;
    gtsam::NonlinearFactorGraph m_graph;
    pcl::IterativeClosestPoint<PointType, PointType> m_icp;
};
