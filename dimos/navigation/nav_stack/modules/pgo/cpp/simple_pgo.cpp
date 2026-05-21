#include "simple_pgo.h"

#include <cstdio>
#include <limits>

SimplePGO::SimplePGO(const Config &config) : m_config(config)
{
    gtsam::ISAM2Params isam2_params;
    isam2_params.relinearizeThreshold = 0.01;
    isam2_params.relinearizeSkip = 1;
    m_isam2 = std::make_shared<gtsam::ISAM2>(isam2_params);
    m_initial_values.clear();
    m_graph.resize(0);
    m_r_offset.setIdentity();
    m_t_offset.setZero();

    m_icp.setMaximumIterations(50);
    m_icp.setMaxCorrespondenceDistance(10);
    m_icp.setTransformationEpsilon(1e-6);
    m_icp.setEuclideanFitnessEpsilon(1e-6);
    m_icp.setRANSACIterations(0);

    m_scan_context_config.n_rings = m_config.scan_context_num_rings;
    m_scan_context_config.n_sectors = m_config.scan_context_num_sectors;
    m_scan_context_config.max_range_m = m_config.scan_context_max_range_m;
    m_scan_context_config.candidate_top_k = m_config.scan_context_top_k;
    m_scan_context_config.match_threshold = m_config.scan_context_match_threshold;
    m_scan_context_config.lidar_height_m = m_config.scan_context_lidar_height_m;
}

bool SimplePGO::isKeyPose(const PoseWithTime &pose)
{
    if (m_key_poses.size() == 0)
        return true;
    const KeyPoseWithCloud &last_item = m_key_poses.back();
    double delta_trans = (pose.t - last_item.t_local).norm();
    double delta_deg = Eigen::Quaterniond(pose.r).angularDistance(Eigen::Quaterniond(last_item.r_local)) * 57.324;
    if (delta_trans > m_config.key_pose_delta_trans || delta_deg > m_config.key_pose_delta_deg)
        return true;
    return false;
}
bool SimplePGO::addKeyPose(const CloudWithPose &cloud_with_pose)
{
    bool is_key_pose = isKeyPose(cloud_with_pose.pose);
    if (!is_key_pose)
        return false;
    size_t idx = m_key_poses.size();
    M3D init_r = m_r_offset * cloud_with_pose.pose.r;
    V3D init_t = m_r_offset * cloud_with_pose.pose.t + m_t_offset;
    // 添加初始值
    m_initial_values.insert(idx, gtsam::Pose3(gtsam::Rot3(init_r), gtsam::Point3(init_t)));
    if (idx == 0)
    {
        // 添加先验约束
        gtsam::noiseModel::Diagonal::shared_ptr noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Ones() * 1e-12);
        m_graph.add(gtsam::PriorFactor<gtsam::Pose3>(idx, gtsam::Pose3(gtsam::Rot3(init_r), gtsam::Point3(init_t)), noise));
    }
    else
    {
        // 添加里程计约束
        const KeyPoseWithCloud &last_item = m_key_poses.back();
        M3D r_between = last_item.r_local.transpose() * cloud_with_pose.pose.r;
        V3D t_between = last_item.r_local.transpose() * (cloud_with_pose.pose.t - last_item.t_local);
        gtsam::noiseModel::Diagonal::shared_ptr noise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-6).finished());
        m_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(idx - 1, idx, gtsam::Pose3(gtsam::Rot3(r_between), gtsam::Point3(t_between)), noise));
    }
    KeyPoseWithCloud item;
    item.time = cloud_with_pose.pose.second;
    item.r_local = cloud_with_pose.pose.r;
    item.t_local = cloud_with_pose.pose.t;
    item.body_cloud = cloud_with_pose.cloud;
    item.r_global = init_r;
    item.t_global = init_t;
    m_key_poses.push_back(item);

    // Cache the Scan Context descriptor + ring-key for this keyframe.
    if (cloud_with_pose.cloud) {
        scan_context::Descriptor descriptor =
            scan_context::make_descriptor(*cloud_with_pose.cloud, m_scan_context_config);
        m_scan_context_ring_keys.push_back(scan_context::make_ring_key(descriptor));
        m_scan_context_descriptors.push_back(std::move(descriptor));
    } else {
        m_scan_context_descriptors.emplace_back();
        m_scan_context_ring_keys.emplace_back();
    }

    return true;
}

CloudType::Ptr SimplePGO::getSubMap(int idx, int half_range, double resolution)
{
    assert(idx >= 0 && idx < static_cast<int>(m_key_poses.size()));
    int min_idx = std::max(0, idx - half_range);
    int max_idx = std::min(static_cast<int>(m_key_poses.size()) - 1, idx + half_range);

    CloudType::Ptr ret(new CloudType);
    for (int i = min_idx; i <= max_idx; i++)
    {

        CloudType::Ptr body_cloud = m_key_poses[i].body_cloud;
        CloudType::Ptr global_cloud(new CloudType);
        pcl::transformPointCloud(*body_cloud, *global_cloud, m_key_poses[i].t_global, Eigen::Quaterniond(m_key_poses[i].r_global));
        *ret += *global_cloud;
    }
    if (resolution > 0)
    {
        pcl::VoxelGrid<PointType> voxel_grid;
        voxel_grid.setLeafSize(resolution, resolution, resolution);
        voxel_grid.setInputCloud(ret);
        voxel_grid.filter(*ret);
    }
    return ret;
}

int SimplePGO::searchByPosition() const
{
    size_t cur_idx = m_key_poses.size() - 1;
    const KeyPoseWithCloud &last_item = m_key_poses.back();
    pcl::PointXYZ last_pose_pt;
    last_pose_pt.x = last_item.t_global(0);
    last_pose_pt.y = last_item.t_global(1);
    last_pose_pt.z = last_item.t_global(2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr key_poses_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < cur_idx; i++)
    {
        pcl::PointXYZ pt;
        pt.x = m_key_poses[i].t_global(0);
        pt.y = m_key_poses[i].t_global(1);
        pt.z = m_key_poses[i].t_global(2);
        key_poses_cloud->push_back(pt);
    }
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(key_poses_cloud);
    std::vector<int> ids;
    std::vector<float> sqdists;
    int neighbors = kdtree.radiusSearch(last_pose_pt, m_config.loop_search_radius, ids, sqdists);
    if (neighbors == 0)
        return -1;

    for (size_t i = 0; i < ids.size(); i++)
    {
        int idx = ids[i];
        if (std::abs(last_item.time - m_key_poses[idx].time) > m_config.loop_time_thresh)
        {
            return idx;
        }
    }
    return -1;
}

int SimplePGO::searchByScanContext(int& out_sector_shift) const
{
    out_sector_shift = 0;
    if (m_scan_context_descriptors.empty() || m_scan_context_descriptors.back().size() == 0) {
        return -1;
    }
    const auto& query = m_scan_context_descriptors.back();
    const auto& query_key = m_scan_context_ring_keys.back();
    const double current_time = m_key_poses.back().time;

    // Two-stage retrieval: first rank candidates by ring-key L2 distance
    // (fast coarse filter), then score the top-K via column-shifted cosine
    // distance on the full descriptor.
    std::vector<std::pair<float, int>> ranked;  // (ring-key dist, idx)
    ranked.reserve(m_scan_context_descriptors.size());
    const size_t cur_idx = m_key_poses.size() - 1;
    for (size_t i = 0; i < cur_idx; i++) {
        if (m_scan_context_descriptors[i].size() == 0) continue;
        if (std::abs(current_time - m_key_poses[i].time) <= m_config.loop_time_thresh) {
            continue;  // too recent — not a true loop candidate
        }
        const float key_dist = (m_scan_context_ring_keys[i] - query_key).norm();
        ranked.emplace_back(key_dist, static_cast<int>(i));
    }
    if (ranked.empty()) return -1;

    const int top_k_count = std::min(
        static_cast<int>(ranked.size()), m_scan_context_config.candidate_top_k);
    std::partial_sort(
        ranked.begin(), ranked.begin() + top_k_count, ranked.end(),
        [](const std::pair<float, int>& a, const std::pair<float, int>& b) {
            return a.first < b.first;
        });

    float best_dist = std::numeric_limits<float>::max();
    int best_idx_unfiltered = -1;
    float best_dist_filtered = static_cast<float>(m_scan_context_config.match_threshold);
    int best_idx = -1;
    int best_shift = 0;
    for (int rank = 0; rank < top_k_count; rank++) {
        const int idx = ranked[rank].second;
        const auto [distance, shift] = scan_context::best_distance(
            query, m_scan_context_descriptors[idx]);
        if (distance < best_dist) {
            best_dist = distance;
            best_idx_unfiltered = idx;
        }
        if (distance < best_dist_filtered) {
            best_dist_filtered = distance;
            best_idx = idx;
            best_shift = shift;
        }
    }

    out_sector_shift = best_shift;
    return best_idx;
}

void SimplePGO::searchForLoopPairs()
{
    if (m_key_poses.size() < 10)
        return;
    if (m_config.min_loop_detect_duration > 0.0)
    {
        if (m_history_pairs.size() > 0)
        {
            double current_time = m_key_poses.back().time;
            double last_time = m_key_poses[m_history_pairs.back().second].time;
            if (current_time - last_time < m_config.min_loop_detect_duration)
                return;
        }
    }

    size_t cur_idx = m_key_poses.size() - 1;

    int loop_idx = -1;
    int sector_shift = 0;
    if (m_config.use_scan_context) {
        loop_idx = searchByScanContext(sector_shift);
    }
    if (loop_idx < 0) {
        // Fallback (or sole path if SC disabled): kdtree on past positions.
        loop_idx = searchByPosition();
    }

    if (loop_idx < 0)
        return;

    // Use Scan Context's column shift to seed ICP with a yaw-aligned initial
    // guess, which dramatically improves convergence on revisits at
    // different headings. Both submaps are in *global* frame, so a naive
    // rotation about the world origin would translate the source cloud
    // kilometers off (e.g. at world position (3500, 350), rotating by 90°
    // sends it to (-350, 3500)). Build a transform that rotates about the
    // source keyframe's own global position instead:
    //     init = T(source_position) · Rz(yaw) · T(-source_position)
    // → init · p = rotation · (p - source_position) + source_position
    Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
    if (m_config.use_scan_context && sector_shift != 0) {
        const double yaw = scan_context::yaw_from_shift(sector_shift, m_scan_context_config.n_sectors);
        Eigen::AngleAxisf yaw_axis_angle(
            static_cast<float>(yaw), Eigen::Vector3f::UnitZ());
        Eigen::Matrix3f rotation = yaw_axis_angle.toRotationMatrix();
        Eigen::Vector3f source_position = m_key_poses[cur_idx].t_global.cast<float>();
        init_guess.block<3, 3>(0, 0) = rotation;
        init_guess.block<3, 1>(0, 3) = source_position - rotation * source_position;
    }

    CloudType::Ptr target_cloud = getSubMap(loop_idx, m_config.loop_submap_half_range, m_config.submap_resolution);
    CloudType::Ptr source_cloud = getSubMap(cur_idx, 0, m_config.submap_resolution);
    CloudType::Ptr align_cloud(new CloudType);

    m_icp.setInputSource(source_cloud);
    m_icp.setInputTarget(target_cloud);
    m_icp.align(*align_cloud, init_guess);

    if (!m_icp.hasConverged() || m_icp.getFitnessScore() > m_config.loop_score_thresh)
        return;

    M4F loop_transform = m_icp.getFinalTransformation();

    LoopPair one_pair;
    one_pair.source_id = cur_idx;
    one_pair.target_id = loop_idx;
    one_pair.score = m_icp.getFitnessScore();
    M3D r_refined = loop_transform.block<3, 3>(0, 0).cast<double>() * m_key_poses[cur_idx].r_global;
    V3D t_refined = loop_transform.block<3, 3>(0, 0).cast<double>() * m_key_poses[cur_idx].t_global + loop_transform.block<3, 1>(0, 3).cast<double>();
    one_pair.r_offset = m_key_poses[loop_idx].r_global.transpose() * r_refined;
    one_pair.t_offset = m_key_poses[loop_idx].r_global.transpose() * (t_refined - m_key_poses[loop_idx].t_global);
    m_cache_pairs.push_back(one_pair);
    m_history_pairs.emplace_back(one_pair.target_id, one_pair.source_id);
}

void SimplePGO::smoothAndUpdate()
{
    bool has_loop = !m_cache_pairs.empty();
    // 添加回环因子
    if (has_loop)
    {
        for (LoopPair &pair : m_cache_pairs)
        {
            m_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(pair.target_id, pair.source_id,
                                                           gtsam::Pose3(gtsam::Rot3(pair.r_offset),
                                                                        gtsam::Point3(pair.t_offset)),
                                                           gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Ones() * pair.score)));
        }
        std::vector<LoopPair>().swap(m_cache_pairs);
    }
    // smooth and mapping
    m_isam2->update(m_graph, m_initial_values);
    m_isam2->update();
    if (has_loop)
    {
        m_isam2->update();
        m_isam2->update();
        m_isam2->update();
        m_isam2->update();
    }
    m_graph.resize(0);
    m_initial_values.clear();

    // update key poses
    gtsam::Values estimate_values = m_isam2->calculateBestEstimate();
    for (size_t i = 0; i < m_key_poses.size(); i++)
    {
        gtsam::Pose3 pose = estimate_values.at<gtsam::Pose3>(i);
        m_key_poses[i].r_global = pose.rotation().matrix().cast<double>();
        m_key_poses[i].t_global = pose.translation().matrix().cast<double>();
    }
    // update offset
    const KeyPoseWithCloud &last_item = m_key_poses.back();
    m_r_offset = last_item.r_global * last_item.r_local.transpose();
    m_t_offset = last_item.t_global - m_r_offset * last_item.t_local;
}
