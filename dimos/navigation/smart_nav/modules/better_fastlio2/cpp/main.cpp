// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// BetterFastLio2 native module for dimos NativeModule framework.
//
// Faithful reimplementation of https://github.com/Yixin-F/better_fastlio2
// adapted to DimOS: ROS replaced by LCM pub/sub, sensor SDK replaced by
// LCM input subscriptions.
//
// Usage: ./better_fastlio2 --lidar <topic> --imu <topic> --odometry <topic>
//        --registered_scan <topic> --global_map <topic> --corrected_path <topic>
//        [config args...]

#include "common_lib.h"
#include "preprocess.h"
#include "IMU_Processing.hpp"
#include "ikd_Tree.h"
#include "Scancontext.h"
#include "tgrs.h"

#include <omp.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <lcm/lcm-cpp.hpp>
#include "dimos_native_module.hpp"
#include "nav_msgs/Odometry.hpp"
#include "nav_msgs/Path.hpp"
#include "sensor_msgs/Imu.hpp"
#include "sensor_msgs/PointCloud2.hpp"
#include "sensor_msgs/PointField.hpp"
#include "geometry_msgs/Quaternion.hpp"
#include "geometry_msgs/Vector3.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

using namespace std;
using dimos::time_from_seconds;
using dimos::make_header;

// ============================================================================
// Constants
// ============================================================================
#define INIT_TIME       0.1
#define LASER_POINT_COV 0.001
// NUM_MATCH_POINTS defined in common_lib.h
// MP_PROC_NUM defined via CMake -DMP_PROC_NUM=N

// ============================================================================
// Global state
// ============================================================================
static atomic<bool> g_running{true};
static lcm::LCM* g_lcm = nullptr;

// LCM topics (filled from CLI args)
static string g_lidar_topic, g_imu_topic;
static string g_odom_topic, g_scan_topic, g_map_topic, g_path_topic;

// Preprocessor
static shared_ptr<Preprocess> p_pre(new Preprocess());

// IMU processor
static shared_ptr<ImuProcess> p_imu(new ImuProcess());

// iKdtree
static KD_TREE<PointType> ikdtree;

// EKF
static esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
static state_ikfom state_point;
static V3D pos_lid;

// Scan Context
static ScanContext::SCManager scLoop;

// Dynamic removal
static TGRS remover;

// GTSAM
static gtsam::NonlinearFactorGraph gtSAMgraph;
static gtsam::Values initialEstimate;
static gtsam::Values isamCurrentEstimate;
static gtsam::ISAM2* isam = nullptr;

// Config
static int NUM_MAX_ITERATIONS = 3;
static double filter_size_map_min = 0.2;
static double cube_len = 1500.0;
static double fov_deg = 180.0;
static double DET_RANGE = 100.0;
static double FOV_DEG = 0, HALF_FOV_COS = 0;
static bool extrinsic_est_en = false;
static int kd_step = 30;
static bool recontructKdTree = false;
static float keyframeAddingDistThreshold = 1.0;
static float keyframeAddingAngleThreshold = 0.2;
static bool dense_pub_en = false;
static float publish_map_frequency = 0.0;

// Loop closure
static bool loopClosureEnableFlag = false;
static float loopClosureFrequency = 1.0;
static float historyKeyframeSearchRadius = 10.0;
static float historyKeyframeSearchTimeDiff = 30.0;
static int historyKeyframeSearchNum = 2;
static float historyKeyframeFitnessScore = 0.2;

// Segment
static bool ground_en = false;
static bool tollerance_en = false;
static float sensor_height = 1.5;
static float z_tollerance = 2.0;
static float rotation_tollerance = 0.2;

// Dynamic removal
static bool dynamic_removal_enable = false;

// lidar end time (for loop closure thread)
static double lidar_end_time = 0.0;

// Buffers
static mutex mtx_buffer;
static condition_variable sig_buffer;
static deque<double> time_buffer;
static deque<pcl::PointCloud<PointType>::Ptr> lidar_buffer;
static deque<shared_ptr<ImuData>> imu_buffer;
static double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;

// Extrinsics
static vector<double> extrinT(3, 0.0);
static vector<double> extrinR(9, 0.0);
static V3D Lidar_T_wrt_IMU;
static M3D Lidar_R_wrt_IMU;

// IMU noise
static double gyr_cov = 0.1, acc_cov = 0.1;
static double b_gyr_cov = 0.0001, b_acc_cov = 0.0001;

// Processing state
static double first_lidar_time = 0.0;
static bool flg_first_scan = true, flg_EKF_inited = false;
static MeasureGroup Measures;
static pcl::PointCloud<PointType>::Ptr feats_undistort(new pcl::PointCloud<PointType>());
static pcl::PointCloud<PointType>::Ptr feats_down_body(new pcl::PointCloud<PointType>());
static pcl::PointCloud<PointType>::Ptr feats_down_world(new pcl::PointCloud<PointType>());
static pcl::PointCloud<PointType>::Ptr normvec(new pcl::PointCloud<PointType>());
static pcl::PointCloud<PointType>::Ptr laserCloudOri(new pcl::PointCloud<PointType>(100000, 1));
static pcl::PointCloud<PointType>::Ptr corr_normvect(new pcl::PointCloud<PointType>(100000, 1));
static pcl::PointCloud<PointType>::Ptr featsFromMap(new pcl::PointCloud<PointType>());
static pcl::PointCloud<PointType>::Ptr _featsArray;

static pcl::VoxelGrid<PointType> downSizeFilterSurf;
static pcl::VoxelGrid<PointType> downSizeFilterMap;
static pcl::VoxelGrid<PointType> downSizeFilterICP;

static int feats_down_size = 0;
static int effct_feat_num = 0;
static double total_residual = 0.0;
static double res_mean_last = 0.05;
static float res_last[100000] = {0.0};
static bool point_selected_surf[100000] = {0};
static double kdtree_incremental_time = 0.0;
static int add_point_size = 0;

static vector<BoxPointType> cub_needrm;
static vector<PointVector> Nearest_Points;
static vector<vector<int>> pointSearchInd_surf;

// Keyframe storage
static pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D(new pcl::PointCloud<PointType>());
static pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D(new pcl::PointCloud<PointTypePose>());
static pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D(new pcl::PointCloud<PointType>());
static pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D(new pcl::PointCloud<PointTypePose>());
static vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;

static float transformTobeMapped[6] = {0};

// Loop closure queues (from upstream — factors queued in loop thread, drained in saveKeyFramesAndFactor)
static mutex mtx;  // matches upstream naming
static map<int, int> loopIndexContainer;
static vector<pair<int, int>> loopIndexQueue;
static vector<gtsam::Pose3> loopPoseQueue;
static vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;

// iKdtree reconstruction (from upstream)
static int updateKdtreeCount = 0;
static float globalMapVisualizationSearchRadius = 10.0;
static float globalMapVisualizationPoseDensity = 10.0;
static float globalMapVisualizationLeafSize = 1.0;

// Quaternion for publishing
static Eigen::Quaterniond geoQuat;
static V3D euler_cur;

// ============================================================================
// Forward declarations (implemented below, matching upstream logic)
// ============================================================================
static void pointBodyToWorld(const PointType* pi, PointType* po);
static void lasermap_fov_segment();
static void map_incremental();
static bool sync_packages(MeasureGroup& meas);
static void h_share_model(state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data);
static void publish_odometry(double timestamp);
static void publish_registered_scan(double timestamp);
static void publish_global_map(double timestamp);
static void publish_path(double timestamp);
static void getCurPose(state_ikfom cur_state);
static bool saveFrame();
static void addOdomFactor();
static void addLoopFactor();
static void recontructIKdTree();
static void saveKeyFramesAndFactor();
static void correctPoses();
static bool detectLoopClosureDistance(int* latestID, int* closestID);
static void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum);
static void performLoopClosure();
static void loopClosureThread();

// ============================================================================
// Transform helpers (from upstream laserMapping.cpp)
// ============================================================================
static void pointBodyToWorld(const PointType* pi, PointType* po) {
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);
    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

// calc_dist is in common_lib.h

// ============================================================================
// getCurPose (from upstream laserMapping.cpp:436-453, faithful)
// ============================================================================
static void getCurPose(state_ikfom cur_state) {
    // Matches upstream: eulerAngles(2,1,0) = ZYX intrinsic = [yaw, pitch, roll]
    Eigen::Vector3d eulerAngle = cur_state.rot.matrix().eulerAngles(2, 1, 0);

    transformTobeMapped[0] = eulerAngle(2);    // roll
    transformTobeMapped[1] = eulerAngle(1);    // pitch
    transformTobeMapped[2] = eulerAngle(0);    // yaw
    transformTobeMapped[3] = cur_state.pos(0); // x
    transformTobeMapped[4] = cur_state.pos(1); // y
    transformTobeMapped[5] = cur_state.pos(2); // z

    if (tollerance_en) {
        transformTobeMapped[0] = constraintTransformation(transformTobeMapped[0], rotation_tollerance);
        transformTobeMapped[1] = constraintTransformation(transformTobeMapped[1], rotation_tollerance);
        transformTobeMapped[5] = constraintTransformation(transformTobeMapped[5], z_tollerance);
    }
}

// ============================================================================
// Pose conversion helpers
// ============================================================================
static gtsam::Pose3 pclPointTogtsamPose3(const PointTypePose& p) {
    return gtsam::Pose3(
        gtsam::Rot3::RzRyRx(double(p.roll), double(p.pitch), double(p.yaw)),
        gtsam::Point3(double(p.x), double(p.y), double(p.z)));
}

static gtsam::Pose3 trans2gtsamPose(float transformIn[]) {
    return gtsam::Pose3(
        gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]),
        gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
}

static PointTypePose trans2PointTypePose(float transformIn[]) {
    PointTypePose p;
    p.x = transformIn[3];
    p.y = transformIn[4];
    p.z = transformIn[5];
    p.roll = transformIn[0];
    p.pitch = transformIn[1];
    p.yaw = transformIn[2];
    return p;
}

static pcl::PointCloud<PointType>::Ptr transformPointCloud(
    pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn) {
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(
        transformIn->x, transformIn->y, transformIn->z,
        transformIn->roll, transformIn->pitch, transformIn->yaw);

    #pragma omp parallel for num_threads(4)
    for (int i = 0; i < cloudSize; ++i) {
        const auto& pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y +
                                transCur(0, 2) * pointFrom.z + transCur(0, 3);
        cloudOut->points[i].y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y +
                                transCur(1, 2) * pointFrom.z + transCur(1, 3);
        cloudOut->points[i].z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y +
                                transCur(2, 2) * pointFrom.z + transCur(2, 3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }
    return cloudOut;
}

// ============================================================================
// saveFrame (from upstream laserMapping.cpp:524-547)
// ============================================================================
static bool aLoopIsClosed = false;

static bool saveFrame() {
    if (cloudKeyPoses3D->points.empty()) return true;

    // Affine-transform-based keyframe selection (matches upstream exactly)
    Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
    Eigen::Affine3f transFinal = trans2Affine3f(transformTobeMapped);
    Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);

    if (abs(roll) < keyframeAddingAngleThreshold &&
        abs(pitch) < keyframeAddingAngleThreshold &&
        abs(yaw) < keyframeAddingAngleThreshold &&
        sqrt(x * x + y * y + z * z) < keyframeAddingDistThreshold) {
        return false;
    }
    return true;
}

// ============================================================================
// addOdomFactor (from upstream laserMapping.cpp:549-581)
// ============================================================================
static void addOdomFactor() {
    if (cloudKeyPoses3D->points.empty()) {
        gtsam::noiseModel::Diagonal::shared_ptr priorNoise =
            gtsam::noiseModel::Diagonal::Variances(
                (gtsam::Vector(6) << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12).finished());
        gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(
            0, trans2gtsamPose(transformTobeMapped), priorNoise));
        initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));
    } else {
        gtsam::noiseModel::Diagonal::shared_ptr odometryNoise =
            gtsam::noiseModel::Diagonal::Variances(
                (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
        gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back());
        gtsam::Pose3 poseTo = trans2gtsamPose(transformTobeMapped);
        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(
            cloudKeyPoses3D->size() - 1, cloudKeyPoses3D->size(),
            poseFrom.between(poseTo), odometryNoise));
        initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
    }
}

// ============================================================================
// addLoopFactor (from upstream laserMapping.cpp:583-609)
// ============================================================================
static void addLoopFactor() {
    if (loopIndexQueue.empty()) return;

    for (int i = 0; i < (int)loopIndexQueue.size(); ++i) {
        int indexFrom = loopIndexQueue[i].first;
        int indexTo = loopIndexQueue[i].second;
        gtsam::Pose3 poseBetween = loopPoseQueue[i];
        gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loopNoiseQueue[i];
        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(
            indexFrom, indexTo, poseBetween, noiseBetween));
    }

    loopIndexQueue.clear();
    loopPoseQueue.clear();
    loopNoiseQueue.clear();
    aLoopIsClosed = true;
}

// ============================================================================
// recontructIKdTree (from upstream laserMapping.cpp:612-669)
// ============================================================================
static void recontructIKdTree() {
    if (updateKdtreeCount == kd_step) {
        pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMapPoses(new pcl::KdTreeFLANN<PointType>());
        pcl::PointCloud<PointType>::Ptr subMapKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr subMapKeyPosesDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr subMapKeyFrames(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr subMapKeyFramesDS(new pcl::PointCloud<PointType>());

        std::vector<int> pointSearchIndGlobalMap;
        std::vector<float> pointSearchSqDisGlobalMap;
        mtx.lock();
        kdtreeGlobalMapPoses->setInputCloud(cloudKeyPoses3D);
        kdtreeGlobalMapPoses->radiusSearch(cloudKeyPoses3D->back(),
            globalMapVisualizationSearchRadius, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
        mtx.unlock();

        for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i) {
            subMapKeyPoses->push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);
        }

        pcl::VoxelGrid<PointType> downSizeFilterSubMapKeyPoses;
        downSizeFilterSubMapKeyPoses.setLeafSize(globalMapVisualizationPoseDensity,
            globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity);
        downSizeFilterSubMapKeyPoses.setInputCloud(subMapKeyPoses);
        downSizeFilterSubMapKeyPoses.filter(*subMapKeyPosesDS);

        for (int i = 0; i < (int)subMapKeyPosesDS->size(); ++i) {
            if (pointDistance(subMapKeyPosesDS->points[i], cloudKeyPoses3D->back()) > globalMapVisualizationSearchRadius)
                continue;
            int thisKeyInd = (int)subMapKeyPosesDS->points[i].intensity;
            *subMapKeyFrames += *transformPointCloud(surfCloudKeyFrames[thisKeyInd],
                &cloudKeyPoses6D->points[thisKeyInd]);
        }

        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames;
        downSizeFilterGlobalMapKeyFrames.setLeafSize(globalMapVisualizationLeafSize,
            globalMapVisualizationLeafSize, globalMapVisualizationLeafSize);
        downSizeFilterGlobalMapKeyFrames.setInputCloud(subMapKeyFrames);
        downSizeFilterGlobalMapKeyFrames.filter(*subMapKeyFramesDS);

        ikdtree.reconstruct(subMapKeyFramesDS->points);
        updateKdtreeCount = 0;
        printf("[better_fastlio2] Reconstructed ikdtree\n");
    }
    updateKdtreeCount++;
}

// ============================================================================
// saveKeyFramesAndFactor (from upstream laserMapping.cpp:680-766, faithful)
// ============================================================================
static void saveKeyFramesAndFactor() {
    // Check keyframe (matches upstream saveFrame())
    if (saveFrame() == false)
        return;

    // Odometry factor (matches upstream addOdomFactor())
    addOdomFactor();

    // Loop closure factor (matches upstream addLoopFactor())
    addLoopFactor();

    // Optimize (matches upstream laserMapping.cpp:695-709)
    isam->update(gtSAMgraph, initialEstimate);
    isam->update();

    // Extra ISAM updates when loop is closed (matches upstream:699-705)
    if (aLoopIsClosed == true) {
        printf("[better_fastlio2] pose is updated by isam\n");
        isam->update();
        isam->update();
        isam->update();
        isam->update();
    }

    gtSAMgraph.resize(0);
    initialEstimate.clear();

    // calculateBestEstimate (matches upstream:717)
    isamCurrentEstimate = isam->calculateBestEstimate();

    // Save keyframe pose (matches upstream:719-739)
    PointType thisPose3D;
    PointTypePose thisPose6D;
    gtsam::Pose3 latestEstimate;

    latestEstimate = isamCurrentEstimate.at<gtsam::Pose3>(isamCurrentEstimate.size() - 1);
    thisPose3D.x = latestEstimate.translation().x();
    thisPose3D.y = latestEstimate.translation().y();
    thisPose3D.z = latestEstimate.translation().z();
    thisPose3D.intensity = cloudKeyPoses3D->size();
    cloudKeyPoses3D->push_back(thisPose3D);

    // thisPose3D.z = 0.0 hack from upstream (laserMapping.cpp:732)
    thisPose3D.z = 0.0;  // FIXME: upstream hack, replicated for faithfulness

    // thisPose6D from ISAM estimate, NOT from transformTobeMapped (matches upstream:734-738)
    thisPose6D.x = thisPose3D.x;
    thisPose6D.y = thisPose3D.y;
    thisPose6D.z = thisPose3D.z;
    thisPose6D.intensity = thisPose3D.intensity;
    thisPose6D.roll = latestEstimate.rotation().roll();
    thisPose6D.pitch = latestEstimate.rotation().pitch();
    thisPose6D.yaw = latestEstimate.rotation().yaw();
    thisPose6D.time = lidar_end_time;
    cloudKeyPoses6D->push_back(thisPose6D);

    // ESKF state update (matches upstream:744-754)
    state_ikfom state_updated = kf.get_x();
    Eigen::Vector3d pos(latestEstimate.translation().x(),
                        latestEstimate.translation().y(),
                        latestEstimate.translation().z());
    Eigen::Quaterniond q = EulerToQuat(latestEstimate.rotation().roll(),
                                        latestEstimate.rotation().pitch(),
                                        latestEstimate.rotation().yaw());
    state_updated.pos = pos;
    state_updated.rot = q;
    state_point = state_updated;  // update state_point on every keyframe (upstream:751)

    if (aLoopIsClosed == true) {
        kf.change_x(state_updated);  // only feed back to ESKF on loop closure (upstream:754)
    }

    // Save keyframe cloud (matches upstream:756-758)
    pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
    pcl::copyPointCloud(*feats_undistort, *thisSurfKeyFrame);
    surfCloudKeyFrames.push_back(thisSurfKeyFrame);

    // iKdtree reconstruction (matches upstream:763-765)
    if (recontructKdTree) {
        recontructIKdTree();
    }
}

// ============================================================================
// correctPoses (from upstream laserMapping.cpp:769-805, faithful)
// ============================================================================
static void correctPoses() {
    if (cloudKeyPoses3D->points.empty()) return;
    if (aLoopIsClosed == true) {
        // Update all keyframe poses from ISAM estimate (upstream:779-795)
        int numPoses = isamCurrentEstimate.size();
        for (int i = 0; i < numPoses; ++i) {
            cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().x();
            cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().y();
            cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().z();

            cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
            cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
            cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
            cloudKeyPoses6D->points[i].roll = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().roll();
            cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().pitch();
            cloudKeyPoses6D->points[i].yaw = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().yaw();
        }

        // iKdtree reconstruction after loop closure (matches upstream:798-800)
        if (recontructKdTree) {
            recontructIKdTree();
        }

        printf("[better_fastlio2] ISAM2 Update\n");
        aLoopIsClosed = false;
    }
}

// ============================================================================
// lasermap_fov_segment (from upstream, faithful FOV-based local map management)
// ============================================================================
static bool Localmap_Initialized = false;
static BoxPointType LocalMap_Points;
static int kdtree_delete_counter = 0;

static void lasermap_fov_segment() {
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    V3D pos_LiD = pos_lid;

    // Initialize local map bounding box centered on lidar position
    if (!Localmap_Initialized) {
        for (int i = 0; i < 3; i++) {
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }

    // Check distance to each map boundary face
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++) {
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE ||
            dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) {
            need_move = true;
        }
    }
    if (!need_move) return;

    // Compute new bounding box and boxes to remove
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9,
                         double(DET_RANGE * (MOV_THRESHOLD - 1)));
    for (int i = 0; i < 3; i++) {
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE) {
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) {
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    // Delete points in removed boxes from ikdtree
    if (cub_needrm.size() > 0) {
        kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    }
}

// ============================================================================
// map_incremental (from upstream, faithful)
// ============================================================================
static void map_incremental() {
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);

    for (int i = 0; i < feats_down_size; i++) {
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        if (!Nearest_Points[i].empty() && flg_EKF_inited) {
            const PointVector& points_near = Nearest_Points[i];
            bool need_add = true;
            PointType mid_point;
            mid_point.x = floor(feats_down_world->points[i].x / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            float dist = calc_dist(feats_down_world->points[i], mid_point);

            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min &&
                fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min &&
                fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min) {
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }

            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++) {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist) {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        } else {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false);
}

// ============================================================================
// h_share_model (from upstream, faithful — measurement model for iESKF)
// ============================================================================
static void h_share_model(state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data) {
    laserCloudOri->clear();
    corr_normvect->clear();
    total_residual = 0.0;

#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
    #pragma omp parallel for
#endif
    for (int i = 0; i < feats_down_size; i++) {
        PointType& point_body = feats_down_body->points[i];
        PointType& point_world = feats_down_world->points[i];

        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
        auto& points_near = Nearest_Points[i];

        if (ekfom_data.converge) {
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false
                : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
        }

        if (!point_selected_surf[i]) continue;

        VF(4) pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f)) {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y +
                         pabcd(2) * point_world.z + pabcd(3);
            float s_val = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());
            if (s_val > 0.9) {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }

    effct_feat_num = 0;
    for (int i = 0; i < feats_down_size; i++) {
        if (point_selected_surf[i]) {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num++;
        }
    }

    if (effct_feat_num < 1) {
        ekfom_data.valid = false;
        fprintf(stderr, "[better_fastlio2] No effective points!\n");
        return;
    }

    res_mean_last = total_residual / effct_feat_num;

    ekfom_data.h_x = Eigen::MatrixXd::Zero(effct_feat_num, 12);
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++) {
        const PointType& laser_p = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);

        const PointType& norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        V3D C(s.rot.conjugate() * norm_vec);
        V3D A(point_crossmat * C);

        if (extrinsic_est_en) {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C);
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z,
                VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        } else {
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z,
                VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }
        ekfom_data.h(i) = -norm_p.intensity;
    }
}

// ============================================================================
// sync_packages (from upstream, adapted for LCM buffers)
// ============================================================================
static bool sync_packages(MeasureGroup& meas) {
    if (lidar_buffer.empty() || imu_buffer.empty()) return false;

    if (!meas.lidar) {
        meas.lidar.reset(new pcl::PointCloud<PointType>());
    }

    // Get lidar scan
    *(meas.lidar) = *(lidar_buffer.front());
    meas.lidar_beg_time = time_buffer.front();

    double local_lidar_end_time;
    if (meas.lidar->points.size() <= 1) {
        local_lidar_end_time = meas.lidar_beg_time + 0.1;
    } else {
        local_lidar_end_time = meas.lidar_beg_time +
                         meas.lidar->points.back().curvature / 1000.0;
    }
    meas.lidar_end_time = local_lidar_end_time;

    // Check if we have enough IMU data
    if (last_timestamp_imu < local_lidar_end_time) return false;

    // Collect IMU data for this lidar frame
    meas.imu.clear();
    while (!imu_buffer.empty()) {
        double imu_time = imu_buffer.front()->timestamp;
        if (imu_time > local_lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    return true;
}

// ============================================================================
// LCM publish helpers
// ============================================================================
static void publish_pointcloud_lcm(const string& topic, pcl::PointCloud<PointType>::Ptr cloud,
                                    double timestamp, const string& frame_id = "map") {
    if (!g_lcm || cloud->empty() || topic.empty()) return;

    int num_points = cloud->size();
    sensor_msgs::PointCloud2 pc;
    pc.header = make_header(frame_id, timestamp);
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

static void publish_odometry(double timestamp) {
    if (!g_lcm || g_odom_topic.empty()) return;

    nav_msgs::Odometry odom;
    odom.header = make_header("map", timestamp);
    odom.child_frame_id = "body";

    odom.pose.pose.position.x = state_point.pos(0);
    odom.pose.pose.position.y = state_point.pos(1);
    odom.pose.pose.position.z = state_point.pos(2);
    odom.pose.pose.orientation.x = geoQuat.x();
    odom.pose.pose.orientation.y = geoQuat.y();
    odom.pose.pose.orientation.z = geoQuat.z();
    odom.pose.pose.orientation.w = geoQuat.w();

    odom.twist.twist.linear.x = state_point.vel(0);
    odom.twist.twist.linear.y = state_point.vel(1);
    odom.twist.twist.linear.z = state_point.vel(2);

    g_lcm->publish(g_odom_topic, &odom);
}

static void publish_registered_scan(double timestamp) {
    if (g_scan_topic.empty()) return;

    pcl::PointCloud<PointType>::Ptr laserCloudWorld(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr source = dense_pub_en ? feats_undistort : feats_down_body;
    int size = source->size();
    laserCloudWorld->resize(size);

    for (int i = 0; i < size; i++) {
        pointBodyToWorld(&source->points[i], &laserCloudWorld->points[i]);
    }

    publish_pointcloud_lcm(g_scan_topic, laserCloudWorld, timestamp);
}

static void publish_global_map(double timestamp) {
    if (g_map_topic.empty() || cloudKeyPoses3D->empty()) return;

    pcl::PointCloud<PointType>::Ptr globalMap(new pcl::PointCloud<PointType>());
    for (size_t i = 0; i < surfCloudKeyFrames.size(); ++i) {
        *globalMap += *transformPointCloud(surfCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
    }

    pcl::VoxelGrid<PointType> ds;
    ds.setLeafSize(0.2f, 0.2f, 0.2f);
    ds.setInputCloud(globalMap);
    pcl::PointCloud<PointType>::Ptr filtered(new pcl::PointCloud<PointType>());
    ds.filter(*filtered);

    publish_pointcloud_lcm(g_map_topic, filtered, timestamp);
}

static void publish_path(double timestamp) {
    // Path publishing via LCM (nav_msgs::Path)
    // TODO: implement if needed for visualization
    (void)timestamp;
}

// ============================================================================
// LCM callbacks
// ============================================================================
static void on_lidar(const lcm::ReceiveBuffer* /*rbuf*/, const string& /*channel*/,
                     const sensor_msgs::PointCloud2* msg) {
    if (!g_running.load()) return;

    double timestamp = msg->header.stamp.sec + msg->header.stamp.nsec * 1e-9;

    // Convert LCM PointCloud2 to PCL
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());

    int num_points = msg->width * msg->height;
    if (num_points == 0) return;

    // Parse fields to find offsets
    int x_off = -1, y_off = -1, z_off = -1, int_off = -1;
    int time_off = -1, ring_off = -1, tag_off = -1;
    for (const auto& f : msg->fields) {
        if (f.name == "x") x_off = f.offset;
        else if (f.name == "y") y_off = f.offset;
        else if (f.name == "z") z_off = f.offset;
        else if (f.name == "intensity") int_off = f.offset;
        else if (f.name == "time" || f.name == "offset_time" || f.name == "t") time_off = f.offset;
        else if (f.name == "ring") ring_off = f.offset;
        else if (f.name == "tag") tag_off = f.offset;
    }

    if (x_off < 0 || y_off < 0 || z_off < 0) return;

    // Convert LCM PointCloud2 to lidar-type-specific PCL cloud and call Preprocess::process()
    // This matches the upstream approach where ROS callbacks convert to PCL then call process()
    pcl::PointCloud<PointType>::Ptr processed(new pcl::PointCloud<PointType>());

    if (p_pre->lidar_type == VELO16 || p_pre->lidar_type == RS) {
        // Velodyne/Robosense: need ring and time fields
        pcl::PointCloud<velodyne_ros::Point> pl_orig;
        pl_orig.resize(num_points);
        for (int i = 0; i < num_points; ++i) {
            const uint8_t* ptr = msg->data.data() + i * msg->point_step;
            auto& pt = pl_orig.points[i];
            pt.x = *reinterpret_cast<const float*>(ptr + x_off);
            pt.y = *reinterpret_cast<const float*>(ptr + y_off);
            pt.z = *reinterpret_cast<const float*>(ptr + z_off);
            pt.intensity = int_off >= 0 ? *reinterpret_cast<const float*>(ptr + int_off) : 0.0f;
            pt.ring = ring_off >= 0 ? *reinterpret_cast<const uint16_t*>(ptr + ring_off) : 0;
            if (time_off >= 0) {
                // Check field datatype
                for (const auto& f : msg->fields) {
                    if (f.offset == time_off) {
                        if (f.datatype == sensor_msgs::PointField::UINT32) {
                            pt.time = static_cast<float>(*reinterpret_cast<const uint32_t*>(ptr + time_off)) / 1e9f;
                        } else {
                            pt.time = *reinterpret_cast<const float*>(ptr + time_off);
                        }
                        break;
                    }
                }
            } else {
                pt.time = 0.0f;
            }
        }
        p_pre->process(pl_orig, processed);
    } else if (p_pre->lidar_type == OUST64) {
        // Ouster
        pcl::PointCloud<ouster_ros::Point> pl_orig;
        pl_orig.resize(num_points);
        for (int i = 0; i < num_points; ++i) {
            const uint8_t* ptr = msg->data.data() + i * msg->point_step;
            auto& pt = pl_orig.points[i];
            pt.x = *reinterpret_cast<const float*>(ptr + x_off);
            pt.y = *reinterpret_cast<const float*>(ptr + y_off);
            pt.z = *reinterpret_cast<const float*>(ptr + z_off);
            pt.intensity = int_off >= 0 ? *reinterpret_cast<const float*>(ptr + int_off) : 0.0f;
            pt.ring = ring_off >= 0 ? *reinterpret_cast<const uint8_t*>(ptr + ring_off) : 0;
            pt.t = time_off >= 0 ? *reinterpret_cast<const uint32_t*>(ptr + time_off) : 0;
            pt.reflectivity = 0;
            pt.ambient = 0;
            pt.range = 0;
        }
        p_pre->process(pl_orig, processed);
    } else {
        // LIVOX (livox_ros format via LCM PointCloud2)
        pcl::PointCloud<livox_ros::Point> pl_orig;
        pl_orig.resize(num_points);
        for (int i = 0; i < num_points; ++i) {
            const uint8_t* ptr = msg->data.data() + i * msg->point_step;
            auto& pt = pl_orig.points[i];
            pt.x = *reinterpret_cast<const float*>(ptr + x_off);
            pt.y = *reinterpret_cast<const float*>(ptr + y_off);
            pt.z = *reinterpret_cast<const float*>(ptr + z_off);
            pt.intensity = int_off >= 0 ? *reinterpret_cast<const float*>(ptr + int_off) : 0.0f;
            pt.tag = tag_off >= 0 ? *reinterpret_cast<const uint8_t*>(ptr + tag_off) : 0;
            pt.line = ring_off >= 0 ? *reinterpret_cast<const uint8_t*>(ptr + ring_off) : 0;
            pt.reflectivity = 0;
            pt.offset_time = time_off >= 0 ? *reinterpret_cast<const uint32_t*>(ptr + time_off) : 0;
        }
        p_pre->process(pl_orig, processed);
    }

    if (processed->empty()) return;

    lock_guard<mutex> lock(mtx_buffer);
    if (timestamp < last_timestamp_lidar) {
        fprintf(stderr, "[better_fastlio2] lidar loop detected, clearing buffer\n");
        lidar_buffer.clear();
        time_buffer.clear();
    }
    last_timestamp_lidar = timestamp;
    lidar_buffer.push_back(processed);
    time_buffer.push_back(timestamp);
    sig_buffer.notify_all();
}

static void on_imu(const lcm::ReceiveBuffer* /*rbuf*/, const string& /*channel*/,
                   const sensor_msgs::Imu* msg) {
    if (!g_running.load()) return;

    double timestamp = msg->header.stamp.sec + msg->header.stamp.nsec * 1e-9;

    auto imu = make_shared<ImuData>();
    imu->timestamp = timestamp;
    imu->acc = V3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    imu->gyro = V3D(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

    lock_guard<mutex> lock(mtx_buffer);
    if (timestamp < last_timestamp_imu) {
        fprintf(stderr, "[better_fastlio2] imu loop detected, clearing buffer\n");
        imu_buffer.clear();
    }
    last_timestamp_imu = timestamp;
    imu_buffer.push_back(imu);
    sig_buffer.notify_all();
}

// ============================================================================
// detectLoopClosureDistance (from upstream laserMapping.cpp:815-850)
// ============================================================================
static pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses(new pcl::KdTreeFLANN<PointType>());

static bool detectLoopClosureDistance(int* latestID, int* closestID) {
    int loopKeyCur = copy_cloudKeyPoses3D->size() - 1;
    int loopKeyPre = -1;

    auto it = loopIndexContainer.find(loopKeyCur);
    if (it != loopIndexContainer.end()) return false;

    std::vector<int> pointSearchIndLoop;
    std::vector<float> pointSearchSqDisLoop;
    kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses3D);
    kdtreeHistoryKeyPoses->radiusSearch(copy_cloudKeyPoses3D->back(),
        historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);

    for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i) {
        int id = pointSearchIndLoop[i];
        if (abs(copy_cloudKeyPoses6D->points[id].time - lidar_end_time) > historyKeyframeSearchTimeDiff) {
            loopKeyPre = id;
            break;
        }
    }

    if (loopKeyPre == -1 || loopKeyCur == loopKeyPre) return false;
    *latestID = loopKeyCur;
    *closestID = loopKeyPre;
    return true;
}

// ============================================================================
// loopFindNearKeyframes (from upstream laserMapping.cpp:856-883)
// ============================================================================
static void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr& nearKeyframes,
                                   const int& key, const int& searchNum) {
    nearKeyframes->clear();
    int cloudSize = copy_cloudKeyPoses6D->size();
    for (int i = -searchNum; i <= searchNum; ++i) {
        int keyNear = key + i;
        if (keyNear < 0 || keyNear >= cloudSize) continue;
        if (i == 0) {
            *nearKeyframes += *surfCloudKeyFrames[keyNear];
        } else {
            Eigen::Affine3f keyTrans = pcl::getTransformation(
                copy_cloudKeyPoses6D->points[key].x, copy_cloudKeyPoses6D->points[key].y,
                copy_cloudKeyPoses6D->points[key].z, copy_cloudKeyPoses6D->points[key].roll,
                copy_cloudKeyPoses6D->points[key].pitch, copy_cloudKeyPoses6D->points[key].yaw);
            Eigen::Affine3f keyNearTrans = pcl::getTransformation(
                copy_cloudKeyPoses6D->points[keyNear].x, copy_cloudKeyPoses6D->points[keyNear].y,
                copy_cloudKeyPoses6D->points[keyNear].z, copy_cloudKeyPoses6D->points[keyNear].roll,
                copy_cloudKeyPoses6D->points[keyNear].pitch, copy_cloudKeyPoses6D->points[keyNear].yaw);
            Eigen::Affine3f finalTrans = keyTrans.inverse() * keyNearTrans;
            pcl::PointCloud<PointType>::Ptr tmp(new pcl::PointCloud<PointType>());
            int tmpSize = surfCloudKeyFrames[keyNear]->size();
            tmp->resize(tmpSize);
            for (int j = 0; j < tmpSize; ++j) {
                const auto& p = surfCloudKeyFrames[keyNear]->points[j];
                tmp->points[j].x = finalTrans(0,0)*p.x + finalTrans(0,1)*p.y + finalTrans(0,2)*p.z + finalTrans(0,3);
                tmp->points[j].y = finalTrans(1,0)*p.x + finalTrans(1,1)*p.y + finalTrans(1,2)*p.z + finalTrans(1,3);
                tmp->points[j].z = finalTrans(2,0)*p.x + finalTrans(2,1)*p.y + finalTrans(2,2)*p.z + finalTrans(2,3);
                tmp->points[j].intensity = p.intensity;
            }
            *nearKeyframes += *tmp;
        }
    }
    if (nearKeyframes->empty()) return;
}

// ============================================================================
// performLoopClosure (from upstream laserMapping.cpp:890-1018)
// ============================================================================
static void performLoopClosure() {
    if (cloudKeyPoses3D->points.empty()) return;

    // Thread-safe copy (matches upstream:900-903)
    mtx.lock();
    *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
    *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
    mtx.unlock();

    int loopKeyCur, loopKeyPre;

    // Distance-based loop detection (matches upstream:909)
    if (detectLoopClosureDistance(&loopKeyCur, &loopKeyPre) == false) return;

    printf("[better_fastlio2] [Nearest Pose found] curKeyFrame: %d loopKeyFrame: %d\n",
           loopKeyCur, loopKeyPre);

    // Build multi-frame submaps (matches upstream:916-929)
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
    loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, historyKeyframeSearchNum);
    loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum);

    // Scan Context verification (matches upstream:932-943)
    Eigen::MatrixXd cureKeyframeSC = scLoop.makeScancontext(*cureKeyframeCloud);
    Eigen::MatrixXd prevKeyframeSC = scLoop.makeScancontext(*prevKeyframeCloud);
    std::pair<double, int> simScore = scLoop.distanceBtnScanContext(cureKeyframeSC, prevKeyframeSC);
    double dist = simScore.first;
    int align = simScore.second;
    if (dist > scLoop.SC_DIST_THRES) {
        printf("[better_fastlio2] but they can not be detected by SC.\n");
        return;
    }
    printf("[better_fastlio2] [SC Loop found] curKeyFrame: %d loopKeyFrame: %d distance: %.3f nn_align: %.1f deg.\n",
           loopKeyCur, loopKeyPre, dist, align * scLoop.PC_UNIT_SECTORANGLE);

    // ICP (matches upstream:947-974)
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(200);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    // Apply SC yaw alignment (matches upstream:954-962)
    float com_yaw = align * scLoop.PC_UNIT_SECTORANGLE;
    PointTypePose com;
    com.x = 0.0; com.y = 0.0; com.z = 0.0;
    com.yaw = -com_yaw; com.pitch = 0.0; com.roll = 0.0;
    cureKeyframeCloud = transformPointCloud(cureKeyframeCloud, &com);

    icp.setInputSource(cureKeyframeCloud);
    icp.setInputTarget(prevKeyframeCloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);

    if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore) {
        printf("[better_fastlio2] but they can not be registered by ICP. icpFitnessScore: %.3f\n",
               icp.getFitnessScore());
        return;
    }
    printf("[better_fastlio2] [ICP Registration success] curKeyFrame: %d loopKeyFrame: %d icpFitnessScore: %.3f\n",
           loopKeyCur, loopKeyPre, icp.getFitnessScore());

    // Compute corrected pose (matches upstream:990-1001)
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();
    Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
    Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;
    pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);

    gtsam::Pose3 poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw),
                                          gtsam::Point3(x, y, z));
    gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);

    // Noise from ICP score (matches upstream:1004-1007)
    gtsam::Vector Vector6(6);
    float noiseScore = icp.getFitnessScore();
    Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
    gtsam::noiseModel::Diagonal::shared_ptr constraintNoise =
        gtsam::noiseModel::Diagonal::Variances(Vector6);

    // Queue loop factor (matches upstream:1011-1017)
    mtx.lock();
    loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
    loopPoseQueue.push_back(poseFrom.between(poseTo));
    loopNoiseQueue.push_back(constraintNoise);
    loopIndexContainer[loopKeyCur] = loopKeyPre;
    mtx.unlock();
}

// ============================================================================
// loopClosureThread (from upstream laserMapping.cpp:1021-1038)
// ============================================================================
static void loopClosureThread() {
    if (loopClosureEnableFlag == false) {
        printf("[better_fastlio2] loopClosureEnableFlag == false\n");
        return;
    }

    auto sleep_time = chrono::milliseconds(static_cast<int>(1000.0 / loopClosureFrequency));
    while (g_running.load()) {
        this_thread::sleep_for(sleep_time);
        performLoopClosure();
    }
}

// ============================================================================
// Signal handling
// ============================================================================
static void signal_handler(int /*sig*/) {
    g_running.store(false);
}

// ============================================================================
// Main
// ============================================================================
int main(int argc, char** argv) {
    dimos::NativeModule mod(argc, argv);

    // Required topics
    g_lidar_topic = mod.has("lidar") ? mod.topic("lidar") : "";
    g_imu_topic = mod.has("imu") ? mod.topic("imu") : "";
    g_odom_topic = mod.has("odometry") ? mod.topic("odometry") : "";
    g_scan_topic = mod.has("registered_scan") ? mod.topic("registered_scan") : "";
    g_map_topic = mod.has("global_map") ? mod.topic("global_map") : "";
    g_path_topic = mod.has("corrected_path") ? mod.topic("corrected_path") : "";

    if (g_lidar_topic.empty() || g_imu_topic.empty()) {
        fprintf(stderr, "Error: --lidar and --imu topics are required\n");
        return 1;
    }

    // Config from CLI args
    p_pre->lidar_type = mod.arg_int("lidar_type", 2);
    p_pre->N_SCANS = mod.arg_int("scan_line", 16);
    p_pre->blind = mod.arg_float("blind", 2.0);
    p_pre->feature_enabled = mod.arg("feature_enabled", "false") == "true";
    p_pre->point_filter_num = mod.arg_int("point_filter_num", 4);
    p_pre->SCAN_RATE = mod.arg_int("scan_rate", 10);
    p_pre->time_unit = mod.arg_int("time_unit", US);

    acc_cov = mod.arg_float("acc_cov", 0.1);
    gyr_cov = mod.arg_float("gyr_cov", 0.1);
    b_acc_cov = mod.arg_float("b_acc_cov", 0.0001);
    b_gyr_cov = mod.arg_float("b_gyr_cov", 0.0001);
    NUM_MAX_ITERATIONS = mod.arg_int("max_iteration", 3);
    extrinsic_est_en = mod.arg("extrinsic_est_en", "false") == "true";
    DET_RANGE = mod.arg_float("det_range", 100.0);
    fov_deg = mod.arg_float("fov_degree", 180.0);
    cube_len = mod.arg_float("cube_len", 1500.0);
    filter_size_map_min = mod.arg_float("filter_size_map_min", 0.2);
    keyframeAddingDistThreshold = mod.arg_float("keyframe_dist_threshold", 1.0);
    keyframeAddingAngleThreshold = mod.arg_float("keyframe_angle_threshold", 0.2);
    kd_step = mod.arg_int("kd_step", 30);
    recontructKdTree = mod.arg("reconstruct_kdtree", "false") == "true";
    dense_pub_en = mod.arg("dense_publish_en", "false") == "true";
    publish_map_frequency = mod.arg_float("publish_map_frequency", 0.0);

    // Loop closure config
    loopClosureEnableFlag = mod.arg("loop_closure_enable", "false") == "true";
    loopClosureFrequency = mod.arg_float("loop_closure_frequency", 1.0);
    historyKeyframeSearchRadius = mod.arg_float("history_keyframe_search_radius", 10.0);
    historyKeyframeSearchTimeDiff = mod.arg_float("history_keyframe_search_time_diff", 30.0);
    historyKeyframeSearchNum = mod.arg_int("history_keyframe_search_num", 2);
    historyKeyframeFitnessScore = mod.arg_float("history_keyframe_fitness_score", 0.2);

    // Dynamic removal
    dynamic_removal_enable = mod.arg("dynamic_removal_enable", "false") == "true";

    // Segment
    ground_en = mod.arg("ground_en", "false") == "true";
    tollerance_en = mod.arg("tollerance_en", "false") == "true";
    sensor_height = mod.arg_float("sensor_height", 1.5);
    SENSOR_HEIGHT = sensor_height;  // Wire to T-GRS dynamic removal
    z_tollerance = mod.arg_float("z_tolerance", 2.0);
    rotation_tollerance = mod.arg_float("rotation_tolerance", 0.2);

    // Extrinsics (parse comma-separated list)
    string ext_t_str = mod.arg("extrinsic_T", "0,0,0");
    string ext_r_str = mod.arg("extrinsic_R", "1,0,0,0,1,0,0,0,1");
    // Parse extrinsic_T
    {
        stringstream ss(ext_t_str);
        string item;
        int idx = 0;
        while (getline(ss, item, ',') && idx < 3) {
            extrinT[idx++] = stod(item);
        }
    }
    // Parse extrinsic_R
    {
        stringstream ss(ext_r_str);
        string item;
        int idx = 0;
        while (getline(ss, item, ',') && idx < 9) {
            extrinR[idx++] = stod(item);
        }
    }

    printf("[better_fastlio2] Starting enhanced FAST-LIO2 module\n");
    printf("[better_fastlio2] lidar: %s | imu: %s | odom: %s\n",
           g_lidar_topic.c_str(), g_imu_topic.c_str(), g_odom_topic.c_str());
    printf("[better_fastlio2] lidar_type=%d scan_line=%d blind=%.1f\n",
           p_pre->lidar_type, p_pre->N_SCANS, p_pre->blind);
    printf("[better_fastlio2] loop_closure=%s dynamic_removal=%s\n",
           loopClosureEnableFlag ? "on" : "off",
           dynamic_removal_enable ? "on" : "off");

    // Signal handlers
    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);

    // Init LCM
    lcm::LCM lcm;
    if (!lcm.good()) {
        fprintf(stderr, "Error: LCM init failed\n");
        return 1;
    }
    g_lcm = &lcm;

    // Subscribe to inputs
    lcm::LCM::HandlerFunction<sensor_msgs::PointCloud2> lidar_handler = on_lidar;
    lcm::LCM::HandlerFunction<sensor_msgs::Imu> imu_handler = on_imu;
    lcm.subscribe(g_lidar_topic, lidar_handler);
    lcm.subscribe(g_imu_topic, imu_handler);

    // Init GTSAM
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam = new gtsam::ISAM2(parameters);

    // Init EKF
    FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    HALF_FOV_COS = cos((FOV_DEG) * 0.5 * M_PI / 180.0);

    _featsArray.reset(new pcl::PointCloud<PointType>());
    std::fill(point_selected_surf, point_selected_surf + 100000, true);
    std::fill(res_last, res_last + 100000, -1000.0f);

    float mappingSurfLeafSize = filter_size_map_min;
    downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
    downSizeFilterICP.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);

    // Set IMU/lidar extrinsics
    Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);
    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    // Init EKF with process/measurement models
    double epsi[23] = {0.001};
    fill(epsi, epsi + 23, 0.001);
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

    // Start loop closure thread
    thread loopthread(loopClosureThread);

    printf("[better_fastlio2] Initialized. Waiting for data...\n");

    // Main processing loop
    auto last_map_publish = chrono::steady_clock::now();

    while (g_running.load()) {
        lcm.handleTimeout(10);

        {
            lock_guard<mutex> lock(mtx_buffer);
            if (!sync_packages(Measures)) continue;
        }

        // First scan
        if (flg_first_scan) {
            first_lidar_time = Measures.lidar_beg_time;
            p_imu->first_lidar_time = first_lidar_time;
            flg_first_scan = false;
            continue;
        }

        // IMU processing: forward propagation + undistortion
        p_imu->Process(Measures, kf, feats_undistort);
        state_point = kf.get_x();
        pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

        if (feats_undistort->empty()) {
            fprintf(stderr, "[better_fastlio2] No undistorted points, skip\n");
            continue;
        }

        // Dynamic object removal (T-GRS 2024)
        if (dynamic_removal_enable && feats_undistort->points.size() > 0) {
            SSC ssc_cur(feats_undistort, 0);
            remover.cluster(ssc_cur.apri_vec, ssc_cur.hash_cloud, ssc_cur.cluster_vox);
            remover.recognizePD(ssc_cur);
            // Replace undistorted cloud with non-dynamic points
            feats_undistort->points.clear();
            *feats_undistort += *ssc_cur.cloud_nd;
        }

        flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? false : true;

        // FOV segmentation
        lasermap_fov_segment();

        // Downsample
        downSizeFilterSurf.setInputCloud(feats_undistort);
        downSizeFilterSurf.filter(*feats_down_body);
        feats_down_size = feats_down_body->points.size();

        // Initialize iKdtree
        if (ikdtree.Root_Node == nullptr) {
            if (feats_down_size > 5) {
                ikdtree.set_downsample_param(filter_size_map_min);
                feats_down_world->resize(feats_down_size);
                for (int i = 0; i < feats_down_size; i++) {
                    pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                }
                ikdtree.Build(feats_down_world->points);
            }
            continue;
        }

        if (feats_down_size < 5) continue;

        // Prepare for ICP
        normvec->resize(feats_down_size);
        feats_down_world->resize(feats_down_size);
        pointSearchInd_surf.resize(feats_down_size);
        Nearest_Points.resize(feats_down_size);

        // Iterated EKF update
        double solve_H_time = 0;
        kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
        state_point = kf.get_x();
        euler_cur = SO3ToEuler(state_point.rot);
        pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
        geoQuat.x() = state_point.rot.coeffs()[0];
        geoQuat.y() = state_point.rot.coeffs()[1];
        geoQuat.z() = state_point.rot.coeffs()[2];
        geoQuat.w() = state_point.rot.coeffs()[3];

        // Update lidar_end_time for loop closure thread (matches upstream global)
        lidar_end_time = Measures.lidar_end_time;

        // Backend: keyframes + factor graph
        getCurPose(state_point);
        saveKeyFramesAndFactor();
        correctPoses();

        // Publish odometry
        publish_odometry(Measures.lidar_end_time);

        // Map incremental update
        map_incremental();

        // Publish registered scan
        publish_registered_scan(Measures.lidar_end_time);

        // Periodic global map publish
        if (publish_map_frequency > 0) {
            auto now = chrono::steady_clock::now();
            double elapsed = chrono::duration<double>(now - last_map_publish).count();
            if (elapsed >= 1.0 / publish_map_frequency) {
                publish_global_map(Measures.lidar_end_time);
                last_map_publish = now;
            }
        }

        // iKdtree reconstruction is now handled inside saveKeyFramesAndFactor()
        // via recontructIKdTree() matching upstream's approach
    }

    // Cleanup
    printf("[better_fastlio2] Shutting down...\n");
    g_running.store(false);
    loopthread.join();
    delete isam;
    g_lcm = nullptr;

    printf("[better_fastlio2] Done.\n");
    return 0;
}
