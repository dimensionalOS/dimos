//
// Created by shibo zhao on 2020-09-27.
// Ported from ROS2 to plain C++ for DimOS (LCM-based)
//

#ifndef arise_slam_mid360_LASERMAPPING_H
#define arise_slam_mid360_LASERMAPPING_H

#include <cmath>
#include <iostream>
#include <queue>
#include <string>
#include <vector>
#include <iomanip>
#include <mutex>
#include <thread>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <functional>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include "arise_slam_mid360/LidarProcess/LocalMap.h"
#include "arise_slam_mid360/common.h"
#include "arise_slam_mid360/tic_toc.h"
#include "arise_slam_mid360/utils/Twist.h"
#include "arise_slam_mid360/container/MapRingBuffer.h"
#include "arise_slam_mid360/config/parameter.h"
#include "arise_slam_mid360/LidarProcess/LidarSlam.h"


namespace arise_slam {

    struct laser_mapping_config{
        float period;
        float lineRes;
        float planeRes;
        int max_iterations;
        bool debug_view_enabled;
        bool enable_ouster_data;
        bool publish_only_feature_points;
        bool use_imu_roll_pitch;
        int max_surface_features;
        double velocity_failure_threshold;
        bool auto_voxel_size;
        bool forget_far_chunks;
        float visual_confidence_factor;
        float pos_degeneracy_threshold;
        float ori_degeneracy_threshold;
        float shift_avg_ratio;
        bool shift_undistortion;
        float yaw_ratio;
        std::string relocalization_map_path;
        bool local_mode;
        float init_x;
        float init_y;
        float init_z;
        float init_roll;
        float init_pitch;
        float init_yaw;
        bool read_pose_file;
        bool trust_fallback_odom;  // Override SLAM position with fallback odom (for sim)
    };

    struct OdometryData {
        double timestamp;
        double duration;
        double x, y, z;
        double roll, pitch, yaw;
    };

    // Plain C++ odometry data used in place of nav_msgs::msg::Odometry
    struct OdomMsg {
        double timestamp;
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;
        // covariance[0] used for degenerate flag, covariance[1] for init stat
        double covariance[36] = {};
    };

    using OdomMsgPtr = std::shared_ptr<OdomMsg>;

    // Callback type for publishing odometry results
    struct LaserMappingOutput {
        double timestamp;
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;
        Eigen::Vector3d velocity;
        bool is_degenerate;
        pcl::PointCloud<PointType>::Ptr registered_scan;

        // Diagnostic data — upstream: laserMapping.cpp publishTopic() lines 927-1111
        pcl::PointCloud<PointType>::Ptr surround_map;   // local map around robot (5x5 chunks)
        pcl::PointCloud<PointType>::Ptr global_map;     // full accumulated map
        Eigen::Vector3d incremental_position = Eigen::Vector3d::Zero();
        Eigen::Quaterniond incremental_orientation = Eigen::Quaterniond::Identity();
        int prediction_source = 0;  // 0=IMU_ORIENTATION, 1=IMU_ODOM, 2=VISUAL_ODOM
        // Stats from optimization
        LidarSLAM::OptimizationStats stats;
    };

    class laserMapping {

    public:
        laserMapping();

        // Initialize parameters and algorithm state (replaces on_configure + on_activate)
        void init(const laser_mapping_config& config);

        void
        initializationParam();

        void
        preprocessDualLidarFeatures(Transformd current_pose, SensorType sensor_type);

        void
        adjustVoxelSize(int &laserCloudCornerStackNum, int &laserCloudSurfStackNum);

        void
        mappingOptimization(Eigen::Vector3i &postion_in_locamap, Transformd start_tf, Eigen::Quaterniond roll_pitch_quat,
                            const int laserCloudCornerStackNum, const int laserCloudSurfStackNum);

        void
        transformAssociateToMap(Transformd T_w_pre, Transformd T_wodom_curr, Transformd T_wodom_pre);

        void
        transformAssociateToMap();

        void
        transformUpdate();

        void
        pointAssociateToMap(PointType const *const pi, PointType *const po);

        void
        pointAssociateToMap(pcl::PointXYZHSV const *const pi, pcl::PointXYZHSV *const po);

        void
        pointAssociateTobeMapped(PointType const *const pi, PointType *const po);

        // Data input methods (replace ROS subscriber callbacks)
        void
        addLaserFeature(pcl::PointCloud<PointType>::Ptr cornerCloud,
                        pcl::PointCloud<PointType>::Ptr surfCloud,
                        pcl::PointCloud<PointType>::Ptr fullCloud,
                        pcl::PointCloud<PointType>::Ptr realsenseCloud,
                        const Eigen::Quaterniond& imuPrediction,
                        double timestamp);

        void
        addIMUOdometry(double timestamp, const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation);

        void
        addVisualOdometry(double timestamp, const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
                          const double* covariance = nullptr);

        void
        extractIMUOdometry(double timeLaserFrame, Transformd &T_w_lidar);

        bool
        extractVisualIMUOdometryAndCheck(Transformd &T_w_lidar);

        void
        getOdometryFromTimestamp(MapRingBuffer<OdomMsgPtr> &buf, const double &timestamp,
                                 Eigen::Vector3d &T, Eigen::Quaterniond &Q);

        void
        extractRelativeTransform(MapRingBuffer<OdomMsgPtr> &buf, Transformd &T_pre_cur, bool imu_prediction);

        void
        setInitialGuess();

        void
        selectposePrediction();

        void
        process();

        bool
        readPointCloud();

        bool
        loadMapFromFile(const std::string& map_path);

        void
        saveLocalizationPose(double timestamp,Transformd &T_w_lidar, const std::string& parentPath);

        void
        readLocalizationPose(const std::string& parentPath);

        void
        save_debug_statistic(const std::string file_name);

        // Get the latest output after process()
        LaserMappingOutput getLatestOutput() const;

        // Diagnostic getters — upstream: laserMapping.cpp publishTopic() lines 927-1111
        pcl::PointCloud<PointType> getSurroundMap() const;   // slam.localMap.get5x5LocalMap()
        pcl::PointCloud<PointType> getGlobalMap() const;     // slam.localMap.getAllLocalMap()
        int getPredictionSource() const;

        // Set callback for output (optional, alternative to polling)
        void setOutputCallback(std::function<void(const LaserMappingOutput&)> callback) {
            output_callback_ = callback;
        }

    private:
        static constexpr float vision_laser_time_offset = 0.0;
        static constexpr int laserCloudCenWidth = 10;
        static constexpr int laserCloudCenHeight = 10;
        static constexpr int laserCloudCenDepth = 5;
        static constexpr int laserCloudWidth = 21;
        static constexpr int laserCloudHeight = 21;
        static constexpr int laserCloudDepth = 11;
        static constexpr int laserCloudNum =laserCloudWidth * laserCloudHeight * laserCloudDepth; // 4851

        MapRingBuffer<OdomMsgPtr> imu_odom_buf;
        MapRingBuffer<OdomMsgPtr> visual_odom_buf;

        int frameCount = 0;
        int waiting_takeoff_timeout = 300;
        int startupCount = 10;
        int localizationCount = 0;
        int laserCloudValidInd[125];
        int laserCloudSurroundInd[125];

        double timeLaserCloudCornerLast = 0;
        double timeLaserCloudSurfLast = 0;
        double timeLaserCloudFullRes = 0;
        double timeLaserOdometry = 0;
        double timeLaserOdometryPrev = 0;
        double timeLatestImuOdometry = 0.0;


        bool got_previous_map = false;
        bool force_initial_guess = false;
        bool odomAvailable = false;
        bool lastOdomAvailable = false;
        bool laser_imu_sync = false;
        bool use_imu_roll_pitch_this_step = false;
        bool initialization = false;
        bool imuodomAvailable = false;
        bool imuorientationAvailable = false;
        bool lastimuodomAvaliable=false;
        bool imu_initialized = false;

        float poseX = 0;
        float poseY = 0;
        float poseZ = 0;

        float shiftX = 0;
        float shiftY = 0;
        float shiftZ = 0;

        pcl::VoxelGrid<PointType> downSizeFilterCorner;
        pcl::VoxelGrid<PointType> downSizeFilterSurf;

        // Buffers (using PCL point clouds directly instead of ROS messages)
        struct LaserFeatureData {
            pcl::PointCloud<PointType>::Ptr corner;
            pcl::PointCloud<PointType>::Ptr surf;
            pcl::PointCloud<PointType>::Ptr fullRes;
            pcl::PointCloud<PointType>::Ptr realsense;
            Eigen::Quaterniond imuPrediction;
            double timestamp;
        };
        std::queue<LaserFeatureData> featureBuf;

        SensorType last_sensor_type_= SensorType::VELODYNE;

        // variables for stacking 2 scans
        std::queue<pcl::PointCloud<PointType>> cornerDualScanBuf;
        std::queue<pcl::PointCloud<PointType>> surfDualScanBuf;
        std::queue<pcl::PointCloud<pcl::PointXYZHSV>> fullResDualScanBuf;
        std::queue<Transformd> scanTransformBuf;
        std::queue<SensorType> sensorTypeBuf;

        pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
        pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;
        pcl::PointCloud<PointType>::Ptr laserCloudRealsense;

        pcl::PointCloud<PointType>::Ptr laserCloudSurround;
        pcl::PointCloud<PointType>::Ptr laserCloudFullRes;
        pcl::PointCloud<PointType>::Ptr laserCloudFullRes_rot;
        pcl::PointCloud<PointType>::Ptr laserCloudRawRes;
        pcl::PointCloud<PointType>::Ptr laserCloudCornerStack;
        pcl::PointCloud<PointType>::Ptr laserCloudSurfStack;
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudRawWithFeatures;
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr velodyneLaserCloudRawWithFeatures;
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr ousterLaserCloudRawWithFeatures;

        pcl::PointCloud<PointType>::Ptr laserCloudPriorOrg;
        pcl::PointCloud<PointType>::Ptr laserCloudPrior;

        Transformd T_w_lidar;
        Transformd last_T_w_lidar;
        Transformd last_imu_T;
        Transformd total_incremental_T;
        Transformd laser_incremental_T;
        Transformd forcedInitialGuess;

        Eigen::Quaterniond q_wmap_wodom;
        Eigen::Vector3d t_wmap_wodom;
        Eigen::Quaterniond q_wodom_curr;
        Eigen::Vector3d t_wodom_curr;
        Eigen::Quaterniond q_wodom_pre;
        Eigen::Vector3d t_wodom_pre;
        Eigen::Quaterniond q_w_imu_pre;
        Eigen::Vector3d t_w_imu_pre;

        LidarSLAM slam;
        laser_mapping_config config_;
        std::mutex mBuf;
        PointType pointOri, pointSel;

        enum class PredictionSource {IMU_ORIENTATION, IMU_ODOM, VISUAL_ODOM};
        PredictionSource prediction_source;
        std::vector<arise_slam::OdometryData> odometryResults;

        // Output callback
        std::function<void(const LaserMappingOutput&)> output_callback_;
        LaserMappingOutput latest_output_;

    }; // class laserMapping

} // namespace arise_slam
#endif //arise_slam_mid360_LASERMAPPING_H
