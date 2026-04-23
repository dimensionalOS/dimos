//
// Created by ubuntu on 2020/9/26.
//

#ifndef LIDARSLAM_H
#define LIDARSLAM_H

#include <atomic>

#include <tbb/concurrent_vector.h>

#include <Eigen/Geometry>

#include "arise_slam_mid360/sensor_data/pointcloud/LidarPoint.h"
#include "arise_slam_mid360/LidarProcess/LocalMap.h"
#include "arise_slam_mid360/LidarProcess/MotionModel.h"
#include "arise_slam_mid360/LidarProcess/PointCloudStorage.h"
#include "arise_slam_mid360/LidarProcess/RollingGrid.h"
#include "arise_slam_mid360/utils/EigenTypes.h"
#include "arise_slam_mid360/utils/Twist.h"
#include "arise_slam_mid360/FeatureExtraction/LidarKeypointExtractor.h"
#include <sophus/se3.hpp>
#include "arise_slam_mid360/LidarProcess/Utilities.h"
#include "arise_slam_mid360/LidarProcess/factor/SE3AbsolutatePoseFactor.h"
#include "arise_slam_mid360/LidarProcess/factor/ceresCostFunction.h"
#include "arise_slam_mid360/LidarProcess/factor/lidarOptimization.h"
#include "arise_slam_mid360/LidarProcess/factor/pose_local_parameterization.h"
#include <ceres/ceres.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>
#include <glog/logging.h>
#include "arise_slam_mid360/tic_toc.h"
#include "arise_slam_mid360/config/parameter.h"

namespace arise_slam {

    class LidarSLAM {


    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        using Point = pcl::PointXYZI;
        using Point2 = PointXYZTIId;
        using PointCloud = pcl::PointCloud<Point>;
        enum class PredictionSource {
            IMU_ORIENTATION, IMU_ODOM, VISUAL_ODOM
        };


    public:

        enum class MatchingMode {
            EGO_MOTION = 0, LOCALIZATION = 1
        };

        enum UndistortionMode {
            //! No undistortion is performed:
            //! -End scan pose is optimized using rigid registration of raw scan and map.
            //! -Raw input scan is added to maps.
            NONE = 0,

            //! Minimal undistortion is performed:
            //! - begin scan pose is linearly interpolated between previous and current end scan poses.
            //! - End scan pose is optimized using rigid registration of undistorted scan and map.
            //! - Scan is linearly undistorted between begin and end scan poses.
            APPROXIMATED = 1,

            //! Ceres-optimized undistorted is performed:
            //! -both begin and end scan are optimized using registration of undistorted scan
            //! and map.
            //! -Scan is linearly undistorted between begin and scan poses.
            OPTIMIZED = 2
        };

        //! Result of keypoint matching, explaining rejection
        enum MatchingResult : uint8_t {
            SUCCESS = 0,               // keypoint has been successfully matched
            NOT_ENOUGH_NEIGHBORS = 1,  // not enough neighbors to match keypoint
            NEIGHBORS_TOO_FAR = 2,     // neighbors are too far to match keypoint
            BAD_PCA_STRUCTURE = 3,     // PCA eigenvalues analysis discards neighborhood fit to model
            INVAVLID_NUMERICAL = 4,    // optimization parameter computation has numerical invalidity
            MSE_TOO_LARGE = 5,         // mean squared error to model is too large to accept fitted model
            UNKNON = 6,                // unkown status (matching not performed yet)
            nRejectionCauses = 7
        };

        enum Feature_observability : uint8_t {
            rx_cross = 0,               // evaluate for x rotation estimation
            neg_rx_cross = 1,           // evaluate for neg x  rotation estimation
            ry_cross = 2,               // evaluate for y rotation estimation
            neg_ry_cross = 3,           // evaluate for neg y rotation estimation
            rz_cross = 4,               // evaluate for z rotation estimation
            neg_rz_cross = 5,           // evaluate for neg z rotation estimation
            tx_dot = 6,                 // evaluate for x translation
            ty_dot = 7,                 // evaluate for y translation
            tz_dot = 8,                    // evaluate for z translation
            nFeatureObs = 9
        };

        enum FeatureType : uint8_t {
            EdgeFeature = 0,
            PlaneFeature = 1,
        };


    public:

        struct LaserOptSet {
            Eigen::Quaterniond imu_roll_pitch;
            bool  debug_view_enabled;
            bool  use_imu_roll_pitch;
            float velocity_failure_threshold;
            float yaw_ratio;
            int max_surface_features;
        };

        struct OptimizationStats {
            int edge_features_from_map = 0;
            int plane_features_from_map = 0;
            int edge_features_from_scan = 0;
            int plane_features_from_scan = 0;
            double translation_norm = 0.0;
            double rotation_norm = 0.0;
            // 6-DoF uncertainty
            double uncertainty_x = 0.0, uncertainty_y = 0.0, uncertainty_z = 0.0;
            double uncertainty_roll = 0.0, uncertainty_pitch = 0.0, uncertainty_yaw = 0.0;
            double average_distance = 0.0;
        };

        //! Estimation of registration error
        struct RegistrationError {
            // Estimation of the maximum position error
            double PositionError = 0.;
            double PositionUncertainty = 0.;
            double MaxPositionError = 0.1;
            double PosInverseConditionNum = 1.0;

            //Estimation of Lidar Uncertainty
            Eigen::VectorXf LidarUncertainty;

            // Direction of the maximum position error
            Eigen::Vector3d PositionErrorDirection = Eigen::Vector3d::Zero();

            // Estimation of the maximum orientation error (in radians)
            double OrientationError = 0.;
            double OrientationUncertainty = 0.;
            double MaxOrientationError = 10;
            double OriInverseConditionNum = 1.0;
            // Direction of the maximum orientation error
            Eigen::Vector3d OrientationErrorDirection = Eigen::Vector3d::Zero();

            // Covariance matrix encoding the estimation of the pose's errors about the 6-DoF parameters
            // (DoF order :  X, Y, Z,rX, rY, rZ)
            Eigen::Matrix<double, 6, 6, Eigen::RowMajor> Covariance = Eigen::Matrix<double, 6, 6, Eigen::RowMajor>::Zero();
        };

        struct eigenValue // Eigen Value ,lamada1 > lamada2 > lamada3;
        {
            double lamada1;
            double lamada2;
            double lamada3;
        };

        struct eigenVector //the eigen vector corresponding to the eigen value
        {
            Eigen::Vector3f principalDirection;
            Eigen::Vector3f middleDirection;
            Eigen::Vector3f normalDirection;
        };

        struct pcaFeature //PCA
        {
            eigenValue values;
            eigenVector vectors;
            double curvature;
            double linear;
            double planar;
            double spherical;
            double linear_2;
            double planar_2;
            double spherical_2;
            pcl::PointNormal pt;
            size_t ptId;
            size_t ptNum = 0;
            std::vector<int> neighbor_indices;
            std::array<int, 4> observability;
            // Add IMLS thought
            double rx_cross;
            double neg_rx_cross;
            double ry_cross;
            double neg_ry_cross;
            double rz_cross;
            double neg_rz_cross;
            double tx_dot;
            double ty_dot;
            double tz_dot;
        };

        struct kdtree_time {
            double timestamp;
            int frameID;
            double kd_tree_building_time;
            double kd_tree_query_time;
        };

        struct LidarOdomUncertainty {
            double uncertainty_x;
            double uncertainty_y;
            double uncertainty_z;
            double uncertainty_roll;
            double uncertainty_pitch;
            double uncertainty_yaw;
        };

        struct OptimizationParameter {
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            MatchingResult match_result;
            FeatureType feature_type;
            pcaFeature feature;
            Eigen::Matrix3d Avalue;
            Eigen::Vector3d Pvalue;
            Eigen::Vector3d Xvalue;
            Eigen::Vector3d NormDir;
            double negative_OA_dot_norm;
            std::pair<Eigen::Vector3d, Eigen::Vector3d> corres;
            double residualCoefficient;
            double TimeValue;
        };

    public:
        LocalMap localMap;
    
        OptimizationStats stats;
        // Variance-Covariance matrix that estimates the localization error about the
        // 6-DoF parameters (DoF order :  rX, rY, rZ, X ,Y ,Z)
        RegistrationError LocalizationUncertainty;
        LidarOdomUncertainty lidarOdomUncer;
        kdtree_time kdtree_time_analysis;
        LaserOptSet OptSet;

        Transformd T_w_lidar;
        Transformd last_T_w_lidar;
        Eigen::Matrix<double, 6, 1> Tworld = Eigen::Matrix<double, 6, 1>::Zero();
        Eigen::Vector3i pos_in_localmap;

        int frame_count;
        int laser_imu_sync;
        int startupCount = 0;

        float Pos_degeneracy_threshold;
        float Ori_degeneracy_threshold;
        float Visual_confidence_factor;

        std::string relocalization_map_path;
        float init_x;
        float init_y;
        float init_z;
        float init_roll;
        float init_pitch;
        float init_yaw;
        float local_mode;
        float update_map;
    
      

        // Pose at the begining of current frame
        Eigen::Isometry3d TworldFrameStart;
        // Transform interpolator to estimate the pose of the sensor within a lidar
        // frame, using poses at the beginning and end of frame.
        LinearTransformInterpolator<double> WithinFrameMotion;

        //---------------------------------------------------
        // current frame edge and planar feature
        PointCloud::Ptr EdgesPoints;
        PointCloud::Ptr PlanarsPoints;
        PointCloud::Ptr BlobsPoints;
        PointCloud::Ptr CurrentFrame;

        PointCloud::Ptr WorldEdgesPoints;
        PointCloud::Ptr WorldPlanarsPoints;
        PointCloud::Ptr WorldBlobsPointsPoints;
        PointCloud::Ptr WorldCurrentFrame;

        bool bInitilization = false;
        bool isDegenerate = false;
        // How the algorithm should undistort the lidar scans.
        // The undistortion should improve the accuracy, but the computation speed
        // may decrease, and the result might be unstable in difficult situations.
        UndistortionMode Undistortion = UndistortionMode::NONE;

        // ICP matching results of keypoints extracted from current input frame
        // (used for debug only)
        std::vector<MatchingResult> EdgePointRejectionLocalization;
        std::vector<MatchingResult> PlanarPointRejectionLocalization;
        std::vector<MatchingResult> BlobPointRejectionLocalization;

        // Histogram of the ICP matching rejection causes
        // (used mainly for debug)
        std::array<std::atomic_int, Feature_observability::nFeatureObs> PlaneFeatureHistogramObs;
        std::array<std::atomic_int, MatchingResult::nRejectionCauses> MatchRejectionHistogramLine;
        std::array<std::atomic_int, MatchingResult::nRejectionCauses> MatchRejectionHistogramPlane;
        std::array<std::atomic_int, MatchingResult::nRejectionCauses> MatchRejectionHistogramBlob;

        // To recover the ego-motion we have to minimize the function
        // f(R, T) = sum(d(point, line)^2) + sum(d(point, plane)^2). In
        // both case the distance between the point and the line / plane
        // can be writen (R*X+T -P).t *A * (R*X+T-P). Where X is the keypoint
        // P is a point on the line / plane. A = (n*n.t) for a plane with n
        // being the normal and A = (I - n*n.t)^2 for a line with n being
        // a director vector of the line
        // - Avalues will store the A matrix
        // - Pvalues will store the P points
        // - Xvalues will store the W points
        // - residualCoefficient will attenuate the distance function for outliers
        // - TimeValues store the time acquisition

        tbb::concurrent_vector<OptimizationParameter> OptimizationData;
        //Eigen::aligned_vector<OptimizationParameter> OptimizationData;
        Eigen::aligned_vector<Eigen::Mat33d> Avalues;
        Eigen::aligned_vector<Eigen::Mat33d> Pvalues;
        Eigen::aligned_vector<Eigen::Mat33d> Xvalues;
        std::vector<double> residualCoefficient;
        std::vector<double> TimeValues;

        //---------------------------------------------------
        // Optimization paramters
        //---------------------------------------------------


        // Maximum number of the iteration
        // in the localization optimization step
        size_t LocalizationLMMaxIter = 15;

        // During the Levenberg-Marquardt algorithm
        // keypoints will have to be match with planes
        // and lines of the previous frame. This parameter
        // indicates how many times we want to do the
        // ICP matching
        size_t LocalizationICPMaxIter = 4;

        // When computing the point<->line and point<->plane distance
        // in the ICP, the kNearest edges/planes points of the current
        // points are selected to approxmate the line/plane using a PCA
        // if the one of the k-nearest points is too far the neigborhood
        // is rejected. We also make a filter upon the ratio of the eigen
        // values of the variance-covariance matrix of the neighborhood
        // to check if the points are distributed upon a line or a plane
        size_t LocalizationLineDistanceNbrNeighbors = 10;
        size_t LocalizationMinmumLineNeighborRejection = 4;
        size_t MaximumSurfFeatures = 2000;
        size_t LocalizationBlobDistanceNbrNeighbors = 25.0;
        size_t LocalizationPlaneDistanceNbrNeighbors = 5;

        double LocalizationLineDistancefactor = 5.0;
        double LocalizationPlaneDistancefactor1 = 16.0;
        double LocalizationPlaneDistancefactor2 = 8.0;

        double LocalizationMaxPlaneDistance = 1.0;
        double LocalizationMaxLineDistance = 0.2;
        double LocalizationLineMaxDistInlier = 0.2;
        double MinNbrMatchedKeypoints = 20.;

        // The max distance allowed between two frames
        // If the distance is over this limit, the ICP
        // matching will not match point and the odometry
        // will fail. It has to be setted according the maximum
        // speed of the vehicle used
        double MaxDistanceForICPMatching = 20.0;


        // Loss saturation properties
        // The loss function used is L(residual) = scale* arctan(residual / scale)
        // where residual is quality of each keypoints match.

        double LocalizationInitLossScale = 0.7;     // Saturation around 2.5 meters
        double LocalizationFinalLosssScale = 0.05;  // saturation around 0.4 meters

        // Maximum distance (in meters) beyond which the residual errors are
        // saturated to robustify the optimization against outlier constraints.
        // The residuals will be robustified by Tukey loss at scale sqrt(SatDist).
        double SaturationDistance = 1.;

        // If Undistortion is enabled, it is necessary to save frame duration
        // (time ellapsed between first and last point measurements)
        double FrameDuration;
        double lasttimeLaserOdometry = 0;
        double fisrt_time = 0;


    public:
        LidarSLAM();

        void Reset(bool resetLog = true);

        void Localization(bool initialization, PredictionSource predictodom, Transformd T_w_lidar,
                          pcl::PointCloud<Point>::Ptr edge_point, pcl::PointCloud<Point>::Ptr planner_point, double timeLaserOdometry);

        void ComputePointInitAndFinalPose(MatchingMode matchingMode,const Point &p,Eigen::Vector3d &pInit,Eigen::Vector3d &pFinal);

        OptimizationParameter ComputeLineDistanceParameters(LocalMap &local_map, const Point &p);

        OptimizationParameter ComputePlaneDistanceParameters(LocalMap &local_map, const Point &p);

        RegistrationError EstimateRegistrationError(ceres::Problem &problem, const double eigen_thresh);


        bool DegeneracyDetection(double eigThreshlod, Eigen::Matrix<double, 6, 6, Eigen::RowMajor> &matAtA,
                                 Eigen::Matrix<double, 6, 1> &matX);

        void FeatureObservabilityAnalysis(pcaFeature &feature, Eigen::Vector3d p_query, Eigen::Vector3d lamada,
                                          Eigen::Vector3d normal_direction, Eigen::Vector3d principal_direction);
        inline void ResetDistanceParameters();

        void MannualYawCorrection();


    };      // class lidarslam
}
#endif  // LIDARSLAM_H
