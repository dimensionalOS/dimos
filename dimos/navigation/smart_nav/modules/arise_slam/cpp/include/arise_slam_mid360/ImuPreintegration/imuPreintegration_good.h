//
// Created by shibo zhao on 2020-09-27.
//
#pragma once
#ifndef IMUPREINTEGRATION_H
#define IMUPREINTEGRATION_H


#include "utility.h"
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
#include "arise_slam_mid360/utils/Twist.h"
#include "arise_slam_mid360/container/MapRingBuffer.h"
#include "arise_slam_mid360/config/parameter.h"
#include <glog/logging.h>


namespace arise_slam {

    using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
    using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
    using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

    class imuPreintegration {

    public:

        imuPreintegration();

        static constexpr double delta_t = 0;
        static constexpr double imu_laser_timedelay= 0.8;
     //   static bool receive_first_laserodom=false;


    public:
        // void
        // setUpROS(ros::NodeHandle *pub_node, ros::NodeHandle *private_node);

        void
        laserodometryHandler(const nav_msgs::msg::Odometry::SharedPtr &odomMsg);

        void
        visualodometryHandler(const nav_msgs::msg::Odometry::SharedPtr &odomMsg);

        void
        imuHandler(const sensor_msgs::msg::Imu::SharedPtr &imu_raw);

        void
        initial_system(double currentCorrectionTime, gtsam::Pose3 lidarPose);

        void
        process_imu_odometry(double currentCorrectionTime, gtsam::Pose3 relativePose);

        template<typename Meas>
        bool
        synchronize_measurements(MapRingBuffer<Meas> &measureBuf,
                                 MapRingBuffer<nav_msgs::msg::Odometry::SharedPtr> &lidarOdomBuf, bool remove_item);

        void
        build_graph(gtsam::Pose3 lidarPose, double curLaserodomtimestamp);

        void
        repropagate_imuodometry(double currentCorrectionTime);

        bool
        failureDetection(const gtsam::Vector3 &velCur,
                         const gtsam::imuBias::ConstantBias &biasCur);

        void
        synchronize_measurements(std::deque<nav_msgs::msg::Odometry::SharedPtr> &laserodomQue,
                                 std::deque<nav_msgs::msg::Odometry::SharedPtr> &visualodomQue,
                                 std::deque<sensor_msgs::msg::Imu> &imuQue);

        void
        obtainCurrodometry(nav_msgs::msg::Odometry::SharedPtr &odomMsg, double &currentCorrectionTime,
                           gtsam::Pose3 &lidarPose,
                           int &currentResetId);

        void
        integrate_imumeasurement(double currentCorrectionTime);

        void
        reset_graph();

        void
        resetOptimization();

        void
        resetParams();

        sensor_msgs::msg::Imu
        imuConverter(const sensor_msgs::msg::Imu &imu_in);

        template<typename T>
        double secs(T msg) {
            return msg->header.stamp.sec + msg->header.stamp.nanosec*1e-9;
        }


    public:
        ros::NodeHandle *pub_node_;
        ros::NodeHandle *private_node_;

        ros::Subscriber subImu;
        ros::Subscriber sublaserOdometry;
        ros::Subscriber subvisualOdometry;

        ros::Publisher pubImuOdometry;
        ros::Publisher pubImuOdometry2;
        ros::Publisher pubSwitchOdometry;
        ros::Publisher pubImuPath;
        ros::Publisher pubSwitchPath;
        ros::Publisher pubHealthStatus;

    public:
        gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
        gtsam::noiseModel::Diagonal::shared_ptr priorVisualPoseNoise;
        gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
        gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
        gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
        gtsam::Vector noiseModelBetweenBias;
        gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;
        gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;
        gtsam::Pose3 prevPose_;
        gtsam::Vector3 prevVel_;
        gtsam::NavState prevState_;
        gtsam::imuBias::ConstantBias prevBias_;
        gtsam::NavState prevStateOdom;
        gtsam::imuBias::ConstantBias prevBiasOdom;
        gtsam::ISAM2 optimizer;
        gtsam::NonlinearFactorGraph graphFactors;
        gtsam::Values graphValues;
        gtsam::Pose3 lidarodom_w_pre;
        gtsam::Pose3 lidarodom_w_cur;

    public:


    public:
        //Modify the extrinsic matrxi between laser and imu, laser and camera
        gtsam::Pose3 imu2cam;
        gtsam::Pose3 cam2Lidar;
        gtsam::Pose3 imu2Lidar;
        gtsam::Pose3 lidar2Imu;

    public:
        tf2::Transform map_to_odom;                     // map -> odom
        tf2_ros::TransformBroadcaster tfMap2Odom;
        tf2_ros::TransformBroadcaster tfOdom2BaseLink;      // odom -> base_link

    public:
        MapRingBuffer<sensor_msgs::msg::Imu> imuBuf;
        MapRingBuffer<sensor_msgs::msg::Imu> imuBufOpt;
        MapRingBuffer<nav_msgs::msg::Odometry::SharedPtr> lidarOdomBuf;
        MapRingBuffer<nav_msgs::msg::Odometry::SharedPtr> visualOdomBuf;


    private:

        bool systemInitialized = false;
        bool doneFirstOpt = false;
        bool use_laserodom = false;
        bool use_visualodom = false;
        bool use_onlyimu = false;
        bool switch_odometry = false;
        bool health_status = true;

        bool imuInitialized = false;
        Eigen::Quaterniond firstImu;

        double first_imu_time_stamp;
        double time_period;
        double last_processed_lidar_time = -1;
        double lastImuT_imu = -1;
        double lastImuT_opt = -1;
        int key = 1;
        int imuPreintegrationResetId = 0;
        int frame_count = 0;
        std::mutex mBuf;
        enum IMU_STATE : uint8_t {
           FAIL=0,    //lose imu information 
           SUCCESS=1, //Obtain the good imu data 
           UNKNOW=2
        };  
      
      IMU_STATE RESULT;
       nav_msgs::msg::Odometry::SharedPtr cur_frame= nullptr;
       nav_msgs::msg::Odometry::SharedPtr last_frame= nullptr;

    };
}

#endif // IMUPREINTEGRATION_H
