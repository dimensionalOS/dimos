//
// Created by shibo zhao on 2020-09-27.
// Ported from ROS2 to plain C++ for DimOS (LCM-based)
//
#pragma once
#ifndef IMUPREINTEGRATION_H
#define IMUPREINTEGRATION_H

#include <deque>
#include <mutex>
#include <memory>
#include <Eigen/Dense>

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
#include "arise_slam_mid360/tic_toc.h"
#include <glog/logging.h>
#include "arise_slam_mid360/sensor_data/imu/imu_data.h"
#include <pcl/common/transforms.h>


namespace arise_slam {

    using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
    using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
    using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
    using FrameId = std::uint64_t;

    struct imuPreintegration_config{
        float imuAccNoise;
        float imuAccBiasN;
        float imuGyrNoise;
        float imuGyrBiasN;
        float imuGravity;
        float lidar_correction_noise;
        float smooth_factor;
        bool  use_imu_roll_pitch;
        bool lidar_flip;
        SensorType sensor;

        double imu_acc_x_offset;
        double imu_acc_y_offset;
        double imu_acc_z_offset;
        double imu_acc_x_limit;
        double imu_acc_y_limit;
        double imu_acc_z_limit;
    };

    // Plain C++ IMU data used in place of sensor_msgs::msg::Imu
    struct ImuMsg {
        double timestamp;
        Eigen::Vector3d linear_acceleration;
        Eigen::Vector3d angular_velocity;
        Eigen::Quaterniond orientation;
    };

    // Plain C++ odometry data used in place of nav_msgs::msg::Odometry (for IMU module)
    struct ImuOdomMsg {
        double timestamp;
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;
        double covariance[36] = {};
    };

    using ImuOdomMsgPtr = std::shared_ptr<ImuOdomMsg>;

    // Output state from IMU preintegration
    struct ImuState {
        double timestamp;
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;
        Eigen::Vector3d velocity;
        Eigen::Vector3d angular_velocity;
        gtsam::imuBias::ConstantBias bias;
        double gravity;
        bool health_status;
        uint8_t result;  // IMU_STATE enum value
    };

    class imuPreintegration {
    public:
        imuPreintegration();

        // Initialize with config (replaces on_configure + on_activate)
        void init(const imuPreintegration_config& config);

        static constexpr double delta_t = 0;
        static constexpr double imu_laser_timedelay= 0.8;

    public:
        // Data input methods (replace ROS subscriber callbacks)
        void
        addLaserOdometry(double timestamp, const Eigen::Vector3d& position,
                         const Eigen::Quaterniond& orientation, double degenerate_flag = 0.0);

        void
        addVisualOdometry(double timestamp, const Eigen::Vector3d& position,
                          const Eigen::Quaterniond& orientation);

        void
        addImuMeasurement(double timestamp, const Eigen::Vector3d& acc,
                          const Eigen::Vector3d& gyro, const Eigen::Quaterniond& orientation);

        // Get latest state
        ImuState getLatestState() const;

        // Set callback for output
        void setOutputCallback(std::function<void(const ImuState&)> callback) {
            output_callback_ = callback;
        }

        void
        initial_system(double currentCorrectionTime, gtsam::Pose3 lidarPose);

        void
        process_imu_odometry(double currentCorrectionTime, gtsam::Pose3 relativePose);

        bool
        build_graph(gtsam::Pose3 lidarPose, double curLaserodomtimestamp);

        void
        repropagate_imuodometry(double currentCorrectionTime);

        bool
        failureDetection(const gtsam::Vector3 &velCur,
                         const gtsam::imuBias::ConstantBias &biasCur);

        void
        integrate_imumeasurement(double currentCorrectionTime);

        void
        reset_graph();

        void
        resetOptimization();

        void
        resetParams();

        void
        addNoMotionFactor(const FrameId& from_id, const FrameId& to_id);

        void
        addZeroVelocityPrior(const FrameId& frame_id);

        ImuMsg
        imuConverter(const ImuMsg &imu_in);

    public:
        gtsam::noiseModel::Diagonal::shared_ptr noMotionNoise;
        gtsam::noiseModel::Diagonal::shared_ptr noVelocityNoise;
        gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
        gtsam::noiseModel::Diagonal::shared_ptr priorVisualPoseNoise;
        gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
        gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
        gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
        gtsam::Vector noiseModelBetweenBias;
        std::shared_ptr<gtsam::PreintegratedImuMeasurements> imuIntegratorOpt_;
        std::shared_ptr<gtsam::PreintegratedImuMeasurements> imuIntegratorImu_;
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
        //Modify the extrinsic matrix between laser and imu, laser and camera
        gtsam::Pose3 imu2cam;
        gtsam::Pose3 cam2Lidar;
        gtsam::Pose3 imu2Lidar;
        gtsam::Pose3 lidar2Imu;

    public:
        MapRingBuffer<Imu::Ptr> imuBuf;
        MapRingBuffer<Imu::Ptr> imuBufOpt;
        std::deque<ImuMsg> imuQueOpt;
        std::deque<ImuMsg> imuQueImu;
        MapRingBuffer<ImuOdomMsgPtr> lidarOdomBuf;
        MapRingBuffer<ImuOdomMsgPtr> visualOdomBuf;
        std::mutex mBuf;
        Imu::Ptr imu_Init = std::make_shared<Imu>();

    public:
        bool systemInitialized = false;
        bool doneFirstOpt = false;
        bool use_laserodom = false;
        bool use_visualodom = false;
        bool use_onlyimu = false;
        bool switch_odometry = false;
        bool health_status = true;
        bool imu_init_success = false;


        Eigen::Quaterniond firstImu;
        Eigen::Vector3d gyr_pre;

        double first_imu_time_stamp;
        double time_period;
        double last_processed_lidar_time = -1;
        double lastImuT_imu = -1;
        double lastImuT_opt = -1;
        int key = 1;
        int imuPreintegrationResetId = 0;
        int frame_count = 0;

        enum IMU_STATE : uint8_t {
        FAIL=0,    //lose imu information
        SUCCESS=1, //Obtain the good imu data
        UNKNOW=2
        };

        IMU_STATE RESULT;
        ImuOdomMsgPtr cur_frame = nullptr;
        ImuOdomMsgPtr last_frame = nullptr;
        imuPreintegration_config config_;

    private:
        std::function<void(const ImuState&)> output_callback_;
        ImuState latest_state_;
    };

}

#endif // IMUPREINTEGRATION_H
