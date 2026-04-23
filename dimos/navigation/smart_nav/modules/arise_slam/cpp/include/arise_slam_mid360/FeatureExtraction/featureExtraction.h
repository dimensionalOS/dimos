//
// Created by shibo zhao on 2020-09-27.
//

#ifndef arise_slam_mid360_FEATUREEXTRACTION_H
#define arise_slam_mid360_FEATUREEXTRACTION_H

#include "arise_slam_mid360/logging.h"
#include "arise_slam_mid360/FeatureExtraction/LidarKeypointExtractor.h"
#include "arise_slam_mid360/FeatureExtraction/DepthImageKeypointExtractor.h"


#include <cmath>
#include <string>
#include <vector>

#include <sophus/so3.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

#include "arise_slam_mid360/common.h"
#include "arise_slam_mid360/container/MapRingBuffer.h"
#include "arise_slam_mid360/sensor_data/imu/imu_data.h"
#include "arise_slam_mid360/sensor_data/pointcloud/point_os.h"
#include "arise_slam_mid360/tic_toc.h"
#include "arise_slam_mid360/utils/Twist.h"
#include "arise_slam_mid360/config/parameter.h"

#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#include <tbb/parallel_for.h>

#include <mutex>
#include <iomanip>
#include <omp.h>

namespace arise_slam {

    using std::atan2;
    using std::cos;
    using std::sin;

    constexpr unsigned int BLOCK_TIME_NS = 55296;   // Time in ns for one block (measurement + recharge)
    constexpr std::size_t NUM_BLOCKS = 12;    // Number of blocks in a Velodyne packet
    constexpr double LIDAR_MESSAGE_TIME = (double)(NUM_BLOCKS * BLOCK_TIME_NS * 151) * 1e-9;

    constexpr double IMU_TIME_LENIENCY = 0.1;


    struct bounds_t
    {
        double blindFront;
        double blindBack;
        double blindRight;
        double blindLeft;
        double blindDiskLow;
        double blindDiskHigh;
        double blindDiskRadius;
    };

    struct feature_extraction_config{
        bounds_t box_size;
        int skipFrame;
        bool lidar_flip;
        int N_SCANS;
        int provide_point_time;
        int point_filter_num;
        bool use_dynamic_mask;
        bool use_imu_roll_pitch;
        bool use_up_realsense_points;
        bool use_down_realsense_points;
        bool debug_view_enabled;
        float min_range;
        float max_range;
        int skip_realsense_points;
        SensorType sensor;
        double livox_pitch;

        double imu_acc_x_offset;
        double imu_acc_y_offset;
        double imu_acc_z_offset;
        double imu_acc_x_limit;
        double imu_acc_y_limit;
        double imu_acc_z_limit;

    };

    typedef feature_extraction_config feature_extraction_config;

    // Output struct for processed features
    // Upstream: published as arise_slam_mid360_msgs::LaserFeature message
    // (see arise_slam_mid360_msgs/msg/LaserFeature.msg)
    struct FeatureExtractionResult {
        pcl::PointCloud<PointType>::Ptr edgePoints;       // cloud_corner
        pcl::PointCloud<PointType>::Ptr plannerPoints;    // cloud_surface
        pcl::PointCloud<PointType>::Ptr depthPoints;      // cloud_realsense
        pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr undistortedCloud;  // cloud_nodistortion
        Eigen::Quaterniond q_w_original_l;                // initial_quaternion_*
        double timestamp;
        bool valid;
        bool imu_available;   // upstream LaserFeature.imu_available
        bool odom_available;  // upstream LaserFeature.odom_available
    };

    class featureExtraction {
    public:
        /* TODO: return this as a parameter */

        static constexpr double scanPeriod = 0.100859904 - 20.736e-6;
        static constexpr double columnTime = 55.296e-6;
        static constexpr double laserTime = 2.304e-6;

        // for livox
        pcl::PointCloud<pcl::PointXYZINormal> pl_full, pl_corn, pl_surf;
        pcl::PointCloud<pcl::PointXYZINormal> pl_buff[128]; //maximum 128 line lidar
        int scan_count = 0;
        bool is_first_lidar = true;
        double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
        std::deque<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> lidar_buffer;
        bool time_sync_en = false;
        bool timediff_set_flg = false;
        double timediff_lidar_wrt_imu = 0.0;

        featureExtraction();

        // Initialize with a config struct instead of ROS parameters
        bool init(const feature_extraction_config& config);

        template <typename PointT>
        void removeClosestFarestPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                                            pcl::PointCloud<PointT> &cloud_out, float min_range, float max_range);
        template <typename Meas>
        bool synchronize_measurements(MapRingBuffer<Meas> &measureBuf,
                                        MapRingBuffer<pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr> &lidarBuf);
        void imuRemovePointDistortion(double lidar_start_time, double lidar_end_time, MapRingBuffer<Imu::Ptr> &imuBuf,
                                    pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr &lidar_msg);
        void feature_extraction(double lidar_start_time, pcl::PointCloud<point_os::PointcloudXYZITR> &laserCloudIn);

        void convert_pointcloud_format(pcl::PointCloud<point_os::PointcloudXYZITR> &laserCloudIn, pcl::PointCloud<LidarKeypointExtractor::Point>::Ptr &cloud_out);

        void undistortionAndscanregistration();

        // Process incoming IMU data
        void addImuData(double timestamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr,
                        const Eigen::Quaterniond& orientation, bool has_orientation);

        // Process incoming point cloud data (standard PointCloud2 format)
        void addPointCloud(double timestamp, pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr pointCloud);

        void assignTimeforPointCloud(pcl::PointCloud<PointType>::Ptr laserCloudIn_ptr_);

        void convert_velodyne_scan_order(pcl::PointCloud<point_os::PointcloudXYZITR> &laserCloudIn);

        Eigen::Matrix3d getPitchTF(double pitch_degrees);

        // Get the latest processed result (call after addImuData/addPointCloud trigger processing)
        FeatureExtractionResult getLatestResult();

        Imu::Ptr imu_Init = std::make_shared<Imu>();
        MapRingBuffer<Imu::Ptr> imuBuf;
        MapRingBuffer<pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr> lidarBuf;

        //parameters can be changed by config
        std::shared_ptr<LidarKeypointExtractor> KeyPointsExtractor =
                std::make_shared<LidarKeypointExtractor>();

        DepthKeypointExtractor DepthFeatureExtraction;

    private:
        int delay_count_;
        std::mutex m_buf;
        int frameCount = 0;

        bool PUB_EACH_LINE = false;
        bool LASER_IMU_SYNC_SCCUESS = false;
        bool LASER_CAMERA_SYNC_SUCCESS = false;
        bool FIRST_LASER_FRAME=false;
        bool IMU_INIT=false;
        double m_imuPeriod;

        Eigen::Quaterniond q_w_original_l;
        Eigen::Vector3d t_w_original_l;
        pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr pointCloudwithTime=nullptr;
        pcl::PointCloud<point_os::OusterPointXYZIRT>::Ptr tmpOusterCloudIn=nullptr ;
        pcl::PointCloud<PointType>::Ptr laserCloud_ringorder=nullptr;
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudWithFeatures_ringorder=nullptr;
        feature_extraction_config config_;

        // Latest result storage
        FeatureExtractionResult latest_result_;
        bool has_new_result_ = false;
    };


} // namespace arise_slam




#endif //arise_slam_mid360_FEATUREEXTRACTION_H
