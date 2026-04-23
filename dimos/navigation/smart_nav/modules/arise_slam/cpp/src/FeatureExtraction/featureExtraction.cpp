//
// Created by shibo zhao on 2020-09-27.
//

#include <arise_slam_mid360/FeatureExtraction/featureExtraction.h>

namespace arise_slam {

    featureExtraction::featureExtraction() {
    }

    bool featureExtraction::init(const feature_extraction_config& config) {
        config_ = config;

        imuBuf.allocate(2000);
        lidarBuf.allocate(50);

        if (config_.N_SCANS != 16 && config_.N_SCANS != 32 && config_.N_SCANS != 64 && config_.N_SCANS != 4)
        {
            printf("[arise_slam] only support velodyne with 16, 32 or 64 scan line! and livox mid 360\n");
            return false;
        }

        printf("[arise_slam] config_.skipFrame: %d\n", config_.skipFrame);
        printf("[arise_slam] config_.lidar_flip: %d\n", config_.lidar_flip);
        printf("[arise_slam] config_.use_imu_roll_pitch %d\n", config_.use_imu_roll_pitch);
        printf("[arise_slam] scan line number %d\n", config_.N_SCANS);
        printf("[arise_slam] use up realsense camera points %d\n", config_.use_up_realsense_points);
        printf("[arise_slam] use down realsense camera points %d\n", config_.use_down_realsense_points);

        KeyPointsExtractor->MinDistanceToSensor = config_.min_range;
        KeyPointsExtractor->MaxDistanceToSensor = config_.max_range;

        delay_count_ = 0;
        m_imuPeriod = 1.0/200;

        return true;
    }

    template <typename PointT>
    void featureExtraction::removeClosestFarestPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                                                          pcl::PointCloud<PointT> &cloud_out, float min_range,
                                                          float max_range)
    {

        if (&cloud_in != &cloud_out)
        {
            cloud_out.header = cloud_in.header;
            cloud_out.points.resize(cloud_in.points.size());
        }
        size_t j = 0;

        PointType point;
        for (size_t i = 0; i < cloud_in.points.size(); ++i)
        {
            //In the bounding box filter
            float pointDis = sqrt(cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y);
            if ((cloud_in.points[i].x > config_.box_size.blindBack && cloud_in.points[i].x < config_.box_size.blindFront &&
                cloud_in.points[i].y > config_.box_size.blindRight && cloud_in.points[i].y < config_.box_size.blindLeft) ||
                (cloud_in.points[i].z > config_.box_size.blindDiskLow && cloud_in.points[i].z < config_.box_size.blindDiskHigh &&
                pointDis < config_.box_size.blindDiskRadius))
            {
                continue;
            }

            // in the range filter
            point.x = cloud_in.points[i].x;
            point.y = cloud_in.points[i].y;
            point.z = cloud_in.points[i].z;

            const Eigen::Vector3f &range = point.getVector3fMap();
            if (std::isfinite(point.x) && std::isfinite(point.y) &&
                std::isfinite(point.z))
            {
                if (range.norm() > config_.min_range && range.norm() < config_.max_range)
                {

                    cloud_out.points[j] = cloud_in.points[i];
                    j++;
                }
            }
        }
        if (j != cloud_in.points.size())
        {
            cloud_out.points.resize(j);
        }

        cloud_out.height = 1;
        cloud_out.width = static_cast<uint32_t>(j);
        cloud_out.is_dense = true;
    }

    template <typename Meas>
    bool featureExtraction::synchronize_measurements(MapRingBuffer<Meas> &measureBuf,
                                                     MapRingBuffer<pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr> &lidarBuf)
    {

            if (lidarBuf.getSize() == 0 or measureBuf.getSize() == 0)
                return false;

            double lidar_start_time;
            lidarBuf.getFirstTime(lidar_start_time);

            pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr lidar_msg;
            lidarBuf.getFirstMeas(lidar_msg);

            double lidar_end_time = lidar_start_time + lidar_msg->back().time;

            // obtain the current imu message
            double meas_start_time=0;
            measureBuf.getFirstTime(meas_start_time);

            double meas_end_time=0;
            measureBuf.getLastTime(meas_end_time);

            if (meas_end_time <= lidar_end_time)
            {
                printf("[arise_slam] meas_end_time < lidar_end_time || message order is not perfect!\n");
                printf("[arise_slam] meas_end_time %f < %f lidar_end_time\n", meas_end_time, lidar_end_time);
                printf("[arise_slam] All the lidar data is more recent than all the imu data. Will throw away lidar frame\n");
                return false;
            }

            if (meas_start_time >= lidar_start_time)
            {
                printf("[arise_slam] throw laser scan, only should happen at the beginning\n");
                lidarBuf.clean(lidar_start_time);
                printf("[arise_slam] removed the lidarBuf size %d, measureBuf size %d\n", lidarBuf.getSize(), measureBuf.getSize());
                printf("[arise_slam] meas_start_time: %f > lidar_start_time: %f\n", meas_start_time, lidar_start_time);
                return false;
            }
            else
            {
                return true;
            }

    }

    void featureExtraction::imuRemovePointDistortion(double lidar_start_time, double lidar_end_time,
                                                     MapRingBuffer<Imu::Ptr> &imuBuf,
                                                     pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr &lidar_msg)
    {

        TicToc t_whole;
        TicToc t_prepare;

        auto &laserCloudIn = *lidar_msg;
        Eigen::Quaterniond q_w_original;
        {
            double t_b_i = lidar_start_time;

            auto after_ptr = imuBuf.measMap_.upper_bound(t_b_i);
            if(after_ptr->first < 0.0001) {
                after_ptr = imuBuf.measMap_.begin();
            }
            if (after_ptr == imuBuf.measMap_.begin())
            {
                q_w_original = after_ptr->second->q_w_i;
            }
            else
            {
                auto before_ptr = after_ptr;
                before_ptr--;

                double ratio_bi = (t_b_i - before_ptr->second->time) /
                                  (after_ptr->second->time - before_ptr->second->time);

                Eigen::Quaterniond q_w_i_before = before_ptr->second->q_w_i;
                Eigen::Quaterniond q_w_i_after = after_ptr->second->q_w_i;

                q_w_original = q_w_i_before.slerp(ratio_bi, q_w_i_after);
            }
        }

        q_w_original_l = q_w_original * T_i_l.rot;
        q_w_original_l.normalized();
        t_w_original_l.x() = 0.0;
        t_w_original_l.y() = 0.0;
        t_w_original_l.z() = 0.0;

        Eigen::Quaterniond q_w_end;
        {
            double t_b_i = lidar_end_time;

            auto after_ptr = imuBuf.measMap_.upper_bound(t_b_i);
            if(after_ptr->first < 0.0001) {
                after_ptr = imuBuf.measMap_.begin();
            }
            if (after_ptr == imuBuf.measMap_.begin())
            {
                q_w_end = after_ptr->second->q_w_i;
            }
            else
            {
                auto before_ptr = after_ptr;
                before_ptr--;

                double ratio_bi = (t_b_i - before_ptr->second->time) /
                                  (after_ptr->second->time - before_ptr->second->time);
                Eigen::Quaterniond q_w_i_before = before_ptr->second->q_w_i;
                Eigen::Quaterniond q_w_i_after = after_ptr->second->q_w_i;

                q_w_end = q_w_i_before.slerp(ratio_bi, q_w_i_after);
            }
        }

        Eigen::Quaterniond q_original_end = q_w_original.inverse() * q_w_end;

        q_original_end.normalized();

        for (auto &point : laserCloudIn)
        {
            double t_b_i = point.time + lidar_start_time;
            Eigen::Quaterniond q_w_i;

            auto after_ptr = imuBuf.measMap_.lower_bound(t_b_i);

            if(after_ptr->first < 0.0001) {
                after_ptr = imuBuf.measMap_.begin();
            }
            if (after_ptr == imuBuf.measMap_.begin())
            {
                q_w_i = after_ptr->second->q_w_i;
            }
            else
            {
                auto before_ptr = after_ptr--;
                double ratio_bi = (t_b_i - before_ptr->second->time) /
                                  (after_ptr->second->time - before_ptr->second->time);

                Eigen::Quaterniond q_w_i_before = before_ptr->second->q_w_i;
                Eigen::Quaterniond q_w_i_after = after_ptr->second->q_w_i;

                q_w_i = q_w_i_before.slerp(ratio_bi, q_w_i_after);
            }

            Eigen::Quaterniond q_original_i = q_w_original.inverse() * q_w_i;
            Transformd T_original_i(q_original_i, Eigen::Vector3d::Zero());

            Transformd T_original_i_l = T_l_i * T_original_i * T_i_l;

            if (std::isfinite(point.x) && std::isfinite(point.y) &&
                std::isfinite(point.z))
            {
                Eigen::Vector3d pt{point.x, point.y, point.z};

                pt = T_original_i_l * pt;
                point.x = pt.x();
                point.y = pt.y();
                point.z = pt.z();
            }
        }
    }

    void featureExtraction::convert_pointcloud_format(pcl::PointCloud<point_os::PointcloudXYZITR> &laserCloudIn,
                                                      pcl::PointCloud<LidarKeypointExtractor::Point>::Ptr &cloud_out)
    {
        LidarKeypointExtractor::Point point;
        for (const auto pt : laserCloudIn)
        {
            if (!std::isfinite(pt.x) ||
                !std::isfinite(pt.y) ||
                !std::isfinite(pt.z))
            {

                continue;
            }
            point.x = pt.x;
            point.y = pt.y;
            point.z = pt.z;
            point.intensity = pt.intensity;
            point.time = pt.time;
            point.laserId = pt.ring;
            cloud_out->push_back(point);
        }
    }

    void featureExtraction::convert_velodyne_scan_order(pcl::PointCloud<point_os::PointcloudXYZITR> &laserCloudIn)
    {

        PointType point;
        std::vector<pcl::PointCloud<PointType>> laserCloudScans(config_.N_SCANS);
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudTobeRegistered(new pcl::PointCloud<pcl::PointXYZHSV>());

        pcl::PointXYZHSV feature_point;
        pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());

        for (const auto &pt : laserCloudIn)
        {
            point.x = pt.x;
            point.y = pt.y;
            point.z = pt.z;

            int scanID = pt.ring;
            point.intensity = pt.intensity;
            laserCloudScans[scanID].push_back(point);
        }

        std::vector<int> scanStartInd(config_.N_SCANS, 0);
        std::vector<int> scanEndInd(config_.N_SCANS, 0);

        laserCloud_ringorder.reset(new pcl::PointCloud<PointType>());

        for (int i = 0; i < config_.N_SCANS; i++)
        {
            scanStartInd[i] = laserCloud->size() + 5;
            *laserCloud += laserCloudScans[i];
            scanEndInd[i] = laserCloud->size() - 6;
        }

        laserCloud_ringorder = laserCloud;
    }


    void featureExtraction::feature_extraction(double lidar_start_time,
                                               pcl::PointCloud<point_os::PointcloudXYZITR> &laserCloudIn)
    {

        TicToc t_whole;

        pcl::PointCloud<LidarKeypointExtractor::Point>::Ptr pc(new pcl::PointCloud<LidarKeypointExtractor::Point>);
        pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());

        if(laserCloudIn.size()<=10) {
            printf("[arise_slam] laserCloudIn size is too small: %zu\n",laserCloudIn.size());
            return;
        }


        convert_pointcloud_format(laserCloudIn, pc);

        convert_velodyne_scan_order(laserCloudIn);

        KeyPointsExtractor->ComputeKeyPoints(pc, config_.N_SCANS, config_.use_dynamic_mask);

        pcl::PointCloud<PointType>::Ptr edgePoints(new pcl::PointCloud<PointType>());
        edgePoints->reserve(KeyPointsExtractor->EdgesPoints->size());
        pcl::PointCloud<PointType>::Ptr plannerPoints(new pcl::PointCloud<PointType>());
        plannerPoints->reserve(KeyPointsExtractor->PlanarsPoints->size());
        pcl::PointCloud<PointType>::Ptr bobPoints(new pcl::PointCloud<PointType>());
        bobPoints->reserve(KeyPointsExtractor->BlobsPoints->size());

        auto convertKeypoints = [this](pcl::PointCloud<LidarKeypointExtractor::Point>::Ptr keyPoints, pcl::PointCloud<PointType>::Ptr &feature_points) {
            for (const auto pt : *keyPoints)
            {
                PointType point;
                if (!std::isfinite(pt.x) ||
                    !std::isfinite(pt.y) ||
                    !std::isfinite(pt.z))
                {

                    continue;
                }
                point.x = pt.x;
                point.y = pt.y;
                point.z = pt.z;
                point.intensity = pt.time;
                feature_points->push_back(point);
            }
        };

        convertKeypoints(KeyPointsExtractor->EdgesPoints, edgePoints);
        convertKeypoints(KeyPointsExtractor->PlanarsPoints, plannerPoints);

        // Store result
        pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr undistortedPoints;
        undistortedPoints = std::make_shared<pcl::PointCloud<point_os::PointcloudXYZITR>>(laserCloudIn);

        latest_result_.edgePoints = edgePoints;
        latest_result_.plannerPoints = plannerPoints;
        latest_result_.depthPoints = bobPoints;
        latest_result_.undistortedCloud = undistortedPoints;
        latest_result_.q_w_original_l = q_w_original_l;
        latest_result_.timestamp = lidar_start_time;
        latest_result_.valid = true;
        // Upstream: featureExtraction.cpp:1121 sets imu_available = true after
        // successful feature extraction (always true when we reach this point)
        latest_result_.imu_available = true;
        latest_result_.odom_available = false;  // no visual odometry source
        has_new_result_ = true;

        if (t_whole.toc() > 100)
            printf("[arise_slam] scan registration process over 100ms\n");

        lidarBuf.clean(lidar_start_time);
    }

    // Upstream: featureExtraction.cpp:1126-1159 — undistortionAndscanregistration()
    // checks IMU/visual odom synchronization before processing.
    // Upstream guards: LASER_IMU_SYNC_SCCUESS (line 1128) and
    //   LASER_CAMERA_SYNC_SUCCESS (line 1130-1135)
    // In upstream, also checks frameCount > 100 for visual odom.
    void featureExtraction::undistortionAndscanregistration()
    {
        LASER_IMU_SYNC_SCCUESS = synchronize_measurements<Imu::Ptr>(imuBuf, lidarBuf);

        if ( LASER_IMU_SYNC_SCCUESS == true and lidarBuf.getSize() > 0)
        {
            double lidar_start_time;
            lidarBuf.getFirstTime(lidar_start_time);
            pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr lidar_msg;
            lidarBuf.getFirstMeas(lidar_msg);

            double lidar_end_time = lidar_start_time + lidar_msg->back().time;

            if (LASER_IMU_SYNC_SCCUESS == true)
            {
                imuRemovePointDistortion(lidar_start_time, lidar_end_time, imuBuf, lidar_msg);
            }

            if(config_.sensor==SensorType::LIVOX)
            {
                pcl::PointCloud<PointType>::Ptr plannerPoints(new pcl::PointCloud<PointType>());
                plannerPoints->reserve(lidar_msg->points.size());
                pcl::PointCloud<PointType>::Ptr edgePoints(new pcl::PointCloud<PointType>());
                edgePoints->reserve(lidar_msg->points.size());
                pcl::PointCloud<PointType>::Ptr bobPoints(new pcl::PointCloud<PointType>());
                bobPoints->reserve(lidar_msg->points.size());

                DepthFeatureExtraction.uniformfeatureExtraction(lidar_msg, plannerPoints, config_.skip_realsense_points,config_.min_range);

                // Store result
                latest_result_.edgePoints = edgePoints;
                latest_result_.plannerPoints = plannerPoints;
                latest_result_.depthPoints = bobPoints;
                latest_result_.undistortedCloud = lidar_msg;
                latest_result_.q_w_original_l = q_w_original_l;
                latest_result_.timestamp = lidar_start_time;
                latest_result_.valid = true;
                latest_result_.imu_available = true;
                latest_result_.odom_available = false;
                has_new_result_ = true;

            }else
            {
                feature_extraction(lidar_start_time, *lidar_msg);
            }

            LASER_IMU_SYNC_SCCUESS = false;

        }else
        {
            printf("[arise_slam] sync unsuccessful, skipping scan frame\n");
        }
    }

    void featureExtraction::addImuData(double timestamp, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr,
                                        const Eigen::Quaterniond& orientation, bool has_orientation)
    {
        m_buf.lock();

        Eigen::Vector3d accel = acc;
        Eigen::Vector3d gyro = gyr;

        if (config_.lidar_flip)
        {
            accel.y() *= -1.0;
            accel.z() *= -1.0;
            gyro.y() *= -1.0;
            gyro.z() *= -1.0;
        }

        accel.x() += config_.imu_acc_x_offset;
        accel.y() += config_.imu_acc_y_offset;
        accel.z() += config_.imu_acc_z_offset;

        double lastImuTime = 0.0;
        double dt = m_imuPeriod;

        if(imuBuf.getLastTime(lastImuTime))
        {
            dt = timestamp - lastImuTime;
            if(abs(dt - m_imuPeriod) > m_imuPeriod * IMU_TIME_LENIENCY)
            {
                dt = m_imuPeriod;
            }
        }

        Imu::Ptr imudata = std::make_shared<Imu>();
        imudata->time = timestamp;
        if(IMU_INIT == true and config_.sensor==SensorType::LIVOX)
        {
            double gravity = 9.8105;

            Eigen::Vector3d rotated_gyr = imu_Init->imu_laser_R_Gravity * gyro;
            Eigen::Vector3d rotated_acc = imu_Init->imu_laser_R_Gravity * accel;

            imudata->acc = rotated_acc * gravity / imu_Init->acc_mean.norm();
            imudata->gyr = rotated_gyr;
        }else
        {
            imudata->acc = accel;
            imudata->gyr = gyro;
        }

        if (!imuBuf.empty())
        {
            const Eigen::Quaterniond rot_last = imuBuf.measMap_.rbegin()->second->q_w_i;
            Eigen::Vector3d gyr_last = imuBuf.measMap_.rbegin()->second->gyr;
            const double &time_last = imuBuf.measMap_.rbegin()->second->time;

            double dt = timestamp - time_last;
            Eigen::Vector3d delta_angle = dt * 0.5 * (imudata->gyr + gyr_last);
            Eigen::Quaterniond delta_r =
                Sophus::SO3d::exp(delta_angle).unit_quaternion();

            Eigen::Quaterniond rot = rot_last * delta_r;
            rot.normalized();
            imudata->q_w_i = rot;
        }
        else
        {
            if (config_.use_imu_roll_pitch && has_orientation)
            {
                // Extract roll/pitch from orientation, zero out yaw
                Eigen::Vector3d euler = orientation.toRotationMatrix().eulerAngles(0, 1, 2);
                double roll = euler[0], pitch = euler[1], yaw = euler[2];

                Eigen::Quaterniond yaw_quat(Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()));
                Eigen::Quaterniond first_orientation = yaw_quat * orientation;

                imudata->q_w_i = first_orientation;
            }
        }

        imuBuf.addMeas(imudata, timestamp);

        double lidar_first_time = 0;
        if(lidarBuf.getFirstTime(lidar_first_time)) {
            if (timestamp > lidar_first_time + LIDAR_MESSAGE_TIME + 0.05)
            {
                if(config_.sensor==SensorType::LIVOX)
                {
                    double first_time = 0.0;
                    imuBuf.getFirstTime(first_time);

                    if (timestamp-first_time>200*m_imuPeriod and IMU_INIT==false)
                    {
                        imu_Init->imuInit(imuBuf);
                        IMU_INIT=true;
                        imuBuf.clean(timestamp);
                        printf("[arise_slam] IMU Initialization Process Finish!\n");
                    }
                    if(IMU_INIT==true)
                    {
                       undistortionAndscanregistration();
                       double lidar_first_time;
                       lidarBuf.getFirstTime(lidar_first_time);
                       lidarBuf.clean(lidar_first_time);
                    }
                }else
                {
                    double lidar_first_time;
                    lidarBuf.getFirstTime(lidar_first_time);
                    lidarBuf.clean(lidar_first_time);
                }
            }
        }

        m_buf.unlock();
    }

    void featureExtraction::addPointCloud(double timestamp, pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr pointCloud)
    {
        if (imuBuf.empty() || delay_count_++ <= 5)
        {
            printf("[arise_slam] imu buf empty, waiting...\n");
            return;
        }

        frameCount = frameCount + 1;
        if (frameCount % config_.skipFrame != 0)
            return;

        m_buf.lock();

        if (config_.lidar_flip)
        {
            for (uint i = 1; i < pointCloud->points.size(); i++)
            {
                pointCloud->points[i].y *= -1.0;
                pointCloud->points[i].z *= -1.0;
            }
        }

        std::size_t curLidarBufferSize = lidarBuf.getSize();

        if(curLidarBufferSize > 2) {
            printf("[arise_slam] Large lidar buffer size: %zu\n", curLidarBufferSize);
            printf("[arise_slam]   IMU and lidar times are likely out of sync\n");
        }

        while (curLidarBufferSize >= 50)
        {
            double lidar_first_time;
            lidarBuf.getFirstTime(lidar_first_time);
            lidarBuf.clean(lidar_first_time);
            printf("[arise_slam] Lidar buffer too large, dropping frame\n");
            curLidarBufferSize = lidarBuf.getSize();
        }

        removeClosestFarestPointCloud(*pointCloud, *pointCloud, config_.min_range, config_.max_range);
        lidarBuf.addMeas(pointCloud, timestamp);

        m_buf.unlock();
    }

    void featureExtraction::assignTimeforPointCloud(pcl::PointCloud<PointType>::Ptr laserCloudIn_ptr_)
    {
        size_t cloud_size = laserCloudIn_ptr_->size();
        pointCloudwithTime.reset(new pcl::PointCloud<point_os::PointcloudXYZITR>());
        pointCloudwithTime->reserve(cloud_size);

        point_os::PointcloudXYZITR point;
        for (size_t i = 0; i < cloud_size; i++)
        {
            point.x = laserCloudIn_ptr_->points[i].x;
            point.y = laserCloudIn_ptr_->points[i].y;
            point.z = laserCloudIn_ptr_->points[i].z;
            point.intensity = laserCloudIn_ptr_->points[i].intensity;

            float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
            int scanID = 0;

            if (config_.N_SCANS == 16)
            {
                scanID = int((angle + 15) / 2 + 0.5);
                if (scanID > (config_.N_SCANS - 1) || scanID < 0)
                {
                    cloud_size--;
                    continue;
                }
            }
            else if (config_.N_SCANS == 32)
            {
                scanID = int((angle + 92.0 / 3.0) * 3.0 / 4.0);
                if (scanID > (config_.N_SCANS - 1) || scanID < 0)
                {
                    cloud_size--;
                    continue;
                }
            }
            else if (config_.N_SCANS == 64)
            {
                if (angle >= -8.83)
                    scanID = int((2 - angle) * 3.0 + 0.5);
                else
                    scanID = config_.N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);

                // use [0 50]  > 50 remove outlies
                if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0)
                {
                    cloud_size--;
                    continue;
                }
            }
            else
            {
                printf("wrong scan number\n");
            }

            point.ring = scanID;
            float rel_time = (columnTime * int(i / config_.N_SCANS) + laserTime * (i % config_.N_SCANS)) / scanPeriod;
            float pointTime = rel_time * scanPeriod;
            point.time = pointTime;
            pointCloudwithTime->push_back(point);
        }
    }

    Eigen::Matrix3d featureExtraction::getPitchTF(double pitch_degrees)
    {
        const double deg2rad = M_PI / 180.0;
        double theta = pitch_degrees * deg2rad;
        Eigen::Matrix3d R_pitch;
        R_pitch << std::cos(theta), 0, std::sin(theta),
                0, 1, 0,
                -std::sin(theta), 0, std::cos(theta);
        return R_pitch;
    }

    FeatureExtractionResult featureExtraction::getLatestResult()
    {
        FeatureExtractionResult result = latest_result_;
        has_new_result_ = false;
        latest_result_.valid = false;
        return result;
    }

} // namespace arise_slam
