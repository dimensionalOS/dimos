//
// Created by shiboz on 2021-10-18.
// Ported from ROS2 to plain C++ for DimOS (LCM-based)
//


#include "arise_slam_mid360/LaserMapping/laserMapping.h"

double parameters[7] = {0, 0, 0, 0, 0, 0, 1};
Eigen::Map<Eigen::Vector3d> t_w_curr(parameters);
Eigen::Map<Eigen::Quaterniond> q_w_curr(parameters+3);

namespace arise_slam {

    laserMapping::laserMapping() {
    }

    void laserMapping::init(const laser_mapping_config& config) {
        config_ = config;

        printf("[arise_slam] DEBUG VIEW: %d\n", config_.debug_view_enabled);
        printf("[arise_slam] ENABLE OUSTER DATA: %d\n", config_.enable_ouster_data);
        printf("[arise_slam] line resolution %f plane resolution %f vision_laser_time_offset %f\n",
                config_.lineRes, config_.planeRes, vision_laser_time_offset);

        downSizeFilterCorner.setLeafSize(config_.lineRes, config_.lineRes, config_.lineRes);
        downSizeFilterSurf.setLeafSize(config_.planeRes, config_.planeRes, config_.planeRes);

        slam.localMap.lineRes_ = config_.lineRes;
        slam.localMap.planeRes_ = config_.planeRes;
        slam.Visual_confidence_factor=config_.visual_confidence_factor;
        slam.Pos_degeneracy_threshold=config_.pos_degeneracy_threshold;
        slam.Ori_degeneracy_threshold=config_.ori_degeneracy_threshold;
        slam.LocalizationICPMaxIter=config_.max_iterations;
        slam.MaximumSurfFeatures=config_.max_surface_features;
        slam.OptSet.debug_view_enabled=config_.debug_view_enabled;
        slam.OptSet.velocity_failure_threshold=config_.velocity_failure_threshold;
        slam.OptSet.max_surface_features=config_.max_surface_features;
        slam.OptSet.yaw_ratio=config_.yaw_ratio;
        slam.relocalization_map_path=config_.relocalization_map_path;
        slam.local_mode=config_.local_mode;
        slam.init_x=config_.init_x;
        slam.init_y=config_.init_y;
        slam.init_z=config_.init_z;
        slam.init_roll=config_.init_roll;
        slam.init_pitch=config_.init_pitch;
        slam.init_yaw=config_.init_yaw;

        prediction_source = PredictionSource::IMU_ORIENTATION;

        timeLatestImuOdometry = 0.0;

        if(config_.read_pose_file)
        {
            readLocalizationPose(config_.relocalization_map_path);
            // Note: config_ already set from Python side, but if read_pose_file
            // is set, override with values from pose file
        }

        initializationParam();
    }

    void laserMapping::initializationParam() {

        laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());
        laserCloudSurround.reset(new pcl::PointCloud<PointType>());
        laserCloudFullRes.reset(new pcl::PointCloud<PointType>());
        laserCloudFullRes_rot.reset(new pcl::PointCloud<PointType>());
        laserCloudRawRes.reset(new pcl::PointCloud<PointType>());
        laserCloudCornerStack.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfStack.reset(new pcl::PointCloud<PointType>());
        laserCloudRealsense.reset(new pcl::PointCloud<PointType>());
        laserCloudPriorOrg.reset(new pcl::PointCloud<PointType>());
        laserCloudPrior.reset(new pcl::PointCloud<PointType>());

        Eigen::Quaterniond q_wmap_wodom_(1, 0, 0, 0);
        Eigen::Vector3d t_wmap_wodom_(0, 0, 0);
        Eigen::Quaterniond q_wodom_curr_(1, 0, 0, 0);
        Eigen::Vector3d t_wodom_curr_(0, 0, 0);
        Eigen::Quaterniond q_wodom_pre_(1, 0, 0, 0);
        Eigen::Vector3d t_wodom_pre_(0, 0, 0);

        q_wmap_wodom = q_wmap_wodom_;
        t_wmap_wodom = t_wmap_wodom_;
        q_wodom_curr = q_wodom_curr_;
        t_wodom_curr = t_wodom_curr_;
        q_wodom_pre = q_wodom_pre_;
        t_wodom_pre = t_wodom_pre_;

        imu_odom_buf.allocate(5000);
        visual_odom_buf.allocate(5000);

        slam.localMap.setOrigin(Eigen::Vector3d(slam.init_x, slam.init_y, slam.init_z));

        if (slam.local_mode) {
            printf("[arise_slam] \033[1;32m Loading map....\033[0m\n");
            if(readPointCloud()) {
                slam.localMap.addSurfPointCloud(*laserCloudPrior);
                printf("[arise_slam] \033[1;32m Loaded map successfully, started SLAM in localization mode.\033[0m\n");
            } else {
                slam.local_mode = false;
                printf("[arise_slam] \033[1;32mCannot read map file, switched to mapping mode.\033[0m\n");
            }
        } else {
            printf("[arise_slam] \033[1;32mStarted SLAM in mapping mode.\033[0m\n");
        }
    }

    bool laserMapping::readPointCloud()
    {
        return loadMapFromFile(slam.relocalization_map_path);
    }

    bool laserMapping::loadMapFromFile(const std::string& map_path)
    {
        laserCloudPriorOrg->clear();
        laserCloudPrior->clear();

        // Check file extension
        std::string extension = map_path.substr(map_path.find_last_of(".") + 1);
        std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);

        if (extension == "pcd") {
            // Load PCD file
            printf("[arise_slam] Loading PCD map from: %s\n", map_path.c_str());
            if (pcl::io::loadPCDFile<PointType>(map_path, *laserCloudPriorOrg) == -1) {
                printf("[arise_slam] ERROR: Failed to load PCD file: %s\n", map_path.c_str());
                return false;
            }
            printf("[arise_slam] Loaded %zu points from PCD file\n", laserCloudPriorOrg->size());
        } else {
            // Load TXT file (original format)
            printf("[arise_slam] Loading TXT map from: %s\n", map_path.c_str());
            FILE *map_file = fopen(map_path.c_str(), "r");
            if (map_file == NULL) {
                printf("[arise_slam] ERROR: Failed to open TXT file: %s\n", map_path.c_str());
                return false;
            }

            PointType pointRead;
            float intensity, time;
            int val1, val2, val3, val4, val5;
            while (1) {
                val1 = fscanf(map_file, "%f", &pointRead.x);
                val2 = fscanf(map_file, "%f", &pointRead.y);
                val3 = fscanf(map_file, "%f", &pointRead.z);
                val4 = fscanf(map_file, "%f", &intensity);
                val5 = fscanf(map_file, "%f", &time);

                if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) break;

                laserCloudPriorOrg->push_back(pointRead);
            }
            fclose(map_file);
            printf("[arise_slam] Loaded %zu points from TXT file\n", laserCloudPriorOrg->size());
        }

        if (laserCloudPriorOrg->empty()) {
            printf("[arise_slam] ERROR: Loaded map is empty!\n");
            return false;
        }

        // Downsample the map
        downSizeFilterSurf.setInputCloud(laserCloudPriorOrg);
        downSizeFilterSurf.filter(*laserCloudPrior);
        laserCloudPriorOrg->clear();

        printf("[arise_slam] Map downsampled to %zu points\n", laserCloudPrior->size());

        return true;
    }

    void laserMapping::readLocalizationPose(const std::string& parentPath)
    {
        std::cerr << "Reading localization pose..." << std::endl;
        std::string saveOdomPath;
        size_t lastSlashPos = parentPath.find_last_of('/');
        if (lastSlashPos != std::string::npos) {
            saveOdomPath=parentPath.substr(0, lastSlashPos + 1);
        }

        std::string localizationPosePath = saveOdomPath + "start_pose.txt";
        std::ifstream file(localizationPosePath);
        if (!file.is_open()) {
            std::cerr << "Error opening file: " << localizationPosePath << std::endl;
            return;
        }

        std::string line;
        while (std::getline(file, line)) {
            if (line.empty()) {
            continue;
    }
            std::istringstream iss(line);
            arise_slam::OdometryData odom;
            if (iss >> odom.x >> odom.y >> odom.z >> odom.roll >> odom.pitch >> odom.yaw >> odom.duration) {
                std::cout << "Read odometry data: " << odom.x << " " << odom.y << " " << odom.z << std::endl;
                odometryResults.push_back(odom);
            } else {
                std::cerr << "Error reading line: " << line << std::endl;
            }
        }
        std::cerr << "\033[1;32m  loaded the localization_pose.txt successfully \033[0m" <<odometryResults[0].x
        <<" "<<odometryResults[0].y<<" "<<odometryResults[0].z<< std::endl;
        file.close();
    }

    // Function to save odometry data to a text file
    void laserMapping::saveLocalizationPose(double timestamp,Transformd &T_w_lidar, const std::string& parentPath) {

        std::string saveOdomPath;
        size_t lastSlashPos = parentPath.find_last_of('/');
        if (lastSlashPos != std::string::npos) {
            saveOdomPath=parentPath.substr(0, lastSlashPos + 1);
            }

        arise_slam::OdometryData odom;
        {
         odom.timestamp = timestamp;
         odom.x = T_w_lidar.pos.x();
         odom.y = T_w_lidar.pos.y();
         odom.z = T_w_lidar.pos.z();
         // Convert quaternion to RPY using Eigen
         Eigen::Vector3d euler = T_w_lidar.rot.toRotationMatrix().eulerAngles(0, 1, 2);
         odom.roll = euler[0];
         odom.pitch = euler[1];
         odom.yaw = euler[2];
        }

        odometryResults.push_back(odom);

        std::string OdomResultPath=saveOdomPath+"start_pose.txt";
        std::ofstream outFile(OdomResultPath, std::ios::app);

        if (!outFile.is_open()) {
            std::cerr << "Error opening file: " << OdomResultPath << std::endl;
            return;
        }

        outFile << std::fixed << " " << odom.x << " " << odom.y << " " << odom.z << " "
        << odom.roll << " " <<odom.pitch << " " << odom.yaw << odom.timestamp-odometryResults[0].timestamp << std::endl;

        outFile.close();
    }

    void laserMapping::transformAssociateToMap(Transformd T_w_pre, Transformd T_wodom_curr, Transformd T_wodom_pre) {

        Transformd T_wodom_pre_curr = T_wodom_pre.inverse() * T_wodom_curr;

        Transformd T_w_curr_predict = T_w_pre * T_wodom_pre_curr;
        q_w_curr = T_w_curr_predict.rot;
        t_w_curr = T_w_curr_predict.pos;

    }

    void laserMapping::transformAssociateToMap() {
        q_w_curr = q_wmap_wodom * q_wodom_curr;
        t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;
    }

    void laserMapping::transformUpdate() {
        q_wmap_wodom = q_w_curr * q_wodom_curr.inverse();
        t_wmap_wodom = t_w_curr - q_wmap_wodom * t_wodom_curr;

    }

    void laserMapping::pointAssociateToMap(PointType const *const pi, PointType *const po) {
        Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
        Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
        po->x = point_w.x();
        po->y = point_w.y();
        po->z = point_w.z();
        po->intensity = pi->intensity;
    }

    void laserMapping::pointAssociateToMap(pcl::PointXYZHSV const *const pi, pcl::PointXYZHSV *const po)
    {
        Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
        Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
        po->x = point_w.x();
        po->y = point_w.y();
        po->z = point_w.z();
        po->h = pi->h;
        po->s = pi->s;
        po->v = pi->v;
    }

    void laserMapping::pointAssociateTobeMapped(PointType const *const pi, PointType *const po) {
        Eigen::Vector3d point_w(pi->x, pi->y, pi->z);
        Eigen::Vector3d point_curr = q_w_curr.inverse() * (point_w - t_w_curr);
        po->x = point_curr.x();
        po->y = point_curr.y();
        po->z = point_curr.z();
        po->intensity = pi->intensity;
    }

    // Data input: add laser feature data (replaces laserFeatureInfoHandler)
    void laserMapping::addLaserFeature(pcl::PointCloud<PointType>::Ptr cornerCloud,
                                        pcl::PointCloud<PointType>::Ptr surfCloud,
                                        pcl::PointCloud<PointType>::Ptr fullCloud,
                                        pcl::PointCloud<PointType>::Ptr realsenseCloud,
                                        const Eigen::Quaterniond& imuPrediction,
                                        double timestamp) {
        mBuf.lock();
        LaserFeatureData data;
        data.corner = cornerCloud;
        data.surf = surfCloud;
        data.fullRes = fullCloud;
        data.realsense = realsenseCloud;
        data.imuPrediction = imuPrediction;
        data.timestamp = timestamp;
        featureBuf.push(data);
        mBuf.unlock();
    }

    // Data input: add IMU odometry (replaces IMUOdometryHandler)
    void laserMapping::addIMUOdometry(double timestamp, const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) {
        mBuf.lock();
        auto odom = std::make_shared<OdomMsg>();
        odom->timestamp = timestamp;
        odom->position = position;
        odom->orientation = orientation;
        imu_odom_buf.addMeas(odom, timestamp);
        timeLatestImuOdometry = timestamp;
        mBuf.unlock();
    }

    // Data input: add visual odometry (replaces visualOdometryHandler)
    void laserMapping::addVisualOdometry(double timestamp, const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
                                          const double* covariance) {
        mBuf.lock();
        auto odom = std::make_shared<OdomMsg>();
        odom->timestamp = timestamp;
        odom->position = position;
        odom->orientation = orientation;
        if (covariance) {
            std::memcpy(odom->covariance, covariance, sizeof(odom->covariance));
        }
        visual_odom_buf.addMeas(odom, timestamp);
        mBuf.unlock();
    }


    void laserMapping::setInitialGuess()
    {
        use_imu_roll_pitch_this_step=false;

        if (initialization == false)  //directly hardset the imu rotation as the first pose
        {
            use_imu_roll_pitch_this_step=true;

            if(use_imu_roll_pitch_this_step)
            {
                // Convert quaternion to RPY using Eigen
                Eigen::Vector3d euler = q_wodom_curr.toRotationMatrix().eulerAngles(0, 1, 2);
                double roll = euler[0], pitch = euler[1], yaw = euler[2];
                printf("[arise_slam] DEBUG: Directly Use IMU Yaw: %f\n", yaw);

                // Create yaw-only quaternion to zero out yaw
                Eigen::Quaterniond yaw_quat(Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()));
                printf("[arise_slam] DEBUG: Start roll, pitch, yaw %f, %f, %f\n", roll, pitch, yaw);

                Eigen::Quaterniond first_orientation = yaw_quat * q_wodom_curr;
                q_w_curr = first_orientation;

                auto q_extrinsic=Eigen::Quaterniond(imu_laser_R);
                q_extrinsic.normalize();
                printf("[arise_slam] DEBUG: q_extrinsic: %f %f %f %f\n", q_extrinsic.w(),q_extrinsic.x(), q_extrinsic.y(), q_extrinsic.z());
                printf("[arise_slam] DEBUG: q_w_curr_pre: %f %f %f %f\n", q_w_curr.w(),q_w_curr.x(), q_w_curr.y(), q_w_curr.z() );

                q_w_curr = q_extrinsic.inverse()*q_w_curr;
                printf("[arise_slam] DEBUG: q_w_curr_pre: %f %f %f %f\n", q_w_curr.w(),q_w_curr.x(), q_w_curr.y(), q_w_curr.z() );

                q_wodom_pre = q_w_curr;
                T_w_lidar.rot=q_w_curr;

                if(slam.local_mode)
                {

                T_w_lidar.pos=Eigen::Vector3d(slam.init_x, slam.init_y, slam.init_z);
                Eigen::Quaterniond quat(
                    Eigen::AngleAxisd(slam.init_yaw, Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(slam.init_pitch, Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(slam.init_roll, Eigen::Vector3d::UnitX())
                );
                T_w_lidar.rot = quat;
                printf("[arise_slam] DEBUG: \033[1;32m  Localization Mode: x: %f y: %f z: %f roll: %f pitch: %f yaw:%f \033[0m\n",
                            slam.init_x,slam.init_y,slam.init_z,slam.init_roll, slam.init_pitch, slam.init_yaw);
                slam.last_T_w_lidar=T_w_lidar;
                }

            }else
            {

                printf("[arise_slam] WARN: start from zero\n");
                q_w_curr = Eigen::Quaterniond(cos(slam.init_yaw / 2), 0, 0, sin(slam.init_yaw / 2));
                q_wodom_pre = Eigen::Quaterniond(cos(slam.init_yaw / 2), 0, 0, sin(slam.init_yaw / 2));
                T_w_lidar.rot=q_w_curr;
                if(slam.local_mode)
                {
                  T_w_lidar.pos=Eigen::Vector3d(slam.init_x, slam.init_y, slam.init_z);
                  Eigen::Quaterniond quat(
                      Eigen::AngleAxisd(slam.init_yaw, Eigen::Vector3d::UnitZ()) *
                      Eigen::AngleAxisd(slam.init_pitch, Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(slam.init_roll, Eigen::Vector3d::UnitX())
                  );
                  T_w_lidar.rot = quat;
                }
            }

        } else if (startupCount>0) // To use IMU orientation for a while for initialization
        {

            printf("[arise_slam] WARN: localization/startup\n");
            use_imu_roll_pitch_this_step = true;
            selectposePrediction();
            if(use_imu_roll_pitch_this_step){
                q_w_curr = T_w_lidar.rot;
            }else{
                q_w_curr = last_T_w_lidar.rot;
            }

            t_w_curr = last_T_w_lidar.pos;
            T_w_lidar.pos = t_w_curr;
            T_w_lidar.rot = q_w_curr;
            startupCount--;
        }
        else
        {

            if (config_.use_imu_roll_pitch)
                use_imu_roll_pitch_this_step=true;
            selectposePrediction();
            q_w_curr = T_w_lidar.rot;
            t_w_curr = T_w_lidar.pos;
        }

        // When trust_fallback_odom is true, override SLAM's initial guess with
        // the fallback odometry position BEFORE ICP runs. This ensures the local
        // map is built at the correct world position, producing clean registered
        // scans. ICP still refines the alignment but starts from ground truth.
        if (config_.trust_fallback_odom && !imu_odom_buf.empty()) {
            Eigen::Vector3d t_odom;
            Eigen::Quaterniond q_odom;
            getOdometryFromTimestamp(imu_odom_buf, timeLaserOdometry, t_odom, q_odom);
            t_w_curr = t_odom;
            q_w_curr = q_odom;
            T_w_lidar.pos = t_odom;
            T_w_lidar.rot = q_odom;
        }

    }

    void laserMapping::selectposePrediction()
    {
        prediction_source =PredictionSource::IMU_ODOM;
        odomAvailable = extractVisualIMUOdometryAndCheck(T_w_lidar);
        if (odomAvailable) {
            return;
        } else {
            // use the incremental imu orientation as the initial guess
            if (imuorientationAvailable == true)
            {
                Eigen::Quaterniond q_w_predict = q_w_curr * q_wodom_pre.inverse() * q_wodom_curr;
                q_w_predict.normalize();
                prediction_source =PredictionSource::IMU_ORIENTATION;
                T_w_lidar.rot = q_w_predict;
                q_wodom_pre = q_wodom_curr;
                T_w_lidar.pos += Eigen::Vector3d{shiftX, shiftY, shiftZ};
                return;
            }
        }
    }

    void laserMapping::getOdometryFromTimestamp(MapRingBuffer<OdomMsgPtr> &buf, const double &timestamp,
                                                Eigen::Vector3d &T, Eigen::Quaterniond &Q) {
        double t_b_i = timestamp;
        auto after_ptr = buf.measMap_.upper_bound(t_b_i);
        if (after_ptr->first < 0.001)
        {
            after_ptr = buf.measMap_.begin();
        }

        if (after_ptr == buf.measMap_.begin())
        {
            Q.x()= after_ptr->second->orientation.x();
            Q.y()= after_ptr->second->orientation.y();
            Q.z()= after_ptr->second->orientation.z();
            Q.w()= after_ptr->second->orientation.x();  // Note: preserving original bug (was orientation.x in original)

            T.x() = after_ptr->second->position.x();
            T.y() = after_ptr->second->position.y();
            T.z() = after_ptr->second->position.z();

        }else
        {

            auto before_ptr = after_ptr;
            before_ptr--;

            double ratio_bi =
                    (t_b_i - before_ptr->first) / (after_ptr->first - before_ptr->first);

            Eigen::Quaterniond q_w_i_before = before_ptr->second->orientation;
            Eigen::Quaterniond q_w_i_after = after_ptr->second->orientation;
            Q = q_w_i_before.slerp(ratio_bi, q_w_i_after);

            Eigen::Vector3d t_w_i_before = before_ptr->second->position;
            Eigen::Vector3d t_w_i_after = after_ptr->second->position;
            T = (1 - ratio_bi) * t_w_i_before + ratio_bi * t_w_i_after;

        }

    }

    void laserMapping::extractRelativeTransform(MapRingBuffer<OdomMsgPtr> &buf, Transformd &T_pre_cur, bool imu_prediction) {

        if (lastOdomAvailable == false)
        {
            lastOdomAvailable = true;
        }
       else
       {

            Eigen::Vector3d t_w_imu_cur;
            Eigen::Quaterniond q_w_imu_cur;
            getOdometryFromTimestamp(buf, timeLaserOdometry, t_w_imu_cur, q_w_imu_cur);
            Eigen::Vector3d t_w_imu_pre;
            Eigen::Quaterniond q_w_imu_pre;
            getOdometryFromTimestamp(buf, timeLaserOdometryPrev, t_w_imu_pre, q_w_imu_pre);

            Transformd T_w_imu_cur(q_w_imu_cur, t_w_imu_cur);
            Transformd T_w_imu_pre(q_w_imu_pre, t_w_imu_pre);
            Transformd T_w_imu_pre_cur = T_w_imu_pre.inverse() * T_w_imu_cur;

            // Extract RPY for debug (using Eigen instead of tf2)
            Eigen::Vector3d euler = T_w_imu_pre_cur.rot.toRotationMatrix().eulerAngles(0, 1, 2);

            T_pre_cur=T_w_imu_pre_cur;
       }
    }

    bool laserMapping::extractVisualIMUOdometryAndCheck(Transformd &T_w_lidar)
    {
        // Assuming VIO won't be available if IMU odom is not available
        if (imu_odom_buf.empty()) {
            return false;
        }

        double imu_odom_first_time, imu_odom_last_time;
        imu_odom_buf.getFirstTime(imu_odom_first_time);
        imu_odom_buf.getLastTime(imu_odom_last_time);

        if ((imu_odom_first_time > timeLaserOdometry) || (imu_odom_last_time < timeLaserOdometry)) {
            printf("[arise_slam] DEBUG: IMU Odometry buffer time not covering laser mapping. Skip odometry initial guess\n");
            return false;
        }


        if (slam.isDegenerate) {
            //check if VIO available
            bool vio_buf_ready = true;
            if (visual_odom_buf.empty()) {
                printf("[arise_slam] DEBUG: Visual Odometry buffer empty.\n");
                vio_buf_ready = false;
            }

            if (vio_buf_ready) {
                double visual_odom_first_time, visual_odom_last_time;
                visual_odom_buf.getFirstTime(visual_odom_first_time);
                visual_odom_buf.getLastTime(visual_odom_last_time);
                if ((visual_odom_first_time > timeLaserOdometryPrev) || (visual_odom_last_time < timeLaserOdometry)) {
                    printf("[arise_slam] DEBUG: Visual Odometry buffer time not covering laser mapping.\n");
                    vio_buf_ready = false;
                }
            }

            if (vio_buf_ready) {

                double t_b_i = timeLaserOdometry;
                auto after_ptr = visual_odom_buf.measMap_.upper_bound(t_b_i);

                if (after_ptr->first < 0.001)
                  {
                        after_ptr = visual_odom_buf.measMap_.begin();
                        vio_buf_ready = false;
                   }else
                    {
                    auto before_ptr = after_ptr;
                    before_ptr--;
                    auto after_init_stat = int(after_ptr->second->covariance[1]);
                    auto before_init_stat = int(before_ptr->second->covariance[1]);
                    auto after_failure_stat = int(after_ptr->second->covariance[0]);
                    auto before_failure_stat = int(before_ptr->second->covariance[0]);
                        if (after_init_stat != 1 || before_init_stat != 1 || after_failure_stat!=0 || before_failure_stat!=0) {
                            printf("[arise_slam] WARN: Visual Odometry current buffer not initialized.\n");
                            vio_buf_ready = false;
                        }
                    }

                    double t_b_prev = timeLaserOdometryPrev;
                    auto after_ptr_prev = visual_odom_buf.measMap_.upper_bound(t_b_prev);

                    if (after_ptr_prev->first < 0.001)
                    {
                        after_ptr_prev = visual_odom_buf.measMap_.begin();
                        vio_buf_ready = false;
                    }else
                    {
                        auto before_ptr_prev = after_ptr_prev;
                        before_ptr_prev--;
                        auto after_init_stat_prev = int(after_ptr_prev->second->covariance[1]);
                        auto before_init_stat_prev = int(before_ptr_prev->second->covariance[1]);
                        auto after_failure_stat_prev = int(after_ptr_prev->second->covariance[0]);
                        auto before_failure_stat_prev = int(before_ptr_prev->second->covariance[0]);
                        if (after_init_stat_prev != 1 || before_init_stat_prev != 1 ||after_failure_stat_prev!=0|| before_failure_stat_prev!=0) {
                            printf("[arise_slam] WARN: Visual Odometry prev buffer not initialized.\n");
                            vio_buf_ready = false;
                        }
                    }

            }

            if (vio_buf_ready) {
                printf("[arise_slam] WARN: VIO is ready for prediction.\n");
                prediction_source = PredictionSource::VISUAL_ODOM;
                Transformd T_pre_cur;
                extractRelativeTransform(visual_odom_buf, T_pre_cur,false);
                T_w_lidar = T_w_lidar * T_pre_cur;

               if(use_imu_roll_pitch_this_step)
               {
                Eigen::Quaterniond q_w_imu_cur;
                Eigen::Vector3d t_w_imu_cur;
                getOdometryFromTimestamp(imu_odom_buf, timeLaserOdometry, t_w_imu_cur, q_w_imu_cur);
                T_w_lidar.rot=q_w_imu_cur;
               }

                return true;
            }
        }

        prediction_source = PredictionSource::IMU_ODOM;
        Transformd T_pre_cur;
        extractRelativeTransform(imu_odom_buf, T_pre_cur,true);
        if (frameCount <= 5 || frameCount % 50 == 0) {
            fprintf(stderr, "[arise_slam] ICP initial guess: T_pre_cur pos=(%.3f,%.3f,%.3f), "
                    "T_w_lidar pos=(%.3f,%.3f,%.3f), imu_odom_buf size=%zu\n",
                    T_pre_cur.pos.x(), T_pre_cur.pos.y(), T_pre_cur.pos.z(),
                    T_w_lidar.pos.x(), T_w_lidar.pos.y(), T_w_lidar.pos.z(),
                    imu_odom_buf.getSize());
        }
        T_w_lidar = T_w_lidar * T_pre_cur;

        if(use_imu_roll_pitch_this_step)
        {
          Eigen::Quaterniond q_w_imu_cur;
          Eigen::Vector3d t_w_imu_cur;
          getOdometryFromTimestamp(imu_odom_buf, timeLaserOdometry, t_w_imu_cur, q_w_imu_cur);
          T_w_lidar.rot=q_w_imu_cur;
        }
        return true;
    }

    LaserMappingOutput laserMapping::getLatestOutput() const {
        return latest_output_;
    }

    // Diagnostic getters — upstream: laserMapping.cpp publishTopic() lines 944-952
    pcl::PointCloud<PointType> laserMapping::getSurroundMap() const {
        return slam.localMap.get5x5LocalMap(slam.pos_in_localmap);
    }

    // Upstream: laserMapping.cpp publishTopic() lines 955-962
    pcl::PointCloud<PointType> laserMapping::getGlobalMap() const {
        return slam.localMap.getAllLocalMap();
    }

    int laserMapping::getPredictionSource() const {
        return static_cast<int>(prediction_source);
    }

    void laserMapping::save_debug_statistic (const std::string file_name)
    {

        slam.kdtree_time_analysis.frameID=frameCount;
        slam.kdtree_time_analysis.timestamp=timeLaserOdometry;

        std::cout << std::endl
             << "Saving important statistic to " << file_name << " ..." << std::endl;
        std::ofstream f;
        f.open(file_name.c_str(), std::ios::out | std::ios::app);
        f << std::fixed;

        double timestamp;
        int frameID;
        float kdtree_build_time;
        float kdtree_query_time;
        timestamp=timeLaserOdometry;
        frameID=frameCount;
        kdtree_build_time=slam.kdtree_time_analysis.kd_tree_building_time;
        kdtree_query_time=slam.kdtree_time_analysis.kd_tree_query_time;

        f <<frameID<<" "<<std::setprecision(6) << timestamp << " "<<kdtree_build_time<<" "<<kdtree_query_time<< std::endl;

        f.close();
    }


    void laserMapping::adjustVoxelSize(int &laserCloudCornerStackNum, int &laserCloudSurfStackNum){

        // Calculate cloud statistics
        bool increase_blind_radius = false;
        if(config_.auto_voxel_size)
        {
            Eigen::Vector3f average(0,0,0);
            int count_far_points = 0;
            for (auto &point : *laserCloudSurfLast)
            {
                average(0) += fabs(point.x);
                average(1) += fabs(point.y);
                average(2) += fabs(point.z);
                if(point.x*point.x + point.y*point.y + point.z*point.z>9){
                    count_far_points++;
                }
            }
            if (count_far_points > 3000)
            {
                increase_blind_radius = true;
            }

            average /= laserCloudSurfLast->points.size();
            slam.stats.average_distance = average(0)*average(1)*average(2);
            if (slam.stats.average_distance < 25)
            {
                config_.lineRes = 0.1;
                config_.planeRes = 0.2;
            }
            else if (slam.stats.average_distance > 65)
            {
                config_.lineRes = 0.4;
                config_.planeRes = 0.8;
            }
            downSizeFilterSurf.setLeafSize(config_.planeRes , config_.planeRes , config_.planeRes );
            downSizeFilterCorner.setLeafSize(config_.lineRes , config_.lineRes , config_.lineRes );
        }

#if 0
        if(increase_blind_radius)
        {
            printf("[arise_slam] DEBUG: increase blind radius\n");
            pcl::CropBox<PointType> boxFilter;
            float min = -2.0;
            float max = -min;
            boxFilter.setMin(Eigen::Vector4f(min, min, min, 1.0));
            boxFilter.setMax(Eigen::Vector4f(max, max, max, 1.0));
            boxFilter.setNegative(true);
            boxFilter.setInputCloud(laserCloudCornerLast);
            boxFilter.filter(*laserCloudCornerLast);
            boxFilter.setInputCloud(laserCloudSurfLast);
            boxFilter.filter(*laserCloudSurfLast);

            printf("[arise_slam] DEBUG: surface size: %zu\n", laserCloudSurfLast->points.size());
            printf("[arise_slam] DEBUG: corner size: %zu\n", laserCloudCornerLast->points.size());
        }
#endif

        laserCloudCornerStack->clear();
        downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
        downSizeFilterCorner.filter(*laserCloudCornerStack);
        laserCloudCornerStackNum = laserCloudCornerStack->points.size();

        laserCloudSurfStack->clear();
        downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
        downSizeFilterSurf.filter(*laserCloudSurfStack);
        laserCloudSurfStackNum = laserCloudSurfStack->points.size();

        slam.localMap.lineRes_ = config_.lineRes;
        slam.localMap.planeRes_ = config_.planeRes;


    }

    void laserMapping::process() {
        while (!featureBuf.empty()) {

                laser_imu_sync = false;

                mBuf.lock();

                auto& feat = featureBuf.front();
                timeLaserOdometry = feat.timestamp;
                timeLaserCloudCornerLast = feat.timestamp;
                timeLaserCloudSurfLast = feat.timestamp;
                timeLaserCloudFullRes = feat.timestamp;

                laserCloudCornerLast = feat.corner;
                laserCloudSurfLast = feat.surf;
                laserCloudFullRes = feat.fullRes;
                if (feat.realsense && !feat.realsense->empty()) {
                    laserCloudRealsense = feat.realsense;
                } else {
                    laserCloudRealsense->clear();
                }

                Eigen::Quaterniond IMUPrediction = feat.imuPrediction;
                IMUPrediction.normalize();

                q_wodom_curr.x() = IMUPrediction.x();
                q_wodom_curr.y() = IMUPrediction.y();
                q_wodom_curr.z() = IMUPrediction.z();
                q_wodom_curr.w() = IMUPrediction.w();

                imuorientationAvailable=true;

                featureBuf.pop();

                // Drain any remaining buffered features (keep only latest)
                while (!featureBuf.empty()) {
                    featureBuf.pop();
                }

                mBuf.unlock();

                setInitialGuess();
                Transformd T_lidar_w = T_w_lidar.inverse();

                if (config_.shift_undistortion) {
                  int laserCloudCornerLastNum = laserCloudCornerLast->points.size();
                  for (int i = 0; i < laserCloudCornerLastNum; i++) {
                    Eigen::Vector3d pt(laserCloudCornerLast->points[i].x, laserCloudCornerLast->points[i].y, laserCloudCornerLast->points[i].z);
                    Eigen::Vector3d pt2 = T_w_lidar * pt;

                    float x3 = pt2.x() + shiftX * laserCloudCornerLast->points[i].intensity / config_.period;
                    float y3 = pt2.y() + shiftY * laserCloudCornerLast->points[i].intensity / config_.period;
                    float z3 = pt2.z() + shiftZ * laserCloudCornerLast->points[i].intensity / config_.period;

                    Eigen::Vector3d pt3 = T_lidar_w * Eigen::Vector3d(x3, y3, z3);

                    laserCloudCornerLast->points[i].x = pt3.x();
                    laserCloudCornerLast->points[i].y = pt3.y();
                    laserCloudCornerLast->points[i].z = pt3.z();
                  }

                  int laserCloudSurfLastNum = laserCloudSurfLast->points.size();
                  for (int i = 0; i < laserCloudSurfLastNum; i++) {
                    Eigen::Vector3d pt(laserCloudSurfLast->points[i].x, laserCloudSurfLast->points[i].y, laserCloudSurfLast->points[i].z);
                    Eigen::Vector3d pt2 = T_w_lidar * pt;

                    float x3 = pt2.x() + shiftX * laserCloudSurfLast->points[i].intensity / config_.period;
                    float y3 = pt2.y() + shiftY * laserCloudSurfLast->points[i].intensity / config_.period;
                    float z3 = pt2.z() + shiftZ * laserCloudSurfLast->points[i].intensity / config_.period;

                    Eigen::Vector3d pt3 = T_lidar_w * Eigen::Vector3d(x3, y3, z3);

                    laserCloudSurfLast->points[i].x = pt3.x();
                    laserCloudSurfLast->points[i].y = pt3.y();
                    laserCloudSurfLast->points[i].z = pt3.z();
                  }

                  int laserCloudFullResNum = laserCloudFullRes->points.size();
                  for (int i = 0; i < laserCloudFullResNum; i++) {
                    Eigen::Vector3d pt(laserCloudFullRes->points[i].x, laserCloudFullRes->points[i].y, laserCloudFullRes->points[i].z);
                    Eigen::Vector3d pt2 = T_w_lidar * pt;

                    float x3 = pt2.x() + shiftX * float(i) / float(laserCloudFullResNum);
                    float y3 = pt2.y() + shiftY * float(i) / float(laserCloudFullResNum);
                    float z3 = pt2.z() + shiftZ * float(i) / float(laserCloudFullResNum);

                    Eigen::Vector3d pt3 = T_lidar_w * Eigen::Vector3d(x3, y3, z3);

                    laserCloudFullRes->points[i].x = pt3.x();
                    laserCloudFullRes->points[i].y = pt3.y();
                    laserCloudFullRes->points[i].z = pt3.z();
                  }
                }

                TicToc t_whole;
                int laserCloudCornerStackNum=0;
                int laserCloudSurfStackNum=0;
                adjustVoxelSize(laserCloudCornerStackNum, laserCloudSurfStackNum);

                if (!laserCloudRealsense->empty())
                {
                    *laserCloudSurfStack = *laserCloudSurfStack + *laserCloudRealsense;
                    laserCloudSurfStackNum = laserCloudSurfStack->points.size();
                }

                Eigen::Quaterniond imu_roll_pitch = Eigen::Quaterniond::Identity();
                if (use_imu_roll_pitch_this_step)
                {
                    // Extract roll/pitch from T_w_lidar orientation using Eigen
                    Eigen::Vector3d euler = T_w_lidar.rot.toRotationMatrix().eulerAngles(0, 1, 2);
                    double imu_roll = euler[0], imu_pitch = euler[1], imu_yaw = euler[2];
                    printf("[arise_slam] Using IMU Roll Pitch in ICP: %f %f %f\n", imu_roll, imu_pitch, imu_yaw);
                    // Create quaternion with only roll and pitch (no yaw)
                    imu_roll_pitch = Eigen::AngleAxisd(imu_pitch, Eigen::Vector3d::UnitY()) *
                                     Eigen::AngleAxisd(imu_roll, Eigen::Vector3d::UnitX());
                }


                if(prediction_source==PredictionSource::VISUAL_ODOM) {

                    slam.OptSet.use_imu_roll_pitch=use_imu_roll_pitch_this_step;
                    slam.OptSet.imu_roll_pitch=imu_roll_pitch;
                    slam.Localization(initialization, LidarSLAM::PredictionSource::VISUAL_ODOM, T_w_lidar,
                                    laserCloudCornerStack, laserCloudSurfStack,timeLaserOdometry);

                }else {

                    slam.OptSet.use_imu_roll_pitch=use_imu_roll_pitch_this_step;
                    slam.OptSet.imu_roll_pitch=imu_roll_pitch;
                    slam.Localization(initialization, LidarSLAM::PredictionSource::IMU_ODOM, T_w_lidar,
                                laserCloudCornerStack, laserCloudSurfStack,timeLaserOdometry);

                }


                // When trust_fallback_odom, keep the odom position (set in
                // setInitialGuess) instead of ICP's result. ICP ran and updated
                // the local map, but we trust the external position source.
                if (config_.trust_fallback_odom && !imu_odom_buf.empty()) {
                    // ICP may have refined orientation, keep that
                    q_w_curr = slam.T_w_lidar.rot;
                    // But position comes from fallback odom (set before ICP)
                    t_w_curr = T_w_lidar.pos;
                    slam.T_w_lidar.pos = T_w_lidar.pos;
                } else {
                    q_w_curr = slam.T_w_lidar.rot;
                    t_w_curr = slam.T_w_lidar.pos;
                }

                T_w_lidar.rot=slam.T_w_lidar.rot;
                T_w_lidar.pos=slam.T_w_lidar.pos;

                last_T_w_lidar=slam.T_w_lidar;

                timeLaserOdometryPrev=timeLaserOdometry;


                startupCount=slam.startupCount;

                frameCount++;

                slam.frame_count=frameCount;
                slam.laser_imu_sync=laser_imu_sync;

                initialization = true;

                // Compute shift for undistortion
                double newPoseX = t_w_curr.x();
                double newPoseY = t_w_curr.y();
                double newPoseZ = t_w_curr.z();

                shiftX = (1.0 - config_.shift_avg_ratio) * shiftX + config_.shift_avg_ratio * (newPoseX - poseX);
                shiftY = (1.0 - config_.shift_avg_ratio) * shiftY + config_.shift_avg_ratio * (newPoseY - poseY);
                shiftZ = (1.0 - config_.shift_avg_ratio) * shiftZ + config_.shift_avg_ratio * (newPoseZ - poseZ);

                poseX = newPoseX;
                poseY = newPoseY;
                poseZ = newPoseZ;

                // Transform full res to world frame for output
                int laserCloudFullResNum = laserCloudFullRes->points.size();
                for (int i = 0; i < laserCloudFullResNum; i++) {
                    PointType const *const &pi = &laserCloudFullRes->points[i];
                    if (pi->x* pi->x+ pi->y * pi->y + pi->z* pi->z < 0.01)
                    {
                        continue;
                    }
                    pointAssociateToMap(&laserCloudFullRes->points[i],
                                        &laserCloudFullRes->points[i]);
                }

                // Build output
                latest_output_.timestamp = timeLaserOdometry;
                latest_output_.position = t_w_curr;
                latest_output_.orientation = q_w_curr;
                latest_output_.is_degenerate = slam.isDegenerate;
                latest_output_.registered_scan = laserCloudFullRes;

                // Diagnostic: prediction source (upstream line 930-942)
                latest_output_.prediction_source = static_cast<int>(prediction_source);

                // Diagnostic: surround map — upstream line 944-952 (every 5 frames)
                if (frameCount % 5 == 0) {
                    latest_output_.surround_map = pcl::PointCloud<PointType>::Ptr(
                        new pcl::PointCloud<PointType>(
                            slam.localMap.get5x5LocalMap(slam.pos_in_localmap)));
                }

                // Diagnostic: global map — upstream line 955-962 (every 20 frames)
                if (frameCount % 20 == 0) {
                    latest_output_.global_map = pcl::PointCloud<PointType>::Ptr(
                        new pcl::PointCloud<PointType>(
                            slam.localMap.getAllLocalMap()));
                }

                // Diagnostic: incremental odometry — upstream line 1042-1075
                if (initialization) {
                    latest_output_.incremental_position = laser_incremental_T.pos;
                    latest_output_.incremental_orientation = laser_incremental_T.rot;
                } else {
                    latest_output_.incremental_position = t_w_curr;
                    latest_output_.incremental_orientation = q_w_curr;
                }

                // Diagnostic: optimization stats — upstream line 1097-1111
                latest_output_.stats = slam.stats;

                if (output_callback_) {
                    output_callback_(latest_output_);
                }
            }
    }




#pragma clang diagnostic pop

} // namespace arise_slam
