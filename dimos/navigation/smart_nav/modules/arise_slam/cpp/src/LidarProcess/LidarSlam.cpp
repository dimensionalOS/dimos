
// LOCAL
#include "arise_slam_mid360/LidarProcess/LidarSlam.h"



double pose_parameters[7] = {0, 0, 0, 0, 0, 0, 1};
Eigen::Map<Eigen::Vector3d> T_w_curr(pose_parameters);
Eigen::Map<Eigen::Quaterniond> Q_w_curr(pose_parameters + 3);

namespace arise_slam {

    LidarSLAM::LidarSLAM() {
        EdgesPoints.reset(new PointCloud());
        PlanarsPoints.reset(new PointCloud());
        WorldEdgesPoints.reset(new PointCloud());
        WorldPlanarsPoints.reset(new PointCloud());
    
    }

    void LidarSLAM::Localization(bool initialization, PredictionSource predictodom, Transformd position,
                                 pcl::PointCloud<Point>::Ptr edge_point,
                                 pcl::PointCloud<Point>::Ptr planner_point,double timeLaserOdometry) {
        bInitilization = initialization;
        T_w_lidar = position;
        Transformd T_w_inital_guess(T_w_lidar);

       

        int Good_Planner_Feature_Num = 0;

        this->EdgesPoints->clear();
        this->PlanarsPoints->clear();

        this->EdgesPoints->reserve(edge_point->size());
        this->PlanarsPoints->reserve(planner_point->size());

        for (const Point &p : *edge_point) {
            this->EdgesPoints->push_back(p);
        }

        for (const Point &p : *planner_point) {
            this->PlanarsPoints->push_back(p);
        }

        if (not bInitilization) {
            localMap.setOrigin(T_w_lidar.pos);

            this->WorldEdgesPoints->clear();
            this->WorldEdgesPoints->points.reserve(this->EdgesPoints->size());
            this->WorldEdgesPoints->header = this->EdgesPoints->header;


            for (const Point &p : *this->EdgesPoints) {
                WorldEdgesPoints->push_back(TransformPointd(p, this->T_w_lidar));
            }
            localMap.addEdgePointCloud(*this->WorldEdgesPoints);

            this->WorldPlanarsPoints->clear();
            this->WorldPlanarsPoints->points.reserve(this->PlanarsPoints->size());
            this->WorldPlanarsPoints->header = this->PlanarsPoints->header;


            for (const Point &p : *this->PlanarsPoints) {
                WorldPlanarsPoints->push_back(TransformPointd(p, this->T_w_lidar));
            }
            localMap.addSurfPointCloud(*this->WorldPlanarsPoints);

            lasttimeLaserOdometry=timeLaserOdometry;
        } else {


            this->pos_in_localmap = localMap.shiftMap(T_w_lidar.pos);
            T_w_curr=this->T_w_lidar.pos;
            Q_w_curr=T_w_lidar.rot;
            

            auto[edgePointsFromMapNum, planarPointsFromMapNum] =
            localMap.get5x5LocalMapFeatureSize(this->pos_in_localmap);
        

            // for debug usage
            stats.edge_features_from_map = edgePointsFromMapNum;
            stats.plane_features_from_map = planarPointsFromMapNum;
            stats.edge_features_from_scan = EdgesPoints->size();
            stats.plane_features_from_scan = PlanarsPoints->size();

            TicToc t_opt;

            //tbb::concurrent_vector<OptimizationParameter> OptimizationData;

            OptimizationData.clear();
            if (planarPointsFromMapNum > 50) {

                for (size_t icpIter = 0; icpIter < this->LocalizationICPMaxIter; ++icpIter) {
                    ResetDistanceParameters();
                    Good_Planner_Feature_Num = 0;
             
                     //TODO: Achieve the TBB to acclerate the process
                    if (not this->EdgesPoints->empty()) {
                    //     auto compute_func = [this](const tbb::blocked_range<size_t> &range) {
                        for (size_t edge_index = 0;
                             edge_index != this->EdgesPoints->points.size(); edge_index++) {
                             const Point &currentPoint = this->EdgesPoints->points[edge_index];
                             auto optimize_data = this->ComputeLineDistanceParameters(
                                    this->localMap, currentPoint);

                            MatchingResult rejection_index = optimize_data.match_result;
                            if (rejection_index == MatchingResult::SUCCESS) {
                                OptimizationData.push_back(optimize_data);
                            }
                            // statistic match result
                            this->MatchRejectionHistogramLine[rejection_index]++;
                        }
                        //   };

                         //tbb::blocked_range<size_t> range(0,this->EdgesPoints->points.size());
                       //  compute_func(range);
                        // tbb::parallel_for(range, compute_func);
                    }  // loop edge edge feature end

                    double sampling_rate = -1.0;
                    int laserCloudSurfStackNum=this->PlanarsPoints->points.size();

                    if(laserCloudSurfStackNum > OptSet.max_surface_features)
                        sampling_rate = 1.0*OptSet.max_surface_features/laserCloudSurfStackNum;

                    if (not this->PlanarsPoints->empty()) {
                            // auto compute_func = [this](const tbb::blocked_range<size_t>&range) {
                        for (size_t planar_index = 0;
                             planar_index != this->PlanarsPoints->points.size();
                             planar_index++) {
                            if(sampling_rate>0.0)
                            {
                                double remainder = fmod(planar_index*sampling_rate, 1.0);
                                if (remainder + 0.001 > sampling_rate)
                                    continue;
                            }

                            const Point &currentPoint =
                                    this->PlanarsPoints->points[planar_index];
                            auto optimize_data = this->ComputePlaneDistanceParameters(
                                    this->localMap, currentPoint);

                            MatchingResult rejection_index = optimize_data.match_result;
                            if (rejection_index == MatchingResult::SUCCESS) {
                                OptimizationData.push_back(optimize_data);
                                std::array<int, 4> obs = optimize_data.feature.observability;
                                this->PlaneFeatureHistogramObs[obs.at(0)]++;
                                this->PlaneFeatureHistogramObs[obs.at(1)]++;
                                this->PlaneFeatureHistogramObs[obs.at(2)]++;
                                // this->PlaneFeatureHistogramObs[obs.at(3)]++;
                            }
                            // statistic match result
                            this->MatchRejectionHistogramPlane[rejection_index]++;
                        }
                       //   };
                         //  tbb::blocked_range<size_t> range(0,this->PlanarsPoints->points.size());
                          // compute_func(range);
                       // tbb::parallel_for(range, compute_func);
                    }  // loop  planar feature end

                   
                   // plane match statistics (first ICP iteration only, for debug)
                   if(icpIter==0)
                   {
                       // Plane match histogram stored in MatchRejectionHistogramPlane
                   }


                //    std::cout<<"SUCCESS:"<<MatchRejectionHistogramLine.at(0)<<std::endl;
                //    std::cout<<"NOT_ENOUGH_NEIGHBORS:"<<MatchRejectionHistogramLine.at(1)<<std::endl;
                //    std::cout<<"NEIGHBORS_TOO_FAR:"<<MatchRejectionHistogramLine.at(2)<<std::endl;
                //    std::cout<<"BAD_PCA_STRUCTURE:"<<MatchRejectionHistogramLine.at(3)<<std::endl;
                //    std::cout<<"INVAVLID_NUMERICAL:"<<MatchRejectionHistogramLine.at(4)<<std::endl;
                //    std::cout<<"MSE_TOO_LARGE:"<<MatchRejectionHistogramLine.at(5)<<std::endl;
                //    std::cout<<"UNKNON:"<<MatchRejectionHistogramLine.at(6)<<std::endl;

                // step construct ceres optimization problem
                //                double lossScale = this->LocalizationInitLossScale +
                //                static_cast<double>(icpIter) *
                //                        (this->LocalizationFinalLosssScale -
                //                        this->LocalizationInitLossScale) / (1.0 *
                //                        this->LocalizationICPMaxIter);
                    
                  

                    ceres::Problem::Options problem_options;

                    ceres::Problem problem(problem_options);

                    problem.AddParameterBlock(pose_parameters, 7,
                                              new PoseLocalParameterization());
                    // ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);

#if 0
                    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                    ceres::Manifold *q_parameterization =
                            new ceres::EigenQuaternionParameterization();
                    problem.AddParameterBlock(pose_parameters, 3);
                    problem.AddParameterBlock(pose_parameters + 3, 4,q_parameterization);
#endif
                 
                    int surf_num = 0; int edge_num=0;
                        for (unsigned int k = 0; k < OptimizationData.size(); ++k) {
                        if (OptimizationData.at(k).feature_type == FeatureType::EdgeFeature) {
                            ceres::CostFunction *cost_function = new EdgeAnalyticCostFunction(
                                    OptimizationData.at(k).Xvalue,
                                    OptimizationData.at(k).corres.first,
                                    OptimizationData.at(k).corres.second);
                            // Use a robustifier to limit the contribution of an outlier match
                            auto *robustifier =
                                    new ceres::TukeyLoss(std::sqrt(3*localMap.lineRes_));
                            // Weight the contribution of the given match by its reliability
                            auto *Robustifier = new ceres::ScaledLoss(
                                    robustifier, OptimizationData.at(k).residualCoefficient,
                                    ceres::TAKE_OWNERSHIP);
                            problem.AddResidualBlock(cost_function, Robustifier,
                                                     pose_parameters);
                         edge_num++;
                        }

                        if (OptimizationData.at(k).feature_type ==
                            FeatureType::PlaneFeature) {
                            Good_Planner_Feature_Num++;
                            ceres::CostFunction *cost_function =
                                    new SurfNormAnalyticCostFunction(
                                            OptimizationData.at(k).Xvalue,
                                            OptimizationData.at(k).NormDir,
                                            OptimizationData.at(k).negative_OA_dot_norm);
                            // Use a robustifier to limit the contribution of an outlier match
                            auto *robustifier =
                                    new ceres::TukeyLoss(std::sqrt(3*localMap.planeRes_));
                            // Weight the contribution of the given match by its reliability
                            auto *Robustifier = new ceres::ScaledLoss(
                                    robustifier, OptimizationData.at(k).residualCoefficient,
                                    ceres::TAKE_OWNERSHIP);
                            problem.AddResidualBlock(cost_function, Robustifier,
                                                     pose_parameters);

                        surf_num++;

                        }
                    }

             
#if 1
                    // add absolute constraints. Specificaly for shaft environments
                    // Note: will turn off for other environments since we are not able to provide reliable information matrix!
   
                    Eigen::Mat66d information;
                    information.setIdentity();
                    information(0, 0) =
                            (1 - lidarOdomUncer.uncertainty_x) * std::max(50, int(Good_Planner_Feature_Num*0.1))* Visual_confidence_factor;
                    information(1, 1) =
                            (1 - lidarOdomUncer.uncertainty_y) * std::max(50, int(Good_Planner_Feature_Num*0.1))* Visual_confidence_factor;
                    information(2, 2) =
                            (1 - lidarOdomUncer.uncertainty_z) * std::max(50, int(Good_Planner_Feature_Num*0.1))* Visual_confidence_factor;

                    information(3, 3) = std::max(10, int(Good_Planner_Feature_Num*0.01)) * Visual_confidence_factor;
                    information(4, 4) = std::max(10, int(Good_Planner_Feature_Num*0.01)) * Visual_confidence_factor;
                    information(5, 5) = std::max(5, int(Good_Planner_Feature_Num*0.001)) * 0;
                    
                    if ((predictodom == PredictionSource::VISUAL_ODOM || predictodom == PredictionSource::IMU_ODOM)
                        and isDegenerate == true and Visual_confidence_factor!=0) {
                        printf("[arise_slam] ADD Absolute factor in lasermapping.\n");
                        SE3AbsolutatePoseFactor *absolutatePoseFactor =
                                new SE3AbsolutatePoseFactor(position, information);
                        problem.AddResidualBlock(absolutatePoseFactor, nullptr, pose_parameters);
                    }
                     
                  
#endif
                    Transformd previous_T(T_w_lidar);
                    ceres::Solver::Options options;
                    options.max_num_iterations = 4;
                    options.linear_solver_type = ceres::DENSE_QR;
                    options.minimizer_progress_to_stdout = false;
                    options.check_gradients = false;
                    options.gradient_check_relative_precision = 1e-4;
                    ceres::Solver::Summary summary;
                    ceres::Solve(options, &problem, &summary);
                    //std::cout << summary.BriefReport()<<"\n";

                    bool converged = (summary.termination_type == ceres::CONVERGENCE);

                 //   LOG(INFO)<<"\033[1;32m covergence status: .\033[0m"<<converged;
                //     if (converged || true) {
                    
                //     this->T_w_lidar.rot = Q_w_curr;
                //     this->T_w_lidar.pos = T_w_curr;   
                //     } else {
                //            LOG(INFO) << "\n###############"
                //                "\n  solve failed "
                //                "\n###############";
                //   {
                //     char tmp;
                //     std::cin >> tmp;
                //   }
                // }   
                 
                   

                    this->T_w_lidar.rot = Q_w_curr;
                    this->T_w_lidar.pos = T_w_curr;


                  
   

                    Transformd incremental_T = previous_T.inverse() * T_w_lidar;
                    stats.translation_norm = incremental_T.pos.norm();
                    stats.rotation_norm =
                            2 * atan2(incremental_T.rot.vec().norm(), incremental_T.rot.w());

   

                    if (OptSet.use_imu_roll_pitch) {
                        // Extract roll, pitch, yaw from current quaternion using Eigen
                        Eigen::Vector3d euler = T_w_lidar.rot.toRotationMatrix().eulerAngles(0, 1, 2);
                        double roll = euler[0], pitch = euler[1], yaw = euler[2];

                        // Construct yaw-only quaternion
                        Eigen::Quaterniond yaw_quat(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));

                        // Combine yaw with IMU roll/pitch
                        Eigen::Quaterniond orientation = yaw_quat * OptSet.imu_roll_pitch;
                        Q_w_curr = orientation;
                        T_w_lidar.rot = Q_w_curr;
                    }

                    if ((summary.num_successful_steps == 1) ||
                        (icpIter == this->LocalizationICPMaxIter - 1)) {

                        this->LocalizationUncertainty =
                                EstimateRegistrationError(problem, 100);

                        break;
                    }
                }
            } 
            else{
                std::cout<<"Not enough features for optimization"<<std::endl;
            }

            MannualYawCorrection();
        
            double time_elapsed = t_opt.toc();
            Transformd total_incremental_T;
            total_incremental_T = T_w_inital_guess.inverse() * T_w_lidar;
            double total_translation = (total_incremental_T).pos.norm();
            double total_rotation = 2 * atan2(total_incremental_T.rot.vec().norm(), total_incremental_T.rot.w());
            Transformd diff_from_last_T = last_T_w_lidar.inverse() * T_w_lidar;

            double translation_from_last = diff_from_last_T.pos.norm();
            double rotation_from_last = 2 * atan2(diff_from_last_T.rot.vec().norm(), diff_from_last_T.rot.w());
            last_T_w_lidar=T_w_lidar;

            bool acceptResult = true;
            double delta_t = timeLaserOdometry - lasttimeLaserOdometry;

            if (translation_from_last/delta_t > OptSet.velocity_failure_threshold)
            {
                T_w_lidar = last_T_w_lidar;
                startupCount=5;
                acceptResult = false;
                printf("[arise_slam] large motion detected, ignoring predictor for a while\n");
            }

            // Try to not accumulate noise when being still
            if ( (translation_from_last < 0.02 && rotation_from_last < 0.005) && stats.edge_features_from_map > 10 && stats.plane_features_from_map > 50)
            {
                acceptResult = false;
                if(translation_from_last < 0.005 && rotation_from_last < 0.005)
                {
                    T_w_lidar = last_T_w_lidar;
                }
                printf("[arise_slam] very small motion, not accumulating. %f\n", translation_from_last);
            }

            lasttimeLaserOdometry=timeLaserOdometry;


        
            localMap.shiftMap(this->T_w_lidar.pos);

            TicToc t_add_feature;
            this->WorldEdgesPoints->clear();

            this->WorldEdgesPoints->points.reserve(this->EdgesPoints->size());
            this->WorldEdgesPoints->header = this->EdgesPoints->header;

            for (const Point &p : *this->EdgesPoints) {
                WorldEdgesPoints->push_back(TransformPointd(p, this->T_w_lidar));
            }

            if(acceptResult)
            localMap.addEdgePointCloud(*this->WorldEdgesPoints);

            this->WorldPlanarsPoints->clear();

            this->WorldPlanarsPoints->points.reserve(this->PlanarsPoints->size());
            this->WorldPlanarsPoints->header = this->PlanarsPoints->header;


            for (const Point &p : *this->PlanarsPoints) {
                WorldPlanarsPoints->push_back(TransformPointd(p, this->T_w_lidar));
            }

            if(acceptResult)
            localMap.addSurfPointCloud(*this->WorldPlanarsPoints);
            kdtree_time_analysis.kd_tree_building_time = t_add_feature.toc();

        }
    }

    void LidarSLAM::ComputePointInitAndFinalPose(
            LidarSLAM::MatchingMode matchingMode, const LidarSLAM::Point &p,
            Eigen::Vector3d &pInit, Eigen::Vector3d &pFinal) {
        // Undistortion can only be done during Localization step
        const bool is_local_lization_step =
                matchingMode == MatchingMode::LOCALIZATION;
        const Eigen::Vector3d pos = p.getVector3fMap().cast<double>();

        // !!! 需要注意的是在定位阶段, 才会进行去畸变,
        // 如果畸变模式选择是OPTIMIZATION时 则同时优化起始点和终止点的位置
        if (this->Undistortion == UndistortionMode::OPTIMIZED and
            is_local_lization_step) {
            // pInit = pos;
            // pFinal = this->WithinFrameMotion(static_cast<double>(p.time)) * pos;

        } else if (this->Undistortion == UndistortionMode::APPROXIMATED and
                   is_local_lization_step) {
            // 先把点投影到末端点, 然后构造一个虚拟的起始点,
            // 这样优化的激光的位姿就是整个相机的位姿
            //   pFinal = this->WithinFrameMotion(p.time) * pos;
            //   pInit = this->T_w_lidar.inverse() * pFinal;
        } else {
            pInit = pos;
            pFinal = this->T_w_lidar * pos;
        }
    }

    LidarSLAM::OptimizationParameter LidarSLAM::ComputeLineDistanceParameters(
            LocalMap &local_map, const LidarSLAM::Point &p) {
        // step1 transform the point using current pose estimation
        // Rigid transform or time continuous motion model to take into
        // account the rolling shutter distortion
        Eigen::Vector3d pInit, pFinal;
        this->ComputePointInitAndFinalPose(MatchingMode::LOCALIZATION, p, pInit,
                                           pFinal);

        size_t min_neighbors;
        size_t requiredNearest;
       // double eigen_value_ratio;
        double square_max_dist;

        min_neighbors = this->LocalizationMinmumLineNeighborRejection;
        requiredNearest = this->LocalizationLineDistanceNbrNeighbors;
       // eigen_value_ratio = this->LocalizationLineDistancefactor;
        square_max_dist = 3*local_map.lineRes_;

        std::vector<Point> nearest_pts;
        std::vector<float> nearest_dist;

        Point pFinal_query;
        pFinal_query.x = pFinal.x();
        pFinal_query.y = pFinal.y();
        pFinal_query.z = pFinal.z();

#if 1

        bool bFind = local_map.nearestKSearchSpecificEdgePoint(
                pFinal_query, nearest_pts, nearest_dist, requiredNearest,
                static_cast<float>(this->LocalizationLineMaxDistInlier));

#endif

        //  bool
        //  bFind=local_map.nearestKSearchEdgePoint(pFinal_query,nearest_pts,nearest_dist);

        OptimizationParameter result;
        // if not find near neighborhood
        if (not bFind) {
            result.match_result = MatchingResult::NOT_ENOUGH_NEIGHBORS;
            return result;
        }

        if (nearest_pts.size() < min_neighbors) {
            result.match_result = MatchingResult::NOT_ENOUGH_NEIGHBORS;
            return result;
        }

        requiredNearest = nearest_pts.size();

        // if the nearest edges are too far from the current edge keypoint.
        // we skip this point
        if (nearest_dist.back() > square_max_dist) {
            result.match_result = MatchingResult::NEIGHBORS_TOO_FAR;
            return result;
        }

        // Check if neighborhood is a good line candidate with PCA

        // Compute PCA to determine best line approximation
        // of the requiredNearest nearest edges points extracted.
        // Thanks to the PCA we will check the shape of the
        // neighborhood and keep it if it is well distributed along a line
        Eigen::MatrixXd data(requiredNearest, 3);
        for (size_t k = 0; k < requiredNearest; k++) {
            const Point &pt = nearest_pts[k];
            data.row(k) << pt.x, pt.y, pt.z;
        }

        Eigen::Vector3d mean;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig = ComputePCA(data, mean);

        // PCA eigenvalues
        Eigen::Vector3d D = eig.eigenvalues();

        // If the first eigen value is significantly higher than the sencond one.
        // it means that the sourrounding points are distributed on an edge line.
        // Otherwise, discard this bad unstructured neighborhood.
        if (D(2) < min_neighbors * D(1)) {
            result.match_result = BAD_PCA_STRUCTURE;

            return result;
        }

        // Compute point-to-line optimization parameters

        // n is the director vector of the line
        Eigen::Vector3d n = eig.eigenvectors().col(2);

        // A = (I-n*n.t).t*(I-n*n.t) = (I-n*n.t)^2
        // since (I-n*n.t) is a symmetric matrix
        // then it comes A (I-n*n.t)^2 = (I-n*n.t)
        // since A is the matrix of a projection endomorphism

        Eigen::Matrix3d A = Eigen::Matrix3d::Identity() - n * n.transpose();

        // Check parameters validity

        // It would bw the case if P1 = P2, for instance if the sensor has some dual
        // returns that hit the same point.
        if (!std::isfinite(A(0, 0))) {
            result.match_result = MatchingResult::INVAVLID_NUMERICAL;

            return result;
        }

        // Evaluate the distance from the fitted line distribution of the neighborhood
        double meanSquareDist = 0.;
        for (const auto &pt : nearest_pts) {
            Eigen::Vector3d Xtemp{pt.x, pt.y, pt.z};
            double squareDist = (Xtemp - mean).transpose() * A * (Xtemp - mean);

            // CHECK invalidate all neighborhood even if only one point is bad?
            if (squareDist > square_max_dist) {
                result.match_result = MatchingResult::MSE_TOO_LARGE;
                return result;
            }

            meanSquareDist += squareDist;
        }

        meanSquareDist /= static_cast<double>(requiredNearest);

        // Add valid parameters for later optimization

        // Quality score of the point-to-line match
        double fitQualityCoeff = 1.0 - std::sqrt(meanSquareDist / square_max_dist);

        Eigen::Vector3d point_a, point_b;
        Eigen::Vector3d point_on_line = mean;

        point_a = 0.1 * n + point_on_line;
        point_b = -0.1 * n + point_on_line;



        // store the distance parameter values
        result.feature_type = FeatureType::EdgeFeature;
        result.match_result = MatchingResult::SUCCESS;
        result.Avalue = A;
        result.Pvalue = mean;
        result.Xvalue = pInit;
        result.corres = std::make_pair(point_a, point_b);
        result.TimeValue =
                static_cast<double>(1.0);  // TODO:should be the point cloud time
        result.residualCoefficient = fitQualityCoeff;

        return result;
    }

    LidarSLAM::OptimizationParameter LidarSLAM::ComputePlaneDistanceParameters(
            LocalMap &local_map, const Point &p) {
        // Transform the point using the current pose estimation
        // Rigid transform or time continuous motion model to take into
        // account the rolling shutter distortion
        Eigen::Vector3d pInit, pFinal;
        this->ComputePointInitAndFinalPose(MatchingMode::LOCALIZATION, p, pInit,
                                           pFinal);

        // Get neighboring points in previous set of keypoints
        size_t requiredNearest;
       // double significantlyFactor1;
       // double significantlyFactor2;
        double squredMaxDist;

        //significantlyFactor1 = this->LocalizationPlaneDistancefactor1;
        //significantlyFactor2 = this->LocalizationPlaneDistancefactor2;
        requiredNearest = this->LocalizationPlaneDistanceNbrNeighbors;
        squredMaxDist = 3*local_map.planeRes_;

        std::vector<Point> nearest_pts;
        std::vector<float> nearest_dist;

        Point pFinal_query;
        pFinal_query.x = pFinal.x();
        pFinal_query.y = pFinal.y();
        pFinal_query.z = pFinal.z();

        TicToc kdtree_query_feature_time;
        auto bFind = local_map.nearestKSearchSurf(pFinal_query, nearest_pts,
                                                  nearest_dist, requiredNearest);
        kdtree_time_analysis.kd_tree_query_time = kdtree_query_feature_time.toc();

        OptimizationParameter result;
        // if not find near neighborhood

        if (not bFind) {
            result.match_result = MatchingResult::NOT_ENOUGH_NEIGHBORS;
            return result;
        }

        if (nearest_dist.size() < 5 or nearest_pts.size() < 5) {
            bFind = false;
            result.match_result = MatchingResult::NOT_ENOUGH_NEIGHBORS;
            return result;
        }

#if 1
        if (nearest_dist.back() > squredMaxDist) {
            result.match_result = MatchingResult::NEIGHBORS_TOO_FAR;
            return result;
        }
#endif
        // Check if neighborhood is a good plane candidate with PCA

        // Compute PCA to determine best plane approximation
        // of the requiredNearest egdes point extracted.
        // Thanks to the PCA, we will check the shape of the neighborhood
        // and keep it if it is will distributed along a plane.
#if 1
        Eigen::MatrixXd data(requiredNearest, 3);
        for (size_t k = 0; k < requiredNearest; k++) {
            const Point &pt = nearest_pts[k];
            data.row(k) << pt.x, pt.y, pt.z;
        }

        Eigen::Vector3d mean;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig = ComputePCA(data, mean);

        // PCA eigenvalues
        Eigen::Vector3d D = eig.eigenvalues();
#endif

        // If the second eigen values is cloese the highest one and bigger than
        // the smallest one, it means the point are distribution along a plane.
        // Otherwise, discard this bad unstructured neighborhood.


#if 0
        if (significantlyFactor2 * D(1) < D(2) ||
            D(1) < significantlyFactor1 * D(0)) {
            result.match_result = MatchingResult::BAD_PCA_STRUCTURE;
            return result;
        }

        // Compute point-to plane optimization paramters

        // n is the normal vector of the plane
        Eigen::Vector3d n = eig.eigenvectors().col(0);
        Eigen::Matrix3d A = n * n.transpose();

        // It would be the case if P1 = P2, P1 = P3 or P2 = P3,
        // for instance if the sensor has some dual returns that
        // hit the same point.

        if (!std::isfinite(A(0, 0))) {
            result.match_result = MatchingResult::INVAVLID_NUMERICAL;
            return result;
        }

        // Evaluate the distance from the fitted plane distribution of the
        // neighborhood
        double mean_square_dist = 0.;
        for (auto &pt : nearest_pts) {
            Eigen::Vector3d Xtemp{pt.x, pt.y, pt.z};
            double square_dist = (Xtemp - mean).transpose() * A * (Xtemp - mean);
            // CHECK invalidate all neighborhood even if only one point is bad?
            if (square_dist > squredMaxDist) {
                result.match_result = MatchingResult::MSE_TOO_LARGE;
                return result;
            }

            mean_square_dist += square_dipublishObservibilityrt(mean_square_dist / squredMaxDist);
        }
#endif

        Eigen::Matrix<double, 5, 3> matA0;
        Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();

        if (bFind) {
            if (nearest_dist.at(4) < squredMaxDist) {
                for (int j = 0; j < 5; j++) {
                    matA0(j, 0) = nearest_pts.at(j).x;
                    matA0(j, 1) = nearest_pts.at(j).y;
                    matA0(j, 2) = nearest_pts.at(j).z;
                    // printf(" pts %f %f %f ", matA0(j, 0), matA0(j, 1), matA0(j,
                    // 2));
                }
            }
        }

        Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
        double negative_OA_dot_norm = 1 / norm.norm();
        norm.normalize();

        bool planeValid = true;
        double point_to_plane_dis;
        double meanSquaredDist = 0.;
        for (int j = 0; j < 5; j++) {
            // if OX * n > 0.2, then plane is not fit well
            point_to_plane_dis=fabs(norm(0) * nearest_pts.at(j).x + norm(1) * nearest_pts.at(j).y +
                                  norm(2) * nearest_pts.at(j).z + negative_OA_dot_norm);
            if (point_to_plane_dis> localMap.planeRes_/2.0) {
                planeValid = false;
                break;
            }
           if (planeValid == false) {
            result.match_result = MatchingResult::MSE_TOO_LARGE;
            return result;
         }
         
        meanSquaredDist += point_to_plane_dis;
        
        }
       
        meanSquaredDist /= 5;

        double fitQualityCoeff = 1.0 - sqrt(meanSquaredDist / squredMaxDist);

        pcaFeature feature;

        // Normal direction criterion: make it to face towards the vehicle.
        // We can use the dot product to find it out, since pointclouds are
        // given in vehicle-frame coordinates.

        // It should be <0 if the normal is pointing to the vehicle.
        // Otherwise, reverse the normal.

        // TODO: need to double check if the normal is correct

        Eigen::Vector3d correct_normal;
        //Eigen::Vector3d curr_point(pInit.x(), pInit.y(), pInit.z());
        Eigen::Vector3d curr_point(pFinal.x(), pFinal.y(), pFinal.z());
        Eigen::Vector3d viewpoint_direction = curr_point;
        Eigen::Vector3d normal=eig.eigenvectors().col(0);
        double dot_product = viewpoint_direction.dot(normal);
        correct_normal=normal;
        if (dot_product < 0)
            correct_normal = -correct_normal;

#if 1
        //TODO: Move feature observability analysis to debug cpp files
        this->FeatureObservabilityAnalysis(
                feature, pFinal, D, correct_normal, eig.eigenvectors().col(2));
#endif



        result.feature_type = FeatureType::PlaneFeature;
        result.feature = feature;
        result.match_result = MatchingResult::SUCCESS;
      //  result.Avalue = A;
        result.Pvalue = mean;
        result.Xvalue = pInit;
        result.NormDir = norm;
        result.negative_OA_dot_norm = negative_OA_dot_norm;
        result.TimeValue =static_cast<double>(1.0);  // TODO:should be the point cloud time
        result.residualCoefficient = fitQualityCoeff;

        return result;
    }

    void LidarSLAM::FeatureObservabilityAnalysis(
            pcaFeature &feature, Eigen::Vector3d p_query, Eigen::Vector3d lamada,
            Eigen::Vector3d normal_direction, Eigen::Vector3d principal_direction) {
        feature.pt.x = p_query.x();
        feature.pt.y = p_query.y();
        feature.pt.z = p_query.z();

        normal_direction.normalized();
        principal_direction.normalized();

        feature.vectors.principalDirection = principal_direction.cast<float>();
        feature.vectors.normalDirection = normal_direction.cast<float>();
        feature.values.lamada1 = std::sqrt(lamada(2));
        feature.values.lamada2 = std::sqrt(lamada(1));
        feature.values.lamada3 = std::sqrt(lamada(0));
        if ((feature.values.lamada1 + feature.values.lamada2 +
             feature.values.lamada3) == 0) {
            feature.curvature = 0;
        } else {
            feature.curvature = feature.values.lamada3 /
                                (feature.values.lamada1 + feature.values.lamada2 +
                                 feature.values.lamada3);
        }
        feature.linear_2 = ((feature.values.lamada1) - (feature.values.lamada2)) /
                           (feature.values.lamada1);
        feature.planar_2 = ((feature.values.lamada2) - (feature.values.lamada3)) /
                           (feature.values.lamada1);
        feature.spherical_2 = (feature.values.lamada3) / (feature.values.lamada1);

        Eigen::Vector3f point(feature.pt.x, feature.pt.y, feature.pt.z);
       
        Eigen::Vector3f x_axis(1, 0, 0);
        Eigen::Vector3f y_axis(0, 1, 0);
        Eigen::Vector3f z_axis(0, 0, 1);
       
        Eigen::Quaternionf rot(T_w_lidar.rot.w(), T_w_lidar.rot.x(),
                               T_w_lidar.rot.y(), T_w_lidar.rot.z());

        rot.normalized();
        
        Eigen::Vector3f rot_x_axis,rot_y_axis,rot_z_axis;

        rot_x_axis = rot * x_axis;
        rot_y_axis = rot * y_axis;
        rot_z_axis = rot * z_axis;

        // TODO: should make sure the normal is correct, then calculate the cross.

        Eigen::Vector3f cross;
        cross = point.cross(feature.vectors.normalDirection);



        feature.rx_cross = cross.dot(rot_x_axis);
        feature.neg_rx_cross = -cross.dot(rot_x_axis);
        feature.ry_cross = cross.dot(rot_y_axis);
        feature.neg_ry_cross = -cross.dot(rot_y_axis);
        feature.rz_cross = cross.dot(rot_z_axis);
        feature.neg_rz_cross = -cross.dot(rot_z_axis);

        // IMLS for translation
        feature.tx_dot = feature.planar_2 * feature.planar_2 *
                         fabs(feature.vectors.normalDirection.dot(rot_x_axis));
        feature.ty_dot = feature.planar_2 * feature.planar_2 *
                         fabs(feature.vectors.normalDirection.dot(rot_y_axis));
        feature.tz_dot = feature.planar_2 * feature.planar_2 *
                         fabs(feature.vectors.normalDirection.dot(rot_z_axis));

        std::vector<std::pair<float, Feature_observability>> rotation_quality;
        std::vector<std::pair<float, Feature_observability>> trans_quality;
        // assign feature quality
        std::pair<float, Feature_observability> quality;
        {
            quality.first = feature.rx_cross;
            quality.second = Feature_observability::rx_cross;
            rotation_quality.push_back(quality);
           quality.first = feature.neg_rx_cross;
           quality.second = Feature_observability::neg_rx_cross;
           rotation_quality.push_back(quality);
            quality.first = feature.ry_cross;
            quality.second = Feature_observability::ry_cross;
            rotation_quality.push_back(quality);
           quality.first = feature.neg_ry_cross;
           quality.second = Feature_observability::neg_ry_cross;
           rotation_quality.push_back(quality);
            quality.first = feature.rz_cross;
            quality.second = Feature_observability::rz_cross;
            rotation_quality.push_back(quality);
           quality.first = feature.neg_rz_cross;
           quality.second = Feature_observability::neg_rz_cross;
           rotation_quality.push_back(quality);
            quality.first = feature.tx_dot;
            quality.second = Feature_observability::tx_dot;
            trans_quality.push_back(quality);
            quality.first = feature.ty_dot;
            quality.second = Feature_observability::ty_dot;
            trans_quality.push_back(quality);
            quality.first = feature.tz_dot;
            quality.second = Feature_observability::tz_dot;
            trans_quality.push_back(quality);
        }

        sort(rotation_quality.begin(), rotation_quality.end(), compare_pair_first);
        sort(trans_quality.begin(), trans_quality.end(), compare_pair_first);

        //    LOG(INFO)<<"rotation_quality 0: "<<rotation_quality.at(0).first;
        //    LOG(INFO)<<"rotation_quality 1: "<<rotation_quality.at(1).first;
        //    LOG(INFO)<<"rotation_quality 2: "<<rotation_quality.at(2).first;
        feature.observability.at(0) = rotation_quality.at(0).second;
        feature.observability.at(1) = rotation_quality.at(1).second;
        feature.observability.at(2) = trans_quality.at(0).second;
        feature.observability.at(3) = trans_quality.at(1).second;
        //    LOG(INFO)<<"trans_quality 0: "<<trans_quality.at(0).first;
        //    LOG(INFO)<<"trans_quality 1: "<<trans_quality.at(1).first;
        //    LOG(INFO)<<"trans_quality 2: "<<trans_quality.at(2).first;




    }

    void LidarSLAM::ResetDistanceParameters() {
        this->OptimizationData.clear();
        for (auto &ele : MatchRejectionHistogramLine) ele = 0;
        for (auto &ele : MatchRejectionHistogramPlane) ele = 0;
        for (auto &ele : MatchRejectionHistogramBlob) ele = 0;
        for (auto &ele : PlaneFeatureHistogramObs) ele = 0;
    }

    LidarSLAM::RegistrationError LidarSLAM::EstimateRegistrationError(
            ceres::Problem &problem, const double eigen_thresh) {
        RegistrationError err;

        // Covariance computation options
        ceres::Covariance::Options covOptions;
        covOptions.apply_loss_function = true;
        covOptions.algorithm_type = ceres::CovarianceAlgorithmType::DENSE_SVD;
        covOptions.null_space_rank = -1;
        covOptions.num_threads = 2;

        ceres::Covariance covarianceSolver(covOptions);
        std::vector<std::pair<const double *, const double *>> covarianceBlocks;
        const double *paramBlock = pose_parameters;
        covarianceBlocks.emplace_back(paramBlock, paramBlock);
        covarianceSolver.Compute(covarianceBlocks, &problem);
        covarianceSolver.GetCovarianceBlockInTangentSpace(paramBlock, paramBlock,
                                                          err.Covariance.data());
        // Degeneracy detection: check if any eigenvalue of the Hessian is too small
        // (indicating unconstrained directions, e.g. XY translation with plane-only features).
        // When degenerate + visual odom available, adds an absolute pose prior to ICP.
        Eigen::Matrix<double, 6, 1> matX;
        matX << 1, 1, 1, 1, 1, 1;
        Eigen::Matrix<double, 6, 6, Eigen::RowMajor> JtJ = err.Covariance.inverse();
        isDegenerate = DegeneracyDetection(Pos_degeneracy_threshold, JtJ, matX);




        // Estimate max position/orientation errors and directions from covariance
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigPosition(err.Covariance.topLeftCorner<3, 3>());

        err.PositionError = std::sqrt(eigPosition.eigenvalues()(2));
        err.PositionErrorDirection = eigPosition.eigenvectors().col(2);
        err.PosInverseConditionNum = std::sqrt(eigPosition.eigenvalues()(0)) / std::sqrt(eigPosition.eigenvalues()(2));

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigOrientation(err.Covariance.bottomRightCorner<3, 3>());
        err.OrientationError = Rad2Deg(std::sqrt(eigOrientation.eigenvalues()(2)));
        err.OrientationErrorDirection = eigOrientation.eigenvectors().col(2);
        err.OriInverseConditionNum =
                std::sqrt(eigOrientation.eigenvalues()(0)) / std::sqrt(eigOrientation.eigenvalues()(2));
        
      //  LOG(INFO) << "err.PositionError : " << err.PositionError;
      //  LOG(INFO) << "err.OrientationError: " <<  err.OrientationError;
        
      //  LOG(INFO) << "err.Covariance.data(): ";
      //  LOG(INFO) << err.Covariance.inverse();



        // LOG(INFO) << "err.PosInverseConditionNum: " << err.PosInverseConditionNum;
        // LOG(INFO) << "err.OriInverseConditionNum: " << err.OriInverseConditionNum;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigPosition2(err.Covariance.inverse());



        //LOG(INFO)<<"err.covariance eigen vector: "<<eigPosition2.eigenvectors();


        // float sum_position_lambda =std::sqrt(eigPosition.eigenvalues()(0))
        //                           +std::sqrt(eigPosition.eigenvalues()(1))
        //                           +std::sqrt(eigPosition.eigenvalues()(2));

        // float sum_orientation_lambda = std::sqrt(eigOrientation.eigenvalues()(0))
        //                                +std::sqrt(eigOrientation.eigenvalues()(1))
        //                                +std::sqrt(eigOrientation.eigenvalues()(2));


        // err.LidarUncertainty.resize(6);
        // err.LidarUncertainty.setZero();
        // err.LidarUncertainty<< (std::sqrt(eigPosition.eigenvalues()(0))/sum_position_lambda), //x
        //                   (std::sqrt(eigPosition.eigenvalues()(1))/sum_position_lambda), //y
        //                   (std::sqrt(eigPosition.eigenvalues()(2))/sum_position_lambda), //z
        //                   (std::sqrt(eigOrientation.eigenvalues()(0))/sum_orientation_lambda), //roll
        //                   (std::sqrt(eigOrientation.eigenvalues()(1))/sum_orientation_lambda), //pitch
        //                   (std::sqrt(eigOrientation.eigenvalues()(2))/sum_orientation_lambda); //yaw


        // cout << "err.LidarUncertainty:" << endl << err.LidarUncertainty;


        return err;
    }
    
   void LidarSLAM::MannualYawCorrection()
   {
    Transformd last_current_T = last_T_w_lidar.inverse() * T_w_lidar;
    float translation_norm = last_current_T.pos.norm();

    // Extract roll, pitch, yaw using Eigen
    Eigen::Vector3d euler = T_w_lidar.rot.toRotationMatrix().eulerAngles(0, 1, 2);
    double roll = euler[0], pitch = euler[1], yaw = euler[2];

    double correct_yaw = yaw + translation_norm * OptSet.yaw_ratio * M_PI / 180;

    // Reconstruct quaternion from corrected RPY
    Eigen::Quaterniond correct_rot =
        Eigen::AngleAxisd(correct_yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

    T_w_lidar.rot = correct_rot.normalized();
   }



    bool LidarSLAM::DegeneracyDetection(
            double eigThreshlod, Eigen::Matrix<double, 6, 6, Eigen::RowMajor> &matAtA,
            Eigen::Matrix<double, 6, 1> &matX) {
        bool isDegenerate;
        Eigen::Matrix<double, 1, 6> matE;
        Eigen::Matrix<double, 6, 6> matV;
        Eigen::Matrix<double, 6, 6> matV2;
        Eigen::Matrix<double, 6, 6> matP;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> esolver(matAtA);
        matE = esolver.eigenvalues().real();
        matV = esolver.eigenvectors().real();

        matV2 = matV;
        isDegenerate = false;
        //  float eignThre[6] = { 100, 100, 100, 100, 100, 100 };

        for (int i = 0; i < 6; i++) {
            if (matE(0, i) < eigThreshlod) {
                for (int j = 0; j < 6; j++) {
                    matV2(i, j) = 0;
                }
                isDegenerate = true;
            } else {
                break;
            }
        }

        matP = matV.inverse() * matV2;

        if (isDegenerate) {
            Eigen::Matrix<double, 6, 1> matX2(matX);
            matX = matP * matX2;
        }

        return isDegenerate;
    }

  

} /* arise_slam */
