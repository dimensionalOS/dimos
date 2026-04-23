#include "tgrs.h"
#include "tictoc.hpp"

void TGRS::mergeClusters(std::vector<int>& clusterIdxs_, const int& idx1_, const int& idx2_){
    for(int i = 0; i < clusterIdxs_.size(); i++){
        if(clusterIdxs_[i] == idx1_){
            clusterIdxs_[i] = idx2_;
        }
    }
}

std::vector<int> TGRS::findVoxelNeighbors(const int& range_idx_, const int& sector_idx_, const int& azimuth_idx_, int size_){
    std::vector<int> neighborIdxs;
    if(range_idx_ > RANGE_NUM * 0.6){
        size_ = 1;
    }
    for(int x = range_idx_ - size_; x <= range_idx_ + size_; x++){
        if(x > RANGE_NUM -1 || x < 0) {continue;}
        for(int y = sector_idx_ - size_; y <= sector_idx_ + size_; y++){
            if(y > SECTOR_NUM -1 || y < 0) {continue;}
            for(int z = azimuth_idx_ - size_; z <= azimuth_idx_ + size_; z++){
                if(z > AZIMUTH_NUM - 1 || z < 0) {continue;}
                neighborIdxs.emplace_back(x * SECTOR_NUM + y + z * RANGE_NUM * SECTOR_NUM);
            }
        }
    }
    return neighborIdxs;
}

void TGRS::cluster(const std::vector<PointAPRI>& apri_vec_,
                   std::unordered_map<int, Voxel>& hash_cloud_,
                   std::unordered_map<int, std::vector<int>>& cluster_vox)
{
    int cluster_name = 4;
    std::vector<int> clusterIdxs = std::vector<int>(apri_vec_.size(), -1);

    TicToc cluster_t;

    // vec cluster
    for(int i = 0; i < apri_vec_.size(); i++){
        PointAPRI apri = apri_vec_[i];
        std::unordered_map<int, Voxel>::iterator it_find1;
        std::unordered_map<int, Voxel>::iterator it_find2;
        std::vector<int> neighbors;  // restore a lot of apri-neighbors idxs

        it_find1 = hash_cloud_.find(apri.voxel_idx);
        if(it_find1 != hash_cloud_.end()){
            std::vector<int> neighbor = findVoxelNeighbors(apri.range_idx, apri.sector_idx, apri.azimuth_idx, 1);
            for(int k = 0; k < neighbor.size(); k++){
                it_find2 =  hash_cloud_.find(neighbor[k]);
                if(it_find2 != hash_cloud_.end()){
                    addVec(neighbors, it_find2->second.ptIdx);
                }
            }
        }

        // neighbors dedup removed (was a no-op self-swap)
        if(neighbors.size() > 0){
            for(int n = 0; n < neighbors.size(); n++){
                int oc = clusterIdxs[i];
                int nc = clusterIdxs[neighbors[n]];

                if(oc != -1 && nc != -1){
                    if(oc != nc){
                        mergeClusters(clusterIdxs, oc, nc);  // merge
                    }
                }
                else{
                    if(nc != -1){
                        clusterIdxs[i] = nc;
                    }
                    else{
                        if(oc != -1){
                            clusterIdxs[neighbors[n]] = oc;
                        }
                    }
                }
            }
        }

        if(clusterIdxs[i] == -1){
            cluster_name ++;   //   a new class
            clusterIdxs[i] = cluster_name;  // just encode the cluster name
            for(int m = 0; m < neighbors.size(); m++){
                clusterIdxs[neighbors[m]] = cluster_name;
            }
        }
    }

    // voxels cluster
    std::unordered_map<int, std::vector<int>>::iterator it_v;
    for(size_t i = 0; i < clusterIdxs.size(); i++){
        it_v = cluster_vox.find(clusterIdxs[i]);
        if(it_v != cluster_vox.end()){
            it_v->second.emplace_back(apri_vec_[i].voxel_idx);
            hash_cloud_[apri_vec_[i].voxel_idx].label = it_v->first;
        }
        else{
            std::vector<int> vox_vec;
            vox_vec.emplace_back(apri_vec_[i].voxel_idx);
            cluster_vox.insert(std::make_pair(clusterIdxs[i], vox_vec));
            hash_cloud_[apri_vec_[i].voxel_idx].label = clusterIdxs[i];
        }
    }

    for(auto& it : cluster_vox){
        sampleVec(it.second);
    }
}

std::pair<PointType, PointType> TGRS::getBoundingBoxOfCloud(const pcl::PointCloud<PointType>::Ptr& cloud_){
    PointType point_min, point_max;
    pcl::getMinMax3D(*cloud_, point_min, point_max);
    return std::make_pair(point_min, point_max);
}

pcl::PointCloud<PointType>::Ptr TGRS::getCloudByVec(const std::vector<int>& vec_, const pcl::PointCloud<PointType>::Ptr& cloud_){
    pcl::PointCloud<PointType>::Ptr cloudReturn(new pcl::PointCloud<PointType>());
    for(auto &it : vec_){
        cloudReturn->points.emplace_back(cloud_->points[it]);
    }
    return cloudReturn;
}

void TGRS::recognizePD(SSC& ssc){
    for(auto& it : ssc.cluster_vox){
        std::vector<int> voxIdx;
        for(auto& id : it.second){
            voxIdx.emplace_back(ssc.hash_cloud[id].ptVoxIdx);
        }
        pcl::PointCloud<PointType>::Ptr voxels(new pcl::PointCloud<PointType>());
        *voxels += *getCloudByVec(voxIdx, ssc.cloud_vox);
        std::pair<PointType, PointType> heightPair = getBoundingBoxOfCloud(voxels);
        if((heightPair.first.z <= -(SENSOR_HEIGHT - 0.2)) && ((heightPair.second.z + SENSOR_HEIGHT) <= PD_HEIGHT)){
            ssc.PD_cluster.emplace_back(it.first);
        }
    }
    std::cout << "There are " << ssc.PD_cluster.size() << " PD objects." << std::endl;
}

void TGRS::trackPD(SSC& ssc_pre, PointTypePose* pose_pre, SSC& ssc_next, PointTypePose* pose_next){
    // Step 1: get voxel cloud
    pcl::PointCloud<PointType>::Ptr voxCloud_pre(new pcl::PointCloud<PointType>());
    *voxCloud_pre += *ssc_pre.cloud_vox;
    pcl::PointCloud<PointType>::Ptr voxCloud_next(new pcl::PointCloud<PointType>());
    *voxCloud_next += *ssc_next.cloud_vox;

    // Step 2: transform voxel cloud
    Eigen::Affine3f trans_pre = pcl::getTransformation(pose_pre->x, pose_pre->y, pose_pre->z, pose_pre->roll, pose_pre->pitch, pose_pre->yaw);
    Eigen::Affine3f trans_next = pcl::getTransformation(pose_next->x, pose_next->y, pose_next->z, pose_next->roll, pose_next->pitch, pose_next->yaw);
    Eigen::Affine3f trans_n2p = trans_pre.inverse() * trans_next;
    pcl::PointCloud<PointType>::Ptr voxCloud_nextTrans(new pcl::PointCloud<PointType>());
    transformPointCloud(voxCloud_next, trans_n2p, voxCloud_nextTrans);

    // Step 3: PD projection (tracking)
    for(auto& pd : ssc_next.PD_cluster){
        std::vector<int> projIdx;
        for(auto& voxIdx : ssc_next.cluster_vox[pd]){
            PointType voxPt = voxCloud_nextTrans->points[ssc_next.hash_cloud[voxIdx].ptVoxIdx];
            float dis = pointDistance2d(voxPt);
            float angle = getPolarAngle(voxPt);
            float azimuth = getAzimuth(voxPt);
            int range_idx = std::ceil((dis - MIN_DIS) / RANGE_RES) - 1;
            int sector_idx = std::ceil((angle - MIN_ANGLE) / SECTOR_RES) - 1;
            int azimuth_idx = std::ceil((azimuth - MIN_AZIMUTH) / AZIMUTH_RES) -1;
            int voxel_idx = azimuth_idx * RANGE_NUM * SECTOR_NUM + range_idx * SECTOR_NUM + sector_idx;

            std::vector<int> neighbor = findVoxelNeighbors(range_idx, sector_idx, azimuth_idx, 1);

            // choice one: find neighbors
            addVec(projIdx, neighbor);
        }
        sampleVec(projIdx);
        std::cout << "cur pd Idx: " << ssc_next.cluster_vox[pd].size() << std::endl;

        // Step 4: HD detection
        int all = projIdx.size();
        int success = 0;
        std::unordered_map<int, Voxel>::iterator it_find;
        for(auto& proj : projIdx){
            it_find = ssc_pre.hash_cloud.find(proj);
            if(it_find != ssc_pre.hash_cloud.end()){
                success ++;
            }
        }
        float overlapRatio = (float)success / (float)all;
        std::cout << "name: " << pd << " success: " << success << " all: " << all << " overlap ratio: " << overlapRatio << std::endl;
        if(overlapRatio <= HD_RATIO){
            ssc_next.HD_cluster.emplace_back(pd);  // PD to HD
        }
        else{
            ssc_next.AS_cluster.emplace_back(pd);  // PD to AS
        }
    }

    std::vector<int> AS_ptIdx;
    for(auto& as : ssc_next.AS_cluster){
        addVec(AS_ptIdx, ssc_next.hash_cloud[as].ptIdx);
    }
    *ssc_next.cloud_nd += *getCloudByVec(AS_ptIdx, ssc_next.cloud_use);
    *ssc_next.cloud_nd += *ssc_next.cloud_g;

    std::cout << ANSI_COLOR_GREEN << " PD num: " << ssc_next.PD_cluster.size() << "\n"
              << ANSI_COLOR_RED << " HD num: " << ssc_next.HD_cluster.size() << ANSI_COLOR_RESET << std::endl;
}

void TGRS::saveColorCloud(SSC& ssc, const std::string& path){
    // NOTE: Original used cv::RNG for random colors. Using simple deterministic coloring instead
    // to avoid OpenCV dependency. Replace with your preferred RNG if needed.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    int color_seed = 12345;
    for(auto& it : ssc.cluster_vox){
        // Simple deterministic pseudo-random colors based on cluster id
        int r = ((it.first * 1103515245 + color_seed) >> 16) % 180 + 20;
        int g = ((it.first * 1103515245 + color_seed + 1) >> 16) % 180 + 20;
        int b = ((it.first * 1103515245 + color_seed + 2) >> 16) % 180 + 20;
        std::vector<int> ptIdx;
        for(auto& vox : it.second){
            addVec(ptIdx, ssc.hash_cloud[vox].ptIdx);
        }
        pcl::PointCloud<PointType>::Ptr cloudGrab(new pcl::PointCloud<PointType>());
        *cloudGrab += *getCloudByVec(ptIdx, ssc.cloud_use);
        for(size_t i = 0; i < cloudGrab->points.size(); i++){
            pcl::PointXYZRGB rgb;
            rgb.x = cloudGrab->points[i].x;
            rgb.y = cloudGrab->points[i].y;
            rgb.z = cloudGrab->points[i].z;
            rgb.r = r;
            rgb.g = g;
            rgb.b = b;
            colorCloud->points.emplace_back(rgb);
        }
    }
    colorCloud->height = 1;
    colorCloud->width = colorCloud->points.size();
    std::cout << "segmented cloud size: " << colorCloud->points.size() << std::endl;
    pcl::io::savePCDFile(path, *colorCloud);
}
