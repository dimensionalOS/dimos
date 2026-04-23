// paper accepted by TGRS @Yixin Fang, but it's just simple version ...
// Ported to DimOS: ROS dependencies removed.

#pragma once

#include "common_lib.h"

#include <unordered_map>
#include <iostream>
#include <cstdio>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <boost/shared_ptr.hpp>

#include "patchwork.h"
#include "tool_color_printf.h"

// PointType, PointTypePose, rad2deg, deg2rad, pointDistance2d/3d,
// getPolarAngle, getAzimuth are all defined in common_lib.h

template<typename T>
void addVec(std::vector<T>& vec_central_, const std::vector<T>& vec_add_){
    vec_central_.insert(vec_central_.end(), vec_add_.begin(), vec_add_.end());
}

template<typename T>
void sampleVec(std::vector<T>& vec_central_){
    std::sort(vec_central_.begin(), vec_central_.end());
    vec_central_.erase(std::unique(vec_central_.begin(), vec_central_.end()), vec_central_.end());
}

template<typename CloudT>
void transformPointCloud(const CloudT& cloudIn_, const Eigen::Affine3f& transCur_, CloudT& cloudOut_){
    int cloudSize = cloudIn_->points.size();
    cloudOut_->points.resize(cloudSize);

    for(int i = 0; i < cloudSize; i++){
        cloudOut_->points[i].x = transCur_(0,0) * cloudIn_->points[i].x + transCur_(0,1) * cloudIn_->points[i].y + transCur_(0,2) * cloudIn_->points[i].z + transCur_(0,3);
        cloudOut_->points[i].y = transCur_(1,0) * cloudIn_->points[i].x + transCur_(1,1) * cloudIn_->points[i].y + transCur_(1,2) * cloudIn_->points[i].z + transCur_(1,3);
        cloudOut_->points[i].z = transCur_(2,0) * cloudIn_->points[i].x + transCur_(2,1) * cloudIn_->points[i].y + transCur_(2,2) * cloudIn_->points[i].z + transCur_(2,3);
        cloudOut_->points[i].intensity = cloudIn_->points[i].intensity;
    }
}

// ---------- End standalone definitions ----------

// Configurable via CLI --sensor_height (default 1.5)
inline double SENSOR_HEIGHT = 1.5;

#define MIN_DIS 1.0
#define MAX_DIS 50.0
#define MIN_ANGLE 0.0
#define MAX_ANGLE 360.0
#define MIN_AZIMUTH -30.0
#define MAX_AZIMUTH 60.0

#define RANGE_RES 0.25
#define SECTOR_RES 2.0
#define AZIMUTH_RES 3.0

#define RANGE_NUM (int)std::ceil((MAX_DIS - MIN_DIS) / RANGE_RES)
#define SECTOR_NUM (int)std::ceil((MAX_ANGLE - MIN_ANGLE) / SECTOR_RES)
#define AZIMUTH_NUM (int)std::ceil((MAX_AZIMUTH - MIN_AZIMUTH) / AZIMUTH_RES)
#define BIN_NUM RANGE_NUM * SECTOR_NUM * AZIMUTH_NUM

#define PD_HEIGHT (SENSOR_HEIGHT + 0.5)

#define HD_RATIO (float)0.7  // FIXME: check

#define VALID_NUM 5

// apiric-format of point
struct PointAPRI{
    float x, y, z;
    float range;
    float angle;
    float azimuth;
    int range_idx = -1;
    int sector_idx = -1 ;
    int azimuth_idx = -1;
    int voxel_idx = -1;  // id in voxel cloud
};

// voxel-type in hash cloud
struct Voxel{
    int range_idx;
    int sector_idx;
    int azimuth_idx;
    int label = -1;
    PointType center;   // the point center's intensity is its id in voxel cloud
    std::vector<int> ptIdx;  // the vector of id cloud_use
    int ptVoxIdx;  // id in voxel's center cloud
};

// frame_SSC
class SSC{
public:
    int frame_id;

    int range_num;
    int sector_num;
    int azimuth_num;
    int bin_num;

    std::vector<PointAPRI> apri_vec;
    std::unordered_map<int, Voxel> hash_cloud;
    std::unordered_map<int, std::vector<int>> cluster_vox;  // cluster name + voxel id
    std::vector<int> PD_cluster;   // PD cluster name
    std::vector<int> HD_cluster;   // HD cluster name
    std::vector<int> AS_cluster;   // HD cluster name

    boost::shared_ptr<PatchWork<PointType>> PatchworkGroundSeg;   // patchwork
    pcl::PointCloud<PointType>::Ptr cloud_g; // ground
    pcl::PointCloud<PointType>::Ptr cloud_ng;

    pcl::PointCloud<PointType>::Ptr cloud_use;

    pcl::PointCloud<PointType>::Ptr cloud_d;  // dynamic
    pcl::PointCloud<PointType>::Ptr cloud_nd;

    pcl::PointCloud<PointType>::Ptr cloud_vox; // voxel's center cloud

    void allocateMemory(){
        PatchworkGroundSeg.reset(new PatchWork<PointType>());
        cloud_g.reset(new pcl::PointCloud<PointType>());
        cloud_ng.reset(new pcl::PointCloud<PointType>());
        cloud_use.reset(new pcl::PointCloud<PointType>());
        cloud_d.reset(new pcl::PointCloud<PointType>());
        cloud_nd.reset(new pcl::PointCloud<PointType>());
        cloud_vox.reset(new pcl::PointCloud<PointType>());
    }

    void extractGroudByPatchWork(const pcl::PointCloud<PointType>::Ptr& cloud_in){
        double time_pw;
        PatchworkGroundSeg->set_sensor(SENSOR_HEIGHT);
        PatchworkGroundSeg->estimate_ground(*cloud_in, *cloud_g, *cloud_ng, time_pw);
        std::cout << "Ground Extract: " << " all pt num: " << cloud_in->points.size()
                                        << " sensor height: " << SENSOR_HEIGHT
                                        << " ground pt num: " << cloud_g->points.size()
                                        << " non-ground pt num: " << cloud_ng->points.size()
                                        << " time cost(ms): " << time_pw << std::endl;
    }

    // make apri vector
    void makeApriVec(const pcl::PointCloud<PointType>::Ptr& cloud_){
        for(size_t i = 0; i < cloud_->points.size(); i++){
            PointType pt = cloud_->points[i];
            float dis = pointDistance2d(pt);
            float angle = getPolarAngle(pt);
            float azimuth = getAzimuth(pt);
            if(dis < MIN_DIS || dis > MAX_DIS){
                continue;
            }
            if(angle < MIN_ANGLE || angle > MAX_ANGLE){
                continue;
            }
            if(azimuth < MIN_AZIMUTH || azimuth > MAX_AZIMUTH){
                continue;
            }

            cloud_use->points.push_back(pt);

            PointAPRI apri;
            apri.x = pt.x;
            apri.y = pt.y;
            apri.z = pt.z;
            apri.range = dis;
            apri.angle = angle;
            apri.azimuth = azimuth;
            apri.range_idx = std::ceil((dis - MIN_DIS) / RANGE_RES) - 1;
            apri.sector_idx = std::ceil((angle - MIN_ANGLE) / SECTOR_RES) - 1;
            apri.azimuth_idx = std::ceil((azimuth - MIN_AZIMUTH) / AZIMUTH_RES) -1;
            apri.voxel_idx = apri.azimuth_idx * RANGE_NUM * SECTOR_NUM + apri.range_idx * SECTOR_NUM + apri.sector_idx;
            if(apri.voxel_idx > BIN_NUM){
                fprintf(stderr, "[WARN] pt %d can't find its bin\n", (int)i);
                continue;
            }
            apri_vec.emplace_back(apri);
        }
        std::cout << "apri vec size: " << apri_vec.size() << " py use num: " << cloud_use->points.size() << std::endl;
    }

    // make hash table for voxels
    void makeHashCloud(const std::vector<PointAPRI>& apriIn_){
        std::unordered_map<int, Voxel>::iterator it_find;
        int count = 0;
        for(size_t i = 0; i < apriIn_.size(); i++){
            PointAPRI apri = apriIn_[i];
            it_find = hash_cloud.find(apri.voxel_idx);
            if(it_find != hash_cloud.end()){
                it_find->second.ptIdx.emplace_back(i);
            }
            else{
                Voxel voxel;
                voxel.ptIdx.emplace_back(i);
                voxel.range_idx = apri.range_idx;
                voxel.sector_idx = apri.sector_idx;
                voxel.azimuth_idx = apri.azimuth_idx;
                float range_center = (apri.range_idx * 2 + 1) / 2 * RANGE_RES + MIN_DIS;
                float sector_center = deg2rad((apri.sector_idx * 2 + 1) / 2 * SECTOR_RES) + MIN_ANGLE;
                float azimuth_center = deg2rad((apri.azimuth_idx * 2 + 1) / 2 * AZIMUTH_RES) + deg2rad(MIN_AZIMUTH);
                voxel.center.x = range_center * std::cos(sector_center);
                voxel.center.y = range_center * std::sin(sector_center);
                voxel.center.z = range_center * std::tan(azimuth_center);
                voxel.center.intensity = apri.voxel_idx;
                cloud_vox->points.emplace_back(voxel.center);
                voxel.ptVoxIdx = count;
                count ++;
                hash_cloud.insert(std::make_pair(apri.voxel_idx, voxel));
            }
        }
        std::cout << "hash cloud size: " << hash_cloud.size() << std::endl;
    }

    ~SSC(){}
    SSC(const pcl::PointCloud<PointType>::Ptr& cloud_in, const int& id){
        frame_id = id;
        std::cout << ANSI_COLOR_GREEN << "current frame id: " << frame_id << ANSI_COLOR_RESET << std::endl;
        allocateMemory();
        extractGroudByPatchWork(cloud_in);
        makeApriVec(cloud_ng);
        makeHashCloud(apri_vec);
    }

};

class TGRS{
public:
    TGRS(){}
    ~TGRS(){}

    // cluster
    std::vector<int> findVoxelNeighbors(const int& range_idx_, const int& sector_idx_, const int& azimuth_idx_, int size_);
    void mergeClusters(std::vector<int>& clusterIdxs_, const int& idx1_, const int& idx2_);
    void cluster(const std::vector<PointAPRI>& apri_vec_,
                 std::unordered_map<int, Voxel>& hash_cloud_,
                 std::unordered_map<int, std::vector<int>>& cluster_vox);
    std::pair<PointType, PointType> getBoundingBoxOfCloud(const pcl::PointCloud<PointType>::Ptr& cloud_);

    // classification
    pcl::PointCloud<PointType>::Ptr getCloudByVec(const std::vector<int>& vec_, const pcl::PointCloud<PointType>::Ptr& cloud_);
    void recognizePD(SSC& ssc);

    // tracking
    void trackPD(SSC& ssc_pre, PointTypePose* pose_pre, SSC& ssc_next, PointTypePose* pose_next);

    // save
    void saveColorCloud(SSC& ssc, const std::string& path);
};
