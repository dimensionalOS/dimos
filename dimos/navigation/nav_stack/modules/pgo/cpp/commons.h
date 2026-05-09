#pragma once
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using PointType = pcl::PointXYZI;
using CloudType = pcl::PointCloud<PointType>;
using PointVec = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

using M3D = Eigen::Matrix3d;
using V3D = Eigen::Vector3d;
using M3F = Eigen::Matrix3f;
using V3F = Eigen::Vector3f;
using M4F = Eigen::Matrix4f;
using V4F = Eigen::Vector4f;

struct PoseWithTime {
    V3D t;
    M3D r;
    int32_t sec;
    uint32_t nsec;
    double second;
    void setTime(int32_t sec, uint32_t nsec);
};

struct CloudWithPose {
    CloudType::Ptr cloud;
    PoseWithTime pose;
};
