// common_lib.h - ROS-free common definitions for better_fastlio2 port to DimOS
// Ported from better_fastlio2/include/common_lib.h
// Removed: all ROS, gtsam, tf, cv_bridge, image_transport, fast_lio_sam message dependencies
// Kept: Eigen, PCL, math helpers, core types needed by IMU_Processing.hpp

#pragma once

// eigen
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>

// std
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <string>
#include <limits>
#include <array>
#include <omp.h>
#include <mutex>
#include <math.h>

// namesapce
using namespace std;
using namespace Eigen;

// define
#define USE_IKFOM

#define PI_M (3.14159265358)
#define G_m_s2 (9.81)         // Gravity const in GuangDong/China
#define DIM_STATE (18)        // Dimension of states (Let Dim(SO(3)) = 3)
#define DIM_PROC_N (12)       // Dimension of process noise (Let Dim(SO(3)) = 3)
#define CUBE_LEN  (6.0)
#define LIDAR_SP_LEN    (2)
#define INIT_COV   (1)
#define NUM_MATCH_POINTS    (5)
#define MAX_MEAS_DIM        (10000)

#define SKEW_SYM_MATRX(v) 0.0,-v[2],v[1],v[2],0.0,-v[0],-v[1],v[0],0.0
#define VEC_FROM_ARRAY(v)        v[0],v[1],v[2]
#define MAT_FROM_ARRAY(v)        v[0],v[1],v[2],v[3],v[4],v[5],v[6],v[7],v[8]
#define CONSTRAIN(v,min,max)     ((v>min)?((v<max)?v:max):min)
#define ARRAY_FROM_EIGEN(mat)    mat.data(), mat.data() + mat.rows() * mat.cols()
#define STD_VEC_FROM_EIGEN(mat)  vector<decltype(mat)::Scalar> (mat.data(), mat.data() + mat.rows() * mat.cols())
#define DEBUG_FILE_DIR(name)     (string("Log/") + string(name))

typedef pcl::PointXYZINormal PointType;
typedef vector<PointType, Eigen::aligned_allocator<PointType>>  PointVector;
typedef Vector3d V3D;
typedef Matrix3d M3D;
typedef Vector3f V3F;
typedef Matrix3f M3F;

#define MD(a,b)  Matrix<double, (a), (b)>
#define VD(a)    Matrix<double, (a), 1>
#define MF(a,b)  Matrix<float, (a), (b)>
#define VF(a)    Matrix<float, (a), 1>

// These must be inline to avoid multiple-definition errors in header-only usage
inline M3D Eye3d(M3D::Identity());
inline M3F Eye3f(M3F::Identity());
inline V3D Zero3d(0, 0, 0);
inline V3F Zero3f(0, 0, 0);

// Pose6D struct (replaces fast_lio_sam::Pose6D ROS message)
struct Pose6D {
    double offset_time;
    double acc[3];
    double gyr[3];
    double vel[3];
    double pos[3];
    double rot[9];
};

// PointTypePose for trajectory
struct PointXYZIRPYT {
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
    (float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)(double, time, time))

typedef PointXYZIRPYT PointTypePose;

// ImuData struct (replaces sensor_msgs::Imu::ConstPtr)
struct ImuData {
    double timestamp;
    Eigen::Vector3d acc;   // linear acceleration (m/s^2)
    Eigen::Vector3d gyro;  // angular velocity (rad/s)
};

// MeasureGroup: lidar and imu data for a single processing frame
struct MeasureGroup
{
    MeasureGroup()
    {
        lidar_beg_time = 0.0;
        this->lidar.reset(new pcl::PointCloud<PointType>());
    };
    double lidar_beg_time;
    double lidar_end_time;
    pcl::PointCloud<PointType>::Ptr lidar;
    deque<std::shared_ptr<ImuData>> imu;
};

// functions
template<typename T>
auto set_pose6d(const double t, const Matrix<T, 3, 1> &a, const Matrix<T, 3, 1> &g, \
                const Matrix<T, 3, 1> &v, const Matrix<T, 3, 1> &p, const Matrix<T, 3, 3> &R)
{
    Pose6D rot_kp;
    rot_kp.offset_time = t;
    for (int i = 0; i < 3; i++)
    {
        rot_kp.acc[i] = a(i);
        rot_kp.gyr[i] = g(i);
        rot_kp.vel[i] = v(i);
        rot_kp.pos[i] = p(i);
        for (int j = 0; j < 3; j++)  rot_kp.rot[i*3+j] = R(i,j);
    }
    return move(rot_kp);
}

template<typename T>
inline T rad2deg(T radians)
{
  return radians * 180.0 / PI_M;
}

template<typename T>
inline T deg2rad(T degrees)
{
  return degrees * PI_M / 180.0;
}

inline float calc_dist(PointType p1, PointType p2){
    float d = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
    return d;
}

template<typename T>
bool esti_normvector(Matrix<T, 3, 1> &normvec, const PointVector &point, const T &threshold, const int &point_num){
    MatrixXf A(point_num, 3);
    MatrixXf b(point_num, 1);
    b.setOnes();
    b *= -1.0f;

    for (int j = 0; j < point_num; j++)
    {
        A(j,0) = point[j].x;
        A(j,1) = point[j].y;
        A(j,2) = point[j].z;
    }
    normvec = A.colPivHouseholderQr().solve(b);

    for (int j = 0; j < point_num; j++)
    {
        if (fabs(normvec(0) * point[j].x + normvec(1) * point[j].y + normvec(2) * point[j].z + 1.0f) > threshold)
        {
            return false;
        }
    }

    normvec.normalize();
    return true;
}

// FOV segment constant (from upstream laserMapping.cpp)
const float MOV_THRESHOLD = 1.5f;

template<typename T>
Eigen::Matrix<T, 3, 1> RotMtoEuler(const Eigen::Matrix<T, 3, 3> &rot)
{
    T sy = sqrt(rot(0,0)*rot(0,0) + rot(1,0)*rot(1,0));
    bool singular = sy < 1e-6;
    T x, y, z;
    if(!singular)
    {
        x = atan2(rot(2, 1), rot(2, 2));
        y = atan2(-rot(2, 0), sy);
        z = atan2(rot(1, 0), rot(0, 0));
    }
    else
    {
        x = atan2(-rot(1, 2), rot(1, 1));
        y = atan2(-rot(2, 0), sy);
        z = 0;
    }
    Eigen::Matrix<T, 3, 1> ang(x, y, z);
    return ang;
}

template<typename T>
bool esti_plane(Matrix<T, 4, 1> &pca_result, const PointVector &point, const T &threshold){
    Matrix<T, NUM_MATCH_POINTS, 3> A;
    Matrix<T, NUM_MATCH_POINTS, 1> b;
    A.setZero();
    b.setOnes();
    b *= -1.0f;

    for (int j = 0; j < NUM_MATCH_POINTS; j++){
        A(j,0) = point[j].x;
        A(j,1) = point[j].y;
        A(j,2) = point[j].z;
    }

    Matrix<T, 3, 1> normvec = A.colPivHouseholderQr().solve(b);

    T n = normvec.norm();
    pca_result(0) = normvec(0) / n;
    pca_result(1) = normvec(1) / n;
    pca_result(2) = normvec(2) / n;
    pca_result(3) = 1.0 / n;

    for (int j = 0; j < NUM_MATCH_POINTS; j++)
    {
        if (fabs(pca_result(0) * point[j].x + pca_result(1) * point[j].y + pca_result(2) * point[j].z + pca_result(3)) > threshold)
        {
            return false;
        }
    }
    return true;
}

inline float pointDistance(PointType p)
{
    return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

inline float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}

template<typename PointT>
float pointDistance3d(const PointT& p1, const PointT& p2){
    return (float)sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

template<typename PointT>
float pointDistance2d(const PointT& p1, const PointT& p2){
    return (float)sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y));
}

template<typename PointT>
float pointDistance3d(const PointT& p1){
    return (float)sqrt((p1.x)*(p1.x) + (p1.y)*(p1.y) + (p1.z)*(p1.z));
}

template<typename PointT>
float pointDistance2d(const PointT& p1){
    return (float)sqrt((p1.x)*(p1.x) + (p1.y)*(p1.y));
}

template<typename PointT>
float getPolarAngle(const PointT& p){
    if(p.x == 0 && p.y == 0){
        return 0.f;
    }
    else if(p.y >= 0){
        return (float)rad2deg((float)atan2(p.y, p.x));
    }
    else if(p.y < 0){
        return (float)rad2deg((float)atan2(p.y, p.x) + 2*M_PI);
    }
    return 0.f;
}

template<typename PointT>
float getAzimuth(const PointT& p){
    return (float)rad2deg((float)atan2(p.z, (float)pointDistance2d(p)));
}

// SO3 Exp map (Rodrigues formula) - used by IMU_Processing
inline M3D Exp(const V3D &ang)
{
    double ang_norm = ang.norm();
    if (ang_norm < 1e-10)
    {
        return M3D::Identity();
    }
    V3D ax = ang / ang_norm;
    M3D K;
    K << 0, -ax(2), ax(1),
         ax(2), 0, -ax(0),
         -ax(1), ax(0), 0;
    return M3D::Identity() + sin(ang_norm) * K + (1 - cos(ang_norm)) * K * K;
}

inline M3D Exp(const V3D &ang_vel, const double &dt)
{
    return Exp(ang_vel * dt);
}

// Exp from roll/pitch/yaw (used in upstream for R from euler angles)
inline M3D Exp(double roll, double pitch, double yaw)
{
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    return q.toRotationMatrix();
}

// Affine3f helpers (from upstream common_lib.h)
inline Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint)
{
    return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z,
                                  thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
}

inline Eigen::Affine3f trans2Affine3f(float transformIn[])
{
    return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5],
                                  transformIn[0], transformIn[1], transformIn[2]);
}

// constraintTransformation (from upstream laserMapping.cpp)
inline float constraintTransformation(float value, float limit)
{
    if (value < -limit) value = -limit;
    if (value > limit) value = limit;
    return value;
}

// EulerToQuat (from upstream common_lib.h)
inline Eigen::Quaterniond EulerToQuat(float roll_, float pitch_, float yaw_)
{
    Eigen::AngleAxisd roll(double(roll_), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch(double(pitch_), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw(double(yaw_), Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = yaw * pitch * roll;
    q.normalize();
    return q;
}
