#pragma once

#include "common_lib.h"

using namespace std;

#define IS_VALID(a) ((abs(a) > 1e8) ? true : false)

enum LID_TYPE
{
  LIVOX = 1,
  VELO16,
  OUST64,
  RS
}; //{1, 2, 3, 4}

enum LIVOX_TYPE
{
  LIVOX_CUS = 1,
  LIVOX_ROS
};

enum TIME_UNIT
{
  SEC = 0,
  MS = 1,
  US = 2,
  NS = 3
};

enum Feature
{
  Nor,
  Poss_Plane,
  Real_Plane,
  Edge_Jump,
  Edge_Plane,
  Wire,
  ZeroPoint
};

enum Surround
{
  Prev,
  Next
};

enum E_jump
{
  Nr_nor,
  Nr_zero,
  Nr_180,
  Nr_inf,
  Nr_blind
};

struct orgtype
{
  double range;
  double dista;
  double angle[2];
  double intersect;
  E_jump edj[2];
  Feature ftype;
  orgtype()
  {
    range = 0;
    edj[Prev] = Nr_nor;
    edj[Next] = Nr_nor;
    ftype = Nor;
    intersect = 2;
  }
};

namespace livox_ros
{
  struct EIGEN_ALIGN16 Point
  {
    PCL_ADD_POINT4D;
    float intensity;
    uint8_t tag;
    uint8_t line;
    uint8_t reflectivity;
    uint32_t offset_time;
    PCL_ADD_RGB;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}
POINT_CLOUD_REGISTER_POINT_STRUCT(livox_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)
                                  (float, intensity, intensity)(std::uint8_t, tag, tag)(std::uint8_t, line, line)(std::uint8_t, reflectivity, reflectivity)(std::uint32_t, offset_time, offset_time)(float, rgb, rgb))

namespace velodyne_ros
{
  struct EIGEN_ALIGN16 Point
  {
    PCL_ADD_POINT4D;
    float intensity;
    float time;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)
                                  (float, intensity, intensity)(float, time, time)(std::uint16_t, ring, ring))

namespace ouster_ros
{
  struct EIGEN_ALIGN16 Point
  {
      PCL_ADD_POINT4D;
      float intensity;
      uint32_t t;
      uint16_t reflectivity;
      uint8_t  ring;
      uint16_t ambient;
      uint32_t range;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
    (std::uint32_t, t, t)(std::uint16_t, reflectivity, reflectivity)(std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)(std::uint32_t, range, range)
)

namespace rslidar_ros
{
  struct EIGEN_ALIGN16 Point
  {
    PCL_ADD_POINT4D;
    float intensity;
    float time;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}
POINT_CLOUD_REGISTER_POINT_STRUCT(rslidar_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)
                                  (float, intensity, curvature)(float, time, normal_x)(uint16_t, ring, ring))

/**
 * @brief Preprocess class: LiDAR point cloud preprocessing
 *
 * Ported from better_fastlio2 to work without ROS.
 * Takes raw PCL point clouds (already converted from LCM PointCloud2)
 * instead of ROS messages.
 */
class Preprocess
{
public:
  Preprocess();
  ~Preprocess();

  // Process a PCL point cloud (already converted from LCM PointCloud2)
  void process(const pcl::PointCloud<velodyne_ros::Point>& pl_orig, pcl::PointCloud<PointType>::Ptr& pcl_out);
  void process(const pcl::PointCloud<ouster_ros::Point>& pl_orig, pcl::PointCloud<PointType>::Ptr& pcl_out);
  void process(const pcl::PointCloud<livox_ros::Point>& pl_orig, pcl::PointCloud<PointType>::Ptr& pcl_out);

  void set(bool feat_en, int lid_type, double bld, int pfilt_num);

  pcl::PointCloud<PointType> pl_full, pl_corn, pl_surf;
  pcl::PointCloud<PointType> pl_buff[128];    // maximum 128 line lidar
  vector<orgtype> typess[128];                // maximum 128 line lidar
  float time_unit_scale;
  int lidar_type, livox_type, point_filter_num, N_SCANS, SCAN_RATE, time_unit = SEC;
  double blind;
  bool feature_enabled, given_offset_time;

private:
  void velodyne_handler(const pcl::PointCloud<velodyne_ros::Point>& pl_orig);
  void oust64_handler(const pcl::PointCloud<ouster_ros::Point>& pl_orig);
  void livoxros_handler(const pcl::PointCloud<livox_ros::Point>& pl_orig);
  void give_feature(pcl::PointCloud<PointType> &pl, vector<orgtype> &types);
  int plane_judge(const pcl::PointCloud<PointType> &pl, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool small_plane(const pcl::PointCloud<PointType> &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool edge_jump_judge(const pcl::PointCloud<PointType> &pl, vector<orgtype> &types, uint i, Surround nor_dir);

  int group_size;
  double disA, disB, inf_bound;
  double limit_maxmid, limit_midmin, limit_maxmin;
  double p2l_ratio;
  double jump_up_limit, jump_down_limit;
  double cos160;
  double edgea, edgeb;
  double smallp_intersect, smallp_ratio;
  double vx, vy, vz;
};
