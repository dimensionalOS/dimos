//
// Created by shibo zhao on 2020/9/27.
//

#ifndef LOCALMAP_H
#define LOCALMAP_H

#include <cmath>
#include <iostream>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#include <tbb/parallel_for.h>

#include <Eigen/Dense>

#include "arise_slam_mid360/common.h"
#include "arise_slam_mid360/tic_toc.h"

struct MapBlock
{

  MapBlock() = default;

  pcl::PointCloud<PointType>::Ptr pcorner_pc_ = nullptr;
  pcl::PointCloud<PointType>::Ptr psurf_pc_ = nullptr;
  pcl::KdTreeFLANN<PointType>::Ptr pkdtree_corner_from_block_ = nullptr;
  pcl::KdTreeFLANN<PointType>::Ptr pkdtree_surf_from_block_ = nullptr;

  bool dirty = false;
  bool in_use = false;

  //  kdtree::DynamicKdTree3d<PointType, point_cloud::PointCloud> dynamic_kdtreeCornerFromMap;
  //  kdtree::DynamicKdTree3d<PointType, point_cloud::PointCloud>  dynamic_kdtreeSurfFromMap;

  bool bnull_ = true;
  bool bcorner_null_ = true;
  bool bsurf_null_ = true;
  bool bnewcorner_points_add_ = false;
  bool bnewsurf_points_add_ = false;

  inline void clear()
  {
    pcorner_pc_ = nullptr;
    psurf_pc_ = nullptr;
    pkdtree_corner_from_block_ = nullptr;
    pkdtree_surf_from_block_ = nullptr;

    bnull_ = true;
    bcorner_null_ = true;
    bsurf_null_ = true;
    bnewcorner_points_add_ = false;
    bnewsurf_points_add_ = false;
  }

  inline void insert_corner(const PointType &point)
  {
    if (pcorner_pc_ == nullptr)
    {
      pcorner_pc_.reset(new pcl::PointCloud<PointType>());
    }
    if (bnull_ or bcorner_null_)
    {
      bnull_ = false;
      bcorner_null_ = false;
    }
    pcorner_pc_->push_back(point);
  }

  inline void insert_surf(const PointType &point)
  {
    if (psurf_pc_ == nullptr)
    {
      psurf_pc_.reset(new pcl::PointCloud<PointType>());
    }
    if (bnull_ or bsurf_null_)
    {
      bnull_ = false;
      bcorner_null_ = false;
    }
    psurf_pc_->push_back(point);
  }

  inline int corner_pc_size() const
  {
    if (pcorner_pc_ == nullptr)
      return 0;
    return pcorner_pc_->size();
  }

  inline int surf_pc_size() const
  {
    if (psurf_pc_ == nullptr)
      return 0;
    return psurf_pc_->size();
  }

  inline bool empty() const { return bnull_; }

  inline bool corner_points_isempty() const { return bcorner_null_; }

  inline bool surf_points_isempty() const { return bsurf_null_; }

  inline bool have_newcorner_points() const { return bnewcorner_points_add_; }

  inline bool have_newsurf_points() const { return bnewsurf_points_add_; }
};

class LocalMap
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr const int laserCloudWidth = 21;
  static constexpr const int laserCloudHeight = 21;
  static constexpr int laserCloudDepth = 11;

  static constexpr int laserCloudNum =
      laserCloudWidth * laserCloudHeight * laserCloudDepth; // 4851

  static constexpr double voxelResulation = 30;
  static constexpr double halfVoxelResulation = voxelResulation * 0.5;

public:
  LocalMap()
  {

    origin_ = Eigen::Vector3i(laserCloudWidth * 0.5, laserCloudHeight * 0.5,
                              laserCloudDepth * 0.5);
  }


  Eigen::Vector3i shiftMap(const Eigen::Vector3d &t_w_cur)
  {

    // 计算当前激光的位置相对于栅格地图的位置
    int centerCubeI =
        int((t_w_cur.x() + halfVoxelResulation) / voxelResulation) +
        origin_.x();
    int centerCubeJ =
        int((t_w_cur.y() + halfVoxelResulation) / voxelResulation) +
        origin_.y();
    int centerCubeK =
        int((t_w_cur.z() + halfVoxelResulation) / voxelResulation) +
        origin_.z();

    if (t_w_cur.x() + halfVoxelResulation < 0)
      centerCubeI--;
    if (t_w_cur.y() + halfVoxelResulation < 0)
      centerCubeJ--;
    if (t_w_cur.z() + halfVoxelResulation < 0)
      centerCubeK--;

    while (centerCubeI < 3)
    {
      for (int j = 0; j < laserCloudHeight; j++)
      {
        for (int k = 0; k < laserCloudDepth; k++)
        {
          int i = laserCloudWidth - 1;
          for (; i >= 1; i--)
          {
            map_[i + laserCloudWidth * j +
                 laserCloudWidth * laserCloudHeight * k] =
                map_[i - 1 + laserCloudWidth * j +
                     laserCloudWidth * laserCloudHeight * k];
          }

          map_[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k].clear();
        }
      }

      centerCubeI++;
      origin_.x() = origin_.x() + 1;
    }

    while (centerCubeI >= laserCloudWidth - 3)
    {
      for (int j = 0; j < laserCloudHeight; j++)
      {
        for (int k = 0; k < laserCloudDepth; k++)
        {
          int i = 0;
          for (; i < laserCloudWidth - 1; i++)
          {
            map_[i + laserCloudWidth * j +
                 laserCloudWidth * laserCloudHeight * k] =
                map_[i + 1 + laserCloudWidth * j +
                     laserCloudWidth * laserCloudHeight * k];
          }
          map_[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k]
              .clear();
        }
      }

      centerCubeI--;
      origin_.x() = origin_.x() - 1;
    }

    while (centerCubeJ < 3)
    {
      for (int i = 0; i < laserCloudWidth; i++)
      {
        for (int k = 0; k < laserCloudDepth; k++)
        {

          int j = laserCloudHeight - 1;

          for (; j >= 1; j--)
          {
            map_[i + laserCloudWidth * j +
                 laserCloudWidth * laserCloudHeight * k] =
                map_[i + laserCloudWidth * (j - 1) +
                     laserCloudWidth * laserCloudHeight * k];
          }

          map_[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k]
              .clear();
        }
      }

      centerCubeJ++;
      origin_.y() = origin_.y() + 1;
    }

    while (centerCubeJ >= laserCloudHeight - 3)
    {
      for (int i = 0; i < laserCloudWidth; i++)
      {
        for (int k = 0; k < laserCloudDepth; k++)
        {
          int j = 0;

          for (; j < laserCloudHeight - 1; j++)
          {
            map_[i + laserCloudWidth * j +
                 laserCloudWidth * laserCloudHeight * k] =
                map_[i + laserCloudWidth * (j + 1) +
                     laserCloudWidth * laserCloudHeight * k];
          }
          map_[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k]
              .clear();
        }
      }

      centerCubeJ--;
      origin_.y() = origin_.y() - 1;
    }

    while (centerCubeK < 3)
    {
      for (int i = 0; i < laserCloudWidth; i++)
      {
        for (int j = 0; j < laserCloudHeight; j++)
        {
          int k = laserCloudDepth - 1;

          for (; k >= 1; k--)
          {
            map_[i + laserCloudWidth * j +
                 laserCloudWidth * laserCloudHeight * k] =
                map_[i + laserCloudWidth * j +
                     laserCloudWidth * laserCloudHeight * (k - 1)];
          }
          map_[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k]
              .clear();
        }
      }

      centerCubeK++;
      origin_.z() = origin_.z() + 1;
    }

    while (centerCubeK >= laserCloudDepth - 3)
    {
      for (int i = 0; i < laserCloudWidth; i++)
      {
        for (int j = 0; j < laserCloudHeight; j++)
        {
          int k = 0;

          for (; k < laserCloudDepth - 1; k++)
          {
            map_[i + laserCloudWidth * j +
                 laserCloudWidth * laserCloudHeight * k] =
                map_[i + laserCloudWidth * j +
                     laserCloudWidth * laserCloudHeight * (k + 1)];
          }
          map_[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k]
              .clear();
        }
      }

      centerCubeK--;
      origin_.z() = origin_.z() - 1;
    }

    return Eigen::Vector3i{centerCubeI, centerCubeJ, centerCubeK};
  } // function shiftMap

  /// \brief obtain 5x5 local grid map corner, surf
  /// \param position　the position of robot in local grid
  /// \return std::tuple<int, int> corner, surf point size
  std::tuple<int, int>
  get_5x5localmap_featuresize(const Eigen::Vector3i &position)
  {

    int centerCubeI, centerCubeJ, centerCubeK;
    centerCubeI = position.x();
    centerCubeJ = position.y();
    centerCubeK = position.z();

    int laserCloudCornerFromMapNum = 0;
    int laserCloudSurfFromMapNum = 0;

    for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++)
    {
      for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++)
      {
        for (int k = centerCubeK - 1; k <= centerCubeK + 1; k++)
        {

          if (i >= 0 && i < laserCloudWidth && j >= 0 && j < laserCloudHeight &&
              k >= 0 && k < laserCloudDepth)
          {

            int cubeInd = i + laserCloudWidth * j +
                          laserCloudWidth * laserCloudHeight * k;
            laserCloudCornerFromMapNum += map_[cubeInd].corner_pc_size();
            laserCloudSurfFromMapNum += map_[cubeInd].surf_pc_size();
          }
        }
      }
    }

    // RCLCPP_INFO(this->get_logger(),"laserCloudCornerFromMapNum: %d", laserCloudCornerFromMapNum);
    // RCLCPP_INFO(this->get_logger(),"laserCloudSurfFromMapNum: %d", laserCloudSurfFromMapNum);

    return std::make_tuple(laserCloudCornerFromMapNum,
                           laserCloudSurfFromMapNum);
  } // function get_localmap_featuresize

  /// \brief  search nearest coner point
  /// \param pt_query
  /// \param k_indices
  /// \param k_sqr_distances
  /// \return
  bool nearestKSearch_corner(const PointType &pt_query,
                             std::vector<PointType> &k_pts,
                             std::vector<float> &k_sqr_distances) const
  {

    k_pts.clear();

    int cubeI =
        int((pt_query.x + halfVoxelResulation) / voxelResulation) + origin_.x();
    int cubeJ =
        int((pt_query.y + halfVoxelResulation) / voxelResulation) + origin_.y();
    int cubeK =
        int((pt_query.z + halfVoxelResulation) / voxelResulation) + origin_.z();

    if (pt_query.x + halfVoxelResulation < 0)
      cubeI--;
    if (pt_query.y + halfVoxelResulation < 0)
      cubeJ--;
    if (pt_query.z + halfVoxelResulation < 0)
      cubeK--;

    if (!(cubeI >= 0 && cubeI < laserCloudWidth && cubeJ >= 0 &&
          cubeJ < laserCloudHeight && cubeK >= 0 && cubeK < laserCloudDepth))
    {
      return false;
    }

    int cubeInd = cubeI + laserCloudWidth * cubeJ +
                  laserCloudWidth * laserCloudHeight * cubeK;
    if (map_[cubeInd].pkdtree_corner_from_block_ == nullptr)
      return false;

    std::vector<int> k_indices;
    map_[cubeInd].pkdtree_corner_from_block_->nearestKSearch(
        pt_query, 5, k_indices, k_sqr_distances);
    for (auto id : k_indices)
      k_pts.push_back(map_[cubeInd].pcorner_pc_->points[id]);

    return true;
  } // // function nearestKSearch_corner

  /// \brief search nearset surf point
  /// \param pt_query
  /// \param k_indices
  /// \param k_sqr_distances
  /// \return
  bool nearestKSearch_surf(const PointType &pt_query,
                           std::vector<PointType> &k_pts,
                           std::vector<float> &k_sqr_distances) const
  {

    k_pts.clear();

    int cubeI =
        int((pt_query.x + halfVoxelResulation) / voxelResulation) + origin_.x();
    int cubeJ =
        int((pt_query.y + halfVoxelResulation) / voxelResulation) + origin_.y();
    int cubeK =
        int((pt_query.z + halfVoxelResulation) / voxelResulation) + origin_.z();

    if (pt_query.x + halfVoxelResulation < 0)
      cubeI--;
    if (pt_query.y + halfVoxelResulation < 0)
      cubeJ--;
    if (pt_query.z + halfVoxelResulation < 0)
      cubeK--;

    if (!(cubeI >= 0 && cubeI < laserCloudWidth && cubeJ >= 0 &&
          cubeJ < laserCloudHeight && cubeK >= 0 && cubeK < laserCloudDepth))
    {
      return false;
    }

    int cubeInd = cubeI + laserCloudWidth * cubeJ +
                  laserCloudWidth * laserCloudHeight * cubeK;

    if (map_[cubeInd].pkdtree_surf_from_block_ == nullptr)
      return false;

    std::vector<int> k_indices;

    map_[cubeInd].pkdtree_surf_from_block_->nearestKSearch(
        pt_query, 5, k_indices, k_sqr_distances);

    for (auto id : k_indices)
      k_pts.push_back(map_[cubeInd].psurf_pc_->points[id]);

    return true;
  } // function nearestKSearch_surf


  void addCornerPointCloud(pcl::PointCloud<PointType> &laserCloudCornerStack)
  {

    //step1:Determine which block the point cloud of the new frame is distributed in, and 
    // add the laser point to the corresponding block
    std::set<int> blockInd;
    for (const auto &point : laserCloudCornerStack)
    {
      int cubeI =
          int((point.x + halfVoxelResulation) / voxelResulation) + origin_.x();
      int cubeJ =
          int((point.y + halfVoxelResulation) / voxelResulation) + origin_.y();
      int cubeK =
          int((point.z + halfVoxelResulation) / voxelResulation) + origin_.z();

      if (point.x + halfVoxelResulation < 0)
        cubeI--;
      if (point.y + halfVoxelResulation < 0)
        cubeJ--;
      if (point.z + halfVoxelResulation < 0)
        cubeK--;

      if (cubeI >= 0 && cubeI < laserCloudWidth && cubeJ >= 0 &&
          cubeJ < laserCloudHeight && cubeK >= 0 && cubeK < laserCloudDepth)
      {
        int cubeInd = cubeI + laserCloudWidth * cubeJ +
                      laserCloudWidth * laserCloudHeight * cubeK;
        blockInd.insert(cubeInd);
        map_[cubeInd].insert_corner(point);
        map_[cubeInd].dirty = true;
      }
    }


    std::vector<int> vblockInd(blockInd.begin(), blockInd.end());

    auto compute_func =
        [&](const tbb::blocked_range<std::vector<int>::iterator> &range) {
          for (auto &iter : range)
          {
            if(map_[iter].in_use == false && forget_far_chunks_){
              map_[iter].pcorner_pc_->points.clear();
              return;
            }

            pcl::PointCloud<PointType>::Ptr tmpCorner(
                new pcl::PointCloud<PointType>());
            pcl::VoxelGrid<PointType> downSizeFilterCorner;

            // RCLCPP_INFO(this->get_logger(), "pre filter size: %d", (int) map_[iter].pcorner_pc_->size());
            downSizeFilterCorner.setLeafSize(lineRes_, lineRes_, lineRes_);
            downSizeFilterCorner.setInputCloud(map_[iter].pcorner_pc_);
            downSizeFilterCorner.filter(*tmpCorner);

            map_[iter].pcorner_pc_ = tmpCorner;
            // RCLCPP_INFO(this->get_logger(), "pos filter size: %d", (int)map_[iter].pcorner_pc_->size());

            if (map_[iter].pkdtree_corner_from_block_ == nullptr)
              map_[iter].pkdtree_corner_from_block_.reset(
                  new pcl::KdTreeFLANN<PointType>());

            map_[iter].pkdtree_corner_from_block_->setInputCloud(
                map_[iter].pcorner_pc_);
          }
        };

    tbb::blocked_range<std::vector<int>::iterator> range(vblockInd.begin(),
                                                         vblockInd.end());
    tbb::parallel_for(range, compute_func);

    for (auto &block : map_)
    {
      block.in_use = block.dirty;
      // block.dirty = false;
    }
  } // function addCornerPointCloud

  /// \brief add surf point, and build kdtree
  /// \param laserCloudSurfStack
  void addSurfPointCloud(pcl::PointCloud<PointType> &laserCloudSurfStack)
  {

    std::set<int> blockInd;

    for (const auto &point : laserCloudSurfStack)
    {

      int cubeI =
          int((point.x + halfVoxelResulation) / voxelResulation) + origin_.x();
      int cubeJ =
          int((point.y + halfVoxelResulation) / voxelResulation) + origin_.y();
      int cubeK =
          int((point.z + halfVoxelResulation) / voxelResulation) + origin_.z();

      if (point.x + halfVoxelResulation < 0)
        cubeI--;
      if (point.y + halfVoxelResulation < 0)
        cubeJ--;
      if (point.z + halfVoxelResulation < 0)
        cubeK--;

      if (cubeI >= 0 && cubeI < laserCloudWidth && cubeJ >= 0 &&
          cubeJ < laserCloudHeight && cubeK >= 0 && cubeK < laserCloudDepth)
      {
        int cubeInd = cubeI + laserCloudWidth * cubeJ +
                      laserCloudWidth * laserCloudHeight * cubeK;

        blockInd.insert(cubeInd);

        map_[cubeInd].insert_surf(point);
        map_[cubeInd].dirty = true;
      }
      
    }

    std::vector<int> vblockInd(blockInd.begin(), blockInd.end());

    auto compute_func =
        [&](const tbb::blocked_range<std::vector<int>::iterator> &range) {
          for (auto &iter : range)
          {
            pcl::PointCloud<PointType>::Ptr tmpSurf(
                new pcl::PointCloud<PointType>());
            pcl::VoxelGrid<PointType> downSizeFilterSurf;

            double voxel_size = planeRes_;
            if(map_[iter].in_use == false && forget_far_chunks_){
              map_[iter].psurf_pc_->points.clear();
              return;
            }

            // RCLCPP_INFO(this->get_logger(), "voxel_size: %f", voxel_size);
            downSizeFilterSurf.setLeafSize(voxel_size, voxel_size, voxel_size);
            downSizeFilterSurf.setInputCloud(map_[iter].psurf_pc_);
            downSizeFilterSurf.filter(*tmpSurf);
            map_[iter].psurf_pc_ = tmpSurf;

            if (map_[iter].pkdtree_surf_from_block_ == nullptr)
              map_[iter].pkdtree_surf_from_block_.reset(
                  new pcl::KdTreeFLANN<PointType>());

            map_[iter].pkdtree_surf_from_block_->setInputCloud(
                map_[iter].psurf_pc_);
          }
        };

    tbb::blocked_range<std::vector<int>::iterator> range(vblockInd.begin(),
                                                         vblockInd.end());
    tbb::parallel_for(range, compute_func);

    for (auto &block : map_)
    {
      block.in_use = block.dirty;
      block.dirty = false;
    }
  } // function addSurfPointCloud

  pcl::PointCloud<PointType> get_all_localmap() const
  {
    pcl::PointCloud<PointType> laserCloudMap;

    for (const auto &cube : map_)
    {
      if (cube.pcorner_pc_)
        laserCloudMap += *(cube.pcorner_pc_);
      if (cube.psurf_pc_)
        laserCloudMap += *(cube.psurf_pc_);
    }

    return laserCloudMap;
  } // function get_all_localmap

  pcl::PointCloud<PointType>
  get_5x5_localmap(const Eigen::Vector3i &position) const
  {
    pcl::PointCloud<PointType> laserCloudMap;

    int centerCubeI, centerCubeJ, centerCubeK;
    centerCubeI = position.x();
    centerCubeJ = position.y();
    centerCubeK = position.z();

    for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++)
    {
      for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++)
      {
        for (int k = centerCubeK - 1; k <= centerCubeK + 1; k++)
        {

          if (i >= 0 && i < laserCloudWidth && j >= 0 && j < laserCloudHeight &&
              k >= 0 && k < laserCloudDepth)
          {

            int cubeInd = i + laserCloudWidth * j +
                          laserCloudWidth * laserCloudHeight * k;
            if (map_[cubeInd].pcorner_pc_)
              laserCloudMap += *(map_[cubeInd].pcorner_pc_);

            if (map_[cubeInd].psurf_pc_)
              laserCloudMap += *(map_[cubeInd].psurf_pc_);
          }
        }
      }
    }

    return laserCloudMap;
  } // function get_5x5_localmap

  pcl::PointCloud<PointType>
  get_5x5_localmap_corner(const Eigen::Vector3i &position) const
  {
    pcl::PointCloud<PointType> laserCloudMap;

    int centerCubeI, centerCubeJ, centerCubeK;
    centerCubeI = position.x();
    centerCubeJ = position.y();
    centerCubeK = position.z();

    for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++)
    {
      for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++)
      {
        for (int k = centerCubeK - 1; k <= centerCubeK + 1; k++)
        {

          if (i >= 0 && i < laserCloudWidth && j >= 0 && j < laserCloudHeight &&
              k >= 0 && k < laserCloudDepth)
          {

            int cubeInd = i + laserCloudWidth * j +
                          laserCloudWidth * laserCloudHeight * k;
            if (map_[cubeInd].pcorner_pc_)
              laserCloudMap += *(map_[cubeInd].pcorner_pc_);
          }
        }
      }
    }

    return laserCloudMap;
  } // function get_5x5_localmap_corner

  pcl::PointCloud<PointType>
  get_5x5_localmap_surf(const Eigen::Vector3i &position) const
  {
    pcl::PointCloud<PointType> laserCloudMap;

    int centerCubeI, centerCubeJ, centerCubeK;
    centerCubeI = position.x();
    centerCubeJ = position.y();
    centerCubeK = position.z();

    for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++)
    {
      for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++)
      {
        for (int k = centerCubeK - 1; k <= centerCubeK + 1; k++)
        {

          if (i >= 0 && i < laserCloudWidth && j >= 0 && j < laserCloudHeight &&
              k >= 0 && k < laserCloudDepth)
          {

            int cubeInd = i + laserCloudWidth * j +
                          laserCloudWidth * laserCloudHeight * k;

            if (map_[cubeInd].psurf_pc_)
              laserCloudMap += *(map_[cubeInd].psurf_pc_);
          }
        }
      }
    }

    return laserCloudMap;
  } // function get_5x5_localmap_corner

public:
  // localmap
  std::array<MapBlock, laserCloudNum> map_;

  float lineRes_ = 0.3;
  float planeRes_ = 0.6;

  bool forget_far_chunks_ = false;

  Eigen::Vector3i origin_;
};

#endif // LOCALMAP_H
