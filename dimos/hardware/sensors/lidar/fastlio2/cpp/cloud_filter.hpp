// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Point cloud filtering utilities: voxel grid downsampling and
// statistical outlier removal using PCL.

#ifndef CLOUD_FILTER_HPP_
#define CLOUD_FILTER_HPP_

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct CloudFilterConfig {
    float voxel_size = 0.1f;
    int sor_mean_k = 50;
    float sor_stddev = 1.0f;
};

/// Apply voxel grid downsample + statistical outlier removal in-place.
/// Returns the filtered cloud (new allocation).
template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr filter_cloud(
    const typename pcl::PointCloud<PointT>::Ptr& input,
    const CloudFilterConfig& cfg) {

    if (!input || input->empty()) return input;

    // Voxel grid downsample
    typename pcl::PointCloud<PointT>::Ptr voxelized(new pcl::PointCloud<PointT>());
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(input);
    vg.setLeafSize(cfg.voxel_size, cfg.voxel_size, cfg.voxel_size);
    vg.filter(*voxelized);

    // Statistical outlier removal
    if (cfg.sor_mean_k > 0 && voxelized->size() > static_cast<size_t>(cfg.sor_mean_k)) {
        typename pcl::PointCloud<PointT>::Ptr cleaned(new pcl::PointCloud<PointT>());
        pcl::StatisticalOutlierRemoval<PointT> sor;
        sor.setInputCloud(voxelized);
        sor.setMeanK(cfg.sor_mean_k);
        sor.setStddevMulThresh(cfg.sor_stddev);
        sor.filter(*cleaned);
        return cleaned;
    }

    return voxelized;
}

#endif
