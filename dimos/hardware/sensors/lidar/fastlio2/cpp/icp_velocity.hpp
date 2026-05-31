// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Scan-to-scan ICP velocity — an independent estimate of how fast the
// lidar is actually moving, derived purely from registering this scan
// against the previous one. Cross-check against the IESKF's velocity
// state; if they disagree by a lot, the IESKF is likely diverging.

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>

#include <chrono>
#include <memory>

namespace icp_velocity {

using PointT = pcl::PointXYZINormal;
using CloudT = pcl::PointCloud<PointT>;

struct Estimator {
    // Last scan in body frame + its sensor-ts. nullptr until the first
    // scan has been ingested.
    CloudT::Ptr last_scan;
    double last_ts = 0.0;

    // ICP parameters. Tuned for Mid-360 scans (~96 pts per packet, several
    // hundred packets per 10 Hz scan, ~30k points / scan). Voxel-downsample
    // to ~1000-2000 points so ICP runs in single-digit ms.
    float voxel_size = 0.2f;
    float max_corr_dist = 1.0f;
    int max_iterations = 20;
    double transform_eps = 1e-6;

    // Estimate per-scan body-frame velocity. Returns
    //   (ok, vx, vy, vz, scan_dt)
    // where vx/vy/vz is the body-frame velocity (m/s) from the last scan
    // to the current one. Caches the (down-sampled) current scan for the
    // next call. ok=false on the first call or when ICP fails to converge.
    struct Result {
        bool ok = false;
        float vx = 0.0f, vy = 0.0f, vz = 0.0f;
        double scan_dt = 0.0;
    };

    Result step(const CloudT::Ptr& raw_scan, double ts) {
        Result r;
        CloudT::Ptr downsampled(new CloudT);
        if (raw_scan && !raw_scan->empty()) {
            pcl::VoxelGrid<PointT> vg;
            vg.setInputCloud(raw_scan);
            vg.setLeafSize(voxel_size, voxel_size, voxel_size);
            vg.filter(*downsampled);
        }
        if (!last_scan || last_scan->empty() || !downsampled || downsampled->size() < 50) {
            // First call, or too few points for a reliable ICP. Cache and exit.
            if (downsampled && downsampled->size() >= 50) {
                last_scan = downsampled;
                last_ts = ts;
            }
            return r;
        }

        pcl::IterativeClosestPoint<PointT, PointT> icp;
        icp.setInputSource(downsampled);
        icp.setInputTarget(last_scan);
        icp.setMaxCorrespondenceDistance(max_corr_dist);
        icp.setMaximumIterations(max_iterations);
        icp.setTransformationEpsilon(transform_eps);
        CloudT aligned;
        icp.align(aligned);

        if (icp.hasConverged()) {
            const Eigen::Matrix4f T = icp.getFinalTransformation();
            const double dt = ts - last_ts;
            if (dt > 0.0) {
                r.ok = true;
                r.vx = T(0, 3) / static_cast<float>(dt);
                r.vy = T(1, 3) / static_cast<float>(dt);
                r.vz = T(2, 3) / static_cast<float>(dt);
                r.scan_dt = dt;
            }
        }

        last_scan = downsampled;
        last_ts = ts;
        return r;
    }
};

}  // namespace icp_velocity
