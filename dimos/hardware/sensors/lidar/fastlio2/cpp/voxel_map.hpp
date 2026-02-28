// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Efficient global voxel map using a hash map.
// Supports O(1) insert/update, distance-based pruning, and
// raycasting-based free space clearing via Amanatides & Woo 3D DDA.
// FOV is discovered dynamically from incoming point cloud data.

#ifndef VOXEL_MAP_HPP_
#define VOXEL_MAP_HPP_

#include <cmath>
#include <cstdint>
#include <unordered_map>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct VoxelKey {
    int32_t x, y, z;
    bool operator==(const VoxelKey& o) const { return x == o.x && y == o.y && z == o.z; }
};

struct VoxelKeyHash {
    size_t operator()(const VoxelKey& k) const {
        // Fast spatial hash — large primes reduce collisions for grid coords
        size_t h = static_cast<size_t>(k.x) * 73856093u;
        h ^= static_cast<size_t>(k.y) * 19349669u;
        h ^= static_cast<size_t>(k.z) * 83492791u;
        return h;
    }
};

struct Voxel {
    float x, y, z;       // running centroid
    float intensity;
    uint32_t count;       // points merged into this voxel
    uint8_t miss_count;   // consecutive scans where a ray passed through without hitting
};

/// Config for raycast-based free space clearing.
struct RaycastConfig {
    int subsample = 4;        // raycast every Nth point
    int max_misses = 3;       // erase after this many consecutive misses
    float fov_margin_rad = 0.035f;  // ~2° safety margin added to discovered FOV
};

class VoxelMap {
public:
    explicit VoxelMap(float voxel_size, float max_range = 100.0f)
        : voxel_size_(voxel_size), max_range_(max_range) {
        map_.reserve(500000);
    }

    /// Insert a point cloud into the map, merging into existing voxels.
    /// Resets miss_count for hit voxels.
    template <typename PointT>
    void insert(const typename pcl::PointCloud<PointT>::Ptr& cloud) {
        if (!cloud) return;
        float inv = 1.0f / voxel_size_;
        for (const auto& pt : cloud->points) {
            VoxelKey key{
                static_cast<int32_t>(std::floor(pt.x * inv)),
                static_cast<int32_t>(std::floor(pt.y * inv)),
                static_cast<int32_t>(std::floor(pt.z * inv))};

            auto it = map_.find(key);
            if (it != map_.end()) {
                // Running average update
                auto& v = it->second;
                float n = static_cast<float>(v.count);
                float n1 = n + 1.0f;
                v.x = (v.x * n + pt.x) / n1;
                v.y = (v.y * n + pt.y) / n1;
                v.z = (v.z * n + pt.z) / n1;
                v.intensity = (v.intensity * n + pt.intensity) / n1;
                v.count++;
                v.miss_count = 0;
            } else {
                map_.emplace(key, Voxel{pt.x, pt.y, pt.z, pt.intensity, 1, 0});
            }
        }
    }

    /// Cast rays from sensor origin through each point in the cloud.
    /// Discovers the sensor FOV from the cloud's elevation angle range,
    /// then marks intermediate voxels as missed and erases those exceeding
    /// the miss threshold within the discovered FOV.
    ///
    /// Orientation quaternion (qx,qy,qz,qw) is body→world.
    template <typename PointT>
    void raycast_clear(float ox, float oy, float oz,
                       float qx, float qy, float qz, float qw,
                       const typename pcl::PointCloud<PointT>::Ptr& cloud,
                       const RaycastConfig& cfg) {
        if (!cloud || cloud->empty() || cfg.max_misses <= 0) return;

        // Phase 0: discover FOV from this scan's elevation angles in sensor-local frame
        update_fov<PointT>(ox, oy, oz, qx, qy, qz, qw, cloud);

        // Skip raycasting until we have a valid FOV (need at least a few scans)
        if (!fov_valid_) return;

        float inv = 1.0f / voxel_size_;
        int n_pts = static_cast<int>(cloud->size());
        float fov_up = fov_up_ + cfg.fov_margin_rad;
        float fov_down = fov_down_ - cfg.fov_margin_rad;

        // Phase 1: walk rays, increment miss_count for intermediate voxels
        for (int i = 0; i < n_pts; i += cfg.subsample) {
            const auto& pt = cloud->points[i];
            raycast_single(ox, oy, oz, pt.x, pt.y, pt.z, inv);
        }

        // Phase 2: erase voxels that exceeded miss threshold and are within FOV
        for (auto it = map_.begin(); it != map_.end();) {
            if (it->second.miss_count > static_cast<uint8_t>(cfg.max_misses)) {
                if (in_sensor_fov(ox, oy, oz, qx, qy, qz, qw,
                                  it->second.x, it->second.y, it->second.z,
                                  fov_up, fov_down)) {
                    it = map_.erase(it);
                    continue;
                }
            }
            ++it;
        }
    }

    /// Remove voxels farther than max_range from the given position.
    void prune(float px, float py, float pz) {
        float r2 = max_range_ * max_range_;
        for (auto it = map_.begin(); it != map_.end();) {
            float dx = it->second.x - px;
            float dy = it->second.y - py;
            float dz = it->second.z - pz;
            if (dx * dx + dy * dy + dz * dz > r2)
                it = map_.erase(it);
            else
                ++it;
        }
    }

    /// Export all voxel centroids as a point cloud.
    template <typename PointT>
    typename pcl::PointCloud<PointT>::Ptr to_cloud() const {
        typename pcl::PointCloud<PointT>::Ptr cloud(
            new pcl::PointCloud<PointT>(map_.size(), 1));
        size_t i = 0;
        for (const auto& [key, v] : map_) {
            auto& pt = cloud->points[i++];
            pt.x = v.x;
            pt.y = v.y;
            pt.z = v.z;
            pt.intensity = v.intensity;
        }
        return cloud;
    }

    size_t size() const { return map_.size(); }
    void clear() { map_.clear(); }
    void set_max_range(float r) { max_range_ = r; }
    float fov_up_deg() const { return fov_up_ * 180.0f / static_cast<float>(M_PI); }
    float fov_down_deg() const { return fov_down_ * 180.0f / static_cast<float>(M_PI); }
    bool fov_valid() const { return fov_valid_; }

private:
    std::unordered_map<VoxelKey, Voxel, VoxelKeyHash> map_;
    float voxel_size_;
    float max_range_;

    // Dynamically discovered sensor FOV (accumulated over scans)
    float fov_up_ = -static_cast<float>(M_PI);    // start narrow, expand from data
    float fov_down_ = static_cast<float>(M_PI);
    int fov_scan_count_ = 0;
    bool fov_valid_ = false;
    static constexpr int FOV_WARMUP_SCANS = 5;  // require N scans before trusting FOV

    /// Update discovered FOV from a scan's elevation angles in sensor-local frame.
    template <typename PointT>
    void update_fov(float ox, float oy, float oz,
                    float qx, float qy, float qz, float qw,
                    const typename pcl::PointCloud<PointT>::Ptr& cloud) {
        // Inverse quaternion for world→sensor rotation
        float nqx = -qx, nqy = -qy, nqz = -qz;

        for (const auto& pt : cloud->points) {
            float wx = pt.x - ox, wy = pt.y - oy, wz = pt.z - oz;

            // Rotate to sensor-local frame
            float tx = 2.0f * (nqy * wz - nqz * wy);
            float ty = 2.0f * (nqz * wx - nqx * wz);
            float tz = 2.0f * (nqx * wy - nqy * wx);
            float lx = wx + qw * tx + (nqy * tz - nqz * ty);
            float ly = wy + qw * ty + (nqz * tx - nqx * tz);
            float lz = wz + qw * tz + (nqx * ty - nqy * tx);

            float horiz_dist = std::sqrt(lx * lx + ly * ly);
            if (horiz_dist < 1e-6f) continue;
            float elevation = std::atan2(lz, horiz_dist);

            if (elevation > fov_up_) fov_up_ = elevation;
            if (elevation < fov_down_) fov_down_ = elevation;
        }

        if (++fov_scan_count_ >= FOV_WARMUP_SCANS && !fov_valid_) {
            fov_valid_ = true;
            printf("[voxel_map] FOV discovered: [%.1f, %.1f] deg\n",
                   fov_down_deg(), fov_up_deg());
        }
    }

    /// Amanatides & Woo 3D DDA: walk from (ox,oy,oz) to (px,py,pz),
    /// incrementing miss_count for all intermediate voxels.
    void raycast_single(float ox, float oy, float oz,
                        float px, float py, float pz, float inv) {
        float dx = px - ox, dy = py - oy, dz = pz - oz;
        float len = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (len < 1e-6f) return;
        dx /= len; dy /= len; dz /= len;

        int32_t cx = static_cast<int32_t>(std::floor(ox * inv));
        int32_t cy = static_cast<int32_t>(std::floor(oy * inv));
        int32_t cz = static_cast<int32_t>(std::floor(oz * inv));
        int32_t ex = static_cast<int32_t>(std::floor(px * inv));
        int32_t ey = static_cast<int32_t>(std::floor(py * inv));
        int32_t ez = static_cast<int32_t>(std::floor(pz * inv));

        int sx = (dx >= 0) ? 1 : -1;
        int sy = (dy >= 0) ? 1 : -1;
        int sz = (dz >= 0) ? 1 : -1;

        // tMax: parametric distance along ray to next voxel boundary per axis
        // tDelta: parametric distance to cross one full voxel per axis
        float tMaxX = (std::abs(dx) < 1e-10f) ? 1e30f
            : (((dx > 0 ? cx + 1 : cx) * voxel_size_ - ox) / dx);
        float tMaxY = (std::abs(dy) < 1e-10f) ? 1e30f
            : (((dy > 0 ? cy + 1 : cy) * voxel_size_ - oy) / dy);
        float tMaxZ = (std::abs(dz) < 1e-10f) ? 1e30f
            : (((dz > 0 ? cz + 1 : cz) * voxel_size_ - oz) / dz);

        float tDeltaX = (std::abs(dx) < 1e-10f) ? 1e30f : std::abs(voxel_size_ / dx);
        float tDeltaY = (std::abs(dy) < 1e-10f) ? 1e30f : std::abs(voxel_size_ / dy);
        float tDeltaZ = (std::abs(dz) < 1e-10f) ? 1e30f : std::abs(voxel_size_ / dz);

        // Walk through voxels (skip endpoint — it was hit)
        int max_steps = static_cast<int>(len * inv) + 3;  // safety bound
        for (int step = 0; step < max_steps; ++step) {
            if (cx == ex && cy == ey && cz == ez) break;  // reached endpoint

            VoxelKey key{cx, cy, cz};
            auto it = map_.find(key);
            if (it != map_.end() && it->second.miss_count < 255) {
                it->second.miss_count++;
            }

            // Step to next voxel on the axis with smallest tMax
            if (tMaxX < tMaxY && tMaxX < tMaxZ) {
                cx += sx; tMaxX += tDeltaX;
            } else if (tMaxY < tMaxZ) {
                cy += sy; tMaxY += tDeltaY;
            } else {
                cz += sz; tMaxZ += tDeltaZ;
            }
        }
    }

    /// Check if a voxel centroid falls within the sensor's vertical FOV.
    /// Rotates the vector (sensor→voxel) into sensor-local frame using the
    /// inverse of the body→world quaternion, then checks elevation angle.
    static bool in_sensor_fov(float ox, float oy, float oz,
                              float qx, float qy, float qz, float qw,
                              float vx, float vy, float vz,
                              float fov_up_rad, float fov_down_rad) {
        // Vector from sensor origin to voxel in world frame
        float wx = vx - ox, wy = vy - oy, wz = vz - oz;

        // Rotate by quaternion inverse (conjugate): q* = (-qx,-qy,-qz,qw)
        float nqx = -qx, nqy = -qy, nqz = -qz;
        // t = 2 * cross(q.xyz, v)
        float tx = 2.0f * (nqy * wz - nqz * wy);
        float ty = 2.0f * (nqz * wx - nqx * wz);
        float tz = 2.0f * (nqx * wy - nqy * wx);
        // v' = v + qw * t + cross(q.xyz, t)
        float lx = wx + qw * tx + (nqy * tz - nqz * ty);
        float ly = wy + qw * ty + (nqz * tx - nqx * tz);
        float lz = wz + qw * tz + (nqx * ty - nqy * tx);

        // Elevation angle in sensor-local frame
        float horiz_dist = std::sqrt(lx * lx + ly * ly);
        if (horiz_dist < 1e-6f) return true;  // directly above/below, treat as in FOV
        float elevation = std::atan2(lz, horiz_dist);

        return elevation >= fov_down_rad && elevation <= fov_up_rad;
    }
};

#endif
