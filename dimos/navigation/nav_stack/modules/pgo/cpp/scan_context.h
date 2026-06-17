// Scan Context — polar-binned lidar place-recognition descriptor.
//
// Each scan becomes an (N_rings × N_sectors) matrix where cell [i, j]
// holds the max z value among points falling in the (range, azimuth)
// bin. The "ring key" — the per-row mean — is the coarse feature used
// for fast kd-tree retrieval; the full matrix is then column-shifted
// against the candidate to measure rotation-invariant cosine distance.
//
// Inspired by Kim & Kim 2018 "Scan Context: Egocentric Spatial
// Descriptor for Place Recognition within 3D Point Cloud Map" and the
// reference implementation at github.com/irapkaist/scancontext (MIT).
// Reimplemented locally to keep the PGO module self-contained and to
// avoid the OpenCV/external-yaml deps the upstream version carries.

#pragma once

#include "commons.h"

#include <Eigen/Dense>

#include <vector>

namespace scan_context {

struct Config {
    int n_rings = 20;             // radial bins
    int n_sectors = 60;           // azimuth bins
    double max_range_m = 80.0;    // ignore points beyond this
    int candidate_top_k = 10;     // kd-tree neighbours to score
    double match_threshold = 0.4; // accepted cosine distance (0..2)
    // Shifts body-frame z so all cells are positive before cosine distance,
    // matching irapkaist/scancontext's LIDAR_HEIGHT convention. Ground points
    // sit near -lidar_height_m in the body frame; without this shift, negative
    // cells make cosine similarity meaningless for revisits.
    double lidar_height_m = 2.0;
};

using Descriptor = Eigen::MatrixXf;   // (n_rings × n_sectors)
using RingKey = Eigen::VectorXf;      // length n_rings
using SectorKey = Eigen::VectorXf;    // length n_sectors

// Build the polar-max-z descriptor for a body-frame scan. Points
// outside ``max_range_m`` or with negative ring index are ignored.
Descriptor make_descriptor(const CloudType& cloud, const Config& config);

// Mean per row — the coarse feature used for kd-tree retrieval.
RingKey make_ring_key(const Descriptor& descriptor);

// Mean per column — only used for the optional sector-key alignment.
SectorKey make_sector_key(const Descriptor& descriptor);

// Cosine distance between two descriptors after column-shifting
// ``candidate`` by ``shift`` columns. 0 = identical, 2 = opposite.
float column_cosine_distance(const Descriptor& query,
                             const Descriptor& candidate,
                             int shift);

// Best (min-distance, best-shift) pair across all column shifts.
// Returns {distance, shift_columns}. To recover yaw rotation from the
// shift: yaw_rad = -2*M_PI * shift / n_sectors.
std::pair<float, int> best_distance(const Descriptor& query,
                                    const Descriptor& candidate);

// Convert sector shift to yaw rotation (radians).
// shift comes from best_distance, which scans [0, n_sectors-1], so
// the raw yaw lies in (-2pi, 0]; wrap into [-pi, pi].
inline double yaw_from_shift(int shift, int n_sectors) {
    double yaw = -2.0 * M_PI * static_cast<double>(shift) /
                 static_cast<double>(n_sectors);
    if (yaw < -M_PI) yaw += 2.0 * M_PI;
    return yaw;
}

}  // namespace scan_context
