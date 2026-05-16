#include "scan_context.h"

#include <algorithm>
#include <cmath>

namespace scan_context {

Descriptor make_descriptor(const CloudType& cloud, const Config& config) {
    // Empty cells stay at 0; we shift z by lidar_height so real points
    // are strictly positive and "no point here" is distinguishable from
    // ground level. Matches irapkaist/scancontext's NO_POINT convention
    // closely enough that the column-wise cosine distance behaves.
    Descriptor descriptor = Descriptor::Constant(config.n_rings, config.n_sectors, 0.0f);
    if (config.n_rings <= 0 || config.n_sectors <= 0 || config.max_range_m <= 0.0) {
        return descriptor;
    }

    const double ring_step = config.max_range_m / static_cast<double>(config.n_rings);
    const double sector_step = 2.0 * M_PI / static_cast<double>(config.n_sectors);
    const float height_offset = static_cast<float>(config.lidar_height_m);

    for (const auto& point : cloud.points) {
        const double x = point.x;
        const double y = point.y;
        const double z = point.z;

        const double range = std::sqrt(x * x + y * y);
        if (range >= config.max_range_m || range <= 1e-6) {
            continue;
        }

        int ring = static_cast<int>(std::floor(range / ring_step));
        if (ring < 0 || ring >= config.n_rings) {
            continue;
        }

        double azimuth = std::atan2(y, x);
        if (azimuth < 0.0) {
            azimuth += 2.0 * M_PI;
        }
        int sector = static_cast<int>(std::floor(azimuth / sector_step));
        if (sector < 0) sector = 0;
        if (sector >= config.n_sectors) sector = config.n_sectors - 1;

        const float shifted_z = static_cast<float>(z) + height_offset;
        // Clip to >= 0 — points slightly below the sensor frame (rare in
        // properly-mounted lidars) shouldn't pull the cell negative.
        const float cell_value = shifted_z > 0.0f ? shifted_z : 0.0f;
        float& cell = descriptor(ring, sector);
        if (cell_value > cell) {
            cell = cell_value;
        }
    }
    return descriptor;
}

RingKey make_ring_key(const Descriptor& descriptor) {
    RingKey key = RingKey::Zero(descriptor.rows());
    if (descriptor.cols() == 0) return key;
    for (int i = 0; i < descriptor.rows(); i++) {
        key(i) = descriptor.row(i).mean();
    }
    return key;
}

SectorKey make_sector_key(const Descriptor& descriptor) {
    SectorKey key = SectorKey::Zero(descriptor.cols());
    if (descriptor.rows() == 0) return key;
    for (int j = 0; j < descriptor.cols(); j++) {
        key(j) = descriptor.col(j).mean();
    }
    return key;
}

float column_cosine_distance(const Descriptor& query,
                             const Descriptor& candidate,
                             int shift) {
    if (query.rows() != candidate.rows() || query.cols() != candidate.cols()) {
        return 2.0f;
    }
    const int cols = static_cast<int>(query.cols());
    if (cols == 0) return 2.0f;

    float total = 0.0f;
    int valid_cols = 0;
    for (int j = 0; j < cols; j++) {
        const int shifted_j = ((j + shift) % cols + cols) % cols;
        const auto query_column = query.col(j);
        const auto candidate_column = candidate.col(shifted_j);
        const float query_norm = query_column.norm();
        const float candidate_norm = candidate_column.norm();
        if (query_norm <= 1e-6f || candidate_norm <= 1e-6f) {
            continue;
        }
        const float cos_sim = query_column.dot(candidate_column) /
                              (query_norm * candidate_norm);
        total += (1.0f - cos_sim);
        valid_cols++;
    }
    if (valid_cols == 0) return 2.0f;
    return total / static_cast<float>(valid_cols);
}

std::pair<float, int> best_distance(const Descriptor& query,
                                    const Descriptor& candidate) {
    const int cols = static_cast<int>(query.cols());
    float min_distance = 2.0f;
    int best_shift = 0;
    for (int shift = 0; shift < cols; shift++) {
        const float distance = column_cosine_distance(query, candidate, shift);
        if (distance < min_distance) {
            min_distance = distance;
            best_shift = shift;
        }
    }
    return {min_distance, best_shift};
}

}  // namespace scan_context
