// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Pose readiness check shared by the SLAM-backed lidar drivers (fastlio2,
// pointlio).

#pragma once

#include <cmath>
#include <vector>

namespace dimos {

// A pose that has not been written yet holds uninitialized bytes, whose
// quaternion part is almost never unit length.
inline constexpr double kQuatNormTolerance = 1e-3;

// True once the estimator has produced a real pose. The SLAM cores hand out
// their pose through an odometry result that starts as uninitialized memory, so
// a nonzero position does not mean the estimator has run. A real estimate
// always carries a normalized quaternion, which uninitialized bytes do not.
inline bool has_estimate(const std::vector<double>& pose) {
    if (pose.size() != 7) {
        return false;
    }
    const double qn = pose[3] * pose[3] + pose[4] * pose[4] + pose[5] * pose[5] +
                      pose[6] * pose[6];
    return std::abs(qn - 1.0) < kQuatNormTolerance;
}

}  // namespace dimos
