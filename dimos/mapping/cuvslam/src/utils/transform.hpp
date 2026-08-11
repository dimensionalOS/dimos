// Copyright 2026 Dimensional Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Quaternion and rigid-transform algebra shared by the cuVSLAM module sources.

#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace transform_math {

/// Rigid transform, rotation as xyzw to match cuvslam::Pose.
struct Transform {
    std::array<double, 4> rotation{0.0, 0.0, 0.0, 1.0};
    std::array<double, 3> translation{0.0, 0.0, 0.0};
};

inline std::array<double, 4> quat_multiply(const std::array<double, 4>& left,
                                           const std::array<double, 4>& right) {
    return {left[3] * right[0] + left[0] * right[3] + left[1] * right[2] - left[2] * right[1],
            left[3] * right[1] - left[0] * right[2] + left[1] * right[3] + left[2] * right[0],
            left[3] * right[2] + left[0] * right[1] - left[1] * right[0] + left[2] * right[3],
            left[3] * right[3] - left[0] * right[0] - left[1] * right[1] - left[2] * right[2]};
}

inline std::array<double, 3> cross(const std::array<double, 3>& left,
                                   const std::array<double, 3>& right) {
    return {left[1] * right[2] - left[2] * right[1], left[2] * right[0] - left[0] * right[2],
            left[0] * right[1] - left[1] * right[0]};
}

inline std::array<double, 3> quat_rotate(const std::array<double, 4>& rotation,
                                         const std::array<double, 3>& vector) {
    const std::array<double, 3> axis{rotation[0], rotation[1], rotation[2]};
    std::array<double, 3> inner = cross(axis, vector);
    for (int component = 0; component < 3; ++component) {
        inner[component] += rotation[3] * vector[component];
    }
    const std::array<double, 3> outer = cross(axis, inner);
    return {vector[0] + 2.0 * outer[0], vector[1] + 2.0 * outer[1], vector[2] + 2.0 * outer[2]};
}

inline Transform compose(const Transform& outer, const Transform& inner) {
    const std::array<double, 3> rotated = quat_rotate(outer.rotation, inner.translation);
    return Transform{quat_multiply(outer.rotation, inner.rotation),
                     {outer.translation[0] + rotated[0], outer.translation[1] + rotated[1],
                      outer.translation[2] + rotated[2]}};
}

inline Transform invert(const Transform& transform) {
    const std::array<double, 4> conjugate{-transform.rotation[0], -transform.rotation[1],
                                          -transform.rotation[2], transform.rotation[3]};
    const std::array<double, 3> rotated = quat_rotate(conjugate, transform.translation);
    return Transform{conjugate, {-rotated[0], -rotated[1], -rotated[2]}};
}

inline double translation_between(const Transform& from, const Transform& to) {
    const double dx = to.translation[0] - from.translation[0];
    const double dy = to.translation[1] - from.translation[1];
    const double dz = to.translation[2] - from.translation[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/// Angle of the relative rotation between two orientations, radians.
inline double angle_between(const std::array<double, 4>& from,
                            const std::array<double, 4>& to) {
    const double dot =
        std::abs(from[0] * to[0] + from[1] * to[1] + from[2] * to[2] + from[3] * to[3]);
    return 2.0 * std::acos(std::min(dot, 1.0));
}

}  // namespace transform_math
