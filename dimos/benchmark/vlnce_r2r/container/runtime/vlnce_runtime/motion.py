# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Pure fixed-period planar motion math shared with the Habitat adapter."""

import math

import numpy as np


class PlanarMotionError(RuntimeError):
    """A public motion input or accepted Habitat pose was invalid."""


def integrate_planar(position, rotation_xyzw, linear_x, linear_y, angular_z, period_seconds):
    """Return requested world position and yaw-updated xyzw orientation."""

    values = (
        tuple(position)
        + tuple(rotation_xyzw)
        + (
            linear_x,
            linear_y,
            angular_z,
            period_seconds,
        )
    )
    if not all(math.isfinite(float(value)) for value in values) or period_seconds <= 0:
        raise PlanarMotionError("planar motion values must be finite with a positive period")
    rotation = np.asarray(rotation_xyzw, dtype=np.float64)
    norm = np.linalg.norm(rotation)
    if norm <= 0.0:
        raise PlanarMotionError("planar motion orientation must be non-zero")
    rotation = rotation / norm
    local_delta = np.array(
        [-linear_y * period_seconds, 0.0, -linear_x * period_seconds],
        dtype=np.float64,
    )
    requested = np.asarray(position, dtype=np.float64) + _rotate(rotation, local_delta)
    half_yaw = angular_z * period_seconds / 2.0
    yaw = np.array([0.0, math.sin(half_yaw), 0.0, math.cos(half_yaw)])
    return requested, _multiply(yaw, rotation)


def record_accepted_motion(trajectory, requested, accepted):
    """Append exactly one finite accepted pose and report pathfinder clipping."""

    accepted_array = np.asarray(accepted, dtype=np.float64)
    if accepted_array.shape != (3,) or not np.all(np.isfinite(accepted_array)):
        raise PlanarMotionError("Habitat returned an invalid accepted position")
    trajectory.append(accepted_array.tolist())
    return not np.allclose(requested, accepted_array, rtol=0.0, atol=1e-5)


def _rotate(rotation_xyzw, vector):
    xyz = rotation_xyzw[:3]
    scalar = rotation_xyzw[3]
    return (
        2.0 * np.dot(xyz, vector) * xyz
        + (scalar * scalar - np.dot(xyz, xyz)) * vector
        + 2.0 * scalar * np.cross(xyz, vector)
    )


def _multiply(left_xyzw, right_xyzw):
    left_xyz, left_w = left_xyzw[:3], left_xyzw[3]
    right_xyz, right_w = right_xyzw[:3], right_xyzw[3]
    result = np.concatenate(
        (
            left_w * right_xyz + right_w * left_xyz + np.cross(left_xyz, right_xyz),
            [left_w * right_w - np.dot(left_xyz, right_xyz)],
        )
    )
    return result / np.linalg.norm(result)
