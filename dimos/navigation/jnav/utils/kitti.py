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

"""Official KITTI odometry error metric (translational % + rotational deg/m).

The KITTI leaderboard metric: for every sub-sequence length in {100,200,...,800}m,
take every start frame, find the end frame ~that path-length away, and measure the
relative pose error between estimated and ground-truth. Average translational
error (as a fraction of length) and rotational error (rad per metre) over all
(start, length) pairs. Reported as translational % and rotational deg/m. This is
the devkit's `evaluate_odometry` algorithm.

Poses are 4x4 body->world transforms (frame 0 = identity), one per frame.
"""

from __future__ import annotations

import numpy as np

LENGTHS = (100.0, 200.0, 300.0, 400.0, 500.0, 600.0, 700.0, 800.0)
STEP = 10  # evaluate every 10th frame as a start (devkit convention)


def trajectory_distances(poses: list[np.ndarray]) -> list[float]:
    """Cumulative path length at each frame."""
    distances = [0.0]
    for index in range(1, len(poses)):
        delta = poses[index][:3, 3] - poses[index - 1][:3, 3]
        distances.append(distances[-1] + float(np.linalg.norm(delta)))
    return distances


def _last_frame_for_length(distances: list[float], first: int, length: float) -> int:
    target = distances[first] + length
    for index in range(first, len(distances)):
        if distances[index] >= target:
            return index
    return -1


def _rotation_error(pose_error: np.ndarray) -> float:
    trace = pose_error[0, 0] + pose_error[1, 1] + pose_error[2, 2]
    return float(np.arccos(np.clip((trace - 1.0) / 2.0, -1.0, 1.0)))


def _translation_error(pose_error: np.ndarray) -> float:
    return float(np.linalg.norm(pose_error[:3, 3]))


def kitti_odometry_error(
    estimated: list[np.ndarray], ground_truth: list[np.ndarray]
) -> dict[str, float]:
    """Average translational (%) and rotational (deg/m) error, devkit-style."""
    count = min(len(estimated), len(ground_truth))
    estimated, ground_truth = estimated[:count], ground_truth[:count]
    distances = trajectory_distances(ground_truth)

    translational_errors: list[float] = []
    rotational_errors: list[float] = []
    for first in range(0, count, STEP):
        for length in LENGTHS:
            last = _last_frame_for_length(distances, first, length)
            if last < 0:
                continue
            # Relative pose error: how far the estimated motion from first->last
            # diverges from ground truth's.
            gt_delta = np.linalg.inv(ground_truth[first]) @ ground_truth[last]
            estimated_delta = np.linalg.inv(estimated[first]) @ estimated[last]
            pose_error = np.linalg.inv(gt_delta) @ estimated_delta
            translational_errors.append(_translation_error(pose_error) / length)
            rotational_errors.append(_rotation_error(pose_error) / length)

    if not translational_errors:
        return {
            "translational_percent": float("nan"),
            "rotational_deg_per_m": float("nan"),
            "pairs": 0,
        }
    return {
        "translational_percent": float(np.mean(translational_errors)) * 100.0,
        "rotational_deg_per_m": float(np.degrees(np.mean(rotational_errors))),
        "pairs": len(translational_errors),
    }
