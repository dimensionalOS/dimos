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

"""Validate that loop closure produces a ``map -> odom`` correction that
absorbs accumulating odom drift.

This is the user-facing reason for running RTAB-Map alongside FastLIO2:
FastLIO2's odometry drifts open-loop; RTAB-Map's loop closures rebind the
global map back to the world, and the ``rtab_tf`` correction (map -> odom)
is what carries that rebind to downstream consumers.
"""

from __future__ import annotations

import numpy as np

from dimos.navigation.nav_stack.modules.rtab_map._rtab_runner import (
    LiteRtabRunner,
    RunnerStepResult,
)
from dimos.navigation.nav_stack.modules.rtab_map.tests.conftest import (
    translated_pose,
)


def _scene_scan() -> np.ndarray:
    """A small but distinctive scan that re-matches when the robot revisits."""
    grid = np.linspace(-1.5, 1.5, 12)
    wall_x_pos = np.stack([np.full_like(grid, 1.5), grid, np.zeros_like(grid)], axis=1)
    wall_y_pos = np.stack([grid, np.full_like(grid, 1.5), np.zeros_like(grid)], axis=1)
    floor = np.stack([grid, np.zeros_like(grid), np.full_like(grid, -0.5)], axis=1)
    return np.concatenate([wall_x_pos, wall_y_pos, floor])


def _trajectory_with_drift(drift_per_step: float, n_loops: int = 2) -> list[np.ndarray]:
    """Closed-loop trajectory with accumulating translational drift on odom."""
    waypoints = [(0.0, 0.0), (0.6, 0.0), (1.2, 0.6), (0.6, 1.2), (0.0, 1.2)]
    poses: list[np.ndarray] = []
    drift = 0.0
    for _ in range(n_loops):
        for x, y in waypoints:
            poses.append(translated_pose(x + drift, y))
            drift += drift_per_step
    # Final return to origin so the closure fires near keyframe #0.
    poses.append(translated_pose(0.0 + drift, 0.0))
    return poses


def test_correction_is_identity_before_first_closure(lite_runner: LiteRtabRunner) -> None:
    """Steps before any loop closure must publish the identity correction —
    the runner has no information to estimate drift yet."""
    scan = _scene_scan()
    poses = _trajectory_with_drift(0.02, n_loops=1)[:3]
    for ts, pose in enumerate(poses):
        result = lite_runner.process(scan, pose, float(ts))
        if not result.loop_closure_event:
            assert np.allclose(result.tf_correction, np.eye(4), atol=1e-9)


def test_correction_diverges_from_identity_after_closure(
    lite_runner: LiteRtabRunner,
) -> None:
    """After a loop closure fires under drift, the correction must shift —
    that's how the drift gets absorbed."""
    scan = _scene_scan()
    poses = _trajectory_with_drift(0.04, n_loops=2)

    closure_step: int | None = None
    closure_result: RunnerStepResult | None = None
    for ts, pose in enumerate(poses):
        result = lite_runner.process(scan, pose, float(ts))
        if result.loop_closure_event:
            closure_step = ts
            closure_result = result
            break

    assert closure_step is not None, "no loop closure produced"
    assert closure_result is not None
    correction_norm = float(np.linalg.norm(closure_result.tf_correction - np.eye(4)))
    assert correction_norm > 1e-3, (
        f"expected non-trivial correction at closure step {closure_step}, "
        f"got norm={correction_norm}"
    )


def test_corrected_pose_pulls_toward_true_position(lite_runner: LiteRtabRunner) -> None:
    """The corrected pose (correction @ odom) should be closer to the
    revisited keyframe's prior global pose than the raw odom is. Concretely:
    on revisit, the corrected x should be smaller than the raw odom x because
    the correction is undoing the linearly-accumulated drift."""
    scan = _scene_scan()
    poses = _trajectory_with_drift(0.05, n_loops=2)

    last_result: RunnerStepResult | None = None
    last_pose: np.ndarray | None = None
    for ts, pose in enumerate(poses):
        last_result = lite_runner.process(scan, pose, float(ts))
        last_pose = pose
        if last_result.loop_closure_event:
            break

    assert last_result is not None and last_pose is not None
    assert last_result.loop_closure_event

    raw_x = float(last_pose[0, 3])
    corrected_x = float(last_result.corrected_pose[0, 3])
    assert corrected_x < raw_x, (
        f"corrected_x={corrected_x} should be < raw odom x={raw_x} "
        "since the correction is undoing the accumulated drift"
    )
