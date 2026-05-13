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

"""Synthetic loop-closure validation.

Inspired by the way the upstream rtabmap_ros repo exercises loop closure on
its dataset bags: drive a closed-loop trajectory, observe the same scene from
two viewpoints, expect at least one closure event and a non-identity correction
once the closure fires.
"""

from __future__ import annotations

import numpy as np

from dimos.navigation.nav_stack.modules.rtab_map._rtab_runner import (
    LiteRtabRunner,
)
from dimos.navigation.nav_stack.modules.rtab_map.tests.conftest import (
    translated_pose,
)


def _square_room_scan() -> np.ndarray:
    """Synthetic scan of four walls + floor, in the body frame.

    The room is centered on the body; walls at x=±1.5, y=±1.5; floor at z=0.
    Stable across keyframe revisits so the lite runner's scan matcher fires.
    """
    grid = np.linspace(-1.5, 1.5, 16)
    wall_x_pos = np.stack([np.full_like(grid, 1.5), grid, np.zeros_like(grid)], axis=1)
    wall_x_neg = np.stack([np.full_like(grid, -1.5), grid, np.zeros_like(grid)], axis=1)
    wall_y_pos = np.stack([grid, np.full_like(grid, 1.5), np.zeros_like(grid)], axis=1)
    wall_y_neg = np.stack([grid, np.full_like(grid, -1.5), np.zeros_like(grid)], axis=1)
    walls = np.concatenate([wall_x_pos, wall_x_neg, wall_y_pos, wall_y_neg])

    xx, yy = np.meshgrid(np.linspace(-1.0, 1.0, 5), np.linspace(-1.0, 1.0, 5))
    floor = np.stack([xx.ravel(), yy.ravel(), np.full(xx.size, -0.5)], axis=1)
    return np.concatenate([walls, floor], axis=0)


def test_closed_trajectory_produces_at_least_one_loop_closure(
    lite_runner: LiteRtabRunner,
) -> None:
    """Drive a closed loop with repeated scans; expect a loop closure."""
    scan_body = _square_room_scan()
    waypoints = [
        translated_pose(0.0, 0.0),
        translated_pose(0.6, 0.0),
        translated_pose(1.2, 0.6),
        translated_pose(1.2, 1.2),
        translated_pose(0.6, 1.2),
        translated_pose(0.0, 1.2),
        # Now drive back through the corridor toward the start — the
        # revisit detector will see we're back near keyframe #0.
        translated_pose(0.0, 0.6),
        translated_pose(0.0, 0.0),
    ]
    closures: list[bool] = []
    for ts, pose in enumerate(waypoints):
        result = lite_runner.process(scan_body, pose, float(ts))
        closures.append(result.loop_closure_event)

    assert any(closures), "expected at least one loop closure on a returning trajectory"
    assert lite_runner.loop_closure_pairs, "loop_closure_pairs should be populated"


def test_loop_closure_changes_tf_correction(lite_runner: LiteRtabRunner) -> None:
    """Before the first closure, the map->odom correction is identity. After
    the closure fires, it must be non-identity (it's absorbing accumulated
    drift)."""
    scan_body = _square_room_scan()
    drift_per_step = 0.03  # m of odom drift per waypoint; must remain inside
    # the revisit radius (default 2.0 m) so the closure still fires.

    ground_truth = [(0.0, 0.0), (0.6, 0.0), (1.2, 0.6), (0.6, 1.2), (0.0, 1.2), (0.0, 0.0)]
    waypoints_with_drift = [
        translated_pose(gx + step * drift_per_step, gy)
        for step, (gx, gy) in enumerate(ground_truth)
    ]

    pre_closure_correction = np.eye(4)
    closure_seen = False
    for ts, pose in enumerate(waypoints_with_drift):
        result = lite_runner.process(scan_body, pose, float(ts))
        if result.loop_closure_event:
            closure_seen = True
            assert not np.allclose(result.tf_correction, np.eye(4), atol=1e-3), (
                "tf_correction should diverge from identity at closure"
            )
            break
        pre_closure_correction = result.tf_correction

    assert closure_seen, "no loop closure observed in drift sequence"
    assert np.allclose(pre_closure_correction, np.eye(4), atol=1e-6)


def test_no_false_closure_on_linear_trajectory(lite_runner: LiteRtabRunner) -> None:
    """A monotonic forward trajectory should NOT trigger loop closure —
    guards against the keyframe revisit detector firing on adjacent frames."""
    scan_body = _square_room_scan()
    waypoints = [translated_pose(0.6 * i, 0.0) for i in range(8)]

    closures = []
    for ts, pose in enumerate(waypoints):
        result = lite_runner.process(scan_body, pose, float(ts))
        closures.append(result.loop_closure_event)

    assert not any(closures), f"unexpected closures: {closures}"
