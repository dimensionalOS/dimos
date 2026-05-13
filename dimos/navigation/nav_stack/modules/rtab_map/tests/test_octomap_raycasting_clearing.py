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

"""Validate that raycasting clears previously-occupied voxels.

This is the behavior the user explicitly asked to default-on: when a later
scan looks through a previously-occupied voxel (sensor sees a target *beyond*
it), the intermediate voxel must drop out of the published OctoMap. Mirrors
the rtabmap `Grid/RayTracing` semantics on top of the OctoMap log-odds model.
"""

from __future__ import annotations

import numpy as np

from dimos.navigation.nav_stack.modules.rtab_map._rtab_runner import (
    LiteRtabRunner,
    LiteRtabRunnerConfig,
    _voxel_traversal,
)
from dimos.navigation.nav_stack.modules.rtab_map.tests.conftest import (
    translated_pose,
)


def _voxel_centroid_set(runner: LiteRtabRunner, tol: float = 1e-6) -> set[tuple[int, int, int]]:
    """Snapshot the runner's occupied-voxel set as a tuple-key set."""
    centroids = runner.occupied_voxel_centroids()
    if len(centroids) == 0:
        return set()
    cell_size = runner._config.cell_size  # type: ignore[attr-defined]
    cells = np.floor(centroids / cell_size + tol).astype(int)
    return {tuple(row) for row in cells}


def test_voxel_traversal_emits_intermediate_cells() -> None:
    """Sanity check: the DDA traversal yields a contiguous voxel chain."""
    path = _voxel_traversal((0, 0, 0), (5, 0, 0))
    assert path == [(0, 0, 0), (1, 0, 0), (2, 0, 0), (3, 0, 0), (4, 0, 0), (5, 0, 0)]
    diagonal = _voxel_traversal((0, 0, 0), (3, 3, 3))
    assert diagonal[0] == (0, 0, 0)
    assert diagonal[-1] == (3, 3, 3)


def test_obstacle_at_long_range_clears_short_range_phantom() -> None:
    """First scan plants an obstacle at +1.0 m. Second scan, taken from the
    same sensor pose, sees through that voxel out to +3.0 m. After the
    second scan the +1.0 m voxel must no longer be occupied.
    """
    runner = LiteRtabRunner(
        LiteRtabRunnerConfig(
            cell_size=0.1,
            ray_tracing=True,
            # Force everything-as-obstacle so a single isolated point counts
            # as an obstacle voxel (otherwise the PCA-fallback ground rule
            # would classify a 1-point cloud as ground).
            ground_is_obstacle=True,
            log_odds_miss=-1.0,
            log_odds_hit=0.85,
            occupied_threshold=0.4,
        )
    )

    sensor_pose = translated_pose(0.0, 0.0, 0.0)

    # Phantom and real obstacles are colinear with the sensor at the origin
    # along the +x axis so the raycast from sensor -> real must pass through
    # the phantom voxel.
    phantom = np.array([[1.0, 0.0, 0.0]])
    result_1 = runner.process(phantom, sensor_pose, timestamp=0.0)
    occupied_1 = _voxel_centroid_set(runner)
    phantom_cell = (10, 0, 0)  # cell_size=0.1
    assert phantom_cell in occupied_1, f"phantom cell missing: {result_1.octomap_voxels}"

    # Second scan: real obstacle 3.0 m ahead, same line. The raycast must
    # decrement the phantom cell's log-odds below the occupied threshold.
    real = np.array([[3.0, 0.0, 0.0]])
    for _ in range(3):
        runner.process(real, sensor_pose, timestamp=1.0)

    occupied_2 = _voxel_centroid_set(runner)
    assert phantom_cell not in occupied_2, "raycasting failed to clear phantom voxel"
    real_cell = (30, 0, 0)
    assert real_cell in occupied_2, "real obstacle voxel missing after clearing pass"


def test_disabling_ray_tracing_preserves_phantom() -> None:
    """Control: with raycasting OFF, the phantom voxel should persist —
    the cleanup is *because of* the ray tracing, not anything else."""
    runner = LiteRtabRunner(
        LiteRtabRunnerConfig(
            cell_size=0.1,
            ray_tracing=False,
            ground_is_obstacle=True,
            log_odds_miss=-1.0,
            log_odds_hit=0.85,
            occupied_threshold=0.4,
        )
    )
    sensor_pose = translated_pose(0.0, 0.0, 0.0)
    runner.process(np.array([[1.0, 0.0, 0.0]]), sensor_pose, timestamp=0.0)
    for _ in range(3):
        runner.process(np.array([[3.0, 0.0, 0.0]]), sensor_pose, timestamp=1.0)
    occupied = _voxel_centroid_set(runner)
    assert (10, 0, 0) in occupied, "phantom should persist when ray tracing is disabled"
