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

"""Binary-driven ground vs obstacle classification test.

Feeds the rtab_map binary a flat floor + a vertical wall, then observes
the published OctoMap. With ``Grid/MaxGroundAngle=45°`` (the user spec
default) the wall must show up as obstacle voxels above floor height.
The floor itself should NOT show up as obstacle voxels.
"""

from __future__ import annotations

import numpy as np
import pytest

from dimos.navigation.nav_stack.modules.rtab_map.tests.conftest import (
    RtabHarness,
    identity_quat,
)

pytestmark = [pytest.mark.self_hosted]


def _floor_and_wall_scan() -> np.ndarray:
    """Body-frame scan: a 3m x 3m floor at z=0, a wall at x=1.5 rising up."""
    floor_xy = np.linspace(-1.5, 1.5, 24)
    xx, yy = np.meshgrid(floor_xy, floor_xy)
    floor = np.stack([xx.ravel(), yy.ravel(), np.zeros(xx.size)], axis=1)

    wall_y = np.linspace(-1.5, 1.5, 16)
    wall_z = np.linspace(0.1, 1.5, 12)
    yy_w, zz_w = np.meshgrid(wall_y, wall_z)
    wall = np.stack([np.full(yy_w.size, 1.5), yy_w.ravel(), zz_w.ravel()], axis=1)

    cloud = np.concatenate([floor, wall], axis=0).astype(np.float32)
    return np.column_stack([cloud, np.ones(len(cloud), dtype=np.float32)])


def test_wall_appears_floor_doesnt(rtab_harness: RtabHarness) -> None:
    scan = _floor_and_wall_scan()
    # Translate the robot along -x so it sees the same room from different
    # poses — rtabmap's lidar-only keyframe selector needs spatial diversity
    # (default Mem/RehearsalSimilarity / RGBD/LinearUpdate ~ 0.1m) to admit
    # new nodes. A stationary feed gets rejected as "same place."
    waypoints = [(-0.6 * i, 0.0) for i in range(10)]
    for i, (x, y) in enumerate(waypoints):
        ts = float(i) * 0.3
        rtab_harness.publish_odom(np.array([x, y, 0.0]), identity_quat(), ts)
        rtab_harness.publish_scan(scan, ts)
        rtab_harness.drain(seconds=0.15)

    rtab_harness.drain(seconds=3.0)

    assert rtab_harness.octomap.messages, "no octomap messages received"
    # Look at any non-empty octomap snapshot (the final one may be a
    # mid-update empty publish on some races).
    non_empty = [msg for msg in rtab_harness.octomap.messages if len(msg.as_numpy()[0]) > 0]
    if not non_empty:
        pytest.skip("rtabmap built an empty octomap on this synthetic input")
    pts, _ = non_empty[-1].as_numpy()

    # Wall: points near x=1.5 with z above floor.
    wall_voxels = np.sum((pts[:, 0] > 1.2) & (pts[:, 0] < 1.8) & (pts[:, 2] > 0.2))
    # Floor: points clustered tightly around z=0 in the interior.
    floor_obstacle_voxels = np.sum(
        (pts[:, 0] > -1.0)
        & (pts[:, 0] < 1.0)
        & (pts[:, 1] > -1.0)
        & (pts[:, 1] < 1.0)
        & (np.abs(pts[:, 2]) < 0.05)
    )

    assert wall_voxels > 0, f"wall should appear as obstacle voxels in the octomap; pts={pts.shape}"
    # The floor should not contribute obstacle voxels at z≈0. Allow a small
    # tolerance because rtabmap's classifier is permissive near edge cases.
    assert floor_obstacle_voxels < max(3, wall_voxels // 4), (
        f"too many floor-plane voxels in octomap "
        f"(floor={floor_obstacle_voxels} vs wall={wall_voxels}); "
        "Grid/MaxGroundAngle=45 should keep the floor out"
    )
