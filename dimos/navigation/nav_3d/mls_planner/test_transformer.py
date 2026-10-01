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

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray
import pytest

pytest.importorskip("dimos_mls_planner")

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.nav_msgs.msg import Path
from dimos_generated.sensor_msgs.msg import PointCloud2
from dimos_generated.std_msgs.msg import Header

from dimos.memory.type.observation import Observation
from dimos.msgs.pointcloud import pointcloud_from_xyz
from dimos.navigation.nav_3d.mls_planner.transformer import MLSPlan


def _obs(
    points: NDArray[np.float32],
    pose: tuple[float, float, float],
    region_bounds: tuple[float, float, float, float, float],
) -> Observation[PointCloud2]:
    return Observation(
        id=0,
        ts=0.0,
        pose=pose,
        tags={"region_bounds": region_bounds},
        _data=PointCloud2.decode(
            pointcloud_from_xyz(
                points,
                header=Header(frame_id="world", stamp=Time(sec=1700000000, nanosec=123456789)),
            ).encode()
        ),
    )


def _flat_floor(half_extent: float = 3.0, spacing: float = 0.1) -> NDArray[np.float32]:
    coords = np.arange(-half_extent, half_extent, spacing, dtype=np.float32)
    xs, ys = np.meshgrid(coords, coords)
    zs = np.zeros_like(xs)
    return np.stack([xs.ravel(), ys.ravel(), zs.ravel()], axis=1)


def test_flat_floor_yields_populated_path_and_planned_true() -> None:
    obs = _obs(
        _flat_floor(),
        pose=(-2.0, -2.0, 1.0),
        region_bounds=(0.0, 0.0, 5.0, -1.0, 2.0),
    )

    [out] = list(MLSPlan(goal=(2.0, 2.0, 0.0), voxel_size=0.2, robot_height=1.0)(iter([obs])))

    decoded = Path.decode(out.data.encode())
    assert decoded.header.frame_id == "world"
    assert decoded.header.stamp.sec == 1700000000
    assert decoded.header.stamp.nanosec == 123456789
    assert all(p.header.stamp.nanosec == 123456789 for p in decoded.poses)
    assert out.tags["planned"] is True
    assert len(out.data.poses) >= 2
    assert out.tags["voxels"] > 0
    assert out.tags["voxel_map"].shape == (out.tags["voxels"], 3)
    assert out.tags["surface_clearance"].shape[1] == 4
    assert set(out.tags["timings"]) == {"update_ms", "plan_ms", "total_ms"}


def test_poseless_obs_is_skipped() -> None:
    points = _flat_floor()
    poseless = Observation(
        id=1,
        ts=0.0,
        pose=None,
        tags={"region_bounds": (0.0, 0.0, 5.0, -1.0, 2.0)},
        _data=PointCloud2.decode(
            pointcloud_from_xyz(
                points,
                header=Header(frame_id="world", stamp=Time(sec=1700000000, nanosec=123456789)),
            ).encode()
        ),
    )
    posed = _obs(points, pose=(-2.0, -2.0, 1.0), region_bounds=(0.0, 0.0, 5.0, -1.0, 2.0))

    results = list(
        MLSPlan(goal=(2.0, 2.0, 0.0), voxel_size=0.2, robot_height=1.0)(iter([poseless, posed]))
    )

    assert len(results) == 1


def test_start_z_is_dropped_by_robot_height() -> None:
    obs = _obs(
        np.zeros((1, 3), dtype=np.float32),
        pose=(1.0, 2.0, 3.0),
        region_bounds=(1.0, 2.0, 5.0, -1.0, 5.0),
    )

    [out] = list(MLSPlan(goal=(10.0, 10.0, 0.0), robot_height=0.4)(iter([obs])))

    assert out.tags["start"] == (1.0, 2.0, 3.0 - 0.4)


def test_no_route_yields_empty_path_with_planned_false() -> None:
    # Random points form no traversable surface, so planning fails.
    rng = np.random.default_rng(0)
    obs = _obs(
        rng.random((50, 3)).astype(np.float32),
        pose=(0.0, 0.0, 0.0),
        region_bounds=(0.5, 0.5, 2.0, -1.0, 2.0),
    )

    [out] = list(MLSPlan(goal=(100.0, 100.0, 100.0))(iter([obs])))

    assert out.tags["planned"] is False
    assert len(out.data.poses) == 0
