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

pytest.importorskip("dimos_voxel_ray_tracing")

from dimos.mapping.ray_tracing.transformer import RayTraceMap
from dimos.memory2.type.observation import Observation
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def _obs(
    points: NDArray[np.float32], ts: float, pose: tuple[float, float, float]
) -> Observation[PointCloud2]:
    return Observation(
        id=0,
        ts=ts,
        pose=pose,
        _data=PointCloud2.from_numpy(points),
    )


def _cube(n: int = 100) -> NDArray[np.float32]:
    rng = np.random.default_rng(0)
    return rng.random((n, 3)).astype(np.float32)


def test_emit_every_n_yields_on_cadence_and_flushes_remainder() -> None:
    points = _cube()
    obs = [_obs(points, ts=float(i), pose=(0.0, 0.0, 0.0)) for i in range(7)]

    results = list(RayTraceMap(emit_every=3)(iter(obs)))

    assert [r.tags["frame_count"] for r in results] == [3, 6, 7]


def test_poseless_obs_are_skipped() -> None:
    points = _cube()
    poseless = Observation(id=1, ts=0.0, pose=None, _data=PointCloud2.from_numpy(points))
    posed = _obs(points, ts=1.0, pose=(0.0, 0.0, 0.0))

    results = list(RayTraceMap()(iter([poseless, posed])))

    assert [r.tags["frame_count"] for r in results] == [1]


def _ring(
    center: tuple[float, float], radius: float, z: float, n: int = 100
) -> NDArray[np.float32]:
    angles = np.linspace(0.0, 2.0 * np.pi, n, endpoint=False)
    xs = center[0] + radius * np.cos(angles)
    ys = center[1] + radius * np.sin(angles)
    zs = np.full_like(xs, z)
    return np.stack([xs, ys, zs], axis=1).astype(np.float32)


def test_tags_region_bounds_around_registered_origin() -> None:
    margin = 0.2 + 0.1
    # Sensor-frame ring centered on the sensor. The pose registers it to (2, 3, 0.5).
    obs = _obs(_ring((0.0, 0.0), radius=1.0, z=0.0), ts=1.0, pose=(2.0, 3.0, 0.5))

    [emitted] = list(RayTraceMap()(iter([obs])))

    cx, cy, radius, z_min, z_max = emitted.tags["region_bounds"]
    assert (cx, cy) == pytest.approx((2.0, 3.0))
    assert radius == pytest.approx(1.0 + margin)
    assert z_min == pytest.approx(0.5 - margin)
    assert z_max == pytest.approx(0.5 + margin)


def test_empty_frame_yields_zero_radius_region_at_robot() -> None:
    empty = np.empty((0, 3), dtype=np.float32)
    obs = _obs(empty, ts=1.0, pose=(1.0, 2.0, 3.0))

    [emitted] = list(RayTraceMap()(iter([obs])))

    assert emitted.tags["region_bounds"] == pytest.approx((1.0, 2.0, 0.0, 3.0, 3.0))


def test_registers_sensor_frame_cloud_by_pose() -> None:
    margin = 0.2 + 0.1
    s = 2.0**-0.5
    # 90-degree pitch maps sensor +x to world -z, then translate by (5, 0, 2),
    # landing the point at world (5, 0, 1).
    point = np.array([[1.0, 0.0, 0.0]], dtype=np.float32)
    obs = Observation(
        id=0,
        ts=1.0,
        pose=(5.0, 0.0, 2.0, 0.0, s, 0.0, s),
        _data=PointCloud2.from_numpy(point),
    )

    [emitted] = list(RayTraceMap()(iter([obs])))

    cx, cy, radius, z_min, z_max = emitted.tags["region_bounds"]
    assert (cx, cy) == pytest.approx((5.0, 0.0))
    assert radius == pytest.approx(0.0 + margin)
    assert z_min == pytest.approx(1.0 - margin)
    assert z_max == pytest.approx(1.0 + margin)
