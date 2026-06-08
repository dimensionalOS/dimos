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


def test_negative_emit_every_is_rejected() -> None:
    with pytest.raises(ValueError):
        RayTraceMap(emit_every=-1)
