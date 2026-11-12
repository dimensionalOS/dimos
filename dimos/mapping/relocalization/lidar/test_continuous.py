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

"""Continuous relocalization on the M20 fixture (continuous_fixture.py): a first fix
that is off gets corrected by the later ones, which only move world -> map."""

from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip("open3d")

from dimos.mapping.relocalization.lidar.continuous_fixture import MAP_IN_WORLD, NAME
from dimos.mapping.relocalization.lidar.module import DiffRelocalization
from dimos.mapping.relocalization.lidar.relocalize import LidarRelocalizer
from dimos.mapping.relocalization.module import fix_error
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.data import get_data

# Diffs folded in between matches: the fixture's diffs are 1 s apart, the module matches every reloc_interval.
DIFFS_PER_MATCH = 2

# How far off the deliberately wrong first fix is, and how close the corrected ones must come.
WRONG_BY = Transform(
    translation=Vector3(0.3, 0.2, 0.0),
    rotation=Quaternion.from_euler(Vector3(0.0, 0.0, np.radians(4.0))),
    frame_id="map",
    child_frame_id="map",
)
CLOSE_M, CLOSE_DEG = 0.1, 1.0


def _diffs() -> list[PointCloud2]:
    with SqliteStore(path=str(get_data(NAME) / "map_update.db"), must_exist=True) as store:
        return [obs.data for obs in store.stream("map_update", PointCloud2).order_by("ts")]


def _run(first_fix: Transform | None) -> list[Transform]:
    """Every fix the module accepts over the fixture, starting from ``first_fix`` when given."""
    map_file = str(get_data(NAME) / "premap.pc2.lcm")
    m = DiffRelocalization(map_file=map_file)
    # Driven by hand, without a bus.
    m.loaded_map = SimpleNamespace(publish=lambda _: None)  # type: ignore[assignment]
    m.register_disposable = lambda disposable: disposable  # type: ignore[method-assign]
    m._load_premap(map_file)
    assert m.premap is not None
    m._relocalizer = LidarRelocalizer(m.premap.pointcloud, m.config.relocalize)
    fixes: list[Transform] = []
    m.fixes.subscribe(fixes.append)
    if first_fix is not None:
        m.submit(first_fix, "test")
    try:
        for index, diff in enumerate(_diffs(), 1):
            m.on_diff(diff)
            if index % DIFFS_PER_MATCH == 0 and m._has_enough_points(cloud := m.live_cloud()):
                m._relocalize(cloud)
    finally:
        m.dispose()
    return fixes


def _off(fix: Transform, reference: Transform) -> tuple[float, float]:
    dyaw, dist_m = fix_error(fix, reference)
    return abs(dyaw), dist_m


@pytest.mark.self_hosted
def test_a_slightly_wrong_first_fix_is_corrected_and_stays_corrected() -> None:
    reference = _run(None)[-1]
    # The fixture's live frame sits MAP_IN_WORLD from the premap, so that is what an unperturbed run finds.
    dyaw, dist_m = _off(reference, MAP_IN_WORLD)
    assert dist_m < 0.2 and dyaw < 2.0, (
        f"unperturbed run off the truth by {dist_m:.2f} m {dyaw:.1f} deg"
    )

    wrong = reference + WRONG_BY
    dyaw, dist_m = _off(wrong, reference)
    assert dist_m >= 0.3 and dyaw >= 3.0

    fixes = _run(wrong)
    errors = [_off(fix, reference) for fix in fixes[1:]]
    close = [dyaw < CLOSE_DEG and dist_m < CLOSE_M for dyaw, dist_m in errors]
    assert any(close), f"never corrected: {errors}"
    first_close = close.index(True)
    assert all(close[first_close:]), f"got worse once corrected: {errors[first_close:]}"
