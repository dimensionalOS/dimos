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

import threading

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.localize.module import PerceptionLocalize
from dimos.perception.localize.support_plane import find_surfaces


def _slab(length, width, yaw_deg, tilt_deg, downhill_deg, center, rng, step=0.01):
    """A flat rectangle of points, 2 mm of noise, tilted down towards a heading."""
    u, v = np.meshgrid(
        np.arange(-length / 2, length / 2, step), np.arange(-width / 2, width / 2, step)
    )
    yaw = np.radians(yaw_deg)
    x = u * np.cos(yaw) - v * np.sin(yaw)
    y = u * np.sin(yaw) + v * np.cos(yaw)
    downhill = np.array([np.cos(np.radians(downhill_deg)), np.sin(np.radians(downhill_deg))])
    z = -np.tan(np.radians(tilt_deg)) * (x * downhill[0] + y * downhill[1])
    points = np.stack([x.ravel(), y.ravel(), z.ravel()], axis=1) + np.asarray(center)
    return points + rng.normal(0.0, 0.002, points.shape)


def _block(center, size, rng):
    """The top and sides of a box standing on z = center[2]."""
    x, y, z = center
    sx, sy, sz = size
    top = rng.uniform([x - sx / 2, y - sy / 2, z + sz], [x + sx / 2, y + sy / 2, z + sz], (400, 3))
    side = rng.uniform([x - sx / 2, y - sy / 2, z], [x + sx / 2, y + sy / 2, z + sz], (400, 3))
    return np.concatenate([top, side])


@pytest.fixture
def rng():
    return np.random.default_rng(0)


def test_level_table_with_blocks(rng):
    table = _slab(0.8, 0.5, 30.0, 0.0, 0.0, (0.5, 0.1, 0.02), rng)
    blocks = [_block((0.4, 0.0, 0.02), (0.05, 0.05, 0.05), rng) for _ in range(3)]
    (surface,) = find_surfaces(np.concatenate([table, *blocks]))

    assert surface.height == pytest.approx(0.02, abs=0.004)
    assert surface.center == pytest.approx((0.5, 0.1, 0.02), abs=0.01)
    assert surface.size == pytest.approx((0.8, 0.5), abs=0.03)
    assert surface.yaw_deg == pytest.approx(30.0, abs=1.5)
    assert surface.tilt_deg < 0.5
    assert surface.rms_m < 0.004


def test_tilted_table_reports_tilt_and_downhill(rng):
    table = _slab(0.8, 0.5, 0.0, 8.0, 90.0, (0.5, 0.0, 0.1), rng)
    (surface,) = find_surfaces(table)

    assert surface.tilt_deg == pytest.approx(8.0, abs=0.5)
    assert surface.downhill_deg == pytest.approx(90.0, abs=2.0)
    # The plane goes down towards +y, so its height under (0, 0) is that at the centre.
    assert surface.height == pytest.approx(0.1, abs=0.005)


def test_max_tilt_drops_steep_surfaces(rng):
    ramp = _slab(0.8, 0.5, 0.0, 35.0, 0.0, (0.5, 0.0, 0.1), rng)
    assert find_surfaces(ramp, max_tilt_deg=25.0) == []
    (surface,) = find_surfaces(ramp, max_tilt_deg=40.0)
    assert surface.tilt_deg == pytest.approx(35.0, abs=0.5)


def test_table_and_floor_biggest_first(rng):
    table = _slab(0.8, 0.5, 0.0, 0.0, 0.0, (0.5, 0.0, 0.0), rng)
    floor = _slab(0.6, 0.3, 0.0, 0.0, 0.0, (1.2, 0.0, -0.7), rng)
    surfaces = find_surfaces(np.concatenate([table, floor]))

    assert [round(s.height, 2) for s in surfaces] == [0.0, -0.7]


def test_reported_in_another_frame(rng):
    table = _slab(0.8, 0.5, 0.0, 0.0, 0.0, (0.5, 0.0, 0.0), rng)
    # A base 0.3 m below the world origin and turned 90 degrees.
    to_base = np.array(
        [[0.0, 1.0, 0.0, 0.0], [-1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.3], [0.0, 0.0, 0.0, 1.0]]
    )
    (surface,) = find_surfaces(table, to_frame=to_base)

    assert surface.height == pytest.approx(0.3, abs=0.004)
    assert surface.center == pytest.approx((0.0, -0.5, 0.3), abs=0.01)
    assert abs(surface.yaw_deg) == pytest.approx(90.0, abs=1.5)


@pytest.fixture
def module():
    module = PerceptionLocalize()
    yield module
    module.stop()


def test_find_surface_skill_reports_in_the_base_frame(module, rng, mocker):
    module._ready = threading.Event()
    module._ready.set()
    module.index = mocker.Mock()
    module.index.get_time_range.return_value = (0.0, 10.0)
    module.index.time_range.return_value = [mocker.Mock(ts=float(t)) for t in range(5)]
    table = PointCloud2.from_numpy(
        _slab(0.8, 0.5, 0.0, 0.0, 0.0, (0.5, 0.0, 0.0), rng).astype(np.float32),
        frame_id="world",
        timestamp=1.0,
    )
    module.rig = mocker.Mock(cameras={"camera": None}, world_frame="world")
    module.rig.backdrop.return_value = table
    # The base stands 0.3 m above the world origin: world to base lowers every point.
    module.rig.tf.get.return_value = Transform(
        translation=Vector3(0.0, 0.0, -0.3), frame_id="link_base", child_frame_id="world"
    )

    answer = module.find_surface(frame="link_base")

    module.rig.tf.get.assert_called_once_with("link_base", "world", 10.0, mocker.ANY)
    assert answer.startswith("surface 1: height -0.30")
    assert "under the origin of link_base" in answer
    assert "long side at 0 deg, level" in answer
    (surface,) = module.surfaces()
    assert surface["height"] == pytest.approx(0.0, abs=0.004)
