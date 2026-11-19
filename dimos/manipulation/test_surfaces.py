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

import numpy as np
import pytest

from dimos.manipulation.surfaces import find_surfaces


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


def test_level_mode_finds_heights_with_blocks_on_the_table(rng):
    table = _slab(0.8, 0.5, 30.0, 0.0, 0.0, (0.5, 0.1, 0.02), rng)
    floor = _slab(0.6, 0.3, 0.0, 0.0, 0.0, (1.2, 0.0, -0.7), rng)
    blocks = [_block((x, 0.0, 0.02), (0.05, 0.05, 0.05), rng) for x in (0.3, 0.45, 0.6)]
    surfaces = find_surfaces(np.concatenate([table, floor, *blocks]), level=True)

    assert [round(s.height, 2) for s in surfaces] == [0.02, -0.7]
    table_surface = surfaces[0]
    assert table_surface.tilt_deg == 0.0
    assert table_surface.size == pytest.approx((0.8, 0.5), abs=0.03)
    assert table_surface.yaw_deg == pytest.approx(30.0, abs=1.5)


def test_level_mode_is_parallel_to_the_reporting_frame(rng):
    """A base tilted 3 degrees under a level table sees the table level to itself."""
    tilt = np.radians(3.0)
    to_base = np.eye(4)
    to_base[:3, :3] = [
        [np.cos(tilt), 0.0, -np.sin(tilt)],
        [0.0, 1.0, 0.0],
        [np.sin(tilt), 0.0, np.cos(tilt)],
    ]
    # Points that are level in the base frame, handed over in the world frame.
    in_base = _slab(0.8, 0.5, 0.0, 0.0, 0.0, (0.5, 0.0, 0.1), rng)
    in_world = in_base @ to_base[:3, :3]

    (surface,) = find_surfaces(in_world, to_frame=to_base, level=True)

    assert surface.tilt_deg == 0.0
    assert surface.height == pytest.approx(0.1, abs=0.004)


@pytest.mark.parametrize("level", [True, False])
def test_a_square_table_reads_square(rng, level):
    """The axes of a square's points are any two; its smallest rectangle is not."""
    table = _slab(1.0, 1.0, 20.0, 0.0, 0.0, (0.5, 0.0, 0.12), rng)

    (surface,) = find_surfaces(table, level=level)

    assert surface.size == pytest.approx((1.0, 1.0), abs=0.03)
    assert surface.yaw_deg % 90.0 == pytest.approx(20.0, abs=1.5)


def test_level_mode_keeps_a_slightly_tilted_table_whole(rng):
    """A base 1.5 degrees off its table: 2.6 cm over a metre, one surface all the same."""
    table = _slab(1.0, 1.0, 0.0, 1.5, 0.0, (0.5, 0.0, 0.12), rng)

    (surface,) = find_surfaces(table, level=True)

    assert surface.tilt_deg == 0.0
    assert surface.height == pytest.approx(0.12, abs=0.004)
    assert surface.size == pytest.approx((1.0, 1.0), abs=0.03)


@pytest.mark.parametrize("level", [True, False])
def test_scattered_block_tops_are_not_a_surface(rng, level):
    """Four 4 cm tops at one height, 20 cm apart, are four patches too small to count."""
    table = _slab(1.0, 1.0, 0.0, 0.0, 0.0, (0.5, 0.0, 0.12), rng)
    tops = [
        _slab(0.04, 0.04, 0.0, 0.0, 0.0, (0.2 + 0.2 * i, 0.0, 0.16), rng, step=0.001)
        for i in range(4)
    ]

    surfaces = find_surfaces(np.concatenate([table, *tops]), level=level)

    assert [round(s.height, 2) for s in surfaces] == [0.12]


def test_a_surface_seen_twice_is_not_bigger(rng):
    table = _slab(0.8, 0.5, 0.0, 0.0, 0.0, (0.5, 0.0, 0.12), rng)
    floor = _slab(1.0, 0.6, 0.0, 0.0, 0.0, (1.2, 0.0, -0.5), rng)

    once = find_surfaces(np.concatenate([table, floor]))
    twice = find_surfaces(np.concatenate([table, table, table, floor]))

    assert [s.points for s in twice] == [s.points for s in once]
    assert [round(s.height, 1) for s in twice] == [-0.5, 0.1]


@pytest.mark.parametrize("level", [True, False])
def test_what_stands_on_a_surface_is_counted(rng, level):
    table = _slab(0.8, 0.5, 0.0, 0.0, 0.0, (0.5, 0.0, 0.12), rng)
    floor = _slab(1.0, 0.6, 0.0, 0.0, 0.0, (1.4, 0.0, -0.5), rng)
    blocks = [_block((x, 0.0, 0.12), (0.05, 0.05, 0.05), rng) for x in (0.35, 0.6)]
    # Past the table's edge: not standing on it.
    beside = _block((1.0, 0.0, 0.12), (0.05, 0.05, 0.05), rng)

    surfaces = find_surfaces(np.concatenate([table, floor, *blocks, beside]), level=level)

    table_surface = next(s for s in surfaces if s.height > 0)
    floor_surface = next(s for s in surfaces if s.height < 0)
    assert table_surface.points_above > 0
    assert floor_surface.points_above == 0
    on_table_only = find_surfaces(np.concatenate([table, *blocks]), level=level)[0].points_above
    assert table_surface.points_above == pytest.approx(on_table_only, rel=0.05)


def test_level_mode_takes_no_ramp_as_parallel(rng):
    """Reported level, a 20 degree ramp would be its own points standing on itself."""
    ramp = _slab(0.8, 0.5, 0.0, 20.0, 0.0, (0.5, 0.0, 0.1), rng)

    assert find_surfaces(ramp, level=True) == []
    (surface,) = find_surfaces(ramp, level=False)
    assert surface.points_above == 0


def test_what_stands_on_a_slightly_tilted_table_is_only_what_is_on_it(rng):
    table = _slab(1.0, 1.0, 0.0, 3.0, 0.0, (0.5, 0.0, 0.12), rng)

    (bare,) = find_surfaces(table, level=True)
    assert bare.tilt_deg == 0.0
    assert bare.points_above == 0
