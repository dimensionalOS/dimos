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

"""The envelope tool's pure half: frames, accumulation, gates, baking."""

from __future__ import annotations

import math

import numpy as np
import pytest

from dimos.navigation.motion.simulation.envelope import (
    Cell,
    Extent,
    achieved_twist,
    arc_inflate,
    command,
    corner_offsets,
    fold,
    to_base,
    union,
    verdict,
    yaw_of,
)


def approx(x):
    return pytest.approx(x, abs=1e-9)


def box(xlo, xhi, ylo, yhi):
    return Extent(np.array([xlo, ylo], float), np.array([xhi, yhi], float))


def test_corner_offsets_spans_the_local_box():
    got = corner_offsets(np.array([[1.0, 2.0, 3.0, 0.5, 0.25, 0.125]]))
    assert got.shape == (1, 8, 3)
    assert np.allclose(got.min(axis=1)[0], [0.5, 1.75, 2.875])
    assert np.allclose(got.max(axis=1)[0], [1.5, 2.25, 3.125])


def test_to_base_translates_then_unrotates():
    pts = np.array([[2.0, 1.0, 0.0], [1.0, 1.0, 5.0]])
    got = to_base(pts, np.array([1.0, 1.0, 0.0]), math.pi / 2)
    # (1, 0) in world with the body facing +y is 1 m to the body's right
    assert np.allclose(got, [[0.0, -1.0], [0.0, 0.0]], atol=1e-12)


def test_to_base_is_identity_at_zero_yaw():
    pts = np.array([[0.3, -0.2, 9.0]])
    assert np.allclose(to_base(pts, np.zeros(3), 0.0), [[0.3, -0.2]])


def test_yaw_of_reads_the_heading():
    assert yaw_of(np.array([1.0, 0.0, 0.0, 0.0])) == 0.0
    c = math.cos(math.pi / 8)
    s = math.sin(math.pi / 8)
    assert yaw_of(np.array([c, 0.0, 0.0, s])) == approx(math.pi / 4)


def test_extent_grows_and_never_shrinks():
    e = Extent()
    assert e.empty
    e.add(np.array([[0.1, 0.2], [-0.3, -0.4]]))
    assert not e.empty
    assert e.length == approx(0.4)
    assert e.width == approx(0.6)
    e.add(np.array([[0.0, 0.0]]))
    assert e.length == approx(0.4)


def test_extent_union_and_centre():
    a = box(-0.5, 0.3, -0.2, 0.2)
    b = box(-0.4, 0.4, -0.25, 0.1)
    u = a | b
    assert u.length == approx(0.9)
    assert u.width == approx(0.45)
    assert u.center_off == approx(-0.05)


def test_command_puts_drift_in_the_body_frame():
    assert np.allclose(command(0.0, 0.5), [0.5, 0.0, 0.0])
    assert np.allclose(command(90.0, 0.5), [0.0, 0.5, 0.0], atol=1e-15)
    assert np.allclose(command(180.0, 0.5, 1.4), [-0.5, 0.0, 1.4], atol=1e-15)


def test_achieved_twist_reads_a_straight_run():
    t = np.linspace(0.0, 2.0, 21)
    pos = np.stack([0.4 * t, np.zeros_like(t), np.zeros_like(t)], axis=1)
    quat = np.tile([1.0, 0.0, 0.0, 0.0], (len(t), 1))
    vx, vy, wz = achieved_twist(t, pos, quat, since=1.0)
    assert vx == approx(0.4)
    assert vy == approx(0.0)
    assert wz == approx(0.0)


def test_achieved_twist_is_body_frame():
    # a body facing +y translating along world +y is going straight ahead
    t = np.linspace(0.0, 2.0, 21)
    pos = np.stack([np.zeros_like(t), 0.4 * t, np.zeros_like(t)], axis=1)
    c, s = math.cos(math.pi / 4), math.sin(math.pi / 4)
    quat = np.tile([c, 0.0, 0.0, s], (len(t), 1))
    vx, vy, _ = achieved_twist(t, pos, quat, since=0.0)
    assert vx == approx(0.4)
    assert vy == approx(0.0)


def test_verdict_accepts_a_tracked_cell():
    assert verdict(90.0, 0.5, 0.0, (0.01, 0.37, -0.004)) == ""


def test_verdict_names_why_a_cell_is_not_a_measurement():
    assert verdict(0.0, 0.2, 0.0, (0.001, 0.0, 0.0)) == "stalled"
    assert verdict(90.0, 0.5, 0.0, (0.4, 0.05, 0.0)) == "drifted"
    assert verdict(0.0, 0.5, 0.0, (0.4, 0.0, 0.4)) == "turned"
    assert verdict(0.0, 0.0, 1.4, (0.0, 0.0, 0.1)) == "no-turn"
    assert verdict(0.0, 0.0, 1.4, (0.0, 0.0, 1.2)) == ""


def test_verdict_wraps_the_drift_comparison():
    # commanded 180, achieved -179.5 deg: half a degree apart, not 359.5
    assert verdict(180.0, 0.5, 0.0, (-0.5, -0.004, 0.0)) == ""


def cell(drift, speed, extent, yaw_rate=0.0):
    return Cell(drift, speed, yaw_rate, extent, (0.0, 0.0, 0.0), "")


def test_a_row_covers_the_mirror_of_the_opposite_sign():
    # planner/revision.md: rows are stored for POSITIVE drift and mirrored by
    # sign at lookup, so -45's box has to fit inside the mirror of the row.
    cells = [
        cell(45.0, 0.35, box(-0.4, 0.35, -0.2, 0.2)),
        cell(-45.0, 0.35, box(-0.4, 0.35, -0.25, 0.15)),
        cell(45.0, 0.95, box(-0.9, 0.9, -0.9, 0.9)),  # out of band, ignored
    ]
    # y spans [-0.2, 0.2] for +45 and, mirrored, [-0.15, 0.25] for -45
    assert fold(cells, (0.35,)) == [(45.0, 0.75, 0.45, -0.025, 0.025)]


def test_fold_puts_a_stand_cell_in_every_row():
    cells = [
        cell(0.0, 0.0, box(-0.45, 0.35, -0.21, 0.21)),
        cell(90.0, 0.35, box(-0.4, 0.35, -0.25, 0.2)),
    ]
    assert fold(cells, (0.0, 0.35)) == [(90.0, 0.8, 0.5, -0.05, 0.0)]


def test_fold_drops_arcs():
    cells = [
        cell(0.0, 0.35, box(-0.4, 0.35, -0.2, 0.2)),
        cell(0.0, 0.35, box(-0.9, 0.9, -0.9, 0.9), yaw_rate=1.4),
    ]
    assert fold(cells, (0.35,)) == [(0.0, 0.75, 0.4, -0.025, 0.0)]


def test_arc_inflate_recovers_a_planted_slope():
    # width grows by 0.05 m per rad/m of yaw-per-metre; the stored number is
    # that slope itself, not a per-edge conversion of it (planner/revision.md)
    cells = [cell(0.0, v, box(-0.4, 0.4, -0.2, 0.2)) for v in (0.35, 0.75)]
    for v in (0.35, 0.75):
        for w in (0.35, 0.7):
            half = 0.2 + 0.05 * (w / v) / 2.0
            cells.append(cell(0.0, v, box(-0.4, 0.4, -half, half), yaw_rate=w))
    assert arc_inflate(cells) == approx(0.05)


def test_arc_inflate_takes_the_worse_turn_direction():
    cells = [cell(0.0, 1.0, box(-0.4, 0.4, -0.2, 0.2))]
    cells.append(cell(0.0, 1.0, box(-0.4, 0.4, -0.2, 0.3), yaw_rate=1.0))
    cells.append(cell(0.0, 1.0, box(-0.4, 0.4, -0.2, 0.25), yaw_rate=-1.0))
    assert arc_inflate(cells) == approx(0.1)


def test_union_covers_every_cell():
    u = union(
        [cell(0.0, 0.5, box(-0.5, 0.3, -0.2, 0.2)), cell(90.0, 0.5, box(-0.4, 0.4, -0.3, 0.1))]
    )
    assert u.length == approx(0.9)
    assert u.width == approx(0.5)
