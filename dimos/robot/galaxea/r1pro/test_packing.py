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

"""Packing decisions include object geometry and gripper release clearance."""

import pytest

from dimos.robot.galaxea.r1pro.packing import OccupiedFootprint, clear_pick_order, empty_slots


def test_five_bottles_get_distinct_neat_slots_and_a_full_tray_stops():
    occupied = []
    chosen = []
    for _ in range(6):
        slots = empty_slots(0.025, tuple(occupied))
        assert slots
        x, y = slots[0]
        occupied.append(OccupiedFootprint(x, y, 0.025))
        chosen.append((round(x, 3), round(y, 3)))
    assert chosen[:5] == [(-0.065, 0.06), (0.0, 0.06), (0.065, 0.06), (-0.065, -0.06), (0.0, -0.06)]
    assert empty_slots(0.025, tuple(occupied)) == []


def test_oversize_bottle_does_not_trigger_a_placement_attempt():
    assert empty_slots(0.11) == []


def test_neighbor_must_clear_the_open_gripper_as_well_as_the_bottle():
    # A centre bottle fits by radius, but opening fingers would hit this neighbour.
    occupied = (OccupiedFootprint(0.0, 0.07, 0.025),)
    assert empty_slots(0.025, occupied, inner_half_size=(0.065, 0.10)) == []


@pytest.mark.parametrize("radius", [0, -0.01, float("nan"), float("inf")])
def test_invalid_geometry_is_rejected(radius):
    with pytest.raises(ValueError, match="geometry"):
        empty_slots(radius)


def test_rear_bottles_wait_until_their_transfer_corridor_is_clear():
    positions = ((0.45, -0.28), (0.34, -0.4), (0.45, -0.4), (0.24, -0.4), (0.34, -0.52))
    order = clear_pick_order(positions, (4, 3, 2, 1, 0))
    assert sorted(order) == list(range(5))
    assert order.index(0) < order.index(2)
    assert order.index(1) < order.index(4)
    assert order[0] == 3


@pytest.mark.parametrize("offset", [-0.003, 0.0, 0.003])
def test_default_order_clears_sources_left_to_right(offset):
    positions = ((0.45, -0.28), (0.34, -0.4), (0.45, -0.4), (0.24, -0.4), (0.34, -0.52))
    jittered = tuple((x + offset * (-1) ** i, y - offset) for i, (x, y) in enumerate(positions))
    assert clear_pick_order(jittered) == [3, 1, 4, 0, 2]
