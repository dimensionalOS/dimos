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

"""Randomized layouts preserve physical fit constraints and object geometry."""

from dataclasses import replace
import math

import numpy as np
import pytest

from dimos.robot.galaxea.r1pro.grasping_sim import TABLE_Z
from dimos.robot.galaxea.r1pro.object_packing_scene import (
    OBJECT_SLOT_BOUNDS,
    OBJECT_TRAY_HALF_SIZE,
    sample_layout,
)
from dimos.robot.galaxea.r1pro.object_packing_task import object_extent
from dimos.robot.galaxea.r1pro.packing import OccupiedFootprint, empty_slots


def test_seed_reproduces_layout_without_assigning_shape_or_position_by_id():
    first = sample_layout(100001)
    assert first == sample_layout(100001)
    other = sample_layout(100002)
    assert first.objects[0].position != other.objects[0].position
    assert len({o.shape for seed in range(12) for o in sample_layout(seed).objects}) == 3


@pytest.mark.parametrize("seed", range(12))
def test_source_objects_rest_on_surface_with_room_for_open_fingers(seed):
    layout = sample_layout(seed)
    for i, obj in enumerate(layout.objects):
        assert obj.position[2] - obj.half_size[2] == pytest.approx(TABLE_Z + 0.001)
        assert 2 * obj.radius < 0.10
        for other in layout.objects[:i]:
            assert (
                abs(obj.position[0] - other.position[0]) > obj.radius + other.radius + 0.026
                or abs(obj.position[1] - other.position[1]) > obj.radius + other.radius + 0.09
            )


def test_partial_occupancy_uses_geometry_and_leaves_an_available_source():
    layout = sample_layout(13, count=5, occupied=3)
    aboard = [o for o in layout.objects if o.in_tray]
    assert len(aboard) == 3
    assert len([o for o in layout.objects if not o.in_tray]) == 2
    for obj in aboard:
        assert obj.position[2] - obj.half_size[2] == pytest.approx(TABLE_Z + 0.016)
        for other in aboard:
            if other.name != obj.name:
                assert math.dist(obj.position[:2], other.position[:2]) > obj.radius + other.radius


def test_box_extent_accounts_for_rotation_and_cylinder_yaw_does_not_change_radius():
    obj = sample_layout(20).objects[0]
    turn = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
    box = replace(obj, shape="box", half_size=(0.01, 0.02, 0.06))
    assert object_extent(box, turn) == pytest.approx([0.02, 0.01, 0.06])
    cylinder = replace(obj, shape="cylinder", half_size=(0.02, 0.02, 0.06))
    assert object_extent(cylinder, turn) == pytest.approx([0.02, 0.02, 0.06])
    tipped = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
    assert object_extent(cylinder, tipped) == pytest.approx([0.06, 0.02, 0.02])


@pytest.mark.parametrize("count,occupied", [(3, 0), (6, 0), (4, 4), (5, -1)])
def test_invalid_object_counts_are_rejected(count, occupied):
    with pytest.raises(ValueError, match="four or five"):
        sample_layout(1, count=count, occupied=occupied)


def test_short_objects_leave_room_to_open_fingers_and_stop_when_tray_is_full():
    occupied = []
    for _ in range(6):
        slots = empty_slots(0.024, tuple(occupied), inner_half_size=OBJECT_SLOT_BOUNDS)
        assert slots
        x, y = slots[0]
        assert OBJECT_TRAY_HALF_SIZE[1] - abs(y) > 0.075
        occupied.append(OccupiedFootprint(x, y, 0.024))
    assert empty_slots(0.024, tuple(occupied), inner_half_size=OBJECT_SLOT_BOUNDS) == []
