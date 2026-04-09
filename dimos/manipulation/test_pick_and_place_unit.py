# Copyright 2025-2026 Dimensional Inc.
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

"""Unit tests for PickAndPlaceModule pure logic (no Drake required)."""

from __future__ import annotations

import threading
from unittest.mock import MagicMock, patch

from dimos.manipulation.manipulation_module import ManipulationState
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.msgs.geometry_msgs.Vector3 import Vector3


def _make_det_object(
    name: str = "cup",
    object_id: str = "abc12345",
    center: tuple[float, float, float] = (0.5, 0.0, 0.3),
    size: tuple[float, float, float] = (0.05, 0.05, 0.10),
) -> MagicMock:
    """Create a lightweight mock DetObject with the attributes used by PickAndPlaceModule."""
    obj = MagicMock()
    obj.name = name
    obj.object_id = object_id
    obj.center = Vector3(x=center[0], y=center[1], z=center[2])
    obj.size = Vector3(x=size[0], y=size[1], z=size[2])
    return obj


def _make_module() -> PickAndPlaceModule:
    """Create a PickAndPlaceModule with mocked __init__."""
    with patch.object(PickAndPlaceModule, "__init__", lambda self: None):
        module = PickAndPlaceModule.__new__(PickAndPlaceModule)
        module._state = ManipulationState.IDLE
        module._lock = threading.Lock()
        module._error_message = ""
        module._robots = {}
        module._planned_paths = {}
        module._planned_trajectories = {}
        module._world_monitor = None
        module._planner = None
        module._kinematics = None
        module._coordinator_client = None
        module._detection_snapshot = []
        module._last_pick_pose = None
        return module


class TestFindObjectInDetections:
    """Test object lookup logic in detection snapshot."""

    def test_find_by_exact_name(self):
        module = _make_module()
        det = _make_det_object(name="cup")
        module._detection_snapshot = [det]

        result = module._find_object_in_detections("cup")
        assert result is det

    def test_find_by_partial_name(self):
        module = _make_module()
        det = _make_det_object(name="red cup")
        module._detection_snapshot = [det]

        result = module._find_object_in_detections("cup")
        assert result is det

    def test_find_by_object_id(self):
        module = _make_module()
        det = _make_det_object(object_id="abc12345")
        module._detection_snapshot = [det]

        # Truncated prefix match
        result = module._find_object_in_detections("anything", object_id="abc1")
        assert result is det

    def test_find_missing_returns_none(self):
        module = _make_module()
        module._detection_snapshot = [_make_det_object(name="bottle")]

        result = module._find_object_in_detections("keyboard")
        assert result is None

    def test_empty_snapshot_returns_none(self):
        module = _make_module()
        module._detection_snapshot = []

        result = module._find_object_in_detections("cup")
        assert result is None


class TestGraspHeuristics:
    """Test grasp orientation and occlusion offset static methods."""

    def test_occlusion_offset_toward_robot(self):
        center = Vector3(x=0.5, y=0.0, z=0.3)
        size = Vector3(x=0.1, y=0.1, z=0.1)

        ox, oy = PickAndPlaceModule._occlusion_offset(center, size)
        # Offset should shift x closer to robot origin (smaller x)
        assert ox < center.x
        assert abs(oy - center.y) < 1e-6  # y should stay ~0

    def test_occlusion_offset_at_origin(self):
        center = Vector3(x=0.0, y=0.0, z=0.3)
        size = Vector3(x=0.1, y=0.1, z=0.1)

        ox, oy = PickAndPlaceModule._occlusion_offset(center, size)
        # At origin, no shift should occur (division-by-zero guard)
        assert abs(ox) < 1e-3
        assert abs(oy) < 1e-3

    def test_grasp_orientation_near_is_top_down(self):
        q = PickAndPlaceModule._grasp_orientation(gx=0.3, gy=0.0, xy_dist=0.3)
        # Near object: pitch = 180° (top-down), tilt = 0, yaw = 0
        # RPY(0, π, 0) → quaternion (x=0, y=1, z=0, w=0)
        assert abs(q.x) < 0.01
        assert abs(q.y - 1.0) < 0.01
        assert abs(q.z) < 0.01
        assert abs(q.w) < 0.01

    def test_grasp_orientation_far_differs_from_near(self):
        q_near = PickAndPlaceModule._grasp_orientation(gx=0.3, gy=0.0, xy_dist=0.3)
        q_far = PickAndPlaceModule._grasp_orientation(gx=1.0, gy=0.0, xy_dist=1.0)
        # Far object should have different orientation (tilted)
        assert not (
            abs(q_near.x - q_far.x) < 0.01
            and abs(q_near.y - q_far.y) < 0.01
            and abs(q_near.z - q_far.z) < 0.01
            and abs(q_near.w - q_far.w) < 0.01
        )


class TestPlaceBack:
    """Test place_back guard logic."""

    def test_place_back_no_pick_pose_errors(self):
        module = _make_module()
        module._last_pick_pose = None

        result = module.place_back()
        assert "Error" in result
        assert "pick" in result.lower()
