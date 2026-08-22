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

from collections.abc import Iterator
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import open3d as o3d
import pytest

from dimos.agents.skill_result import SkillResult
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.experimental.object import Object as DetObject


def _make_det_object(
    name: str = "cup",
    object_id: str = "abc12345",
    center: tuple[float, float, float] = (0.5, 0.0, 0.3),
    size: tuple[float, float, float] = (0.05, 0.05, 0.10),
    identity_status: str | None = None,
    identity_basis: str | None = None,
) -> DetObject:
    """Create a DetObject with the given attributes and sensible defaults."""
    return DetObject(
        name=name,
        object_id=object_id,
        center=Vector3(x=center[0], y=center[1], z=center[2]),
        size=Vector3(x=size[0], y=size[1], z=size[2]),
        pose=PoseStamped(),
        pointcloud=PointCloud2(o3d.geometry.PointCloud()),
        bbox=(0.0, 0.0, 1.0, 1.0),
        track_id=0,
        class_id=0,
        confidence=1.0,
        ts=0.0,
        image=Image(),
        identity_status=identity_status,
        identity_basis=identity_basis,
    )


@pytest.fixture
def module() -> Iterator[PickAndPlaceModule]:
    """Create an unstarted PickAndPlaceModule for pure-logic tests."""
    instance = PickAndPlaceModule()
    yield instance
    instance.stop()


class TestFindObjectInDetections:
    """Test object lookup logic in detection snapshot."""

    def test_find_by_exact_name(self, module):
        det = _make_det_object(name="cup")
        module._detection_snapshot = [det]

        result = module._find_object_in_detections("cup")
        assert result is det

    def test_find_by_partial_name(self, module):
        det = _make_det_object(name="red cup")
        module._detection_snapshot = [det]

        result = module._find_object_in_detections("cup")
        assert result is det

    def test_find_by_object_id(self, module):
        det = _make_det_object(object_id="abc12345")
        module._detection_snapshot = [det]

        # Truncated prefix match
        result = module._find_object_in_detections("anything", object_id="abc1")
        assert result is det

    def test_find_by_object_id_ambiguous_returns_none(self, module):
        det1 = _make_det_object(object_id="abc12345")
        det2 = _make_det_object(object_id="abc19999")
        module._detection_snapshot = [det1, det2]

        result = module._find_object_in_detections("anything", object_id="abc1")
        assert result is None

    def test_find_missing_returns_none(self, module):
        module._detection_snapshot = [_make_det_object(name="bottle")]

        result = module._find_object_in_detections("keyboard")
        assert result is None

    def test_empty_snapshot_returns_none(self, module):
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

    def test_grasp_heuristic_uses_configured_robot_base(self, module):
        det = _make_det_object(
            center=(-1.703, 1.947, 0.978),
            identity_status="privileged_ground_truth",
            identity_basis="pimsim_mechanics_diagnostic",
        )
        base_pose = PoseStamped(position=Vector3(-1.136, 2.237, 1.041))
        module._detection_snapshot = [det]

        grasps = module._generate_grasps_for_pick("cup", base_pose=base_pose)

        assert grasps is not None
        dx, dy, distance = module._xy_from_base(det.center.x, det.center.y, base_pose)
        expected = module._grasp_orientation(dx, dy, distance)
        assert distance == pytest.approx(0.637, abs=0.001)
        assert tuple(grasps[0].orientation) == pytest.approx(tuple(expected))

    def test_mechanics_truth_grasp_uses_exact_center(self, module):
        det = _make_det_object(
            center=(0.5, 0.1, 0.3),
            size=(0.05, 0.05, 0.10),
            identity_status="privileged_ground_truth",
            identity_basis="pimsim_mechanics_diagnostic",
        )
        module._detection_snapshot = [det]

        grasps = module._generate_grasps_for_pick("cup")

        assert grasps is not None
        assert grasps[0].position == det.center

    def test_perception_grasp_keeps_occlusion_correction(self, module):
        det = _make_det_object(center=(0.5, 0.1, 0.3), size=(0.05, 0.05, 0.10))
        module._detection_snapshot = [det]

        grasps = module._generate_grasps_for_pick("cup")

        assert grasps is not None
        assert grasps[0].position != det.center

    def test_grasp_frame_is_transformed_to_tcp(self, module):
        det = _make_det_object(
            center=(0.5, 0.1, 0.3),
            identity_status="privileged_ground_truth",
            identity_basis="pimsim_mechanics_diagnostic",
        )
        module._detection_snapshot = [det]

        grasps = module._generate_grasps_for_pick(
            "cup",
            grasp_frame_to_tcp=Pose(0.0, 0.0, 0.03),
        )

        assert grasps is not None
        assert grasps[0].position.z == pytest.approx(0.27)

    def test_pre_grasp_uses_the_robot_tcp_direction(self, module):
        grasp = Pose(0.5, 0.0, 0.3)

        pre_grasp = module._compute_pre_grasp_pose(
            grasp,
            0.1,
            Vector3(-1.0, 0.0, 0.0),
        )

        assert pre_grasp.position == Vector3(0.4, 0.0, 0.3)

    def test_pick_excludes_selected_object_from_collision_world(self, module):
        det = _make_det_object(name="cup", object_id="cup-1")
        module._detection_snapshot = [det]
        module._world_monitor = MagicMock()

        with (
            patch.object(
                module,
                "_get_robot",
                return_value=(
                    "arm",
                    "robot-1",
                    SimpleNamespace(
                        pre_grasp_offset=0.05,
                        grasp_frame_to_tcp=Pose(),
                        pre_grasp_direction=Vector3(0.0, 0.0, -1.0),
                        base_pose=PoseStamped(),
                    ),
                ),
            ),
            patch.object(module, "_generate_grasps_for_pick", return_value=[Pose()]),
            patch.object(module, "_lift_if_low", return_value=SkillResult.ok()),
            patch.object(module, "plan_to_pose", return_value=False),
        ):
            result = module.pick("cup")

        assert not result.is_success()
        module._world_monitor.remove_object_obstacle.assert_called_once_with("cup-1")


class TestPlaceBack:
    """Test place_back guard logic."""

    def test_place_back_no_pick_pose_errors(self, module):
        module._last_pick_pose = None

        result = module.place_back()
        assert not result.is_success()
        assert result.error_code == "NO_PRIOR_POSE"
        assert "pick" in result.message.lower()
