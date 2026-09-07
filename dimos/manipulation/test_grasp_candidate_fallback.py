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

from collections.abc import Iterator
import math
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import numpy as np
import pytest

from dimos.manipulation.grasp_verification import GripperSettle
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Header import Header


def _box_cloud() -> PointCloud2:
    """An oblong cross-section, so the narrow axis is unambiguous."""
    rng = np.random.default_rng(0)
    points = np.column_stack(
        [
            rng.uniform(-0.05, 0.05, 400),
            rng.uniform(-0.01, 0.01, 400),
            rng.uniform(0.0, 0.10, 400),
        ]
    )
    return PointCloud2.from_numpy(points, frame_id="world", timestamp=1.0)


def _yaw_of(candidate: GraspCandidate) -> float:
    return float(candidate.pose.orientation.to_euler().z)


def test_generator_offers_the_wrist_flip_and_keeps_the_narrow_axis_first() -> None:
    single = HeuristicGraspModule()
    many = HeuristicGraspModule(yaw_candidates=4)
    try:
        cloud = _box_cloud()
        one = single.propose_grasps(cloud)
        several = many.propose_grasps(cloud)

        # The default stays exactly what it was, so the xArm is unaffected.
        assert len(one.candidates) == 1
        assert len(several.candidates) > 1
        assert several.candidates[0].score == pytest.approx(1.0)
        assert _yaw_of(several.candidates[0]) == pytest.approx(_yaw_of(one.candidates[0]))
        assert [c.score for c in several.candidates] == sorted(
            (c.score for c in several.candidates), reverse=True
        )
        # A half turn is the same physical grasp for parallel jaws, so the
        # oblong object gets it and nothing that grasps across the wide axis.
        deltas = [
            abs((_yaw_of(c) - _yaw_of(one.candidates[0]) + math.pi) % (2 * math.pi) - math.pi)
            for c in several.candidates[1:]
        ]
        assert deltas and all(delta == pytest.approx(math.pi, abs=1e-6) for delta in deltas)
    finally:
        single.stop()
        many.stop()


@pytest.fixture
def module() -> Iterator[PickAndPlaceModule]:
    instance = PickAndPlaceModule(planning_frame="world")
    instance._scene = MagicMock()
    instance._grasp_generator = MagicMock()
    instance._manipulation = MagicMock()
    instance._manipulation.list_planning_groups.return_value = [
        SimpleNamespace(id="arm/tool", has_gripper=True, tip_frame="tool")
    ]
    instance._manipulation.get_state.return_value = SimpleNamespace(
        groups={"arm/tool": SimpleNamespace(gripper_position=0.5, end_effector_pose=None)}
    )
    instance._manipulation.execute.return_value = SimpleNamespace(succeeded=True, message="")
    instance._manipulation.set_gripper_position.return_value = SimpleNamespace(
        succeeded=True, message=""
    )
    instance._objects = {"cup-1": {"object_id": "cup-1", "name": "cup"}}
    instance._scene.get_object_pointcloud_by_object_id.return_value = MagicMock()
    instance.config.grasp_verification.enabled = False
    yield instance
    instance.stop()


def test_pick_skips_a_candidate_that_will_not_plan(
    module: PickAndPlaceModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(
        "dimos.manipulation.pick_and_place_module.await_gripper_settle",
        lambda read, target, config: GripperSettle(True, target, True, 0.1),
    )
    first = Pose(Vector3(0.4, 0.0, 0.2), Quaternion.from_euler(Vector3(-math.pi, 0.0, 0.0)))
    second = Pose(Vector3(0.4, 0.0, 0.2), Quaternion.from_euler(Vector3(-math.pi, 0.0, math.pi)))
    module._grasp_generator.propose_grasps.return_value = GraspCandidateArray(
        Header(1.0, "world"),
        [GraspCandidate(first, score=1.0), GraspCandidate(second, score=0.95)],
    )

    planned: list[PoseStamped] = []
    first_orientation = None

    def plan(targets: dict[str, PoseStamped], **_: Any) -> SimpleNamespace:
        nonlocal first_orientation
        pose = next(iter(targets.values()))
        planned.append(pose)
        if first_orientation is None:
            first_orientation = pose.orientation
        rejected = pose.orientation == first_orientation
        return SimpleNamespace(succeeded=not rejected, message="unreachable wrist angle")

    module._manipulation.plan_to_poses.side_effect = plan

    result = module.pick_object("cup-1")

    assert result.success, result.message
    assert result.metadata["rank"] == 1
    assert result.metadata["candidates"] == 2
    assert module._holding_object
    # The first proposal was planned for and abandoned, not executed.
    assert planned[0].orientation == first_orientation
    assert planned[-1].orientation != first_orientation
