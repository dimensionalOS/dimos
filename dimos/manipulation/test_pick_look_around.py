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

"""pick_object looking at the cup from more viewpoints before grasping."""

import time

import numpy as np
import pytest

from dimos.manipulation.manipulation_spec import PlanResult, PlanStatus
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule

# The pick-and-place unit tests' arm and cup, and their fixtures.
from dimos.manipulation.test_pick_and_place_unit import (  # noqa: F401
    CUP_AT,
    PLANNED,
    _detections,
    _joints,
    module,
    settled_gripper,
)
from dimos.utils.transform_utils import pose_to_matrix

UNREACHABLE = PlanResult(PlanStatus.FAILED, "IK failed")

# A camera 17 cm behind the tool tip, looking along its Z axis.
CAMERA_FROM_TIP = (
    (1.0, 0.0, 0.0, 0.0),
    (0.0, 1.0, 0.0, 0.0),
    (0.0, 0.0, 1.0, -0.17),
    (0.0, 0.0, 0.0, 1.0),
)


def test_pick_looks_around_the_target_first_skipping_viewpoints_out_of_reach(
    module: PickAndPlaceModule,
) -> None:
    module.config.scan_views = 2
    module.config.camera_from_tip = CAMERA_FROM_TIP
    # The first viewpoint tried (opposite the scan's) is out of reach.
    module._manipulation.plan_to_poses.side_effect = [UNREACHABLE, PLANNED, PLANNED] + [
        PLANNED
    ] * 20
    module._scene.scan_scene.return_value = _detections(ts=time.time() + 60.0)

    result = module.pick_object("cup-1")

    assert result.metadata["scan_views"] == {"views": 2, "unreachable": 1, "points": [2, 2]}
    # The viewpoints, then the pre-grasp: each one's camera looks at the cup.
    looks = [
        call.args[0]["arm/tool"] for call in module._manipulation.plan_to_poses.call_args_list[:3]
    ]
    for pose in looks:
        camera = pose_to_matrix(pose) @ np.asarray(CAMERA_FROM_TIP)
        to_cup = np.array([CUP_AT.x, CUP_AT.y, CUP_AT.z]) - camera[:3, 3]
        assert camera[:3, 2] == pytest.approx(to_cup / np.linalg.norm(to_cup))
    # A detection at each of the two views reached, before grasping.
    assert module._scene.scan_scene.call_count >= 2


def test_pick_without_a_camera_offset_skips_looking_around(module: PickAndPlaceModule) -> None:
    module.config.scan_views = 2

    result = module.pick_object("cup-1")

    assert result.metadata["scan_views"] == {
        "skipped": "no camera_from_tip in the config, or no tool tip pose"
    }


def test_pick_keeps_its_target_in_the_scene_registration_while_looking_around(
    module: PickAndPlaceModule,
) -> None:
    module.config.scan_views = 1
    module.config.camera_from_tip = CAMERA_FROM_TIP
    module._scene.scan_scene.return_value = _detections(ts=time.time() + 60.0)

    module.pick_object("cup-1")

    module._scene.keep_object.assert_called_once_with("cup-1")


def test_pick_skips_looking_around_a_target_the_registration_lost(
    module: PickAndPlaceModule,
) -> None:
    module.config.scan_views = 2
    module.config.camera_from_tip = CAMERA_FROM_TIP
    module._scene.keep_object.return_value = False

    result = module.pick_object("cup-1")

    assert result.metadata["scan_views"] == {
        "skipped": "the scene registration has no object cup-1"
    }


def test_look_around_backs_away_when_the_planner_will_not_start_from_a_view(
    module: PickAndPlaceModule,
) -> None:
    module.config.scan_views = 1
    module.config.camera_from_tip = CAMERA_FROM_TIP
    module._scene.scan_scene.return_value = _detections(ts=time.time() + 60.0)
    module._scan_joints = {"arm/tool": _joints(0.3)}
    stuck = PlanResult(PlanStatus.FAILED, "Start configuration is in collision")
    module._manipulation.plan_to_joints.side_effect = [stuck, PLANNED] + [PLANNED] * 10

    result = module.pick_object("cup-1")

    assert "stopped" not in result.metadata["scan_views"]
    # The first move with collision checking off backs the camera away from the cup.
    backing = next(
        c
        for c in module._manipulation.move_linear.call_args_list
        if c.kwargs.get("check_collision") is False
    )
    delta = np.array(backing.args[:3])
    assert np.linalg.norm(delta) == pytest.approx(0.35 / 3.0)
