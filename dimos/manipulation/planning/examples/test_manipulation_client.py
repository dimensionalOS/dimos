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

from __future__ import annotations

import numpy as np
import pytest

from dimos.manipulation.manipulation_spec import PlanningGroupInfo, PlanResult, PlanStatus
from dimos.manipulation.pick_and_place_spec import (
    DetectedObject,
    PickAndPlaceSpec,
    PickPlaceStatus,
    PickResult,
    PlaceResult,
    ScanResult,
)
from dimos.manipulation.planning.examples.manipulation_client import run_motion, run_pick_place
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.sdk.manipulation import Arm, MotionError


@pytest.fixture
def arm(mocker):
    client = mocker.Mock(spec=Arm)
    client.info = PlanningGroupInfo("arm", ("j0", "j1"), "base", "tip", True)
    client.joints.return_value = np.array([0.1, 0.2])
    client.pose.return_value = PoseStamped(frame_id="world", position=[0.4, 0.0, 0.3])
    return client


def test_motion_example_uses_sdk_and_restores_initial_joints(arm):
    group = run_motion(arm)

    assert group == "arm"
    np.testing.assert_allclose(arm.move_joints.call_args_list[0].args[0], [0.12, 0.2])
    np.testing.assert_array_equal(arm.move_joints.call_args_list[1].args[0], [0.1, 0.2])
    np.testing.assert_array_equal(arm.joints.return_value, [0.1, 0.2])
    arm.move_pose.assert_called_once_with([0.4, 0.0, 0.31], speed_scale=0.2)
    arm.move_linear.assert_called_once_with(dz=-0.01, check_collision=True)
    arm.open_gripper.assert_called_once_with()


def test_motion_example_stops_after_sdk_failure(arm):
    failure = MotionError("move_joints", PlanResult(PlanStatus.FAILED, "unreachable"))
    arm.move_joints.side_effect = failure

    with pytest.raises(MotionError, match="unreachable") as error:
        run_motion(arm)

    assert error.value is failure
    arm.move_pose.assert_not_called()
    arm.open_gripper.assert_not_called()


@pytest.fixture
def pick_place(mocker):
    module = mocker.Mock(spec=PickAndPlaceSpec)
    module.scan_objects.return_value = ScanResult(
        PickPlaceStatus.SUCCEEDED, objects=(DetectedObject("cup-1", "cup"),)
    )
    module.pick_object.return_value = PickResult(
        PickPlaceStatus.SUCCEEDED, object_id="cup-1", holding_object=True
    )
    module.place_at.return_value = PlaceResult(PickPlaceStatus.SUCCEEDED)
    return module


def test_example_uses_scanned_id_and_explicit_destination(pick_place):
    run_pick_place(pick_place, "cup", (0.4, 0.0, 0.2), "arm")

    pick_place.pick_object.assert_called_once_with("cup-1", "arm")
    pick_place.place_at.assert_called_once_with(0.4, 0.0, 0.2, planning_group="arm")


def test_example_does_not_guess_between_objects(pick_place):
    pick_place.scan_objects.return_value = ScanResult(
        PickPlaceStatus.SUCCEEDED,
        objects=(DetectedObject("cup-1", "cup"), DetectedObject("cup-2", "cup")),
    )

    with pytest.raises(RuntimeError, match="exactly one object"):
        run_pick_place(pick_place, "cup", (0.4, 0.0, 0.2), "arm")

    pick_place.pick_object.assert_not_called()


def test_example_stops_after_failed_pick_even_when_holding(pick_place):
    pick_place.pick_object.return_value = PickResult(
        PickPlaceStatus.EXECUTION_FAILED, "retract failed", holding_object=True
    )

    with pytest.raises(RuntimeError, match="retract failed"):
        run_pick_place(pick_place, "cup", (0.4, 0.0, 0.2), "arm")

    pick_place.place_at.assert_not_called()
