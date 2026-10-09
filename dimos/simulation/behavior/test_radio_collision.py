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

"""Restoration and fail-closed scene ownership at the native API boundary."""

from contextlib import nullcontext
from threading import RLock

import numpy as np
import pytest

from dimos.manipulation.planning.spec.enums import ObstacleType
from dimos.manipulation.planning.spec.models import Obstacle
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.simulation.behavior.radio_collision import temporary_carried_radio_geometry


@pytest.fixture
def world(mocker):
    native = mocker.Mock()
    world = mocker.Mock()
    world._lock = RLock()
    world._usable = True
    world._require_scene.return_value = native
    world.scratch_context.return_value = nullcontext(object())
    wrist = np.eye(4)
    gripper = np.eye(4)
    gripper[:3, 3] = [-0.0295, 0, -0.16065]
    world.get_link_pose.side_effect = [wrist, gripper]
    world.get_obstacles.return_value = [
        Obstacle(
            name=name,
            obstacle_type=ObstacleType.BOX,
            pose=PoseStamped(frame_id="world", position=[1, 2, 3]),
            dimensions=(0.1, 0.1, 0.1),
        )
        for name in ("body", "handle")
    ]
    return world, native


def test_carried_parts_restored_after_rejected_planning_without_collision_exemptions(world):
    world, native = world
    with pytest.raises(RuntimeError, match="path rejected"):
        with temporary_carried_radio_geometry(
            world, "right_gripper_link", {"body": np.eye(4), "handle": np.eye(4)}
        ):
            assert [c.args[1] for c in native.updateGeometryPlacement.call_args_list] == [
                "right_arm_link7",
                "right_arm_link7",
            ]
            raise RuntimeError("path rejected")
    calls = native.updateGeometryPlacement.call_args_list
    np.testing.assert_allclose(calls[0].args[2][:3, 3], [-0.0295, 0, -0.16065])
    assert [(c.args[0], c.args[1]) for c in calls[2:]] == [
        ("handle", "dimos_world"),
        ("body", "dimos_world"),
    ]
    np.testing.assert_array_equal(calls[-1].args[2][:3, 3], [1, 2, 3])
    native.setCollisions.assert_not_called()
    native.removeGeometry.assert_not_called()
    assert world._usable


def test_partial_native_setup_failure_restores_every_attempted_part(world):
    world, native = world
    native.updateGeometryPlacement.side_effect = [None, RuntimeError("setup"), None, None]
    with pytest.raises(RuntimeError, match="setup"):
        with temporary_carried_radio_geometry(
            world, "right_gripper_link", {"body": np.eye(4), "handle": np.eye(4)}
        ):
            pytest.fail("partial setup must not yield")
    assert native.updateGeometryPlacement.call_count == 4
    assert world._usable


def test_restoration_failure_invalidates_world_and_still_restores_other_parts(world):
    world, native = world
    native.updateGeometryPlacement.side_effect = [None, None, RuntimeError("restore"), None]
    with pytest.raises(RuntimeError, match="world unusable"):
        with temporary_carried_radio_geometry(
            world, "right_gripper_link", {"body": np.eye(4), "handle": np.eye(4)}
        ):
            pass
    assert native.updateGeometryPlacement.call_count == 4
    assert not world._usable


@pytest.mark.parametrize("matrix", [np.full((4, 4), np.nan), np.eye(3), np.diag([-1, 1, 1, 1])])
def test_invalid_transform_rejected_before_scene_mutation(world, matrix):
    world, native = world
    with pytest.raises(ValueError, match="rigid transforms"):
        with temporary_carried_radio_geometry(world, "right_gripper_link", {"body": matrix}):
            pytest.fail("must reject")
    native.updateGeometryPlacement.assert_not_called()


def test_unregistered_part_rejected_before_scene_mutation(world):
    world, native = world
    with pytest.raises(ValueError, match="already be registered"):
        with temporary_carried_radio_geometry(world, "right_gripper_link", {"missing": np.eye(4)}):
            pytest.fail("must reject")
    native.updateGeometryPlacement.assert_not_called()
