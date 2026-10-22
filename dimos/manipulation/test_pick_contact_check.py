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

"""pick_object with the grasp contact check: the gripper's shape says which
grasps would touch the cup off the finger pads or hit the scene."""

from typing import Any
from unittest.mock import MagicMock

import numpy as np
import pytest

from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.manipulation.planning.gripper_contact import GripperContactModel

# The pick-and-place unit tests' arm and cup, and their fixtures.
from dimos.manipulation.test_pick_and_place_unit import (  # noqa: F401
    _candidate,
    module,
    settled_gripper,
)
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Header import Header


def _table() -> PointCloud2:
    """Camera points of a table top just under the cup (z = 0.175), 2 cm apart, 1 m square."""
    xs = np.arange(-0.4, 0.6, 0.02)
    grid = np.stack(np.meshgrid(xs, xs, [0.175], indexing="ij"), axis=-1).reshape(-1, 3)
    return PointCloud2.from_numpy(grid.astype(np.float32), frame_id="world", timestamp=2.0)


@pytest.fixture
def gripper(module: PickAndPlaceModule) -> MagicMock:
    """The gripper's shape on the arm's tool tip; its check says which grasps pass."""
    module.config.robot = MagicMock()
    model = MagicMock()
    model.keep_clear.side_effect = lambda *args: GripperContactModel.keep_clear(model, *args)
    module._contact_models["tool"] = model
    scene: Any = module._scene
    scene.get_full_scene_pointcloud.return_value = _table()
    module._grasp_generator.propose_grasps.return_value = GraspCandidateArray(
        Header(1.0, "world"), [_candidate(0.1, 0.9), _candidate(0.12, 0.5)]
    )
    return model


def test_pick_skips_grasps_the_contact_check_rejects(
    module: PickAndPlaceModule, gripper: MagicMock
) -> None:
    gripper.check.return_value = np.array([False, True])

    result = module.pick_object("cup-1")

    assert result.message == "Pick complete"
    assert result.metadata["score"] == 0.5
    assert result.metadata["contact_rejected"] == 1
    assert [c.score for c in module.get_grasp_candidates().candidates] == [0.5]
    poses, target, obstacles, contact_margin, obstacle_margin = gripper.check.call_args.args
    assert poses.shape == (2, 4, 4)
    assert target.shape == (2, 3)
    # The table under the cup, cropped to the gripper's reach.
    assert 0 < len(obstacles) < len(_table().points_f32())
    assert (contact_margin, obstacle_margin) == (0.005, 0.003)


def test_pick_says_when_the_contact_check_rejects_every_grasp(
    module: PickAndPlaceModule, gripper: MagicMock
) -> None:
    gripper.check.return_value = np.array([False, False])

    result = module.pick_object("cup-1")

    assert result.message == (
        "Generated 0 grasp candidates for object cup-1. The contact check dropped 2 grasp(s) "
        "whose gripper would touch the object off its finger pads or hit the scene."
    )
    assert result.metadata["contact_rejected"] == 2
    module._manipulation.plan_to_poses.assert_not_called()


def test_pick_tries_every_grasp_when_the_check_fails(
    module: PickAndPlaceModule, gripper: MagicMock
) -> None:
    gripper.check.side_effect = ValueError("bad mesh")

    result = module.pick_object("cup-1")

    assert result.metadata["score"] == 0.9
    assert result.metadata["contact_check_skipped"] == "ValueError: bad mesh"
