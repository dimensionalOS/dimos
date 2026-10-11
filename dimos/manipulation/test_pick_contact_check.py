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

"""pick_object with the grasp contact check: the manipulation module says which
grasps' grippers would touch the cup off the finger pads or hit the scene."""

from typing import Any

import pytest

from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.manipulation.planning.spec.gripper import GraspContact

# The pick-and-place unit tests' arm, cup and table, and their fixtures.
from dimos.manipulation.test_pick_and_place_unit import (  # noqa: F401
    _candidate,
    _table,
    module,
    settled_gripper,
)
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.std_msgs.Header import Header


@pytest.fixture
def contact_check(module: PickAndPlaceModule) -> PickAndPlaceModule:
    module.config.grasp_contact_check = True
    scene: Any = module._scene
    scene.get_full_scene_pointcloud.return_value = _table()
    module._grasp_generator.propose_grasps.return_value = GraspCandidateArray(
        Header(1.0, "world"), [_candidate(0.1, 0.9), _candidate(0.12, 0.5)]
    )
    return module


def test_pick_skips_grasps_the_contact_check_rejects(contact_check: PickAndPlaceModule) -> None:
    manipulation: Any = contact_check._manipulation
    manipulation.check_grasp_contacts.return_value = [
        GraspContact(("target", "table"), while_closing=True),
        GraspContact(),
    ]

    result = contact_check.pick_object("cup-1")

    assert result.message == "Pick complete"
    assert result.metadata["score"] == 0.5
    assert [c.score for c in contact_check.get_grasp_candidates().candidates] == [0.5]
    facts = result.metadata["contact_rejected"]
    assert {
        name: facts[name] for name in ("checked", "rejected", "target", "table", "objects")
    } == {
        "checked": 2,
        "rejected": 1,
        "target": 1,
        "table": 1,
        "objects": 0,
    }
    assert facts["while_closing"] == 1
    candidates, target, obstacles, contact_margin, obstacle_margin, group = (
        manipulation.check_grasp_contacts.call_args.args
    )
    assert len(candidates.candidates) == 2
    assert target.frame_id == "world"
    assert set(obstacles) == {"table", "objects"}
    assert (contact_margin, obstacle_margin, group) == (0.005, 0.003, "arm/tool")
    assert manipulation.check_grasp_contacts.call_args.kwargs == {"min_gap_points": 10}


def test_pick_counts_grasps_that_close_on_nothing(contact_check: PickAndPlaceModule) -> None:
    manipulation: Any = contact_check._manipulation
    manipulation.check_grasp_contacts.return_value = [GraspContact(empty_gap=True), GraspContact()]

    result = contact_check.pick_object("cup-1")

    assert result.metadata["score"] == 0.5
    facts = result.metadata["contact_rejected"]
    assert (facts["rejected"], facts["empty_gap"], facts["target"]) == (1, 1, 0)


def test_pick_with_the_gap_rule_off_asks_for_no_gap(contact_check: PickAndPlaceModule) -> None:
    contact_check.config.grasp_require_object_in_gap = False
    manipulation: Any = contact_check._manipulation
    manipulation.check_grasp_contacts.return_value = [GraspContact(), GraspContact()]

    result = contact_check.pick_object("cup-1")

    assert manipulation.check_grasp_contacts.call_args.kwargs == {"min_gap_points": 0}
    assert "empty_gap" not in result.metadata["contact_rejected"]


def test_pick_says_when_the_contact_check_rejects_every_grasp(
    contact_check: PickAndPlaceModule,
) -> None:
    manipulation: Any = contact_check._manipulation
    manipulation.check_grasp_contacts.return_value = [GraspContact(("target",))] * 2

    result = contact_check.pick_object("cup-1")

    assert result.message == (
        "Generated 0 grasp candidates for object cup-1. The contact check dropped 2 grasp(s) "
        "whose gripper would touch the object off its finger pads or hit the scene."
    )
    assert result.metadata["contact_rejected"]["rejected"] == 2
    manipulation.plan_to_poses.assert_not_called()


def test_pick_tries_every_grasp_without_a_gripper_spec(contact_check: PickAndPlaceModule) -> None:
    contact_check._manipulation.check_grasp_contacts.return_value = None

    result = contact_check.pick_object("cup-1")

    assert result.metadata["score"] == 0.9
    assert result.metadata["contact_check_skipped"] == "the robot model has no gripper spec"
    assert "contact_rejected" not in result.metadata


def test_pick_with_the_contact_check_off_says_so(module: PickAndPlaceModule) -> None:
    result = module.pick_object("cup-1")

    assert result.metadata["contact_check_skipped"] == "off"
    module._manipulation.check_grasp_contacts.assert_not_called()
