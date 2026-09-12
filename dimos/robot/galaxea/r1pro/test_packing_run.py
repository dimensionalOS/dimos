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

"""An ACT pick must leave all unrequested bottles undisturbed."""

from copy import deepcopy

import pytest

from dimos.robot.galaxea.r1pro.packing_checks import validate_other_bottles
from dimos.robot.galaxea.r1pro.packing_run import run_bottle_pick


@pytest.fixture
def inventory():
    return [
        {
            "bottle": i + 1,
            "bottle_position": [i * 0.1, 0.0, 0.77],
            "inside_bin": i == 1,
            "upright": True,
            "released": True,
        }
        for i in range(3)
    ]


@pytest.mark.parametrize("failure", ["wrong_pick", "tipped", "spilled", "missing"])
def test_rejects_disturbing_an_unrequested_bottle(inventory, failure):
    after = deepcopy(inventory)
    if failure == "wrong_pick":
        after[2]["bottle_position"][2] += 0.02
    elif failure == "tipped":
        after[2]["upright"] = False
    elif failure == "spilled":
        after[1]["inside_bin"] = False
    else:
        after.pop()
    with pytest.raises(RuntimeError, match="unrequested|inventory"):
        validate_other_bottles(inventory, after, selected=0)


def test_selected_bottle_can_move_while_neighbors_settle(inventory):
    after = deepcopy(inventory)
    after[0].update(bottle_position=[1.0, 1.0, 1.0], released=False)
    after[2]["bottle_position"][0] += 0.001
    assert validate_other_bottles(inventory, after, selected=0) is None


def test_wrong_object_stops_live_rollout_and_does_not_claim_success(mocker, inventory):
    sim, policy = mocker.Mock(), mocker.Mock()
    sim.select_bottle.return_value = {"selected": True}
    after = deepcopy(inventory)
    after[2]["released"] = False
    sim.packing_state.side_effect = [
        {"bottles": inventory},
        {"bottles": after, "selected": {"pick_complete": False}},
        {"bottles": inventory, "selected": {"pick_complete": False}},
    ]
    policy.preflight_rollout.return_value = {
        "policy_ready": True,
        "observations_ready": True,
        "last_error": None,
    }
    policy.start_rollout.return_value = {"active": True}
    policy.rollout_status.return_value = {"active": True, "last_error": None}
    report = {}
    with pytest.raises(RuntimeError, match="unrequested bottle_3"):
        run_bottle_pick(policy, sim, 0, report, pause=lambda _: None)
    policy.stop_rollout.assert_called_once_with()
    assert report["success"] is False
