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

"""Navigation rejects obstructed routes and stops base commands on cargo failure."""

import pytest

from dimos.robot.galaxea.r1pro.navigation_delivery import (
    follow_navigation_path,
    prepare_navigation_map,
)
from dimos.robot.galaxea.r1pro.navigation_sim import NAV_TASK


@pytest.fixture
def navigation_state():
    return {
        "base_pose": [0.0, 0.0, 0.0],
        "sim_time": 10.0,
        "obstacles": [],
        "inside_bin": True,
        "upright": True,
        "tray": {"support_geoms": [], "bimanual_grasp": True, "tilt_radians": 0.0},
    }


def test_obstructed_route_never_dispatches_a_control_task(mocker):
    sim, control = mocker.Mock(), mocker.Mock()
    sim.validate_navigation_path.side_effect = RuntimeError("obstructed")
    with pytest.raises(RuntimeError, match="obstructed"):
        follow_navigation_path(control, sim, [[0, 0, 0], [1, 0, 0]], "carry", {"stages": []})
    control.task_invoke.assert_not_called()


@pytest.mark.parametrize("failure", ["bottle", "grasp", "obstacle"])
def test_cargo_failure_cancels_task_and_holds_measured_base(mocker, navigation_state, failure):
    sim, control = mocker.Mock(), mocker.Mock()
    state = navigation_state
    if failure == "bottle":
        state["inside_bin"] = False
    elif failure == "grasp":
        state["tray"]["bimanual_grasp"] = False
    else:
        state["obstacles"] = ["cabinet"]
    sim.task_state.return_value = state
    with pytest.raises(RuntimeError, match="obstacle|grasp"):
        follow_navigation_path(control, sim, [[0, 0, 0], [1, 0, 0]], "carry", {"stages": []})
    assert control.task_invoke.call_args == mocker.call(NAV_TASK, "cancel", {})
    sim.stop_navigation_base.assert_called_once_with()


def test_map_must_be_acknowledged_and_is_not_rebuilt_for_each_stage(mocker, tmp_path):
    sim = mocker.Mock()
    sim.navigation_status.side_effect = [
        {"surface_points": 0},
        {"surface_points": 100},
        {"surface_points": 100},
    ]
    cloud = tmp_path / "house.npy"
    prepare_navigation_map(sim, cloud)
    prepare_navigation_map(sim, cloud)
    sim.publish_navigation_map.assert_called_once_with(str(cloud.resolve()))
