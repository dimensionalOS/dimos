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

from types import SimpleNamespace

import pytest

from dimos.robot.manipulators.dual_openyam.sim_demo import (
    SimDemoSkills,
    inside_bin,
    planning_group_for_y,
)


@pytest.mark.parametrize(
    ("y", "group"),
    [(0.2, "left_manipulator"), (0.0, "right_manipulator"), (-0.2, "right_manipulator")],
)
def test_arm_choice_follows_object_side(y, group):
    assert planning_group_for_y(y) == group


def test_bin_success_requires_object_extent_inside_after_release():
    container = [[0.5, -0.1, 0.75], [0.7, 0.1, 0.95]]
    assert inside_bin([[0.55, -0.02, 0.77], [0.65, 0.02, 0.93]], container)
    assert not inside_bin([[0.55, -0.02, 0.9], [0.65, 0.02, 1.06]], container)
    assert not inside_bin([[0.65, -0.02, 0.77], [0.75, 0.02, 0.93]], container)


@pytest.fixture
def reset_rig(mocker):
    module = SimDemoSkills()
    module._sim = mocker.Mock()
    module._sim.reset.return_value = True
    module._pick = mocker.Mock()
    module._manipulation = mocker.Mock()
    for method in ("set_gripper_position", "plan_to_joints", "execute"):
        getattr(module._manipulation, method).return_value = SimpleNamespace(succeeded=True)
    clock = SimpleNamespace(now=0.0)

    def advance(seconds):
        clock.now += seconds

    mocker.patch("dimos.robot.manipulators.dual_openyam.sim_demo.time.monotonic", lambda: clock.now)
    mocker.patch("dimos.robot.manipulators.dual_openyam.sim_demo.time.sleep", advance)
    yield module, clock
    module.stop()


def test_reset_waits_for_bottles_to_stop_moving_before_a_scan_can_start(reset_rig):
    module, clock = reset_rig

    def poses(names):
        height = max(0.75, 0.80 - 0.1 * clock.now)
        return {name: [0.5, 0.0, height, 0.0, 0.0, 0.0, 1.0] for name in names}

    module._sim.get_body_poses.side_effect = poses

    result = module.reset_scene()

    assert result.success
    assert 0.75 <= clock.now < 1.0
    module._pick.reset_selection.assert_called_once_with()


def test_reset_reports_failure_if_the_scene_keeps_moving(reset_rig):
    module, clock = reset_rig

    def poses(names):
        return {name: [0.5, 0.0, clock.now, 0.0, 0.0, 0.0, 1.0] for name in names}

    module._sim.get_body_poses.side_effect = poses

    result = module.reset_scene()

    assert not result.success
    assert result.error_code == "EXECUTION_FAILED"
    assert "settle" in result.message
    assert clock.now >= 3.0
