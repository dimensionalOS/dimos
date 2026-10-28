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


from pathlib import Path

import pytest

from dimos.control.coordinator import ControlCoordinator
from dimos.control.hardware_interface import ConnectedWholeBody
from dimos.hardware.whole_body.mock.adapter import MockWholeBodyAdapter
from dimos.robot.assets.model import LoadedRobotModel, RobotModel
from dimos.robot.manipulators.dual_openyam.blueprints.basic import DualOpenYamCoordinator
from dimos.robot.manipulators.dual_openyam.joints import DUAL_OPENYAM_ARM_JOINTS


def test_joint_offset_handoff_pauses_and_resumes_without_deactivating_hardware(mocker):
    coordinator = DualOpenYamCoordinator(publish_joint_state=False)
    loop = mocker.Mock()
    loop.is_running = True
    loop._tick_thread.is_alive.return_value = False
    loop.stop.side_effect = lambda: setattr(loop, "is_running", False)
    loop.start.side_effect = lambda: setattr(loop, "is_running", True)
    coordinator._tick_loop = loop
    hardware = mocker.Mock()
    coordinator._hardware["dual_openyam"] = hardware
    positions = {name: 0.2 for name in DUAL_OPENYAM_ARM_JOINTS}
    mocker.patch.object(coordinator, "get_joint_positions", return_value=positions)

    try:
        assert coordinator.pause_for_joint_offset_test()
        loop.stop.assert_called_once_with()
        hardware.adapter.deactivate.assert_not_called()

        assert coordinator.resume_after_joint_offset_test()
        loop.start.assert_called_once_with()
        hardware.adapter.deactivate.assert_not_called()
        left = coordinator._tasks["joint_offset_handoff_left"]
        right = coordinator._tasks["joint_offset_handoff_right"]
        assert left.is_active() and right.is_active()
        assert left.compute(mocker.Mock()).positions == [0.2] * 6
        left.on_preempted("teleop_dual_openyam_left", frozenset({"left_joint1"}))
        assert not left.is_active() and right.is_active()
    finally:
        coordinator.stop()


@pytest.mark.parametrize("ports", [{}, {"left_can_port": "left", "right_can_port": "right"}])
def test_setup_supplies_model_limits_to_hardware(mocker, ports):
    names = DUAL_OPENYAM_ARM_JOINTS
    # Different bounds and reverse document order catch accidental positional mapping.
    bounds = {name: (-float(i + 1), float(i + 2)) for i, name in enumerate(names)}
    xml = (
        '<robot name="test"><link name="base"/>'
        + "".join(
            f'<link name="{name}_link"/><joint name="{name}" type="revolute">'
            f'<parent link="base"/><child link="{name}_link"/>'
            f'<limit lower="{bounds[name][0]}" upper="{bounds[name][1]}"/></joint>'
            for name in reversed(names)
        )
        + "</robot>"
    )
    mocker.patch.object(
        RobotModel, "load", return_value=LoadedRobotModel(xml, Path("model.urdf"), {})
    )
    mocker.patch.object(ControlCoordinator, "_setup_from_config")
    coordinator = DualOpenYamCoordinator(**ports)
    try:
        coordinator._setup_from_config()
        component = coordinator.config.hardware[0]
        adapter = MockWholeBodyAdapter(dof=len(component.joints))
        hardware = ConnectedWholeBody(adapter, component)
        limits = hardware.get_limits()
        assert limits is not None
        assert limits.position_lower == [*[bounds[name][0] for name in names], 0.0, 0.0]
        assert limits.position_upper == [*[bounds[name][1] for name in names], 1.0, 1.0]
    finally:
        coordinator.stop()
