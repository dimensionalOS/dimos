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

from typing import Any, cast

from dimos.control.components import HardwareType
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import Blueprint, TransportSpec
from dimos.core.transport import LCMTransport
from dimos.hardware.whole_body.spec import WholeBodyConfig
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.robot.diy.alfred.blueprints.pillar import (
    PILLAR_SERVO_TASK_NAME,
    alfred_pillar,
)
from dimos.robot.diy.alfred.pillar_connection import (
    PILLAR_HARDWARE_ID,
    PILLAR_LIFT_JOINT,
    PillarConnection,
)


def _coordinator_kwargs(blueprint: Blueprint) -> dict[str, Any]:
    return next(
        atom.kwargs
        for atom in blueprint.blueprints
        if isinstance(atom.module, type) and issubclass(atom.module, ControlCoordinator)
    )


def test_alfred_pillar_composes_connection_and_one_joint_coordinator() -> None:
    assert any(atom.module is PillarConnection for atom in alfred_pillar.blueprints)

    kwargs = _coordinator_kwargs(alfred_pillar)
    hardware = kwargs["hardware"][0]
    tasks = cast("list[TaskConfig]", kwargs["tasks"])

    assert hardware.hardware_id == PILLAR_HARDWARE_ID
    assert hardware.hardware_type is HardwareType.WHOLE_BODY
    assert hardware.joints == [PILLAR_LIFT_JOINT]
    assert hardware.adapter_type == "transport_lcm"
    assert hardware.wb_config == WholeBodyConfig(kp=(0.0,), kd=(0.0,))
    assert tasks == [
        TaskConfig(
            name=PILLAR_SERVO_TASK_NAME,
            type="servo",
            joint_names=[PILLAR_LIFT_JOINT],
            priority=10,
            auto_start=True,
        )
    ]


def test_alfred_pillar_pins_its_public_lcm_topics() -> None:
    expected = {
        ("motor_command", MotorCommandArray): f"/{PILLAR_HARDWARE_ID}/motor_command",
        ("motor_states", JointState): f"/{PILLAR_HARDWARE_ID}/motor_states",
        ("joint_command", JointState): f"/{PILLAR_HARDWARE_ID}/joint_command",
        ("coordinator_joint_state", JointState): f"/{PILLAR_HARDWARE_ID}/joints",
    }

    assert set(alfred_pillar.transport_map) == set(expected)
    for key, topic in expected.items():
        transport = alfred_pillar.transport_map[key]
        assert isinstance(transport, TransportSpec)
        assert transport.cls is LCMTransport
        assert transport.args == (topic, key[1])
