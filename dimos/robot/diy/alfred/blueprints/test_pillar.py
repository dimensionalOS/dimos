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

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import Blueprint, TransportSpec
from dimos.core.coordination.module_coordinator import verify_no_name_conflicts
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport, ZenohTransport
from dimos.core.transport_factory import transport_topic
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.diy.alfred.blueprints.pillar import (
    PILLAR_LIFT_VELOCITY_LIMIT_M_S,
    PILLAR_TASK_NAME,
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
    tasks = cast("list[TaskConfig]", kwargs["tasks"])

    assert "hardware" not in kwargs, "the pillar is on the coordinator's ports"
    assert tasks == [
        TaskConfig(
            name=PILLAR_TASK_NAME,
            type="trajectory",
            joint_names=[PILLAR_LIFT_JOINT],
            priority=10,
            params={
                "start_position_tolerance": 0.05,
                "velocity_limits": {PILLAR_LIFT_JOINT: PILLAR_LIFT_VELOCITY_LIMIT_M_S},
            },
        )
    ]


def test_the_coordinator_reaches_the_pillar_on_its_ports() -> None:
    verify_no_name_conflicts(alfred_pillar)
    remaps = alfred_pillar.remapping_map
    connection = PillarConnection.name
    assert remaps[(connection, "position_command")] == f"{PILLAR_HARDWARE_ID}_position_command"
    assert remaps[(connection, "joint_state")] == f"{PILLAR_HARDWARE_ID}_joint_state"


def test_alfred_pillar_pins_its_public_channels_on_the_active_backend() -> None:
    """What `dimos topic` drives and reads the lift on, from another terminal."""
    expected = {
        ("joint_command", JointState): f"/{PILLAR_HARDWARE_ID}/joint_command",
        ("coordinator_joint_state", JointState): f"/{PILLAR_HARDWARE_ID}/joints",
    }
    backend = ZenohTransport if global_config.transport == "zenoh" else LCMTransport

    assert set(alfred_pillar.transport_map) == set(expected)
    for key, channel in expected.items():
        transport = alfred_pillar.transport_map[key]
        assert isinstance(transport, TransportSpec)
        assert transport.cls is backend
        assert transport.args == (transport_topic(channel), key[1])
