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
from typing import Any

import pytest

from dimos.control.connection.mock_joint_connection import MockJointConnection
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import Blueprint
from dimos.core.global_config import global_config
from dimos.robot.all_blueprints import all_blueprints
from dimos.robot.manipulators.dual_openyam.blueprints import leader_teleop
from dimos.robot.manipulators.dual_openyam.config import (
    DUAL_OPENYAM_HOME_PER_ARM,
    dual_openyam_model_config,
)
from dimos.robot.manipulators.dual_openyam.connection import (
    DualOpenYamConnection,
    DualOpenYamLeaderConnection,
)
from dimos.teleop.leader.mapping import LEADER_SIDES

_MODEL = dual_openyam_model_config(robot="arm")
# The connection has no mock fallback, so every parse needs both selectors.
_CAN_PORTS = ["--left-can-port=follower_l", "--right-can-port=follower_r"]


def _atom(blueprint: Blueprint, module_type: type) -> Any:
    return next(atom for atom in blueprint.blueprints if atom.module is module_type)


def test_mappings_drive_every_planning_model_joint_in_order() -> None:
    assert leader_teleop.DUAL_OPENYAM_LEADER_JOINTS == list(_MODEL.joint_names)
    for side, group in zip(LEADER_SIDES, _MODEL.planning_groups, strict=True):
        mapping = leader_teleop.MAPPINGS[side]
        assert mapping.joint_names() == tuple(group.joint_names)
        assert mapping.leader_joint_names() == tuple(
            f"leader/{side}_joint_{index}" for index in range(1, 8)
        )


@pytest.mark.self_hosted
def test_mapping_limits_match_the_urdf() -> None:
    loaded = _MODEL.model.load()
    for mapping in leader_teleop.MAPPINGS.values():
        for joint_mapping in mapping.joints:
            joint = loaded.get_joint(joint_mapping.joint)
            assert joint is not None
            assert joint.lower == pytest.approx(joint_mapping.lower, abs=1e-4)
            assert joint.upper == pytest.approx(joint_mapping.upper, abs=1e-4)


def test_serial_ports_from_the_command_line_select_the_leaders() -> None:
    blueprint = leader_teleop.dual_openyam_leader_teleop
    parsed = BlueprintConfigParser(blueprint).parse(
        ["--right-serial-port=/dev/ttyACM1", *_CAN_PORTS], environ={}
    )
    leaders = DualOpenYamLeaderConnection(
        **parsed.module_kwargs(_atom(blueprint, DualOpenYamLeaderConnection).name)
    )
    try:
        assert leaders.serial_ports() == {"left": "", "right": "/dev/ttyACM1"}
    finally:
        leaders.stop()
    kwargs = parsed.module_kwargs(_atom(blueprint, DualOpenYamConnection).name)
    assert (kwargs["left_can_port"], kwargs["right_can_port"]) == ("follower_l", "follower_r")


@pytest.mark.self_hosted
def test_simulation_mocks_the_leaders_and_the_can_buses(mocker: Any) -> None:
    mocker.patch.object(global_config, "simulation", "mujoco")
    (atom,) = leader_teleop.dual_openyam_leaders().blueprints
    mock_positions = atom.kwargs["mock_positions"]
    for side in LEADER_SIDES:
        home = [mock_positions[f"{side}_joint_{index}"] for index in range(1, 7)]
        assert home == DUAL_OPENYAM_HOME_PER_ARM

    control = leader_teleop.dual_openyam_leader_control()
    assert not any(issubclass(atom.module, DualOpenYamConnection) for atom in control.blueprints)
    mock = _atom(control, MockJointConnection)
    assert mock.kwargs["source"] == "arm"
    assert control.remapping_map[(mock.name, "position_command")] == "arm_position_command"


def test_blueprint_is_registered() -> None:
    assert all_blueprints["dual-openyam-leader-teleop"].endswith(
        "leader_teleop:dual_openyam_leader_teleop"
    )
