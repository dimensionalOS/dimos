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

from typing import cast

import pytest

from dimos.control.components import HardwareType
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.global_config import global_config
from dimos.hardware.whole_body.damiao.config import DamiaoRuntimeConfig
from dimos.manipulation.planning.groups.registry import PlanningGroupRegistry
from dimos.robot.manipulators.openarm.blueprints.basic import openarm_planner_coordinator
from dimos.robot.manipulators.openarm.config import (
    OPENARM_ARM_JOINTS,
    OPENARM_BIMANUAL_MODEL,
    OPENARM_BIMANUAL_XACRO,
    OPENARM_DESCRIPTION_REF,
    OPENARM_DESCRIPTION_SOURCE,
    OPENARM_DESCRIPTION_URL,
    OPENARM_HARDWARE_ID,
    OPENARM_SIDES,
    openarm_bimanual_model_config,
    openarm_hardware,
    openarm_urdf_joints,
)


def test_openarm_model_uses_pinned_official_bimanual_xacro() -> None:
    assert OPENARM_DESCRIPTION_URL == "https://github.com/enactic/openarm_description"
    assert OPENARM_DESCRIPTION_REF == "1fba2cbc05001f05b4514120b70130b4ac06f409"
    assert OPENARM_DESCRIPTION_SOURCE.url == OPENARM_DESCRIPTION_URL
    assert OPENARM_DESCRIPTION_SOURCE.ref == OPENARM_DESCRIPTION_REF
    assert cast("tuple[str, ...]", OPENARM_BIMANUAL_XACRO.parts)[-5:] == (
        "assets",
        "robot",
        "openarm_v2.0",
        "urdf",
        "openarm_v20.urdf.xacro",
    )


def test_openarm_config_exposes_one_bimanual_robot() -> None:
    config = openarm_bimanual_model_config()
    registry = PlanningGroupRegistry([config])

    assert config.model is OPENARM_BIMANUAL_MODEL
    assert config.get_coordinator_joint_names() == OPENARM_ARM_JOINTS
    assert [group.name for group in config.planning_groups] == [
        "left_manipulator",
        "right_manipulator",
    ]
    assert [group.joint_names for group in config.planning_groups] == [
        tuple(openarm_urdf_joints(side)) for side in OPENARM_SIDES
    ]
    assert config.collision_exclusion_pairs == [
        ("openarm_left_ee_link1", "openarm_left_ee_link2"),
        ("openarm_right_ee_link1", "openarm_right_ee_link2"),
    ]
    assert [group.id for group in registry.list()] == [
        "openarm/left_manipulator",
        "openarm/right_manipulator",
    ]


def test_openarm_hardware_defaults_to_mock_without_can_ports(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(global_config, "left_can_port", None)
    monkeypatch.setattr(global_config, "right_can_port", None)

    hardware = openarm_hardware()

    assert (hardware.hardware_id, hardware.hardware_type, hardware.adapter_type) == (
        OPENARM_HARDWARE_ID,
        HardwareType.WHOLE_BODY,
        "mock_whole_body",
    )
    assert hardware.adapter_kwargs == {}


def test_openarm_hardware_uses_physical_adapter_with_explicit_can_ports(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(global_config, "left_can_port", "can1")
    monkeypatch.setattr(global_config, "right_can_port", "can0")

    hardware = openarm_hardware()

    assert hardware.adapter_type == "openarm_damiao"
    runtime_config = hardware.adapter_kwargs["runtime_config"]
    assert isinstance(runtime_config, DamiaoRuntimeConfig)
    assert runtime_config.bus_addresses == {"left": "can1", "right": "can0"}


@pytest.mark.parametrize(
    ("left_can_port", "right_can_port"),
    [("can1", None), (None, "can0")],
)
def test_openarm_hardware_rejects_partial_can_configuration(
    monkeypatch: pytest.MonkeyPatch,
    left_can_port: str | None,
    right_can_port: str | None,
) -> None:
    monkeypatch.setattr(global_config, "left_can_port", left_can_port)
    monkeypatch.setattr(global_config, "right_can_port", right_can_port)

    with pytest.raises(ValueError, match="requires both left and right CAN ports"):
        openarm_hardware()


def test_openarm_can_ports_are_blueprint_cli_options() -> None:
    parsed = BlueprintConfigParser(openarm_planner_coordinator).parse(
        ["--left-can-port", "can1", "--right-can-port", "can0"],
        environ={},
    )

    config = parsed.global_config_values()
    assert config["left_can_port"] == "can1"
    assert config["right_can_port"] == "can0"
