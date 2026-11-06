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

"""The main dual-Metal geometry and task behavior through a connection."""

from pathlib import Path
from types import SimpleNamespace

import pytest

from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.damiao._damiao_test_helpers import mock_connection_rig
from dimos.robot.manipulators.dual_makermods_metal.blueprints.basic import (
    DualMetalManipulationConfig,
    dual_makermods_metal_planner_coordinator,
)
from dimos.robot.manipulators.dual_makermods_metal.config import dual_metal_model_config
from dimos.robot.manipulators.dual_makermods_metal.connection import DualMetalConnection


def test_blueprint_parses_main_mount_options_and_required_can_ports():
    parsed = BlueprintConfigParser(dual_makermods_metal_planner_coordinator).parse(
        [
            "--left-can-port",
            "left",
            "--right-can-port",
            "right",
            "--left-base-xyz",
            "[0,0.25,0]",
            "--right-base-xyz",
            "[0,-0.25,0]",
        ],
        environ={},
    )
    connection = parsed.module_kwargs("dualmetalconnection")
    assert (connection["left_can_port"], connection["right_can_port"]) == ("left", "right")
    config = DualMetalManipulationConfig.model_validate(
        parsed.module_kwargs("dualmetalmanipulationmodule")
    )
    assert config.left_base_xyz == (0.0, 0.25, 0.0)
    assert config.right_base_xyz == (0.0, -0.25, 0.0)
    assert config.model.joint_names == [
        *[f"arm/left_joint{i}" for i in range(1, 7)],
        *[f"arm/right_joint{i}" for i in range(1, 7)],
    ]


def test_independent_grippers_through_the_coordinator(tmp_path):
    [atom] = [
        a
        for a in dual_makermods_metal_planner_coordinator.blueprints
        if issubclass(a.module, ControlCoordinator)
    ]
    coordinator = atom.module(**atom.kwargs)
    with mock_connection_rig(DualMetalConnection, coordinator, tmp_path=tmp_path) as rig:
        coordinator.task_invoke("left_arm_gripper", "set_normalized", {"values": [0.8]})
        rig.tick()
        assert rig.positions["arm/left_gripper"] == 0.8
        assert rig.positions["arm/right_gripper"] == 0.0
        coordinator.task_invoke("right_arm_gripper", "set_normalized", {"values": [0.3]})
        rig.tick()
        assert rig.positions["arm/left_gripper"] == 0.8
        assert rig.positions["arm/right_gripper"] == 0.3


def test_main_attachment_geometry_is_retained(tmp_path: Path, mocker):
    path = tmp_path / "single.urdf"
    path.write_text(
        '<robot name="metal"><link name="base_link"/><link name="tip"/>'
        '<joint name="joint1" type="revolute"><parent link="base_link"/>'
        '<child link="tip"/><limit lower="-1" upper="1" effort="1" velocity="1"/>'
        "</joint></robot>"
    )
    mocker.patch(
        "dimos.robot.manipulators.dual_makermods_metal.config.make_metal_model_config",
        return_value=SimpleNamespace(model=RobotModel.from_file(path)),
    )
    loaded = dual_metal_model_config(
        left_xyz=(0, 0.25, 0), right_xyz=(0, -0.25, 0), right_rpy=(0, 0, 1)
    ).model.load()
    assert loaded.source_path is None
    assert loaded.get_joint("left_mount").origin_xyz == (0, 0.25, 0)
    assert loaded.get_joint("right_mount").origin_rpy == (0, 0, 1)
    assert loaded.get_joint("right_joint1").parent_link == "right_base_link"


@pytest.mark.parametrize(
    "ports",
    [
        {"left_can_port": "same", "right_can_port": "same"},
        {"left_can_port": "", "right_can_port": "right"},
    ],
)
def test_invalid_dual_selectors_are_rejected(ports):
    with pytest.raises(ValueError, match="CAN selector"):
        DualMetalConnection(**ports)


def test_main_ambiguous_mounts_are_rejected():
    with pytest.raises(ValueError, match="distinct"):
        dual_metal_model_config(left_xyz=(0, 0, 0), right_xyz=(0, 0, 0))
