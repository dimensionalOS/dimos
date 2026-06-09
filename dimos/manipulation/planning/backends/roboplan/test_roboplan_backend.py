# Copyright 2025-2026 Dimensional Inc.
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

from __future__ import annotations

from pathlib import Path
from types import ModuleType
from unittest.mock import patch

import pytest

from dimos.manipulation.planning.backends.registry import create_planning_backend
from dimos.manipulation.planning.backends.roboplan.backend import RoboPlanPlanningBackend
from dimos.manipulation.planning.backends.roboplan.config import parse_backend_config
from dimos.manipulation.planning.backends.roboplan.conversion import dimos_path_from_roboplan
from dimos.manipulation.planning.backends.roboplan.imports import RoboPlanBackendUnavailableError
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3


def _robot_config(model_path: Path) -> RobotModelConfig:
    orientation = Quaternion.__new__(Quaternion)
    orientation.x = 0.0
    orientation.y = 0.0
    orientation.z = 0.0
    orientation.w = 1.0
    return RobotModelConfig(
        name="arm",
        model_path=model_path,
        base_pose=PoseStamped(position=Vector3(), orientation=orientation),
        joint_names=["joint1", "joint2", "joint3"],
        end_effector_link="tool0",
        base_link="base_link",
        joint_limits_lower=[-1.0, -2.0, -3.0],
        joint_limits_upper=[1.0, 2.0, 3.0],
    )


def test_registry_preserves_drake_default_and_accepts_roboplan() -> None:
    backend = create_planning_backend("roboplan", options={})

    assert backend.name == "roboplan"


def test_registry_unknown_backend_lists_roboplan() -> None:
    with pytest.raises(ValueError, match="roboplan"):
        create_planning_backend("missing")


def test_missing_roboplan_module_error_names_selected_backend() -> None:
    from dimos.manipulation.planning.backends.roboplan.imports import load_roboplan_components

    def fail_import(module_name: str) -> ModuleType:
        raise ModuleNotFoundError(name=module_name)

    with patch("dimos.manipulation.planning.backends.roboplan.imports.import_module", fail_import):
        with pytest.raises(
            RoboPlanBackendUnavailableError, match="RoboPlan planning backend was selected"
        ):
            load_roboplan_components()


def test_parse_backend_config_requires_joint_limits_and_srdf(tmp_path: Path) -> None:
    model = tmp_path / "robot.urdf"
    backend_model = tmp_path / "backend_robot.urdf"
    srdf = tmp_path / "robot.srdf"
    model.write_text("<robot name='arm'/>")
    backend_model.write_text("<robot name='arm'/>")
    srdf.write_text("<robot name='arm'/>")

    config = parse_backend_config(
        _robot_config(model),
        {
            "model_path": str(backend_model),
            "srdf_path": str(srdf),
            "planning_group": "arm",
            "active_joint_names": ["joint2", "joint1"],
        },
    )

    assert config.model_path == backend_model
    assert config.planning_group == "arm"
    assert config.active_joint_names == ["joint2", "joint1"]
    assert config.retiming == "dimos"
    assert config.rrt.rrt_connect is True


def test_dimos_path_from_roboplan_expands_active_joint_subset() -> None:
    class NativePath:
        joint_names = ["joint2", "joint1"]
        positions = [[0.2, 0.1], [0.4, 0.3]]

    path = dimos_path_from_roboplan(
        NativePath(),
        ["joint1", "joint2", "joint3"],
        ["joint2", "joint1"],
        [9.0, 8.0, 7.0],
    )

    assert [state.position for state in path] == [[0.1, 0.2, 7.0], [0.3, 0.4, 7.0]]


def test_world_pose_targets_are_converted_to_roboplan_base_frame(tmp_path: Path) -> None:
    model = tmp_path / "robot.urdf"
    srdf = tmp_path / "robot.srdf"
    model.write_text("<robot name='arm'/>")
    srdf.write_text("<robot name='arm'/>")
    robot = _robot_config(model)
    robot.base_link = "link_base"
    robot.base_pose = PoseStamped(
        frame_id="world",
        position=Vector3(1.0, 2.0, 3.0),
        orientation=[0.0, 0.0, 0.0, 1.0],
    )
    backend = RoboPlanPlanningBackend(
        options={
            "srdf_path": str(srdf),
            "base_frame": "link_base",
            "end_effector_frame": "tool0",
        }
    )
    backend.add_robot(robot)

    converted = backend._target_pose_for_roboplan(
        PoseStamped(
            frame_id="world",
            position=Vector3(1.25, 2.5, 3.75),
            orientation=[0.0, 0.0, 0.0, 1.0],
        )
    )

    assert converted is not None
    assert converted.frame_id == "link_base"
    assert converted.position.x == pytest.approx(0.25)
    assert converted.position.y == pytest.approx(0.5)
    assert converted.position.z == pytest.approx(0.75)
