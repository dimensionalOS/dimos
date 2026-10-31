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
from typing import Any, cast

import pytest
from pytest_mock import MockerFixture

from dimos.control.tasks.teleop_ik_task.teleop_ik_task import TeleopIKTask
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import Blueprint
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.robot.manipulators.common._damiao_test_helpers import mock_connection_rig
from dimos.robot.manipulators.common.damiao_blueprint import DamiaoTeleopCoordinator
from dimos.robot.manipulators.dual_openyam.blueprints.teleop import (
    _dual_openyam_webxr_tasks,
    teleop_webxr_dual_openyam,
)
from dimos.robot.manipulators.dual_openyam.config import (
    DUAL_OPENYAM_HOME_PER_ARM,
    dual_openyam_arm_joints,
)
from dimos.robot.manipulators.dual_openyam.connection import DualOpenYamConnection
from dimos.robot.manipulators.dual_openyam.teleop_ik import (
    DualOpenYamPinkPoseTargetSolver,
)
from dimos.teleop.webxr.controller_types import Buttons


def _module_kwargs(blueprint: Blueprint, module_type: type) -> dict[str, Any]:
    return next(atom.kwargs for atom in blueprint.blueprints if atom.module is module_type)


def test_webxr_blueprint_selects_physical_hardware_from_both_can_ports() -> None:
    parsed = BlueprintConfigParser(teleop_webxr_dual_openyam).parse(
        [
            "--left-can-port",
            "follower_l",
            "--right-can-port",
            "follower_r",
            "--manipulationmodule.visualization.host=0.0.0.0",
        ],
        environ={},
    )

    connection = parsed.module_kwargs("dualopenyamconnection")
    assert connection["left_can_port"] == "follower_l"
    assert connection["right_can_port"] == "follower_r"
    assert parsed.module_kwargs("manipulationmodule")["visualization"]["host"] == "0.0.0.0"


@pytest.mark.parametrize("engaged", ["left", "right"])
def test_mock_webxr_grip_engages_only_its_own_arm(
    mocker: MockerFixture, tmp_path: Path, engaged: str
) -> None:
    kwargs = _module_kwargs(teleop_webxr_dual_openyam, DamiaoTeleopCoordinator)
    mocker.patch.object(DualOpenYamPinkPoseTargetSolver, "_validate_frame_targets")
    mocker.patch.object(
        DualOpenYamPinkPoseTargetSolver,
        "frame_poses",
        return_value={
            "left_grasp_frame": PoseStamped(position=[0.5, 0.2, 0.4]),
            "right_grasp_frame": PoseStamped(position=[0.5, -0.2, 0.4]),
        },
    )
    mocker.patch.object(
        DualOpenYamPinkPoseTargetSolver,
        "step",
        side_effect=lambda targets, seed, dt: JointState(name=seed.name, position=[0.01] * 6),
    )
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.start")
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.serve_module_rpc")
    coordinator = DamiaoTeleopCoordinator(publish_joint_state=False, **kwargs)

    with mock_connection_rig(
        DualOpenYamConnection, coordinator, tmp_path=tmp_path, mocker=mocker
    ) as rig:
        for side, config in _dual_openyam_webxr_tasks.items():
            task = cast("TeleopIKTask", coordinator._tasks[config.name])
            assert set(task.claim().joints) == {
                f"arm/{name}" for name in dual_openyam_arm_joints(side)
            }
        buttons = Buttons()
        setattr(buttons, f"{engaged}_grip", True)
        rig.inputs["teleop_buttons"].broadcast(None, buttons)
        rig.inputs["left_gripper_command"].broadcast(None, Float32(data=0.75))
        rig.inputs["right_gripper_command"].broadcast(None, Float32(data=0.25))
        rig.inputs["left_cartesian_command"].broadcast(
            None,
            PoseStamped(position=[1.0, 0.0, 0.0]),
        )
        rig.inputs["right_cartesian_command"].broadcast(
            None,
            PoseStamped(position=[-1.0, 0.0, 0.0]),
        )
        rig.tick()

        positions = rig.positions
        idle = "right" if engaged == "left" else "left"
        assert [positions[f"arm/{engaged}_joint{i}"] for i in range(1, 7)] == [0.01] * 6
        assert [positions[f"arm/{idle}_joint{i}"] for i in range(1, 7)] == pytest.approx(
            DUAL_OPENYAM_HOME_PER_ARM
        )
        assert [positions["arm/left_gripper"], positions["arm/right_gripper"]] == pytest.approx(
            [0.75, 0.25], abs=0.01
        )
