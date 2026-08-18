# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

from typing import Any, cast

import pytest
from pytest_mock import MockerFixture

from dimos.control.tasks.teleop_ik_task.teleop_ik_task import TeleopIKTask
from dimos.control.tick_loop import TickLoop
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import Blueprint
from dimos.hardware.whole_body.spec import WholeBodyAdapter
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.dual_openyam.blueprints.basic import (
    DualOpenYamCoordinator,
)
from dimos.robot.manipulators.dual_openyam.blueprints.teleop import (
    DUAL_OPENYAM_QUEST_TASK_NAME,
    teleop_quest_dual_openyam,
)
from dimos.robot.manipulators.dual_openyam.config import (
    DUAL_OPENYAM_ARM_JOINTS,
    DUAL_OPENYAM_GRIPPER_JOINTS,
)
from dimos.robot.manipulators.dual_openyam.teleop_ik import (
    DualOpenYamPinkPoseTargetSolver,
)
from dimos.teleop.quest.quest_types import Buttons


def _module_kwargs(blueprint: Blueprint, module_type: type) -> dict[str, Any]:
    return next(atom.kwargs for atom in blueprint.blueprints if atom.module is module_type)


def test_quest_blueprint_selects_physical_hardware_from_both_can_ports() -> None:
    parsed = BlueprintConfigParser(teleop_quest_dual_openyam).parse(
        [
            "--left-can-port",
            "follower_l",
            "--right-can-port",
            "follower_r",
            "--manipulationmodule.visualization.host=0.0.0.0",
        ],
        environ={},
    )

    coordinator = parsed.module_kwargs("ControlCoordinator")
    assert coordinator["left_can_port"] == "follower_l"
    assert coordinator["right_can_port"] == "follower_r"
    assert parsed.module_kwargs("manipulationmodule")["visualization"]["host"] == "0.0.0.0"


def test_mock_quest_coordinator_commands_both_arms_and_grippers(
    mocker: MockerFixture,
) -> None:
    kwargs = _module_kwargs(teleop_quest_dual_openyam, DualOpenYamCoordinator)
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
        return_value=JointState(name=DUAL_OPENYAM_ARM_JOINTS, position=[0.01] * 12),
    )
    mocker.patch.object(TickLoop, "start")
    coordinator = DualOpenYamCoordinator(publish_joint_state=False, **kwargs)

    coordinator.start()
    try:
        task = cast("TeleopIKTask", coordinator._tasks[DUAL_OPENYAM_QUEST_TASK_NAME])
        assert set(task.claim().joints) == set(
            DUAL_OPENYAM_ARM_JOINTS + DUAL_OPENYAM_GRIPPER_JOINTS
        )
        buttons = Buttons()
        buttons.left_primary = True
        buttons.right_primary = True
        buttons.pack_analog_triggers(left=0.25, right=0.75)
        coordinator._dispatch("teleop_buttons", buttons)
        coordinator._dispatch(
            "left_cartesian_command",
            PoseStamped(frame_id=DUAL_OPENYAM_QUEST_TASK_NAME, position=[1.0, 0.0, 0.0]),
        )
        coordinator._dispatch(
            "right_cartesian_command",
            PoseStamped(frame_id=DUAL_OPENYAM_QUEST_TASK_NAME, position=[-1.0, 0.0, 0.0]),
        )
        assert coordinator._tick_loop is not None
        coordinator._tick_loop._tick()

        adapter = cast("WholeBodyAdapter", coordinator._hardware["dual_openyam"].adapter)
        states = adapter.read_motor_states()
        assert [state.q for state in states[:12]] == [0.01] * 12
        assert [state.q for state in states[12:]] == pytest.approx([0.75, 0.25], abs=0.01)
    finally:
        coordinator.stop()
