# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Dual OpenYAM blueprint driven by a remote OpenPI websocket policy."""

from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.operator_hold_task.operator_hold_task import operator_hold_task
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.models.openpi.camera_bridge import OpenPIYAMCameraBridge
from dimos.models.openpi.client import OpenPIClient
from dimos.robot.manipulators.dual_openyam.blueprints.basic import (
    DualOpenYamCoordinator,
    dual_openyam_trajectory_task,
)
from dimos.robot.manipulators.dual_openyam.joints import DUAL_OPENYAM_GRIPPER_JOINTS


def build_openpi_dual_openyam(
    *,
    policy_host: str = "127.0.0.1",
    policy_port: int = 8000,
    prompt: str = "pick up the red cube and place it in the yellow bin",
    left_can_port: str = "can0",
    right_can_port: str = "can1",
) -> Blueprint:
    """Build the real-camera, dual-CAN OpenPI deployment."""
    cameras = OpenPIYAMCameraBridge.blueprint(
        top_device="/dev/video10",
        left_wrist_device="/dev/video16",
        right_wrist_device="/dev/video4",
        width=640,
        height=480,
        fps=15.0,
    )
    coordinator = DualOpenYamCoordinator.blueprint(
        instance_name="ControlCoordinator",
        left_can_port=left_can_port,
        right_can_port=right_can_port,
        tasks=[
            dual_openyam_trajectory_task(priority=10),
            operator_hold_task(),
            TaskConfig(
                name="left_gripper",
                type="gripper",
                joint_names=[DUAL_OPENYAM_GRIPPER_JOINTS[0]],
                priority=10,
                stream_bind={"gripper_command": "left_gripper_command"},
            ),
            TaskConfig(
                name="right_gripper",
                type="gripper",
                joint_names=[DUAL_OPENYAM_GRIPPER_JOINTS[1]],
                priority=10,
                stream_bind={"gripper_command": "right_gripper_command"},
            ),
        ],
    )
    policy = OpenPIClient.blueprint(host=policy_host, port=policy_port, prompt=prompt)
    return autoconnect(cameras, coordinator, policy).remappings(
        [
            (OpenPIClient, "arm_command", "joint_command"),
        ]
    )


openpi_dual_openyam = autoconnect(build_openpi_dual_openyam())
