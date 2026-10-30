# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

import threading
import time

import numpy as np
import pytest

from dimos.models.openpi.client import OpenPIClient
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint


@pytest.fixture
def client():
    module = OpenPIClient(prompt="pick up the red cube")
    try:
        yield module
    finally:
        module.stop()


def test_infer_reorders_coordinator_state_to_checkpoint_layout(client, mocker):
    actions = np.arange(16 * 14, dtype=np.float32).reshape(16, 14)
    policy = mocker.Mock()
    policy.infer.return_value = {"actions": actions}
    mocker.patch.object(client, "_client", policy)
    image = Image.from_numpy(np.zeros((4, 6, 3), dtype=np.uint8), format=ImageFormat.RGB)
    names = [f"{side}_joint{i}" for side in ("left", "right") for i in range(1, 7)]
    names += ["left_arm/gripper", "right_arm/gripper"]
    state = JointState(name=names, position=list(range(14)))

    trajectory = client._infer({"top": image, "left": image, "right": image, "state": state})

    observation = policy.infer.call_args.args[0]
    np.testing.assert_array_equal(
        observation["state"], [0, 1, 2, 3, 4, 5, 12, 6, 7, 8, 9, 10, 11, 13]
    )
    assert observation["images"]["cam_high"].shape == (3, 224, 224)
    assert observation["prompt"] == "pick up the red cube"
    assert trajectory.num_points == 16
    assert trajectory.points[0].positions == list(range(14))
    assert trajectory.points[-1].time_from_start == 0.5


def test_commands_are_serializable_and_separate_grippers(client, mocker):
    arm = mocker.patch.object(client.arm_command, "publish")
    left = mocker.patch.object(client.left_gripper_command, "publish")
    right = mocker.patch.object(client.right_gripper_command, "publish")
    mocker.patch.object(client._stop, "wait", return_value=False)
    positions = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.25, -0.1, -0.2, -0.3, -0.4, -0.5, -0.6, 0.75]
    trajectory = JointTrajectory(points=[TrajectoryPoint(positions=positions)])

    client._publish_commands(trajectory)

    arm.assert_called_once()
    command = arm.call_args.args[0]
    assert command.name == [f"{side}_joint{i}" for side in ("left", "right") for i in range(1, 7)]
    assert command.position == positions[:6] + positions[7:13]
    assert command.lcm_encode()
    assert left.call_args.args[0].data == 0.25
    assert right.call_args.args[0].data == 0.75


def test_pause_finishes_chunk_and_blocks_next_inference_until_resume(client, mocker):
    chunk_started = threading.Event()
    finish_chunk = threading.Event()
    second_inference = threading.Event()
    calls = 0

    def infer(_values):
        nonlocal calls
        calls += 1
        if calls == 2:
            second_inference.set()
        return JointTrajectory(points=[TrajectoryPoint(positions=[0.0] * 14)])

    def publish(_trajectory):
        chunk_started.set()
        assert finish_chunk.wait(2.0)

    mocker.patch.object(client, "_infer", side_effect=infer)
    mocker.patch.object(client, "_publish_commands", side_effect=publish)
    client._latest = {key: object() for key in ("top", "left", "right", "state")}
    client._received_at = {key: time.monotonic() for key in client._latest}
    client._worker = threading.Thread(target=client._run)
    client._worker.start()
    client._wake.set()
    assert chunk_started.wait(2.0)
    assert client.pause()
    assert not client.is_paused()
    finish_chunk.set()
    deadline = time.monotonic() + 2.0
    while not client.is_paused() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert client.is_paused()
    assert calls == 1
    assert not second_inference.is_set()
    client._received_at = {key: time.monotonic() for key in client._latest}
    assert client.resume()
    assert second_inference.wait(2.0)
