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

import base64
from collections.abc import Iterator
from typing import Any

import cv2
import dream
from dream.transport import Transport
import numpy as np
import pytest
from pytest_mock import MockerFixture

from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import (
    JointTrajectoryTask,
    JointTrajectoryTaskConfig,
)
from dimos.imitation.policy.dreamlabs.module import DreamlabsModule
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState
from dimos.protocol.rpc.pubsubrpc import LCMRPC
from dimos.robot.manipulators.dual_openyam.joints import DUAL_OPENYAM_JOINTS

LEFT = [0.0, 0.5, 0.5, -0.35, 0.0, 0.0]
RIGHT = [0.2, 0.6, 0.7, -0.1, 0.2, -0.3]


@pytest.fixture
def module(mocker: MockerFixture) -> Iterator[tuple[DreamlabsModule, Any]]:
    mocker.patch("dimos.core.module.get_loop", return_value=(mocker.MagicMock(), None))
    for method in ("__init__", "start", "stop", "serve_module_rpc"):
        mocker.patch.object(LCMRPC, method, return_value=None)
    request = mocker.patch.object(
        Transport,
        "request",
        return_value={
            "status": {"code": "DA-0000"},
            "actions": {
                "q_l_chunk": [[*LEFT, 0.2]] * 48,
                "q_r_chunk": [[*RIGHT, 0.8]] * 48,
                "per_key": {"action.left_arm": [LEFT] * 48, "action.right_arm": [RIGHT] * 48},
            },
        },
    )
    mocker.patch(
        "dimos.imitation.policy.dreamlabs.module.create_client",
        side_effect=lambda: dream.DreamClient(api_key="offline-fixture"),
    )
    instance = DreamlabsModule(rpc_transport=LCMRPC)
    try:
        yield instance, request
    finally:
        instance.stop()


def tick(module: DreamlabsModule, timestamp: float, red: int = 210) -> None:
    overhead = Image(
        data=np.full((240, 400, 3), [red, 0, 0], dtype=np.uint8), format=ImageFormat.RGB
    )
    left = Image(data=np.full((240, 400, 3), [0, 220, 0], dtype=np.uint8), format=ImageFormat.RGB)
    right = Image(data=np.full((240, 400, 3), [0, 0, 230], dtype=np.uint8), format=ImageFormat.RGB)
    state = JointState(
        name=list(reversed(DUAL_OPENYAM_JOINTS)), position=list(reversed([*LEFT, *RIGHT, 0.2, 0.8]))
    )
    module.set_observation(overhead, left, right, state, timestamp)


def test_real_colored_images_and_state_reach_client_without_execution(
    module: tuple[DreamlabsModule, Any], mocker: MockerFixture
) -> None:
    instance, request = module
    execute = mocker.patch.object(ControlCoordinator, "execute_trajectory")
    instance.start()
    request.assert_not_called()
    tick(instance, 10.0)
    request.assert_not_called()
    preview = instance.get_action()
    request.assert_called_once()
    sent = request.call_args.args[0]["observation"]
    for role, expected in (
        ("overhead", [0, 0, 210]),
        ("wrist_l", [0, 220, 0]),
        ("wrist_r", [230, 0, 0]),
    ):
        encoded = sent[f"video.{role}"][0]
        assert not encoded.startswith("data:")
        pixels = cv2.imdecode(
            np.frombuffer(base64.b64decode(encoded), dtype=np.uint8), cv2.IMREAD_COLOR
        )
        assert pixels.shape == (176, 320, 3)
        np.testing.assert_allclose(pixels[70, 120], expected, atol=2)
    np.testing.assert_allclose(sent["state.left_eef_xyz"], [0.262523, 0, 0.206904], atol=2e-5)
    np.testing.assert_allclose(preview.points[0].positions, [*LEFT, *RIGHT, 0.2, 0.8])
    assert preview.joint_names == DUAL_OPENYAM_JOINTS
    assert preview.duration == 2.4
    execute.assert_not_called()


def test_sdk_owns_history_and_reset(module: tuple[DreamlabsModule, Any]) -> None:
    instance, request = module
    for index in range(12):
        tick(instance, 10.0 + index * 0.2, red=index * 10)
    instance.get_action()
    history = request.call_args.args[0]["observation"]["video.overhead"]
    assert len(history) == 9
    for encoded, expected in zip(history, range(30, 120, 10), strict=True):
        pixels = cv2.imdecode(
            np.frombuffer(base64.b64decode(encoded), dtype=np.uint8), cv2.IMREAD_COLOR
        )
        np.testing.assert_allclose(pixels[70, 120], [0, 0, expected], atol=2)
    instance.reset_episode()
    with pytest.raises(RuntimeError, match="set_observation"):
        instance.get_action()
    tick(instance, 20.0)
    instance.get_action()
    assert len(request.call_args.args[0]["observation"]["video.overhead"]) == 1


def test_timeout_propagates_without_robot_commands(module: tuple[DreamlabsModule, Any]) -> None:
    instance, request = module
    tick(instance, 10.0)
    request.side_effect = dream.DreamTimeout("offline timeout")
    with pytest.raises(dream.DreamTimeout):
        instance.get_action()


def test_explicit_input_and_normal_client_cleanup(
    module: tuple[DreamlabsModule, Any], mocker: MockerFixture
) -> None:
    instance, _ = module
    client = mocker.MagicMock()
    factory = mocker.patch(
        "dimos.imitation.policy.dreamlabs.module.create_client", return_value=client
    )
    instance.start()
    factory.assert_not_called()
    with pytest.raises(RuntimeError, match="set_observation"):
        instance.get_action()
    factory.assert_not_called()
    tick(instance, 10.0)
    factory.assert_called_once()
    assert client.policy.call_args.args[0]["_record"] is False
    client.get_action.assert_not_called()
    instance.reset_episode()
    client.reset_episode.assert_called_once()
    instance.stop()
    instance.stop()
    client.close.assert_called_once()


def test_existing_coordinator_apis_execute_status_cancel(
    module: tuple[DreamlabsModule, Any], mocker: MockerFixture
) -> None:
    instance, _ = module
    tick(instance, 10.0)
    preview = instance.get_action()
    coordinator = ControlCoordinator(rpc_transport=LCMRPC)
    try:
        task = JointTrajectoryTask(JointTrajectoryTaskConfig(joint_names=list(DUAL_OPENYAM_JOINTS)))
        coordinator.add_task(task, task_type="trajectory")
        mocker.patch.object(
            coordinator,
            "get_joint_positions",
            return_value=dict(zip(DUAL_OPENYAM_JOINTS, [*LEFT, *RIGHT, 0.2, 0.8], strict=True)),
        )
        assert coordinator.execute_trajectory(preview).status.name == "ACCEPTED"
        assert (
            coordinator.task_invoke("joint_trajectory", "get_status", {"t_now": None}).state
            is TrajectoryState.EXECUTING
        )
        assert coordinator.cancel_trajectory().safe
        assert (
            coordinator.task_invoke("joint_trajectory", "get_status", {"t_now": None}).state
            is TrajectoryState.ABORTED
        )
    finally:
        coordinator.stop()
