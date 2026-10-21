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

from collections.abc import Iterator
from threading import Event
import time
from typing import Any

import numpy as np
import pytest
from pytest_mock import MockerFixture
import reactivex as rx
from reactivex.subject import Subject

from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryCancellationResult,
    TrajectoryCancellationStatus,
    TrajectoryExecutionResult,
    TrajectoryExecutionStatus,
)
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.imitation.policy.dreamlabs.rollout import DreamlabsRolloutModule
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState, TrajectoryStatus
from dimos.protocol.rpc.pubsubrpc import LCMRPC
from dimos.robot.manipulators.dual_openyam.blueprints.dreamlabs import (
    dreamlabs_dual_openyam_manual,
)
from dimos.robot.manipulators.dual_openyam.joints import DUAL_OPENYAM_JOINTS

RolloutFixture = tuple[DreamlabsRolloutModule, Any, Event, Event, list[Subject[Any]]]

POSITIONS = [0.0, 0.2, 0.0, 0.0, 0.0, 0.0] * 2 + [0.5, 0.5]


@pytest.fixture
def rollout(mocker: MockerFixture) -> Iterator[RolloutFixture]:
    for method in ("__init__", "start", "stop", "serve_module_rpc"):
        mocker.patch.object(LCMRPC, method, return_value=None)
    mocker.patch("dimos.imitation.policy.dreamlabs.rollout.OBSERVATION_PERIOD", 0.01)
    mocker.patch("dimos.imitation.policy.dreamlabs.rollout.OBSERVATION_WINDOW", 2)
    client = mocker.MagicMock()
    entered, release = Event(), Event()

    def infer() -> dict[str, Any]:
        entered.set()
        assert release.wait(5), "test inference was not released"
        return {
            "q_l_chunk": [[*POSITIONS[:6], 0.5]] * 48,
            "q_r_chunk": [[*POSITIONS[6:12], 0.5]] * 48,
        }

    client.get_action.side_effect = infer
    mocker.patch("dimos.imitation.policy.dreamlabs.module.create_client", return_value=client)
    instance = DreamlabsRolloutModule(rpc_transport=LCMRPC)
    instance._control = mocker.MagicMock()
    instance._control.cancel_trajectory.return_value = TrajectoryCancellationResult(
        TrajectoryCancellationStatus.CANCELLED
    )
    instance._control.execute_trajectory.return_value = TrajectoryExecutionResult(
        TrajectoryExecutionStatus.ACCEPTED
    )
    instance._control.task_invoke.return_value = TrajectoryStatus(state=TrajectoryState.EXECUTING)
    preview = mocker.patch.object(instance.policy_preview, "publish")
    preview.side_effect = lambda _: None
    subjects = [Subject() for _ in range(4)]
    for port, subject in zip(
        (
            instance.overhead_image,
            instance.left_wrist_image,
            instance.right_wrist_image,
            instance.coordinator_joint_state,
        ),
        subjects,
        strict=True,
    ):
        mocker.patch.object(port, "observable", return_value=subject)

    def feed(_: int) -> None:
        now = time.time()
        for subject, color in zip(
            subjects[:3], ([200, 0, 0], [0, 210, 0], [0, 0, 220]), strict=True
        ):
            subject.on_next(
                Image(
                    data=np.full((8, 8, 3), color, dtype=np.uint8), format=ImageFormat.RGB, ts=now
                )
            )
        subjects[3].on_next(JointState(name=DUAL_OPENYAM_JOINTS, position=POSITIONS, ts=now))

    instance.start()
    feeding = rx.interval(0.01).subscribe(feed)
    try:
        yield instance, client, entered, release, subjects
    finally:
        feeding.dispose()
        release.set()
        instance.stop()
        for subject in subjects:
            subject.dispose()


def test_startup_idle_and_preview_never_dispatches(
    rollout: RolloutFixture,
    mocker: MockerFixture,
) -> None:
    instance, client, entered, release, _ = rollout
    assert instance.rollout_status()["active"] is False
    client.get_action.assert_not_called()
    previewed = Event()
    mocker.patch.object(instance.policy_preview, "publish", side_effect=lambda _: previewed.set())
    instance.start_rollout()
    assert entered.wait(5)
    assert client.set_observation.call_args.args[1]["prompt"] == "fold the towel"
    release.set()
    assert previewed.wait(5)
    instance.stop_rollout()
    instance._control.execute_trajectory.assert_not_called()
    instance._control.cancel_trajectory.assert_not_called()


@pytest.mark.parametrize("change_prompt", [False, True])
def test_stop_or_prompt_change_discards_inflight_chunk(
    rollout: RolloutFixture,
    change_prompt: bool,
) -> None:
    instance, client, entered, release, _ = rollout
    instance.start_rollout(dispatch_actions=True)
    assert entered.wait(5)
    status = instance.set_prompt("pick up the cloth") if change_prompt else instance.stop_rollout()
    assert status["active"] is False
    assert status["busy"] is True
    with pytest.raises(RuntimeError, match="draining"):
        instance.start_rollout()
    worker = instance._worker
    assert worker is not None
    release.set()
    worker.result(timeout=5)
    instance._control.execute_trajectory.assert_not_called()
    instance.policy_preview.publish.assert_not_called()
    client.reset_episode.assert_called()
    assert instance.rollout_status()["busy"] is False
    assert instance.rollout_status()["prompt"] == (
        "pick up the cloth" if change_prompt else "fold the towel"
    )


def test_explicit_dispatch_uses_coordinator_and_cancels(
    rollout: RolloutFixture,
    mocker: MockerFixture,
) -> None:
    instance, _, entered, release, _ = rollout
    executed = Event()

    def execute(trajectory: Any) -> TrajectoryExecutionResult:
        assert trajectory.joint_names == DUAL_OPENYAM_JOINTS
        assert trajectory.points[0].positions == POSITIONS
        executed.set()
        return TrajectoryExecutionResult(TrajectoryExecutionStatus.ACCEPTED)

    instance._control.execute_trajectory.side_effect = execute
    instance.start_rollout(dispatch_actions=True)
    assert entered.wait(5)
    release.set()
    assert executed.wait(5)
    instance.stop_rollout()
    instance._control.cancel_trajectory.assert_called_once()
    assert instance.rollout_status()["chunks_executed"] == 1


def test_out_of_model_chunk_fails_without_clipping(
    rollout: RolloutFixture,
    mocker: MockerFixture,
) -> None:
    instance, client, entered, release, _ = rollout
    client.get_action.side_effect = None
    client.get_action.return_value = {
        "q_l_chunk": [[0.0, -0.01749, 0.0, 0.0, 0.0, 0.0, 0.5]] * 48,
        "q_r_chunk": [[*POSITIONS[6:12], 0.5]] * 48,
    }
    failed = Event()
    original = instance.stop_rollout

    def stopped() -> Any:
        result = original()
        failed.set()
        return result

    mocker.patch.object(instance, "stop_rollout", side_effect=stopped)
    instance.start_rollout(dispatch_actions=True)
    assert failed.wait(5)
    assert "left_joint2 targets exceed model limits" in instance.rollout_status()["last_error"]
    instance._control.execute_trajectory.assert_not_called()


def test_manual_blueprint_and_deployment_arguments() -> None:
    blueprint = dreamlabs_dual_openyam_manual
    modules = {atom.name: atom for atom in blueprint.blueprints}
    assert modules["ControlCoordinator"].kwargs["tasks"][0].joint_names == DUAL_OPENYAM_JOINTS
    assert blueprint.remapping_map[("OverheadCamera", "color_image")] == "overhead_image"
    assert blueprint.remapping_map[("LeftWristCamera", "color_image")] == "left_wrist_image"
    assert blueprint.remapping_map[("RightWristCamera", "color_image")] == "right_wrist_image"
    parsed = BlueprintConfigParser(blueprint).parse(
        [
            "--overheadcamera.device",
            "/dev/cam-a",
            "--leftwristcamera.device",
            "/dev/cam-c",
            "--rightwristcamera.device",
            "/dev/cam-b",
            "--controlcoordinator.left-can-port",
            "follower_l",
            "--controlcoordinator.right-can-port",
            "follower_r",
        ],
        environ={},
    )
    assert parsed.module_kwargs("OverheadCamera")["device"] == "/dev/cam-a"
    assert parsed.module_kwargs("ControlCoordinator")["left_can_port"] == "follower_l"


def test_stream_loss_during_inference_rejects_response(rollout: RolloutFixture) -> None:
    instance, _, entered, release, subjects = rollout
    instance.start_rollout(dispatch_actions=True)
    assert entered.wait(5)
    for subject in subjects:
        subject.on_completed()
    # No additional ticks can arrive. Use the captured wall-clock age, without a sleep.
    with instance._control_lock:
        instance._observation_time = time.time() - instance.config.max_observation_age_s - 1
    worker = instance._worker
    assert worker is not None
    release.set()
    worker.result(timeout=5)
    assert "stale during inference" in instance.rollout_status()["last_error"]
    instance._control.execute_trajectory.assert_not_called()
    instance.policy_preview.publish.assert_not_called()


def test_uncertain_cancellation_requires_retry(rollout: RolloutFixture) -> None:
    instance, _, entered, release, _ = rollout
    instance.start_rollout(dispatch_actions=True)
    assert entered.wait(5)
    instance._control.cancel_trajectory.return_value = TrajectoryCancellationResult(
        TrajectoryCancellationStatus.UNCERTAIN, "test timeout"
    )
    with pytest.raises(RuntimeError, match="uncertain"):
        instance.stop_rollout()
    assert instance.rollout_status()["active"] is False
    with pytest.raises(RuntimeError):
        instance.start_rollout()
    instance._control.cancel_trajectory.return_value = TrajectoryCancellationResult(
        TrajectoryCancellationStatus.CANCELLED
    )
    instance.stop_rollout()
    release.set()
