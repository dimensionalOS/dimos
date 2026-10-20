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
from pathlib import Path
from threading import Event, Thread
import time
from typing import Any

from dimos_dreamlabs.runtime import DreamlabsRuntime
import dream
import numpy as np
import pytest
from pytest_mock import MockerFixture

from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryCancellationResult,
    TrajectoryCancellationStatus,
    TrajectoryExecutionResult,
    TrajectoryExecutionStatus,
)
from dimos.imitation.policy.dreamlabs.conversion import CAMERAS, policy_config
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState, TrajectoryStatus
from dimos.protocol.rpc.pubsubrpc import LCMRPC
from dimos.robot.assets.model import LoadedRobotModel, RobotModel
from dimos.robot.manipulators.dual_openyam.joints import DUAL_OPENYAM_JOINTS
from dimos.teleop.webxr.controller_types import Buttons


class FakeClient:
    def __init__(self) -> None:
        self.entered = Event()
        self.release = Event()
        self.closed = Event()
        self.calls: list[dict[str, Any] | None] = []
        self.ticks: list[tuple[dict[str, str], dict[str, Any], float]] = []
        self.reply: dict[str, Any] = {
            "q_l_chunk": [[0.01] * 6 + [0.5]] * 48,
            "q_r_chunk": [[-0.01] * 6 + [0.5]] * 48,
        }
        self.error: Exception | None = None
        self.config: dict[str, Any] = {}

    def policy(self, config: dict[str, Any]) -> None:
        self.config = config

    def set_observation(
        self, frames: dict[str, str], obs: dict[str, Any], *, timestamp: float
    ) -> None:
        self.ticks.append((frames, obs, timestamp))

    def get_action(self, *, commanded: dict[str, Any] | None = None) -> dict[str, Any]:
        self.calls.append(commanded)
        self.entered.set()
        assert self.release.wait(2), "test did not release fake inference"
        if self.error is not None:
            raise self.error
        return self.reply

    def close(self) -> None:
        self.closed.set()


@pytest.fixture
def station(mocker: MockerFixture) -> Iterator[tuple[DreamlabsRuntime, FakeClient, Any]]:
    mocker.patch("dimos.core.module.get_loop", return_value=(mocker.MagicMock(), None))
    for method in ("__init__", "start", "stop", "serve_module_rpc"):
        mocker.patch.object(LCMRPC, method, return_value=None)
    fake = FakeClient()
    mocker.patch.object(dream, "DreamClient", return_value=fake)
    xml = (
        '<robot name="test"><link name="base"/>'
        + "".join(
            f'<joint name="{name}" type="revolute"><limit lower="-3" upper="3"/></joint>'
            for name in DUAL_OPENYAM_JOINTS[:12]
        )
        + "</robot>"
    )
    mocker.patch.object(
        RobotModel, "load", return_value=LoadedRobotModel(xml, Path("test.urdf"), {})
    )
    module = DreamlabsRuntime(
        _isolated_python_runtime=True,
        rpc_transport=LCMRPC,
        allow_remote=True,
        enable_execution=True,
        standard_yam_verified=True,
    )
    control = mocker.MagicMock()
    control.cancel_trajectory.return_value = TrajectoryCancellationResult(
        TrajectoryCancellationStatus.CANCELLED
    )
    control.execute_trajectory.return_value = TrajectoryExecutionResult(
        TrajectoryExecutionStatus.ACCEPTED
    )
    control.get_trajectory_status.return_value = TrajectoryStatus(state=TrajectoryState.IDLE)
    module._control = control
    now = time.time()
    image = Image(data=np.zeros((10, 10, 3), dtype=np.uint8), format=ImageFormat.RGB, ts=now)
    module._on_overhead(image)
    module._on_left(image)
    module._on_right(image)
    module._on_state(
        JointState(ts=now, name=list(DUAL_OPENYAM_JOINTS), position=[0.0] * 12 + [0.5, 0.5])
    )
    module.preflight_rollout()
    try:
        yield module, fake, control
    finally:
        fake.release.set()
        module.stop()


def test_preflight_is_offline_and_start_is_explicit(
    station: tuple[DreamlabsRuntime, FakeClient, Any],
) -> None:
    module, fake, control = station
    assert fake.config == {}
    control.execute_trajectory.assert_not_called()
    module.config.enable_execution = False
    with pytest.raises(ValueError, match="opt-in"):
        module.start_rollout()


def test_stop_during_request_cancels_before_reply_and_blocks_restart(
    station: tuple[DreamlabsRuntime, FakeClient, Any],
) -> None:
    module, fake, control = station
    module.start_rollout()
    assert fake.entered.wait(1)
    cancelled = Event()
    control.cancel_trajectory.side_effect = lambda: (
        cancelled.set(),
        TrajectoryCancellationResult(TrajectoryCancellationStatus.CANCELLED),
    )[1]
    stop = Thread(target=module.stop_rollout)
    stop.start()
    try:
        assert cancelled.wait(1)
        with pytest.raises(RuntimeError, match="still running"):
            module.start_rollout()
        fake.release.set()
    finally:
        stop.join(3)
    assert not stop.is_alive()
    control.execute_trajectory.assert_not_called()
    assert fake.closed.is_set()
    assert module.rollout_status()["active"] is False


@pytest.mark.parametrize("failure", ["timeout", "malformed", "stale", "rejected"])
def test_bad_request_or_response_stops_and_cancels(
    station: tuple[DreamlabsRuntime, FakeClient, Any], failure: str
) -> None:
    module, fake, control = station
    if failure == "timeout":
        fake.error = TimeoutError("secret or payload must not enter status")
    elif failure == "malformed":
        fake.reply = {}
    elif failure == "stale":
        module.config.max_response_age_s = 1e-9
    else:
        control.execute_trajectory.return_value = TrajectoryExecutionResult(
            TrajectoryExecutionStatus.ALREADY_EXECUTING
        )
    fake.release.set()
    module.start_rollout()
    assert fake.closed.wait(2)
    assert module.rollout_status()["active"] is False
    assert module.rollout_status()["last_error"] in {
        "rollout stopped: TimeoutError",
        "rollout stopped: ValueError",
        "rollout stopped: RuntimeError",
    }
    assert module.rollout_status()["chunks_accepted"] == 0
    control.cancel_trajectory.assert_called()


def test_accepted_chunk_uses_coordinator_and_manual_grip_cancels(
    station: tuple[DreamlabsRuntime, FakeClient, Any],
) -> None:
    module, fake, control = station
    executed = Event()
    control.execute_trajectory.side_effect = lambda trajectory: (
        executed.set(),
        setattr(
            control.get_trajectory_status,
            "return_value",
            TrajectoryStatus(state=TrajectoryState.EXECUTING),
        ),
        TrajectoryExecutionResult(TrajectoryExecutionStatus.ACCEPTED),
    )[2]
    fake.release.set()
    module.start_rollout()
    assert executed.wait(1)
    buttons = Buttons()
    buttons.left_grip = True
    module._on_buttons(buttons)
    assert fake.closed.wait(2)
    trajectory = control.execute_trajectory.call_args.args[0]
    assert trajectory.points[1].positions == [0.01] * 6 + [-0.01] * 6 + [0.5, 0.5]
    assert fake.config["_record"] is False
    assert set(fake.ticks[0][0]) == {"video.overhead", "video.wrist_l", "video.wrist_r"}
    assert fake.ticks[0][2] < time.monotonic()
    control.cancel_trajectory.assert_called()


def test_old_images_and_state_cannot_replace_newer_inputs(
    station: tuple[DreamlabsRuntime, FakeClient, Any],
) -> None:
    module, _, _ = station
    old = Image(
        data=np.ones((10, 10, 3), dtype=np.uint8), format=ImageFormat.RGB, ts=time.time() - 2
    )
    module._on_overhead(old)
    module._on_state(JointState(ts=time.time() - 2, name=["bad"], position=[0.0]))
    assert module.preflight_rollout()["preflight_passed"] is True


def test_next_chunk_is_anchored_on_completed_command(
    station: tuple[DreamlabsRuntime, FakeClient, Any], mocker: MockerFixture
) -> None:
    module, fake, control = station
    calls: list[dict[str, Any] | None] = []

    def infer(*, commanded: dict[str, Any] | None = None) -> dict[str, Any]:
        calls.append(commanded)
        if len(calls) == 2:
            raise TimeoutError("end fixture")
        return fake.reply

    def execute(trajectory: Any) -> TrajectoryExecutionResult:
        control.get_trajectory_status.return_value = TrajectoryStatus(
            state=TrajectoryState.COMPLETED
        )
        module._on_state(
            JointState(
                ts=time.time(),
                name=list(DUAL_OPENYAM_JOINTS),
                position=trajectory.points[-1].positions,
            )
        )
        return TrajectoryExecutionResult(TrajectoryExecutionStatus.ACCEPTED)

    mocker.patch.object(fake, "get_action", side_effect=infer)
    control.execute_trajectory.side_effect = execute
    module.start_rollout()
    assert fake.closed.wait(2)
    assert calls == [None, {"q_l_chunk": [0.01] * 6 + [0.5], "q_r_chunk": [-0.01] * 6 + [0.5]}]
    assert module.rollout_status()["chunks_accepted"] == 1


def test_uncertain_cancel_blocks_future_rollout(
    station: tuple[DreamlabsRuntime, FakeClient, Any],
) -> None:
    module, fake, control = station
    module.start_rollout()
    assert fake.entered.wait(1)
    control.cancel_trajectory.return_value = TrajectoryCancellationResult(
        TrajectoryCancellationStatus.UNCERTAIN
    )
    fake.release.set()
    module.stop_rollout()
    assert module.rollout_status()["cancellation_safe"] is False
    with pytest.raises(ValueError, match="cancellation was uncertain"):
        module.start_rollout()


def test_sdk_history_uses_capture_timestamps_without_background_recording(
    mocker: MockerFixture,
) -> None:
    gateway = mocker.MagicMock()
    gateway.get_action.return_value = {"fixture": True}
    opener = mocker.patch.object(dream, "_open_policy", return_value=gateway)
    recorder = mocker.patch("dream.facade.DreamClient._new_recording")
    with dream.DreamClient(api_key="offline-fixture") as client:
        client.policy(policy_config("dreamlabs/dream-1-beta", 1500))
        for index in range(12):
            frames = {f"video.{role}": f"frame-{index}" for role in CAMERAS}
            client.set_observation(frames, {"prompt": "fixture"}, timestamp=10.0 + index * 0.2)
        assert client.get_action() == {"fixture": True}
    sent = gateway.get_action.call_args.args[0]
    assert sent["video.overhead"] == [f"frame-{index}" for index in range(3, 12)]
    assert sent["video.wrist_l"] == sent["video.overhead"]
    assert "_record" not in opener.call_args.args[0]
    recorder.assert_not_called()
