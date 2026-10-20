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


"""Synchronous, bounded Dreamlabs chunks through the existing coordinator."""

from __future__ import annotations

from dataclasses import dataclass, field
from threading import Event, RLock, Thread
import time
from typing import TYPE_CHECKING, Any, Protocol

import numpy as np
from numpy.typing import NDArray
from reactivex.disposable import Disposable

from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryCancellationResult,
    TrajectoryExecutionResult,
    TrajectoryExecutionStatus,
)
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In
from dimos.imitation.policy.dreamlabs.config import DreamlabsConfig
from dimos.imitation.policy.dreamlabs.conversion import (
    CAMERAS,
    action_trajectory,
    encode_image,
    policy_config,
    state_vector,
    validate_times,
)
from dimos.imitation.policy.dreamlabs.kinematics import DreamlabsKinematics
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState, TrajectoryStatus
from dimos.robot.manipulators.dual_openyam.joints import DUAL_OPENYAM_ARM_JOINTS
from dimos.robot.manipulators.dual_openyam.model import DUAL_OPENYAM_MODEL
from dimos.spec.utils import Spec
from dimos.teleop.webxr.controller_types import Buttons

if TYPE_CHECKING:
    from dream import DreamClient


class DreamlabsControlSpec(Spec, Protocol):
    def execute_trajectory(self, trajectory: JointTrajectory) -> TrajectoryExecutionResult: ...
    def cancel_trajectory(self) -> TrajectoryCancellationResult: ...
    def get_trajectory_status(self) -> TrajectoryStatus | None: ...


class DreamlabsRolloutSpec(Spec, Protocol):
    def preflight_rollout(self) -> dict[str, Any]: ...
    def start_rollout(self) -> dict[str, Any]: ...
    def stop_rollout(self) -> dict[str, Any]: ...
    def rollout_status(self) -> dict[str, Any]: ...


def create_client() -> DreamClient:
    # Optional SDK: importing a blueprint or doing local preflight needs no client package.
    import dream

    return dream.DreamClient()


@dataclass
class _Episode:
    stop: Event = field(default_factory=Event)
    thread: Thread | None = None
    sampler: Thread | None = None
    chunks: int = 0
    error: str | None = None
    observation_time: float | None = None
    capture_stamp: float | None = None


class DreamlabsModule(Module):
    """Idle-by-default Dream-1 joint rollout through the existing dual YAM coordinator."""

    config: DreamlabsConfig
    overhead_image: In[Image]
    left_wrist_image: In[Image]
    right_wrist_image: In[Image]
    coordinator_joint_state: In[JointState]
    teleop_buttons: In[Buttons]
    _control: DreamlabsControlSpec

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = RLock()
        self._images: dict[str, tuple[str, float]] = {}
        self._state: JointState | None = None
        self._episode: _Episode | None = None
        self._manual = False
        self._cancellation_safe = True
        self._fk: DreamlabsKinematics | None = None
        self._bounds: tuple[NDArray[np.float64], NDArray[np.float64]] | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.overhead_image.subscribe(self._on_overhead)))
        self.register_disposable(Disposable(self.left_wrist_image.subscribe(self._on_left)))
        self.register_disposable(Disposable(self.right_wrist_image.subscribe(self._on_right)))
        self.register_disposable(Disposable(self.coordinator_joint_state.subscribe(self._on_state)))
        self.register_disposable(Disposable(self.teleop_buttons.subscribe(self._on_buttons)))

    @rpc
    def stop(self) -> None:
        try:
            self.stop_rollout()
        finally:
            super().stop()

    def _on_overhead(self, image: Image) -> None:
        self._on_image("overhead", image)

    def _on_left(self, image: Image) -> None:
        self._on_image("wrist_l", image)

    def _on_right(self, image: Image) -> None:
        self._on_image("wrist_r", image)

    def _on_image(self, role: str, image: Image) -> None:
        with self._lock:
            previous = self._images.get(role)
            if previous is not None and image.ts <= previous[1]:
                return
            try:
                encoded = encode_image(image)
                validate_times([image.ts], time.time(), self.config.max_observation_age_s, 1.0)
            except ValueError:
                self._images.pop(role, None)
                return
            self._images[role] = (encoded, image.ts)

    def _on_state(self, state: JointState) -> None:
        with self._lock:
            if self._state is None or state.ts > self._state.ts:
                self._state = JointState(state)

    def _on_buttons(self, buttons: Buttons) -> None:
        with self._lock:
            self._manual = buttons.left_grip or buttons.right_grip
            if self._manual and self._episode is not None and not self._episode.stop.is_set():
                self._episode.stop.set()
                self._cancel_locked(self._episode)

    def _snapshot(self) -> tuple[dict[str, str], NDArray[np.float64], float]:
        if set(self._images) != set(CAMERAS) or self._state is None:
            raise ValueError("three camera roles and coordinator state are required")
        stamps = [self._images[role][1] for role in CAMERAS] + [self._state.ts]
        validate_times(
            stamps, time.time(), self.config.max_observation_age_s, self.config.max_skew_s
        )
        return (
            {f"video.{role}": self._images[role][0] for role in CAMERAS},
            state_vector(self._state),
            min(stamps),
        )

    def _gates(self) -> None:
        if not self._cancellation_safe:
            raise ValueError(
                "cancellation was uncertain; inspect hardware before restarting this module"
            )
        if not self.config.allow_remote or not self.config.enable_execution:
            raise ValueError("remote inference and execution require explicit opt-in")
        if not self.config.standard_yam_verified:
            raise ValueError("operator must verify standard YAM calibration and camera mapping")
        if self._manual:
            raise ValueError("release manual controller grips")
        if not self.config.prompt.strip():
            raise ValueError("prompt must not be blank")
        self._snapshot()

    @rpc
    def preflight_rollout(self) -> dict[str, Any]:
        with self._lock:
            if self._episode is not None and not self._episode.stop.is_set():
                raise RuntimeError("cannot preflight an active episode")
            self._gates()
            model = DUAL_OPENYAM_MODEL.load()
            lows: list[float] = []
            highs: list[float] = []
            for name in DUAL_OPENYAM_ARM_JOINTS:
                joint = model.get_joint(name)
                if joint is None or joint.lower is None or joint.upper is None:
                    raise ValueError("model joint limits unavailable")
                lows.append(joint.lower)
                highs.append(joint.upper)
            lower = np.array([*lows, 0.0, 0.0], dtype=np.float64)
            upper = np.array([*highs, 1.0, 1.0], dtype=np.float64)
            if not np.isfinite([lower, upper]).all() or np.any(lower >= upper):
                raise ValueError("invalid model bounds")
            if self._fk is None:
                self._fk = DreamlabsKinematics()
            self._bounds = lower, upper
            return self.rollout_status()

    @rpc
    def start_rollout(self) -> dict[str, Any]:
        with self._lock:
            if (
                self._episode is not None
                and self._episode.thread is not None
                and self._episode.thread.is_alive()
            ):
                raise RuntimeError("previous episode is still running or stopping")
            self._gates()
            if self._bounds is None:
                raise RuntimeError("run preflight_rollout first")
            status = self._control.get_trajectory_status()
            if status is None or status.state not in (
                TrajectoryState.IDLE,
                TrajectoryState.COMPLETED,
                TrajectoryState.ABORTED,
            ):
                raise RuntimeError("coordinator trajectory task is unavailable, busy, or faulted")
            episode = _Episode()
            episode.thread = Thread(target=self._run, args=(episode,), name="dreamlabs-rollout")
            self._episode = episode
            episode.thread.start()
            return self.rollout_status()

    @rpc
    def stop_rollout(self) -> dict[str, Any]:
        with self._lock:
            episode = self._episode
            if episode is not None:
                episode.stop.set()
                self._cancel_locked(episode)
        if episode is not None and episode.thread is not None:
            episode.thread.join(timeout=2 * self.config.timeout_ms / 1000 + 3)
            if episode.thread.is_alive():
                with self._lock:
                    episode.error = "inference shutdown pending; restart is blocked"
        return self.rollout_status()

    @rpc
    def rollout_status(self) -> dict[str, Any]:
        with self._lock:
            e = self._episode
            return {
                "active": e is not None and not e.stop.is_set(),
                "stopping": e is not None
                and e.stop.is_set()
                and e.thread is not None
                and e.thread.is_alive(),
                "chunks_accepted": e.chunks if e is not None else 0,
                "last_error": e.error if e is not None else None,
                "preflight_passed": self._bounds is not None,
                "cancellation_safe": self._cancellation_safe,
            }

    def _feed(self, client: DreamClient, episode: _Episode) -> None:
        with self._lock:
            if episode.stop.is_set():
                return
            frames, vector, stamp = self._snapshot()
            if episode.capture_stamp is not None and stamp <= episode.capture_stamp:
                return
            # Convert capture epoch to monotonic, never send wall time to the SDK window.
            captured = time.monotonic() - (time.time() - stamp)
            if episode.observation_time is not None and captured <= episode.observation_time:
                return
            assert self._fk is not None
            client.set_observation(
                frames, self._fk.observation(vector, self.config.prompt), timestamp=captured
            )
            episode.observation_time = captured
            episode.capture_stamp = stamp

    def _sample(self, client: DreamClient, episode: _Episode) -> None:
        while not episode.stop.wait(1 / 30):
            try:
                self._feed(client, episode)
            except Exception as exc:
                self._fail(episode, exc)
                return

    def _fail(self, episode: _Episode, exc: Exception) -> None:
        with self._lock:
            # SDK exceptions may contain payloads. Persist only the exception category.
            episode.error = f"rollout stopped: {type(exc).__name__}"
            episode.stop.set()
            self._cancel_locked(episode)

    def _cancel_locked(self, episode: _Episode) -> None:
        try:
            result = self._control.cancel_trajectory()
            if not result.safe:
                self._cancellation_safe = False
                episode.error = "coordinator cancellation uncertain; inspect E-stop and hardware"
        except Exception:
            self._cancellation_safe = False
            episode.error = "coordinator cancellation failed; inspect E-stop and hardware"

    def _run(self, episode: _Episode) -> None:
        client: DreamClient | None = None
        commanded: dict[str, Any] | None = None
        try:
            client = create_client()
            client.policy(policy_config(self.config.checkpoint, self.config.timeout_ms))
            self._feed(client, episode)
            episode.sampler = Thread(
                target=self._sample, args=(client, episode), name="dreamlabs-observations"
            )
            episode.sampler.start()
            while not episode.stop.is_set():
                with self._lock:
                    self._snapshot()
                    requested = time.monotonic()
                reply = client.get_action(commanded=commanded)
                with self._lock:
                    if episode.stop.is_set() or self._episode is not episode:
                        break
                    if time.monotonic() - requested > self.config.max_response_age_s:
                        raise ValueError("stale inference response")
                    _, vector, _ = self._snapshot()
                    assert self._bounds is not None
                    trajectory = action_trajectory(
                        reply,
                        vector,
                        *self._bounds,
                        self.config.max_joint_speed,
                        self.config.max_gripper_speed,
                    )
                    # Serialize cancellation and dispatch. No response can dispatch after stop.
                    result = self._control.execute_trajectory(trajectory)
                    if result.status is not TrajectoryExecutionStatus.ACCEPTED:
                        raise RuntimeError("coordinator rejected trajectory")
                    episode.chunks += 1
                self._await_completion(episode, trajectory.duration)
                last = trajectory.points[-1].positions
                commanded = {
                    "q_l_chunk": [*last[:6], last[12]],
                    "q_r_chunk": [*last[6:12], last[13]],
                }
        except Exception as exc:
            self._fail(episode, exc)
        finally:
            episode.stop.set()
            if episode.sampler is not None:
                episode.sampler.join()
            with self._lock:
                self._cancel_locked(episode)
            if client is not None:
                try:
                    client.close()
                except Exception as exc:
                    self._fail(episode, exc)

    def _await_completion(self, episode: _Episode, duration: float) -> None:
        deadline = time.monotonic() + duration + self.config.completion_timeout_s
        while not episode.stop.wait(0.01):
            with self._lock:
                self._snapshot()
                status = self._control.get_trajectory_status()
                if status is not None:
                    if status.state == TrajectoryState.COMPLETED:
                        return
                    if status.state != TrajectoryState.EXECUTING:
                        raise RuntimeError("trajectory aborted or faulted")
                else:
                    raise RuntimeError("trajectory task unavailable")
                if time.monotonic() > deadline:
                    raise TimeoutError("coordinator completion timeout")
