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

"""Manual Dreamlabs controls on the normal module loop and coordinator executor."""

from __future__ import annotations

import asyncio
from concurrent.futures import Future
from threading import RLock
import time
from typing import Any, Protocol, TypedDict

import numpy as np
from pydantic import Field
import reactivex as rx
from reactivex import operators as ops
from reactivex.abc import DisposableBase

from dimos.control.tasks.trajectory_task.trajectory_task import (
    JOINT_TRAJECTORY_TASK_NAME,
    TrajectoryCancellationResult,
    TrajectoryExecutionResult,
    TrajectoryExecutionStatus,
)
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.imitation.policy.dreamlabs.config import DreamlabsConfig
from dimos.imitation.policy.dreamlabs.module import DreamlabsModule
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState, TrajectoryStatus
from dimos.robot.manipulators.dual_openyam.joints import DUAL_OPENYAM_ARM_JOINTS
from dimos.robot.manipulators.dual_openyam.model import DUAL_OPENYAM_MODEL
from dimos.spec.utils import Spec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()
OBSERVATION_PERIOD = 0.2
OBSERVATION_WINDOW = 9


class DreamlabsControl(Spec, Protocol):
    def execute_trajectory(self, trajectory: JointTrajectory) -> TrajectoryExecutionResult: ...
    def cancel_trajectory(self) -> TrajectoryCancellationResult: ...
    def task_invoke(self, task_name: str, method: str, kwargs: dict[str, Any]) -> Any: ...


class DreamlabsRolloutConfig(DreamlabsConfig):
    max_observation_age_s: float = Field(default=0.5, gt=0)
    max_observation_skew_s: float = Field(default=0.1, gt=0)


class RolloutStatus(TypedDict):
    active: bool
    dispatch_actions: bool
    busy: bool
    prompt: str
    observations: int
    chunks_received: int
    chunks_executed: int
    last_error: str | None


class DreamlabsRolloutModule(DreamlabsModule):
    """Acquire on startup; infer only on start_rollout; dispatch only by explicit opt-in."""

    config: DreamlabsRolloutConfig
    overhead_image: In[Image]
    left_wrist_image: In[Image]
    right_wrist_image: In[Image]
    coordinator_joint_state: In[JointState]
    policy_preview: Out[JointTrajectory]
    _control: DreamlabsControl

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._control_lock = RLock()
        self._revision = 0
        self._active = False
        self._prepared = False
        self._observation_time = 0.0
        self._dispatch = False
        self._worker: Future[Any] | None = None
        self._observations = 0
        self._chunks_received = 0
        self._chunks_executed = 0
        self._last_error: str | None = None
        self._bounds: dict[str, tuple[float, float]] = {}
        self._observation_subscription: DisposableBase | None = None
        self._last_capture_time = 0.0

    @rpc
    def start(self) -> None:
        super().start()
        # Latest-frame pairing, not a new sensor history or hardware synchronizer.
        paired = rx.combine_latest(
            self.overhead_image.observable(),
            self.left_wrist_image.observable(),
            self.right_wrist_image.observable(),
            self.coordinator_joint_state.observable(),
        ).pipe(ops.sample(OBSERVATION_PERIOD))
        self._observation_subscription = self.process_observable(paired, self._observe)

    async def _observe(self, messages: tuple[Image, Image, Image, JointState]) -> None:
        self._buffer_observation(messages)

    def _buffer_observation(self, messages: tuple[Image, Image, Image, JointState]) -> None:
        with self._control_lock:
            if not self._active or not self._prepared:
                return
            try:
                stamps = np.array([message.ts for message in messages])
                ages = time.time() - stamps
                if (
                    not np.isfinite(stamps).all()
                    or np.any(ages < 0)
                    or np.any(ages > self.config.max_observation_age_s)
                ):
                    raise ValueError("missing, future or stale observation timestamp")
                if np.ptp(stamps) > self.config.max_observation_skew_s:
                    raise ValueError("camera/state timestamp skew exceeds configured tolerance")
                capture_time = float(stamps[0])
                if capture_time <= self._last_capture_time:
                    return
                self.set_observation(*messages, timestamp=time.monotonic() - float(ages[0]))
                self._last_capture_time = capture_time
                self._observations += 1
                self._observation_time = float(stamps.min())
            except Exception as exc:
                self._fail(exc)

    @rpc
    def rollout_status(self) -> RolloutStatus:
        """Report acquisition/inference/dispatch state; busy includes a draining request."""
        with self._control_lock:
            return {
                "active": self._active,
                "dispatch_actions": self._dispatch,
                "busy": self._worker is not None and not self._worker.done(),
                "prompt": self.config.prompt,
                "observations": self._observations,
                "chunks_received": self._chunks_received,
                "chunks_executed": self._chunks_executed,
                "last_error": self._last_error,
            }

    @rpc
    def start_rollout(self, dispatch_actions: bool = False) -> RolloutStatus:
        """Start billed inference; default is preview only. Execution is a separate opt-in.

        Wait for busy=False after stop before starting a new episode. No API retries
        or queued chunks are created; errors leave the module idle.
        """
        with self._control_lock:
            if (
                self._active
                or self._dispatch
                or (self._worker is not None and not self._worker.done())
            ):
                raise RuntimeError("rollout active or previous request still draining")
            if dispatch_actions:
                model = DUAL_OPENYAM_MODEL.load()
                self._bounds = {}
                for name in DUAL_OPENYAM_ARM_JOINTS:
                    joint = model.get_joint(name)
                    if joint is None or joint.lower is None or joint.upper is None:
                        raise ValueError(f"missing model limits for {name}")
                    if (
                        not np.isfinite([joint.lower, joint.upper]).all()
                        or joint.lower >= joint.upper
                    ):
                        raise ValueError(f"invalid model limits for {name}")
                    self._bounds[name] = (joint.lower, joint.upper)
            self._revision += 1
            self._active, self._dispatch = True, dispatch_actions
            self._prepared = False
            self._last_capture_time = 0.0
            self._observations = self._chunks_received = self._chunks_executed = 0
            self._last_error = None
            self._worker = self.spawn(self._run(self._revision))
            return self.rollout_status()

    @rpc
    def stop_rollout(self) -> RolloutStatus:
        """Invalidate pending responses and cancel our current coordinator trajectory.

        An in-flight API call may finish and be billed, but cannot publish or execute.
        Cancellation failure is surfaced in last_error; this RPC is not an E-stop.
        """
        with self._control_lock:
            self._revision += 1
            self._active = False
            if self._dispatch:
                try:
                    result = self._control.cancel_trajectory()
                    if not result.safe:
                        raise RuntimeError(f"trajectory cancellation uncertain: {result.message}")
                except Exception as exc:
                    self._last_error = str(exc)
                    raise
                self._dispatch = False
            return self.rollout_status()

    @rpc
    def set_prompt(self, prompt: str) -> RolloutStatus:
        """Stop, replace task language and remain idle; next start uses fresh SDK history."""
        if not prompt.strip():
            raise ValueError("prompt must not be blank")
        with self._control_lock:
            self.stop_rollout()
            self.config.prompt = prompt
            return self.rollout_status()

    def _fail(self, exc: Exception) -> None:
        logger.error("Dreamlabs rollout failed", error=str(exc))
        self._last_error = str(exc)
        self.stop_rollout()

    def _is_active(self, revision: int) -> bool:
        with self._control_lock:
            return self._active and revision == self._revision

    def _accept_chunk(self, revision: int, trajectory: JointTrajectory) -> bool:
        # The final revision check and execute serialize against stop + cancel.
        with self._control_lock:
            if not self._is_active(revision):
                return False
            if time.time() - self._observation_time > self.config.max_observation_age_s:
                raise ValueError("observation stream stopped or became stale during inference")
            self._chunks_received += 1
            if self._dispatch:
                for name, positions in zip(
                    trajectory.joint_names,
                    np.array([p.positions for p in trajectory.points]).T,
                    strict=True,
                ):
                    if name in self._bounds:
                        lower, upper = self._bounds[name]
                        if np.any(positions < lower) or np.any(positions > upper):
                            raise ValueError(
                                f"{name} targets exceed model limits [{lower}, {upper}]"
                            )
                result = self._control.execute_trajectory(trajectory)
                if result.status is not TrajectoryExecutionStatus.ACCEPTED:
                    raise RuntimeError(
                        f"trajectory rejected: {result.status.name}: {result.message}"
                    )
                self._chunks_executed += 1
            self.policy_preview.publish(trajectory)
            return True

    async def _run(self, revision: int) -> None:
        try:
            await asyncio.to_thread(self.reset_episode)
            with self._control_lock:
                self._prepared = self._is_active(revision)
            while self._is_active(revision):
                with self._control_lock:
                    ready = self._observations >= OBSERVATION_WINDOW
                if not ready:
                    await asyncio.sleep(OBSERVATION_PERIOD)
                    continue
                with self._control_lock:
                    if time.time() - self._observation_time > self.config.max_observation_age_s:
                        raise ValueError("observation stream stopped or became stale")
                trajectory = await asyncio.to_thread(self.get_action)
                if not self._accept_chunk(revision, trajectory):
                    break
                with self._control_lock:
                    dispatch = self._dispatch
                if dispatch:
                    while self._is_active(revision):
                        status = await asyncio.to_thread(
                            self._control.task_invoke,
                            JOINT_TRAJECTORY_TASK_NAME,
                            "get_status",
                            {"t_now": None},
                        )
                        if not isinstance(status, TrajectoryStatus):
                            raise RuntimeError("coordinator returned no trajectory status")
                        if status.state == TrajectoryState.COMPLETED:
                            break
                        if status.state != TrajectoryState.EXECUTING:
                            raise RuntimeError(f"trajectory ended in {status.state.name}")
                        await asyncio.sleep(OBSERVATION_PERIOD)
                else:
                    await asyncio.sleep(trajectory.duration)
        except Exception as exc:
            with self._control_lock:
                if self._is_active(revision):
                    self._fail(exc)
        finally:
            # No new start is accepted until the old request and reset have finished.
            try:
                await asyncio.to_thread(self.reset_episode)
            except Exception as exc:
                with self._control_lock:
                    self._last_error = f"SDK episode reset failed: {exc}"
                logger.error("Dreamlabs episode reset failed", error=str(exc))
                raise

    @rpc
    def stop(self) -> None:
        try:
            self.stop_rollout()
        finally:
            try:
                if self._observation_subscription is not None:
                    self._observation_subscription.dispose()
                worker = self._worker
                if worker is not None:
                    worker.result(timeout=self.config.timeout_ms / 1000 + 10)
            finally:
                try:
                    # Module closes its owned loop without draining its default executor.
                    # A caller-owned loop manages its own executor lifetime.
                    if self._loop_thread is not None and self._loop is not None:
                        self.spawn(self._loop.shutdown_default_executor()).result(timeout=10)
                finally:
                    super().stop()
