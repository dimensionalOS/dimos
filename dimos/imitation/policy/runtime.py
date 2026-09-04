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

"""Backend-neutral live observation alignment and safe rollout execution."""

from __future__ import annotations

from collections import deque
from collections.abc import Callable
from dataclasses import dataclass
import math
from threading import Condition, Event, RLock, Thread, current_thread
import time
from typing import Any, cast

import numpy as np
from numpy.typing import NDArray
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.tasks.trajectory_task.trajectory_task import TrajectoryExecutionStatus
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.imitation.policy.backend import PolicyBackend, PolicyBackendInfo
from dimos.imitation.policy.module import (
    PolicyControlSpec,
    PolicyRolloutConfig,
    RolloutStatus,
    _PolicyModule,
)
from dimos.imitation.profile import ImageSource, JointPositionSource, PolicyIOProfile
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.teleop.quest.quest_types import BUTTON_ALIASES, Buttons
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass(frozen=True)
class _TimedValue:
    value: NDArray[Any]
    ts: float


class _PolicyRuntimeMixin:
    """Runtime implementation mixed into a generated backend declaration."""

    profile: PolicyIOProfile
    backend_type: type[PolicyBackend]
    config: PolicyRolloutConfig
    button_pressed: Any
    _control: PolicyControlSpec
    register_disposable: Callable[[Any], None]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = RLock()
        self._observation_changed = Condition(self._lock)
        self._buffers: dict[str, deque[_TimedValue]] = {
            key: deque(maxlen=64) for key in self.profile.observations
        }
        self._backend = self.backend_type(self.config)
        self._backend_info: PolicyBackendInfo | None = None
        self._stop_event = Event()
        self._thread: Thread | None = None
        self._chunks_accepted = 0
        self._last_error: str | None = None
        self._active = False

    @rpc
    def start(self) -> None:
        Module.start(cast("Module", self))
        streams = {source.stream for source in self.profile.observations.values()}
        for stream_name in streams:
            stream = getattr(self, stream_name)
            self.register_disposable(
                Disposable(
                    stream.subscribe(
                        lambda message, name=stream_name: self._on_observation(name, message)
                    )
                )
            )
        self.register_disposable(Disposable(self.button_pressed.subscribe(self._on_button_pressed)))

    @rpc
    def stop(self) -> None:
        if not self._stop_policy():
            self._cancel_after_stop_timeout()
        Module.stop(cast("Module", self))

    @rpc
    def preflight_rollout(self) -> RolloutStatus:
        with self._lock:
            if self._active:
                self._last_error = "cannot preflight while a policy rollout is active"
                return self._status_locked()
            try:
                self._snapshot_observation(time.time())
            except Exception as exc:
                self._backend_info = None
                self._last_error = str(exc)
                return self._status_locked()

        try:
            tasks = set(self._control.list_tasks())
            if self.config.trajectory_task_name not in tasks:
                raise RuntimeError(
                    "ControlCoordinator is missing configured rollout task "
                    f"{self.config.trajectory_task_name!r}"
                )
            backend_info = self._backend_info or self._backend.load(self.profile)
            self._validate_backend_info(backend_info)
            with self._lock:
                self._backend_info = backend_info
                self._snapshot_observation(time.time())
                self._last_error = None
                return self._status_locked()
        except Exception as exc:
            with self._lock:
                self._backend_info = None
                self._last_error = str(exc)
                return self._status_locked()

    @rpc
    def start_rollout(self) -> RolloutStatus:
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                self._last_error = "a policy rollout is already active"
                return self._status_locked()
            if self._backend_info is None:
                self._last_error = "policy preflight has not passed"
                return self._status_locked()
            try:
                self._snapshot_observation(time.time())
            except RuntimeError as exc:
                self._last_error = str(exc)
                return self._status_locked()

            self._stop_event.clear()
            self._chunks_accepted = 0
            self._last_error = None
            self._active = True
            self._thread = Thread(
                target=self._run_rollout,
                name="policy-rollout",
                daemon=True,
            )
            self._thread.start()
            return self._status_locked()

    @rpc
    def stop_rollout(self) -> RolloutStatus:
        if not self._stop_policy():
            self._cancel_after_stop_timeout()
        return self.rollout_status()

    @rpc
    def rollout_status(self) -> RolloutStatus:
        with self._lock:
            return self._status_locked()

    def _status_locked(self) -> RolloutStatus:
        try:
            self._snapshot_observation(time.time())
            observations_ready = True
        except RuntimeError:
            observations_ready = False
        return {
            "active": self._active,
            "artifact": self.config.artifact,
            "backend": self._backend_info.name if self._backend_info is not None else None,
            "task": self.config.task,
            "device": self.config.device,
            "policy_ready": self._backend_info is not None,
            "observations_ready": observations_ready,
            "chunks_accepted": self._chunks_accepted,
            "last_error": self._last_error,
        }

    def _on_observation(self, stream_name: str, message: object) -> None:
        changed_action_state = False
        with self._observation_changed:
            for key, source in self.profile.observations.items():
                if source.stream != stream_name:
                    continue
                try:
                    value, ts = _read_source(source, message)
                except (TypeError, ValueError) as exc:
                    logger.warning(
                        "Ignoring invalid policy observation",
                        feature=key,
                        error=str(exc),
                    )
                    continue
                self._buffers[key].append(_TimedValue(value=value, ts=ts))
                changed_action_state = changed_action_state or key == self.profile.action_state_key
            if changed_action_state:
                self._observation_changed.notify_all()

    def _on_button_pressed(self, buttons: Buttons) -> None:
        button = BUTTON_ALIASES.get(self.config.rollout_button, self.config.rollout_button)
        if not bool(getattr(buttons, button)):
            return
        with self._lock:
            active = self._active
        if active:
            self.stop_rollout()
        else:
            self.start_rollout()

    def _snapshot_observation(
        self,
        now: float,
    ) -> tuple[dict[str, NDArray[Any]], NDArray[np.float32], float]:
        anchor_key = self.profile.sync.anchor
        anchor_buffer = self._buffers[anchor_key]
        if not anchor_buffer:
            raise RuntimeError(f"no {anchor_key!r} observation has been received")
        anchor = anchor_buffer[-1]
        selected = {anchor_key: anchor}
        tolerance_s = self.profile.sync.tolerance_ms / 1000.0

        for key, buffer in self._buffers.items():
            if key == anchor_key:
                continue
            if not buffer:
                raise RuntimeError(f"no {key!r} observation has been received")
            nearest = min(buffer, key=lambda item: abs(item.ts - anchor.ts))
            skew = abs(nearest.ts - anchor.ts)
            if skew > tolerance_s:
                raise RuntimeError(
                    f"{key!r} is {skew * 1000.0:.1f}ms from anchor {anchor_key!r}; "
                    f"limit is {self.profile.sync.tolerance_ms:.1f}ms"
                )
            selected[key] = nearest

        for key, item in selected.items():
            age = now - item.ts
            if age > self.config.max_observation_age_s:
                raise RuntimeError(f"{key!r} observation is stale by {age:.2f}s")

        state_item = selected[self.profile.action_state_key]
        observations = {key: item.value.copy() for key, item in selected.items()}
        return observations, np.asarray(state_item.value, dtype=np.float32), state_item.ts

    def _run_rollout(self) -> None:
        info: PolicyBackendInfo | None = None
        try:
            with self._lock:
                info = self._backend_info
            if info is None:
                raise RuntimeError("policy preflight has not passed")
            if self._stop_event.is_set():
                return
            self._backend.reset()
            execution_steps = self._execution_steps(info)

            while not self._stop_event.is_set():
                with self._lock:
                    observations, state, state_ts = self._snapshot_observation(time.time())
                action_chunk = np.asarray(
                    self._backend.predict(observations, self.config.task),
                    dtype=np.float32,
                )
                actions = self._validated_actions(action_chunk, info, execution_steps)
                if self._stop_event.is_set():
                    break
                result = self._control.execute_trajectory(
                    self._trajectory(state, actions),
                    task_name=self.config.trajectory_task_name,
                )
                if result.status is TrajectoryExecutionStatus.START_STATE_MISMATCH:
                    self._wait_for_newer_joint_state(state_ts)
                    continue
                if result.status is not TrajectoryExecutionStatus.ACCEPTED:
                    raise RuntimeError(
                        result.message or f"trajectory rejected: {result.status.name}"
                    )
                with self._lock:
                    self._chunks_accepted += 1
                self._stop_event.wait(execution_steps / self.profile.sync.rate_hz)
        except Exception as exc:
            with self._lock:
                self._last_error = str(exc)
            logger.exception("Policy execution stopped", error=str(exc))
        finally:
            self._stop_event.set()
            cancellation_error = self._cancel_trajectory()
            reset_error = self._reset_backend() if info is not None else None
            with self._lock:
                shutdown_errors = [
                    error for error in (cancellation_error, reset_error) if error is not None
                ]
                if shutdown_errors:
                    shutdown_error = "; ".join(shutdown_errors)
                    self._last_error = (
                        f"{self._last_error}; {shutdown_error}"
                        if self._last_error is not None
                        else shutdown_error
                    )
                self._active = False

    def _execution_steps(self, info: PolicyBackendInfo) -> int:
        horizon_steps = math.floor(
            self.config.max_execution_horizon_s * self.profile.sync.rate_hz + 1e-9
        )
        if horizon_steps < 1:
            raise ValueError("max execution horizon is shorter than one policy step")
        return min(info.chunk_length, info.preferred_execution_steps, horizon_steps)

    def _validated_actions(
        self,
        action_chunk: NDArray[np.float32],
        info: PolicyBackendInfo,
        execution_steps: int,
    ) -> NDArray[np.float32]:
        width = len(self.profile.action.demonstration.joints)
        if action_chunk.ndim != 2 or action_chunk.shape[1] != width:
            raise RuntimeError(
                f"policy returned action chunk shape {action_chunk.shape}, expected (steps, {width})"
            )
        if action_chunk.shape[0] < execution_steps:
            raise RuntimeError(
                f"policy returned {action_chunk.shape[0]} steps, expected at least {execution_steps}"
            )
        actions = action_chunk[:execution_steps]
        if not np.all(np.isfinite(actions)):
            raise RuntimeError("policy returned non-finite joint targets")
        if info.action_lower is None:
            return actions
        assert info.action_upper is not None
        bounded = np.clip(actions, info.action_lower, info.action_upper)
        if np.any(actions != bounded):
            logger.warning("Clipped policy actions to backend range")
        return bounded

    def _validate_backend_info(self, info: PolicyBackendInfo) -> None:
        if info.chunk_length <= 0 or info.preferred_execution_steps <= 0:
            raise ValueError("backend chunk and execution step counts must be positive")
        bounds = (info.action_lower, info.action_upper)
        if (bounds[0] is None) != (bounds[1] is None):
            raise ValueError("backend must provide both action bounds or neither")
        if bounds[0] is None:
            return
        assert bounds[1] is not None
        shape = (len(self.profile.action.demonstration.joints),)
        if bounds[0].shape != shape or bounds[1].shape != shape:
            raise ValueError(f"backend action bounds must have shape {shape}")
        if not np.all(np.isfinite(bounds[0])) or not np.all(np.isfinite(bounds[1])):
            raise ValueError("backend action bounds contain non-finite values")
        if np.any(bounds[0] > bounds[1]):
            raise ValueError("backend action lower bound exceeds upper bound")

    def _stop_policy(self) -> bool:
        with self._lock:
            thread = self._thread
            self._stop_event.set()
            self._observation_changed.notify_all()
        if thread is not None and thread is not current_thread():
            thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        return thread is None or not thread.is_alive()

    def _cancel_after_stop_timeout(self) -> None:
        timeout_error = f"policy rollout did not stop within {DEFAULT_THREAD_JOIN_TIMEOUT} seconds"
        cancellation_error = self._cancel_trajectory()
        with self._lock:
            self._last_error = (
                f"{timeout_error}; {cancellation_error}"
                if cancellation_error is not None
                else timeout_error
            )

    def _trajectory(
        self,
        state: NDArray[np.float32],
        actions: NDArray[np.float32],
    ) -> JointTrajectory:
        joints = list(self.profile.action.demonstration.joints)
        zeros = [0.0] * len(joints)
        points = [
            TrajectoryPoint(
                positions=[float(value) for value in state],
                velocities=zeros,
                time_from_start=0.0,
            )
        ]
        points.extend(
            TrajectoryPoint(
                positions=[float(value) for value in action],
                velocities=zeros,
                time_from_start=(index + 1) / self.profile.sync.rate_hz,
            )
            for index, action in enumerate(actions)
        )
        return JointTrajectory(joint_names=joints, points=points)

    def _wait_for_newer_joint_state(self, previous_ts: float) -> None:
        state_buffer = self._buffers[self.profile.action_state_key]
        with self._observation_changed:
            self._observation_changed.wait_for(
                lambda: self._stop_event.is_set()
                or (bool(state_buffer) and state_buffer[-1].ts > previous_ts)
            )

    def _cancel_trajectory(self) -> str | None:
        try:
            result = self._control.cancel_trajectory(task_name=self.config.trajectory_task_name)
        except Exception as exc:
            logger.exception(
                "Failed to cancel policy trajectory",
                task_name=self.config.trajectory_task_name,
            )
            return f"Failed to cancel policy trajectory: {exc}"
        if result.safe:
            return None
        return result.message or "Policy trajectory cancellation was uncertain"

    def _reset_backend(self) -> str | None:
        try:
            self._backend.reset()
        except Exception as exc:
            logger.exception("Failed to reset policy backend")
            return f"Failed to reset policy backend: {exc}"
        return None


def declare_policy_runtime(
    name: str,
    module_name: str,
    declaration: type[_PolicyModule],
    backend_type: type[PolicyBackend],
) -> type[_PolicyModule]:
    """Declare an importable runtime subclass using the common rollout loop."""
    return type(
        name,
        (_PolicyRuntimeMixin, declaration),
        {
            "__module__": module_name,
            "__qualname__": name,
            "backend_type": backend_type,
        },
    )


def _read_source(
    source: ImageSource | JointPositionSource,
    message: object,
) -> tuple[NDArray[Any], float]:
    if isinstance(source, ImageSource):
        if not isinstance(message, Image):
            raise TypeError(f"expected Image, got {type(message).__name__}")
        if message.format != ImageFormat.RGB or message.data.dtype != np.uint8:
            raise ValueError("image must be uint8 RGB")
        if message.data.shape != source.shape:
            raise ValueError(f"image shape {message.data.shape} does not match {source.shape}")
        return np.ascontiguousarray(message.data), message.ts

    if not isinstance(message, JointState):
        raise TypeError(f"expected JointState, got {type(message).__name__}")
    if len(message.name) != len(message.position):
        raise ValueError("JointState names and positions have different lengths")
    if len(message.name) != len(set(message.name)):
        raise ValueError("JointState contains duplicate joint names")
    positions = dict(zip(message.name, message.position, strict=True))
    missing = [joint for joint in source.joints if joint not in positions]
    if missing:
        raise ValueError(f"JointState is missing configured joints: {missing}")
    value = np.asarray([positions[joint] for joint in source.joints], dtype=np.float32)
    if not np.all(np.isfinite(value)):
        raise ValueError("JointState contains non-finite positions")
    return value, message.ts
