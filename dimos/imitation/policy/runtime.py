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

"""Shared observation, lifecycle, and execution loop for isolated policy backends."""

from __future__ import annotations

from functools import partial
from importlib import import_module
import json
from threading import Condition, Event, RLock, Thread, current_thread
import time
from typing import Any, cast

import numpy as np
from numpy.typing import NDArray
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.tasks.trajectory_task.trajectory_task import TrajectoryExecutionStatus
from dimos.core.core import rpc
from dimos.imitation.policy.backend import Images, PolicyBackend
from dimos.imitation.policy.module import PolicyModule, RolloutStatus, policy_class
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.teleop.quest.quest_types import BUTTON_ALIASES, Buttons
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class _PolicyRuntime(PolicyModule):
    """Shared rollout implementation loaded by ``PolicyModule``."""

    _lock: RLock
    _observation_changed: Condition
    _loaded_policy: PolicyBackend | None
    _latest_images: dict[str, tuple[NDArray[np.uint8], float]]
    _latest_joint_state: JointState | None
    _stop_event: Event
    _thread: Thread | None
    _chunks_accepted: int
    _last_error: str | None
    _active: bool

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = RLock()
        self._observation_changed = Condition(self._lock)
        self._loaded_policy = None
        self._latest_images = {}
        self._latest_joint_state = None
        self._stop_event = Event()
        self._thread = None
        self._chunks_accepted = 0
        self._last_error = None
        self._active = False

    @rpc
    def start(self) -> None:
        super().start()
        missing = set(self.config.image_mapping) - self.inputs.keys()
        if missing:
            raise ValueError(f"Image mapping names undeclared policy ports: {sorted(missing)}")
        for port in self.config.image_mapping:
            self.register_disposable(
                Disposable(self.inputs[port].subscribe(partial(self._on_image, port)))
            )
        self.register_disposable(
            Disposable(self.coordinator_joint_state.subscribe(self._on_joint_state))
        )
        self.register_disposable(Disposable(self.button_pressed.subscribe(self._on_button_pressed)))

    @rpc
    def stop(self) -> None:
        if not self._stop_policy():
            self._cancel_after_stop_timeout()
        if self._loaded_policy is not None and (
            self._thread is None or not self._thread.is_alive()
        ):
            self._loaded_policy.close()
            self._loaded_policy = None
        super().stop()

    @rpc
    def preflight_rollout(self) -> RolloutStatus:
        """Validate the checkpoint, coordinator, and live observations without moving."""
        with self._lock:
            if self._active:
                self._last_error = "cannot preflight while a policy rollout is active"
                return self._status_locked()
            loaded_policy = self._loaded_policy
            try:
                self._snapshot_observation(time.time())
            except Exception as exc:
                if loaded_policy is not None:
                    loaded_policy.close()
                self._loaded_policy = None
                self._last_error = str(exc)
                return self._status_locked()

        try:
            tasks = set(self._control.list_tasks())
            if self.config.trajectory_task_name not in tasks:
                raise RuntimeError(
                    "ControlCoordinator is missing configured rollout task "
                    f"{self.config.trajectory_task_name!r}"
                )
            if loaded_policy is None:
                loaded_policy = self._load_policy()
                logger.info(
                    "Loaded policy during preflight",
                    path=self.config.policy_path,
                    runtime_fps=self.config.fps,
                    chunk_size=loaded_policy.chunk_size,
                    n_action_steps=loaded_policy.n_action_steps,
                )
            with self._lock:
                images, state, _ = self._snapshot_observation(time.time())
            sample = loaded_policy.predict(images, state, task=self.config.task)
            if (
                sample.ndim != 2
                or sample.shape[1] != len(self.config.joint_names)
                or sample.shape[0] < loaded_policy.n_action_steps
                or not np.all(np.isfinite(sample))
            ):
                raise ValueError("Policy preflight returned an invalid action chunk")
            loaded_policy.reset()
            with self._lock:
                self._loaded_policy = loaded_policy
                self._snapshot_observation(time.time())
                self._last_error = None
                return self._status_locked()
        except Exception as exc:
            if loaded_policy is not None:
                loaded_policy.close()
            with self._lock:
                self._loaded_policy = None
                self._last_error = str(exc)
                return self._status_locked()

    @rpc
    def start_rollout(self) -> RolloutStatus:
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                self._last_error = "a policy rollout is already active"
                return self._status_locked()
            if self._loaded_policy is None:
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
            "backend": self.config.backend,
            "active": self._active,
            "policy_path": self.config.policy_path,
            "task": self.config.task,
            "device": self.config.device,
            "policy_ready": self._loaded_policy is not None,
            "observations_ready": observations_ready,
            "chunks_accepted": self._chunks_accepted,
            "last_error": self._last_error,
        }

    def _on_image(self, port: str, image: Image) -> None:
        if image.format != ImageFormat.RGB or image.data.dtype != np.uint8:
            logger.warning("Ignoring non-uint8 RGB policy image", port=port)
            return
        if image.data.ndim != 3 or image.data.shape[2] != 3:
            logger.warning("Ignoring malformed RGB policy image", port=port)
            return
        with self._lock:
            self._latest_images[port] = (np.ascontiguousarray(image.data).copy(), image.ts)

    def _on_joint_state(self, state: JointState) -> None:
        with self._observation_changed:
            self._latest_joint_state = JointState(state)
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

    def _snapshot_observation(self, now: float) -> tuple[Images, NDArray[np.float32], float]:
        images: Images = {}
        max_age = self.config.max_observation_age_s
        for port, feature in self.config.image_mapping.items():
            if port not in self._latest_images:
                raise RuntimeError(f"no camera image has been received on {port}")
            image, timestamp = self._latest_images[port]
            if not np.isfinite(timestamp) or not 0 <= now - timestamp <= max_age:
                raise RuntimeError(f"camera image on {port} is stale or has an invalid timestamp")
            images[feature] = image.copy()
        if self._latest_joint_state is None:
            raise RuntimeError("no coordinator joint state has been received")
        state = self._latest_joint_state
        if not np.isfinite(state.ts) or not 0 <= now - state.ts <= max_age:
            raise RuntimeError("joint state is stale or has an invalid timestamp")

        if len(state.name) != len(state.position) or len(set(state.name)) != len(state.name):
            raise RuntimeError("joint state names must be unique and match its position count")
        positions = dict(zip(state.name, state.position, strict=True))
        missing = [name for name in self.config.joint_names if name not in positions]
        if missing:
            raise RuntimeError(f"joint state is missing configured joints: {missing}")
        vector = np.asarray(
            [positions[name] for name in self.config.joint_names],
            dtype=np.float32,
        )
        if not np.all(np.isfinite(vector)):
            raise RuntimeError("joint state contains non-finite positions")
        return images, vector, state.ts

    def _load_policy(self) -> PolicyBackend:
        backend_module = import_module(f"dimos_{self.config.backend}.backend")
        return cast("PolicyBackend", backend_module.Backend(self.config))

    def _run_rollout(self) -> None:
        loaded_policy: PolicyBackend | None = None
        try:
            with self._lock:
                loaded_policy = self._loaded_policy
            if loaded_policy is None:
                raise RuntimeError("policy preflight has not passed")
            if self._stop_event.is_set():
                return
            self._reset_policy(loaded_policy)

            while not self._stop_event.is_set():
                with self._lock:
                    image, state, state_ts = self._snapshot_observation(time.time())
                inference_started = time.perf_counter()
                action_chunk = loaded_policy.predict(image, state, task=self.config.task)
                inference_s = time.perf_counter() - inference_started
                expected_width = len(self.config.joint_names)
                if action_chunk.ndim != 2:
                    raise RuntimeError(
                        f"policy returned action chunk shape {action_chunk.shape}, expected "
                        f"(steps, {expected_width})"
                    )
                if action_chunk.shape[1] != expected_width:
                    raise RuntimeError(
                        f"policy returned action width {action_chunk.shape[1]}, expected {expected_width}"
                    )
                if action_chunk.shape[0] < loaded_policy.n_action_steps:
                    raise RuntimeError(
                        f"policy returned {action_chunk.shape[0]} action steps, but n_action_steps "
                        f"is {loaded_policy.n_action_steps}"
                    )
                actions = action_chunk[: loaded_policy.n_action_steps]
                if not np.all(np.isfinite(actions)):
                    raise RuntimeError("policy returned non-finite joint targets")
                bounded_actions = (
                    np.clip(actions, loaded_policy.action_lower, loaded_policy.action_upper)
                    if loaded_policy.action_lower is not None
                    and loaded_policy.action_upper is not None
                    else actions
                )
                clipped = np.any(actions != bounded_actions, axis=0)
                if np.any(clipped):
                    logger.warning(
                        "Clipped policy actions to checkpoint range",
                        joints=[
                            name
                            for name, was_clipped in zip(
                                self.config.joint_names, clipped, strict=True
                            )
                            if was_clipped
                        ],
                    )
                actions = bounded_actions
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
                logger.info(
                    "Policy chunk accepted",
                    backend=self.config.backend,
                    inference_s=inference_s,
                    execution_steps=loaded_policy.n_action_steps,
                    fps=loaded_policy.fps,
                )
                self._stop_event.wait(loaded_policy.n_action_steps / loaded_policy.fps)
        except Exception as exc:
            with self._lock:
                self._last_error = str(exc)
            logger.exception("Policy execution stopped", error=str(exc))
        finally:
            self._stop_event.set()
            cancellation_error = self._cancel_trajectory()
            if loaded_policy is not None:
                self._reset_policy(loaded_policy)
            with self._lock:
                if cancellation_error is not None:
                    self._last_error = (
                        f"{self._last_error}; {cancellation_error}"
                        if self._last_error is not None
                        else cancellation_error
                    )
                self._active = False

    @staticmethod
    def _reset_policy(loaded_policy: PolicyBackend) -> None:
        loaded_policy.reset()

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
        assert self._loaded_policy is not None
        zeros = [0.0] * len(self.config.joint_names)
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
                time_from_start=(index + 1) / self._loaded_policy.fps,
            )
            for index, action in enumerate(actions)
        )
        return JointTrajectory(joint_names=list(self.config.joint_names), points=points)

    def _wait_for_newer_joint_state(self, previous_ts: float) -> None:
        with self._observation_changed:
            self._observation_changed.wait_for(
                lambda: self._stop_event.is_set()
                or (
                    self._latest_joint_state is not None
                    and self._latest_joint_state.ts > previous_ts
                )
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
        message = result.message or "Policy trajectory cancellation was uncertain"
        logger.error(
            "Policy trajectory cancellation was uncertain",
            error=message,
            task_name=self.config.trajectory_task_name,
        )
        return message


def __getattr__(name: str) -> Any:
    if not name.startswith("PolicyRuntime_"):
        raise AttributeError(name)
    ports = tuple(json.loads(bytes.fromhex(name.removeprefix("PolicyRuntime_")).decode()))
    cls = type(name, (_PolicyRuntime, policy_class(ports)), {"__module__": __name__})
    globals()[name] = cls
    return cls
