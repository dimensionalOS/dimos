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

"""Shared observation, lifecycle and execution for isolated policy backends."""

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
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState, TrajectoryStatus
from dimos.teleop.webxr.controller_types import BUTTON_ALIASES, Buttons
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class _PolicyRuntime(PolicyModule):
    """One rollout controller for local joint policies and remote joint/EEF policies."""

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = RLock()
        self._submission_lock = RLock()
        self._observation_changed = Condition(self._lock)
        self._loaded_policy: PolicyBackend | None = None
        self._latest_images: dict[str, tuple[NDArray[np.uint8], float]] = {}
        self._latest_joint_state: JointState | None = None
        self._stop_event = Event()
        self._thread: Thread | None = None
        self._chunks_accepted = 0
        self._chunks_received = 0
        self._chunks_completed = 0
        self._last_error: str | None = None
        self._active = False
        self._stopping = False
        self._manual_control = False
        self._dispatch = False
        self._submitted_task: str | None = None
        self._last_preview: JointTrajectory | None = None

    @property
    def _task_name(self) -> str:
        return "joint_trajectory"

    @rpc
    def start(self) -> None:
        super().start()
        for port in self.config.image_mapping:
            self.register_disposable(
                Disposable(self.inputs[port].subscribe(partial(self._on_image, port)))
            )
        self.register_disposable(
            Disposable(self.coordinator_joint_state.subscribe(self._on_joint_state))
        )
        self.register_disposable(Disposable(self.button_pressed.subscribe(self._on_button_pressed)))
        self.register_disposable(Disposable(self.teleop_buttons.subscribe(self._on_teleop_buttons)))

    @rpc
    def stop(self) -> None:
        self.stop_rollout()
        with self._lock:
            if not self._busy() and self._loaded_policy is not None:
                self._loaded_policy.close()
                self._loaded_policy = None
        super().stop()

    def _busy(self) -> bool:
        return any(thread is not None and thread.is_alive() for thread in (self._thread,))

    @rpc
    def preflight_rollout(self) -> RolloutStatus:
        """Validate local configuration and live inputs; never request remote inference."""
        with self._lock:
            if self._busy() or self._active or self._stopping:
                self._last_error = "cannot preflight while a rollout is active or draining"
                return self._status_locked()
            try:
                self._snapshot_observation(time.time())
                if self._task_name not in self._control.list_tasks():
                    raise RuntimeError(f"ControlCoordinator is missing task {self._task_name!r}")
                if self._loaded_policy is None:
                    self._loaded_policy = self._load_policy()
                self._last_error = None
            except Exception as exc:
                self._last_error = str(exc)
                policy, self._loaded_policy = self._loaded_policy, None
                if policy is not None:
                    try:
                        policy.close()
                    except Exception as close_error:
                        self._record_error(f"policy close failed: {close_error}")
            return self._status_locked()

    @rpc
    def start_rollout(self, dispatch_actions: bool | None = None) -> RolloutStatus:
        """Start an episode. Dreamlabs defaults to preview; dispatch requires explicit opt-in."""
        with self._lock:
            try:
                if self._manual_control:
                    raise RuntimeError("release the controller grips before starting rollout")
                if self._busy() or self._active or self._stopping:
                    raise RuntimeError("a policy rollout is already active or draining")
                if self._submitted_task is not None:
                    raise RuntimeError("previous trajectory cancellation is uncertain; stop again")
                if self._loaded_policy is None:
                    raise RuntimeError("policy preflight has not passed")
                self._snapshot_observation(time.time())
            except Exception as exc:
                self._last_error = str(exc)
                return self._status_locked()
            self._stop_event.clear()
            self._chunks_accepted = self._chunks_received = self._chunks_completed = 0
            self._last_preview = None
            self._last_error = None
            self._dispatch = True if dispatch_actions is None else dispatch_actions
            self._active = True
            self._thread = Thread(target=self._run_rollout, name="policy-rollout", daemon=True)
            self._thread.start()
            return self._status_locked()

    @rpc
    def stop_rollout(self) -> RolloutStatus:
        """Invalidate pending replies, cancel submitted work, then drain owned workers."""
        with self._observation_changed:
            self._stopping = True
            self._stop_event.set()
            self._active = False
            self._observation_changed.notify_all()
        with self._submission_lock, self._lock:
            self._cancel_locked()
        for thread in (self._thread,):
            if thread is not None and thread is not current_thread():
                thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        with self._lock:
            if self._busy():
                self._record_error("policy workers are still draining")
            self._stopping = False
            self._dispatch = False
            return self._status_locked()

    @rpc
    def set_task(self, task: str) -> RolloutStatus:
        if not task.strip():
            raise ValueError("task must not be blank")
        self.stop_rollout()
        with self._lock:
            if self._busy() or self._submitted_task is not None:
                return self._status_locked()
            self.config.task = task
            return self._status_locked()

    @rpc
    def rollout_status(self) -> RolloutStatus:
        with self._lock:
            return self._status_locked()

    @rpc
    def get_last_action_response(self) -> dict[str, Any] | None:
        with self._lock:
            policy = self._loaded_policy
        return policy.get_last_action_response() if policy is not None else None

    @rpc
    def get_last_preview(self) -> JointTrajectory | None:
        with self._lock:
            return self._last_preview

    def _status_locked(self) -> RolloutStatus:
        try:
            self._snapshot_observation(time.time())
            ready = True
        except RuntimeError:
            ready = False
        return {
            "backend": self.config.backend,
            "active": self._active,
            "busy": self._busy(),
            "dispatch_actions": self._dispatch,
            "policy_path": self.config.policy_path,
            "task": self.config.task,
            "device": self.config.device,
            "policy_ready": self._loaded_policy is not None,
            "observations_ready": ready,
            "chunks_received": self._chunks_received,
            "chunks_accepted": self._chunks_accepted,
            "chunks_completed": self._chunks_completed,
            "last_error": self._last_error,
        }

    def _on_image(self, port: str, image: Image) -> None:
        if (
            image.format != ImageFormat.RGB
            or image.data.dtype != np.uint8
            or image.data.ndim != 3
            or image.data.shape[2] != 3
            or not image.data.size
        ):
            return
        if self.config.image_width is not None and self.config.image_height is not None:
            if image.data.shape != (self.config.image_height, self.config.image_width, 3):
                return
        with self._observation_changed:
            self._latest_images[port] = (np.ascontiguousarray(image.data).copy(), image.ts)
            self._observation_changed.notify_all()

    def _on_joint_state(self, state: JointState) -> None:
        with self._observation_changed:
            self._latest_joint_state = JointState(state)
            self._observation_changed.notify_all()

    def _on_teleop_buttons(self, buttons: Buttons) -> None:
        with self._observation_changed:
            self._manual_control = buttons.left_grip or buttons.right_grip
            take_over = self._manual_control and self._active
            if take_over:
                self._stop_event.set()
                self._active = False
                self._observation_changed.notify_all()
        if take_over:
            with self._submission_lock, self._lock:
                self._cancel_locked()

    def _on_button_pressed(self, buttons: Buttons) -> None:
        button = BUTTON_ALIASES.get(self.config.rollout_button, self.config.rollout_button)
        if bool(getattr(buttons, button)):
            if self.rollout_status()["active"]:
                self.stop_rollout()
            else:
                self.start_rollout()

    def _snapshot_observation(self, now: float) -> tuple[Images, NDArray[np.float32], float]:
        images: Images = {}
        stamps: list[float] = []
        for port, feature in self.config.image_mapping.items():
            if port not in self._latest_images:
                raise RuntimeError(f"no camera image has been received on {port}")
            image, timestamp = self._latest_images[port]
            images[feature] = image.copy()
            stamps.append(timestamp)
        state = self._latest_joint_state
        if state is None:
            raise RuntimeError("no coordinator joint state has been received")
        stamps.append(state.ts)
        if any(
            not np.isfinite(t) or not 0 <= now - t <= self.config.max_observation_age_s
            for t in stamps
        ):
            raise RuntimeError("observation is stale or has an invalid timestamp")
        if max(stamps) - min(stamps) > self.config.max_observation_skew_s:
            raise RuntimeError("camera/state timestamp skew exceeds configured tolerance")
        if len(state.name) != len(state.position) or len(set(state.name)) != len(state.name):
            raise RuntimeError("joint state names must be unique and match its position count")
        positions = dict(zip(state.name, state.position, strict=True))
        missing = set(self.config.joint_names) - positions.keys()
        if missing:
            raise RuntimeError(f"joint state is missing configured joints: {sorted(missing)}")
        vector = np.asarray([positions[name] for name in self.config.joint_names], dtype=np.float32)
        if not np.all(np.isfinite(vector)):
            raise RuntimeError("joint state contains non-finite positions")
        return images, vector, min(stamps)

    def _load_policy(self) -> PolicyBackend:
        module = import_module("dimos_lerobot.backend")
        return cast("PolicyBackend", module.Backend(self.config))

    def _run_rollout(self) -> None:
        policy = self._loaded_policy
        try:
            if policy is None:
                raise RuntimeError("policy preflight has not passed")
            policy.reset()
            while not self._stop_event.is_set():
                with self._lock:
                    images, state, state_ts = self._snapshot_observation(time.time())
                    task = self.config.task
                chunk = policy.predict(images, state, task=task)
                with self._lock:
                    if self._stop_event.is_set():
                        break
                    _, anchor, _ = self._snapshot_observation(time.time())
                    assert self._latest_joint_state is not None
                    state_ts = self._latest_joint_state.ts
                trajectory = self._prepare_trajectory(chunk, anchor, policy)
                retry_state = False
                # Serialize final submission against stop without blocking sensor ingestion.
                with self._submission_lock:
                    with self._lock:
                        if self._stop_event.is_set():
                            break
                        self._snapshot_observation(time.time())
                        self._last_preview = trajectory
                        self._chunks_received += 1
                        dispatch = self._dispatch
                    if dispatch:
                        result = self._control.task_invoke(
                            self._task_name, "execute", {"trajectory": trajectory}
                        )
                        retry_state = (
                            result.status is TrajectoryExecutionStatus.START_STATE_MISMATCH
                            and self.config.backend == "lerobot"
                        )
                        if not retry_state:
                            if result.status is not TrajectoryExecutionStatus.ACCEPTED:
                                raise RuntimeError(
                                    f"trajectory rejected: {result.status.name}: {result.message}"
                                )
                            with self._lock:
                                self._submitted_task = self._task_name
                                self._chunks_accepted += 1
                if retry_state:
                    self._wait_for_newer_joint_state(state_ts)
                    continue
                if self._dispatch:
                    self._wait_for_completion()
                else:
                    self._stop_event.wait(trajectory.duration)
        except Exception as exc:
            with self._lock:
                if not self._stop_event.is_set():
                    self._record_error(str(exc))
                    logger.exception("Policy rollout stopped", error=str(exc))
        finally:
            with self._submission_lock, self._observation_changed:
                self._stop_event.set()
                self._active = False
                self._cancel_locked()
                self._dispatch = False
                self._observation_changed.notify_all()
            if policy is not None:
                try:
                    policy.reset()
                except Exception as exc:
                    with self._lock:
                        self._record_error(f"policy reset failed: {exc}")
                        self._loaded_policy = None
                    try:
                        policy.close()
                    except Exception as close_error:
                        with self._lock:
                            self._record_error(f"policy close failed: {close_error}")

    def _wait_for_completion(self) -> None:
        while not self._stop_event.is_set():
            with self._lock:
                self._snapshot_observation(time.time())
                task = self._submitted_task
            if task is None:
                return
            method = "get_status"
            status = self._control.task_invoke(task, method, {"t_now": None})
            if not isinstance(status, TrajectoryStatus):
                raise RuntimeError("coordinator returned no trajectory status")
            if status.state == TrajectoryState.COMPLETED:
                with self._lock:
                    self._chunks_completed += 1
                    self._submitted_task = None
                return
            if status.state != TrajectoryState.EXECUTING:
                raise RuntimeError(f"trajectory ended in {status.state.name}")
            self._stop_event.wait(0.05)

    def _wait_for_newer_joint_state(self, previous_ts: float) -> None:
        with self._observation_changed:
            self._observation_changed.wait_for(
                lambda: self._stop_event.is_set()
                or (
                    self._latest_joint_state is not None
                    and self._latest_joint_state.ts > previous_ts
                ),
                timeout=self.config.max_observation_age_s,
            )

    def _prepare_trajectory(
        self,
        chunk: NDArray[np.float32],
        state: NDArray[np.float32],
        policy: PolicyBackend,
    ) -> JointTrajectory:
        width = len(self.config.joint_names)
        if chunk.ndim != 2 or chunk.shape[1] != width or chunk.shape[0] < policy.n_action_steps:
            raise ValueError(f"policy returned invalid action chunk shape {chunk.shape}")
        actions = chunk[: policy.n_action_steps]
        if not np.all(np.isfinite(actions)):
            raise ValueError("policy returned non-finite joint targets")
        if policy.action_lower is not None and policy.action_upper is not None:
            actions = np.clip(actions, policy.action_lower, policy.action_upper)
        zeros = [0.0] * width
        points = [TrajectoryPoint(positions=state.tolist(), velocities=zeros, time_from_start=0.0)]
        points.extend(
            TrajectoryPoint(
                positions=row.tolist(), velocities=zeros, time_from_start=(i + 1) / policy.fps
            )
            for i, row in enumerate(actions)
        )
        return JointTrajectory(joint_names=self.config.joint_names, points=points)

    def _record_error(self, message: str) -> None:
        self._last_error = f"{self._last_error}; {message}" if self._last_error else message

    def _cancel_locked(self) -> None:
        if self._submitted_task is None:
            return
        try:
            result = self._control.task_invoke(self._submitted_task, "cancel", {})
            if not result.safe:
                raise RuntimeError(result.message or "trajectory cancellation uncertain")
            self._submitted_task = None
        except Exception as exc:
            self._record_error(f"trajectory cancellation failed: {exc}")


def __getattr__(name: str) -> Any:
    if not name.startswith("PolicyRuntime_"):
        raise AttributeError(name)
    ports = tuple(json.loads(bytes.fromhex(name.removeprefix("PolicyRuntime_")).decode()))
    cls = type(name, (_PolicyRuntime, policy_class(ports)), {"__module__": __name__})
    globals()[name] = cls
    return cls
