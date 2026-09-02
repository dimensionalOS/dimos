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

"""Run trained LeRobot policies in an isolated Python environment."""

from __future__ import annotations

from contextlib import nullcontext
from dataclasses import dataclass
from threading import Condition, Event, RLock, Thread, current_thread
import time
from typing import Any

from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.factory import get_policy_class, make_pre_post_processors
from lerobot.policies.pretrained import PreTrainedPolicy
from lerobot.policies.utils import prepare_observation_for_inference
from lerobot.processor import PolicyProcessorPipeline
from lerobot.types import PolicyAction, RobotObservation
from lerobot.utils.import_utils import register_third_party_plugins
import numpy as np
from numpy.typing import NDArray
from reactivex.disposable import Disposable
import torch

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.tasks.trajectory_task.trajectory_task import TrajectoryExecutionStatus
from dimos.core.core import rpc
from dimos.imitation.policy.lerobot.module import (
    LeRobotPolicyModule,
    RolloutStatus,
)
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.teleop.quest.quest_types import BUTTON_ALIASES, Buttons
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_IMAGE_FEATURE = "observation.images.wrist"
_STATE_FEATURE = "observation.state"
_ACTION_FEATURE = "action"

RawObservation = dict[str, NDArray[np.uint8] | NDArray[np.float32]]


@dataclass(frozen=True)
class _LoadedPolicy:
    policy: PreTrainedPolicy
    device: torch.device
    preprocessor: PolicyProcessorPipeline[RobotObservation, RobotObservation]
    postprocessor: PolicyProcessorPipeline[PolicyAction, PolicyAction]
    use_amp: bool
    chunk_size: int | None
    n_action_steps: int


class LeRobotPolicyRuntime(LeRobotPolicyModule):
    """Concrete LeRobot implementation loaded by ``LeRobotPolicyModule``."""

    _lock: RLock
    _observation_changed: Condition
    _loaded_policy: _LoadedPolicy | None
    _latest_image: tuple[NDArray[np.uint8], float] | None
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
        self._latest_image = None
        self._latest_joint_state = None
        self._stop_event = Event()
        self._thread = None
        self._chunks_accepted = 0
        self._last_error = None
        self._active = False

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.color_image.subscribe(self._on_color_image)))
        self.register_disposable(
            Disposable(self.coordinator_joint_state.subscribe(self._on_joint_state))
        )
        self.register_disposable(Disposable(self.button_pressed.subscribe(self._on_button_pressed)))

    @rpc
    def stop(self) -> None:
        self._stop_policy()
        super().stop()

    @rpc
    def start_rollout(self) -> RolloutStatus:
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                self._last_error = "a policy rollout is already active"
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
                name="lerobot-policy-rollout",
                daemon=True,
            )
            self._thread.start()
            return self._status_locked()

    @rpc
    def stop_rollout(self) -> RolloutStatus:
        self._cancel_trajectory()
        self._stop_policy()
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
            "policy_path": self.config.policy_path,
            "task": self.config.task,
            "device": self.config.device,
            "observations_ready": observations_ready,
            "chunks_accepted": self._chunks_accepted,
            "last_error": self._last_error,
        }

    def _on_color_image(self, image: Image) -> None:
        rgb = image.to_rgb()
        if rgb.format != ImageFormat.RGB or rgb.data.dtype != np.uint8:
            logger.warning("Ignoring non-uint8 RGB policy image", image=str(image))
            return
        if rgb.data.ndim != 3 or rgb.data.shape[2] != 3:
            logger.warning("Ignoring policy image with unexpected shape", shape=rgb.data.shape)
            return
        with self._lock:
            self._latest_image = (np.ascontiguousarray(rgb.data), rgb.ts)

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

    def _snapshot_observation(
        self, now: float
    ) -> tuple[NDArray[np.uint8], NDArray[np.float32], float]:
        if self._latest_image is None:
            raise RuntimeError("no camera image has been received")
        if self._latest_joint_state is None:
            raise RuntimeError("no coordinator joint state has been received")

        image, image_ts = self._latest_image
        state = self._latest_joint_state
        max_age = self.config.max_observation_age_s
        if now - image_ts > max_age:
            raise RuntimeError(f"camera image is stale by {now - image_ts:.2f}s")
        if now - state.ts > max_age:
            raise RuntimeError(f"joint state is stale by {now - state.ts:.2f}s")

        positions = dict(zip(state.name, state.position, strict=False))
        missing = [name for name in self.config.joint_names if name not in positions]
        if missing:
            raise RuntimeError(f"joint state is missing configured joints: {missing}")
        vector = np.asarray(
            [positions[name] for name in self.config.joint_names],
            dtype=np.float32,
        )
        if not np.all(np.isfinite(vector)):
            raise RuntimeError("joint state contains non-finite positions")
        return image.copy(), vector, state.ts

    def _load_policy(self) -> _LoadedPolicy:
        register_third_party_plugins()
        policy_config = PreTrainedConfig.from_pretrained(self.config.policy_path)
        if self.config.device is not None:
            policy_config.device = self.config.device
        if policy_config.device is None:
            raise RuntimeError("LeRobot did not resolve an inference device")

        self._validate_features(policy_config)
        device = torch.device(policy_config.device)
        if device.type == "cuda" and not torch.cuda.is_available():
            raise RuntimeError(
                f"Policy requested device {policy_config.device!r}, but CUDA is not available"
            )

        policy_class = get_policy_class(policy_config.type)
        loaded_policy = policy_class.from_pretrained(self.config.policy_path, config=policy_config)
        preprocessor, postprocessor = make_pre_post_processors(
            policy_cfg=policy_config,
            pretrained_path=self.config.policy_path,
            preprocessor_overrides={"device_processor": {"device": str(device)}},
        )
        return _LoadedPolicy(
            policy=loaded_policy,
            device=device,
            preprocessor=preprocessor,
            postprocessor=postprocessor,
            use_amp=bool(policy_config.use_amp),
            chunk_size=_optional_int_attribute(policy_config, "chunk_size"),
            n_action_steps=_positive_int_attribute(policy_config, "n_action_steps"),
        )

    def _validate_features(self, policy_config: PreTrainedConfig) -> None:
        inputs = policy_config.input_features or {}
        outputs = policy_config.output_features or {}
        missing = {_IMAGE_FEATURE, _STATE_FEATURE} - set(inputs)
        if missing:
            raise ValueError(
                "Policy is incompatible with the DimOS single-camera runtime; "
                f"missing input features: {sorted(missing)}"
            )
        if _ACTION_FEATURE not in outputs:
            raise ValueError(f"Policy has no {_ACTION_FEATURE!r} output feature")
        if getattr(policy_config, "temporal_ensemble_coeff", None) is not None:
            raise ValueError("Policies using temporal ensembling are not supported")

        state_shape = tuple(inputs[_STATE_FEATURE].shape)
        action_shape = tuple(outputs[_ACTION_FEATURE].shape)
        joint_count = len(self.config.joint_names)
        if not state_shape or state_shape[0] != joint_count:
            raise ValueError(
                f"Policy state dimension {state_shape} does not match {joint_count} configured joints"
            )
        if not action_shape or action_shape[0] != joint_count:
            raise ValueError(
                f"Policy action dimension {action_shape} does not match {joint_count} configured joints"
            )

    def _predict(
        self,
        loaded_policy: _LoadedPolicy,
        image: NDArray[np.uint8],
        state: NDArray[np.float32],
        *,
        task: str,
    ) -> NDArray[np.float32]:
        observation: RawObservation = {
            _IMAGE_FEATURE: image,
            _STATE_FEATURE: state,
        }
        with (
            torch.inference_mode(),
            torch.autocast(device_type="cuda")
            if loaded_policy.device.type == "cuda" and loaded_policy.use_amp
            else nullcontext(),
        ):
            prepared = prepare_observation_for_inference(
                observation,
                loaded_policy.device,
                task=task,
                robot_type=self.config.robot_type,
            )
            prepared = loaded_policy.preprocessor(prepared)
            predict = getattr(loaded_policy.policy, "predict_action_chunk", None)
            if not callable(predict):
                raise TypeError("Policy does not provide predict_action_chunk()")
            action_chunk = loaded_policy.postprocessor(predict(prepared))
        return np.asarray(action_chunk.to("cpu").numpy(), dtype=np.float32)

    def _run_rollout(self) -> None:
        loaded_policy: _LoadedPolicy | None = None
        try:
            with self._lock:
                loaded_policy = self._loaded_policy
            if loaded_policy is None:
                loaded_policy = self._load_policy()
                with self._lock:
                    self._loaded_policy = loaded_policy
                logger.info(
                    "Loaded LeRobot policy",
                    path=self.config.policy_path,
                    runtime_fps=self.config.fps,
                    chunk_size=loaded_policy.chunk_size,
                    n_action_steps=loaded_policy.n_action_steps,
                    action_horizon_s=loaded_policy.n_action_steps / self.config.fps,
                )
            if self._stop_event.is_set():
                return
            self._reset_policy(loaded_policy)

            while not self._stop_event.is_set():
                with self._lock:
                    image, state, state_ts = self._snapshot_observation(time.time())
                action_chunk = self._predict(loaded_policy, image, state, task=self.config.task)
                expected_width = len(self.config.joint_names)
                if action_chunk.ndim != 3 or action_chunk.shape[0] != 1:
                    raise RuntimeError(
                        f"policy returned action chunk shape {action_chunk.shape}, expected "
                        f"(1, steps, {expected_width})"
                    )
                if action_chunk.shape[2] != expected_width:
                    raise RuntimeError(
                        f"policy returned action width {action_chunk.shape[2]}, expected {expected_width}"
                    )
                if action_chunk.shape[1] < loaded_policy.n_action_steps:
                    raise RuntimeError(
                        f"policy returned {action_chunk.shape[1]} action steps, but n_action_steps "
                        f"is {loaded_policy.n_action_steps}"
                    )
                actions = action_chunk[0, : loaded_policy.n_action_steps]
                if not np.all(np.isfinite(actions)):
                    raise RuntimeError("policy returned non-finite joint targets")
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
                self._stop_event.wait(loaded_policy.n_action_steps / self.config.fps)
        except Exception as exc:
            self._cancel_trajectory()
            with self._lock:
                self._last_error = str(exc)
            logger.exception("LeRobot policy execution stopped", error=str(exc))
        finally:
            self._stop_event.set()
            if loaded_policy is not None:
                self._reset_policy(loaded_policy)
            with self._lock:
                self._active = False

    @staticmethod
    def _reset_policy(loaded_policy: _LoadedPolicy) -> None:
        _reset(loaded_policy.policy)
        _reset(loaded_policy.preprocessor)
        _reset(loaded_policy.postprocessor)

    def _stop_policy(self) -> bool:
        with self._lock:
            thread = self._thread
            was_running = thread is not None and thread.is_alive()
            self._stop_event.set()
            self._active = False
            self._observation_changed.notify_all()
        if thread is not None and thread is not current_thread():
            thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        return was_running

    def _trajectory(
        self,
        state: NDArray[np.float32],
        actions: NDArray[np.float32],
    ) -> JointTrajectory:
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
                time_from_start=(index + 1) / self.config.fps,
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

    def _cancel_trajectory(self) -> None:
        try:
            self._control.cancel_trajectory(task_name=self.config.trajectory_task_name)
        except Exception:
            logger.exception(
                "Failed to cancel policy trajectory",
                task_name=self.config.trajectory_task_name,
            )


def _reset(instance: object) -> None:
    reset = getattr(instance, "reset", None)
    if not callable(reset):
        raise TypeError(f"{type(instance).__name__} does not provide reset()")
    reset()


def _optional_int_attribute(instance: object, name: str) -> int | None:
    value = getattr(instance, name, None)
    if value is not None and not isinstance(value, int):
        raise TypeError(f"{name} must be an int, got {type(value).__name__}")
    return value


def _positive_int_attribute(instance: object, name: str) -> int:
    value = _optional_int_attribute(instance, name)
    if value is None or value <= 0:
        raise ValueError(f"{name} must be a positive int")
    return value
