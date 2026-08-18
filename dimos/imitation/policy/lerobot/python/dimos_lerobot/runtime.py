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
from threading import Event, RLock, Thread, current_thread
import time
from typing import Any, Protocol, cast

from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.factory import get_policy_class, make_pre_post_processors
from lerobot.policies.pretrained import PreTrainedPolicy
from lerobot.policies.utils import prepare_observation_for_inference
from lerobot.processor import PolicyProcessorPipeline
from lerobot.utils.import_utils import register_third_party_plugins
import numpy as np
from numpy.typing import NDArray
from reactivex.disposable import Disposable
import torch
from torch import Tensor

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.imitation.policy.lerobot.module import (
    LeRobotPolicyConfig,
    LeRobotPolicyModule,
    PolicyStatus,
)
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_IMAGE_FEATURE = "observation.images.image"
_STATE_FEATURE = "observation.state"
_ACTION_FEATURE = "action"

RawObservation = dict[str, NDArray[np.uint8] | NDArray[np.float32]]
PreparedObservation = dict[str, Tensor | str]
PolicyBatch = dict[str, Tensor]


class _Resettable(Protocol):
    def reset(self) -> None: ...


@dataclass(frozen=True)
class _LoadedPolicy:
    policy: PreTrainedPolicy
    device: torch.device
    preprocessor: PolicyProcessorPipeline[PreparedObservation, PreparedObservation]
    postprocessor: PolicyProcessorPipeline[Tensor, Tensor]
    use_amp: bool


class LeRobotPolicyRuntime(LeRobotPolicyModule):
    """Concrete LeRobot implementation loaded by ``LeRobotPolicyModule``."""

    _lock: RLock
    _loaded_policies: dict[str, _LoadedPolicy]
    _latest_image: tuple[NDArray[np.uint8], float] | None
    _latest_joint_state: JointState | None
    _stop_event: Event
    _thread: Thread | None
    _commands_sent: int
    _last_error: str | None
    _active_policy_name: str | None
    _active_task: str

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = RLock()
        self._loaded_policies = {}
        self._latest_image = None
        self._latest_joint_state = None
        self._stop_event = Event()
        self._thread = None
        self._commands_sent = 0
        self._last_error = None
        self._active_policy_name = None
        self._active_task = ""

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.color_image.subscribe(self._on_color_image)))
        self.register_disposable(
            Disposable(self.coordinator_joint_state.subscribe(self._on_joint_state))
        )

    @rpc
    def stop(self) -> None:
        self._stop_policy()
        super().stop()

    @rpc
    def execute_learned_policy(
        self,
        policy_name: str,
        duration: float | None = None,
    ) -> str:
        policy = self.config.policies.get(policy_name)
        if policy is None:
            available = ", ".join(sorted(self.config.policies))
            return f"Unknown learned policy {policy_name!r}. Available policies: {available}."

        execution_duration = policy.default_duration if duration is None else duration
        if execution_duration <= 0:
            return "Duration must be greater than zero."

        try:
            with self._lock:
                if self._thread is not None and self._thread.is_alive():
                    return f"Learned policy {self._active_policy_name!r} is already running."
                self._snapshot_observation(time.time())
                loaded_policy = self._loaded_policies.get(policy_name)

            if loaded_policy is None:
                newly_loaded_policy = self._load_policy(policy)
                with self._lock:
                    loaded_policy = self._loaded_policies.setdefault(
                        policy_name, newly_loaded_policy
                    )
                logger.info(
                    "Loaded LeRobot policy",
                    policy=policy_name,
                    path=policy.policy_path,
                )

            with self._lock:
                if self._thread is not None and self._thread.is_alive():
                    return f"Learned policy {self._active_policy_name!r} is already running."
                self._snapshot_observation(time.time())
                cast("_Resettable", loaded_policy.policy).reset()
                cast("_Resettable", loaded_policy.preprocessor).reset()
                cast("_Resettable", loaded_policy.postprocessor).reset()
                self._stop_event.clear()
                self._commands_sent = 0
                self._last_error = None
                self._active_policy_name = policy_name
                self._active_task = policy.task
                self._thread = Thread(
                    target=self._run_policy,
                    args=(loaded_policy, execution_duration, policy.task),
                    name=f"lerobot-policy-{policy_name}",
                    daemon=True,
                )
                self._thread.start()
            return (
                f"Learned policy {policy_name!r} started for up to {execution_duration:.1f}s. "
                "Use stop_learned_policy to stop early."
            )
        except Exception as exc:
            with self._lock:
                self._last_error = str(exc)
            return f"Learned policy did not start: {exc}"

    @rpc
    def stop_learned_policy(self) -> str:
        was_running = self._stop_policy()
        return "Learned policy stopped." if was_running else "Learned policy was not running."

    @rpc
    def policy_status(self) -> PolicyStatus:
        with self._lock:
            running = (
                self._thread is not None
                and self._thread.is_alive()
                and not self._stop_event.is_set()
            )
            observation_error: str | None = None
            try:
                self._snapshot_observation(time.time())
            except RuntimeError as exc:
                observation_error = str(exc)
            return {
                "running": running,
                "observations_ready": observation_error is None,
                "observation_error": observation_error,
                "active_policy": self._active_policy_name,
                "policy_path": (
                    self.config.policies[self._active_policy_name].policy_path
                    if self._active_policy_name is not None
                    else None
                ),
                "available_policies": sorted(self.config.policies),
                "task": self._active_task,
                "commands_sent": self._commands_sent,
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
        with self._lock:
            self._latest_joint_state = JointState(state)

    def _snapshot_observation(self, now: float) -> tuple[NDArray[np.uint8], NDArray[np.float32]]:
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
        return image.copy(), vector

    def _load_policy(self, policy: LeRobotPolicyConfig) -> _LoadedPolicy:
        register_third_party_plugins()
        policy_config = PreTrainedConfig.from_pretrained(policy.policy_path)
        if policy.device is not None:
            policy_config.device = policy.device
        if policy_config.device is None:
            raise RuntimeError("LeRobot did not resolve an inference device")

        self._validate_features(policy_config)
        device = torch.device(policy_config.device)
        if device.type == "cuda" and not torch.cuda.is_available():
            raise RuntimeError(
                f"Policy requested device {policy_config.device!r}, but CUDA is not available"
            )

        policy_class = get_policy_class(policy_config.type)
        loaded_policy = policy_class.from_pretrained(policy.policy_path, config=policy_config)
        preprocessor, postprocessor = make_pre_post_processors(
            policy_cfg=policy_config,
            pretrained_path=policy.policy_path,
            preprocessor_overrides={"device_processor": {"device": str(device)}},
        )
        return _LoadedPolicy(
            policy=loaded_policy,
            device=device,
            preprocessor=cast(
                "PolicyProcessorPipeline[PreparedObservation, PreparedObservation]", preprocessor
            ),
            postprocessor=postprocessor,
            use_amp=bool(policy_config.use_amp),
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
            prepared = cast(
                "PreparedObservation",
                prepare_observation_for_inference(
                    observation,
                    loaded_policy.device,
                    task=task,
                    robot_type=self.config.robot_type,
                ),
            )
            prepared = loaded_policy.preprocessor(prepared)
            action = loaded_policy.policy.select_action(cast("PolicyBatch", prepared))
            action = loaded_policy.postprocessor(action)
        return np.asarray(action.squeeze(0).to("cpu").numpy(), dtype=np.float32)

    def _run_policy(
        self,
        loaded_policy: _LoadedPolicy,
        duration: float,
        task: str,
    ) -> None:
        period = 1.0 / self.config.fps
        deadline = time.monotonic() + duration
        try:
            while not self._stop_event.is_set() and time.monotonic() < deadline:
                tick_started = time.monotonic()
                with self._lock:
                    image, state = self._snapshot_observation(time.time())
                action = np.asarray(
                    self._predict(loaded_policy, image, state, task=task),
                    dtype=np.float32,
                ).reshape(-1)
                if action.shape != (len(self.config.joint_names),):
                    raise RuntimeError(
                        f"policy returned {action.shape}, expected "
                        f"({len(self.config.joint_names)},)"
                    )
                if not np.all(np.isfinite(action)):
                    raise RuntimeError("policy returned non-finite joint targets")
                if self._stop_event.is_set() or time.monotonic() >= deadline:
                    break

                self.joint_command.publish(
                    JointState(
                        name=list(self.config.joint_names),
                        position=action.astype(float).tolist(),
                    )
                )
                with self._lock:
                    self._commands_sent += 1
                self._stop_event.wait(max(0.0, period - (time.monotonic() - tick_started)))
        except Exception as exc:
            with self._lock:
                self._last_error = str(exc)
            logger.exception("LeRobot policy execution stopped", error=str(exc))
        finally:
            self._stop_event.set()

    def _stop_policy(self) -> bool:
        with self._lock:
            thread = self._thread
            was_running = thread is not None and thread.is_alive()
            self._stop_event.set()
        if thread is not None and thread is not current_thread():
            thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        return was_running
