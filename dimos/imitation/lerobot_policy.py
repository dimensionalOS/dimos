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

"""Run a trained LeRobot policy against live DimOS observations."""

from __future__ import annotations

from contextlib import nullcontext
from copy import copy
from importlib import import_module
from threading import Event, RLock, Thread, current_thread
import time
from typing import Any, Protocol

import numpy as np
from numpy.typing import NDArray
from pydantic import Field
from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.agents.capabilities import CAP_MOVEMENT
from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.protocol.service.spec import BaseConfig
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_IMAGE_FEATURE = "observation.images.image"
_STATE_FEATURE = "observation.state"
_ACTION_FEATURE = "action"
_TOOL_NAME = "execute_learned_policy"


class PolicyBackend(Protocol):
    """Minimal inference interface, separated so module behavior is testable without torch."""

    def reset(self) -> None: ...

    def predict(
        self,
        image: NDArray[np.uint8],
        state: NDArray[np.float32],
        *,
        task: str,
        robot_type: str,
    ) -> NDArray[np.float32]: ...


class LeRobotPolicyConfig(BaseConfig):
    """Configuration for one named learned policy."""

    policy_path: str
    task: str = ""
    device: str | None = None
    default_duration: float = Field(default=10.0, gt=0)


class LeRobotPolicyModuleConfig(ModuleConfig):
    policies: dict[str, LeRobotPolicyConfig] = Field(min_length=1)
    joint_names: list[str] = Field(min_length=1)
    fps: float = Field(default=15.0, gt=0)
    robot_type: str = ""
    max_observation_age_s: float = Field(default=0.5, gt=0)


class _LeRobotBackend:
    """Lazy LeRobot/torch adapter using the upstream inference pipeline."""

    def __init__(
        self,
        config: LeRobotPolicyModuleConfig,
        policy: LeRobotPolicyConfig,
    ) -> None:
        try:
            torch = import_module("torch")
            PreTrainedConfig = import_module("lerobot.configs.policies").PreTrainedConfig
            policy_factory = import_module("lerobot.policies.factory")
            get_policy_class = policy_factory.get_policy_class
            make_pre_post_processors = policy_factory.make_pre_post_processors
            prepare_observation_for_inference = import_module(
                "lerobot.policies.utils"
            ).prepare_observation_for_inference
            register_third_party_plugins = import_module(
                "lerobot.utils.import_utils"
            ).register_third_party_plugins
        except ImportError as exc:
            raise ImportError(
                "LeRobot policy inference is not installed. Run `uv sync --extra lerobot`."
            ) from exc

        register_third_party_plugins()
        policy_config = PreTrainedConfig.from_pretrained(policy.policy_path)
        if policy.device is not None:
            policy_config.device = policy.device
        if policy_config.device is None:
            raise RuntimeError("LeRobot did not resolve an inference device")

        self._validate_features(policy_config, len(config.joint_names))
        self._device = torch.device(policy_config.device)
        if self._device.type == "cuda" and not torch.cuda.is_available():
            raise RuntimeError(
                f"Policy requested device {policy_config.device!r}, but CUDA is not available"
            )

        policy_class = get_policy_class(policy_config.type)
        self._policy = policy_class.from_pretrained(policy.policy_path, config=policy_config)
        self._preprocessor, self._postprocessor = make_pre_post_processors(
            policy_cfg=policy_config,
            pretrained_path=policy.policy_path,
            preprocessor_overrides={"device_processor": {"device": str(self._device)}},
        )
        self._prepare_observation = prepare_observation_for_inference
        self._torch = torch
        self._use_amp = bool(policy_config.use_amp)

    @staticmethod
    def _validate_features(policy_config: Any, joint_count: int) -> None:
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
        if not state_shape or state_shape[0] != joint_count:
            raise ValueError(
                f"Policy state dimension {state_shape} does not match {joint_count} configured joints"
            )
        if not action_shape or action_shape[0] != joint_count:
            raise ValueError(
                f"Policy action dimension {action_shape} does not match {joint_count} configured joints"
            )

    def reset(self) -> None:
        self._policy.reset()
        self._preprocessor.reset()
        self._postprocessor.reset()

    def predict(
        self,
        image: NDArray[np.uint8],
        state: NDArray[np.float32],
        *,
        task: str,
        robot_type: str,
    ) -> NDArray[np.float32]:
        observation: dict[str, NDArray[Any]] = {
            _IMAGE_FEATURE: image,
            _STATE_FEATURE: state,
        }
        torch = self._torch
        with (
            torch.inference_mode(),
            torch.autocast(device_type="cuda")
            if self._device.type == "cuda" and self._use_amp
            else nullcontext(),
        ):
            prepared = self._prepare_observation(
                copy(observation),
                self._device,
                task=task,
                robot_type=robot_type,
            )
            prepared = self._preprocessor(prepared)
            action = self._policy.select_action(prepared)
            action = self._postprocessor(action)
        return np.asarray(action.squeeze(0).to("cpu").numpy(), dtype=np.float32)


def _load_policy_backend(
    config: LeRobotPolicyModuleConfig,
    policy: LeRobotPolicyConfig,
) -> PolicyBackend:
    return _LeRobotBackend(config, policy)


class LeRobotPolicyModule(Module):
    """Convert live image and joint state observations into streaming joint targets."""

    dedicated_worker = True
    config: LeRobotPolicyModuleConfig

    color_image: In[Image]
    coordinator_joint_state: In[JointState]
    joint_command: Out[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        if len(set(self.config.joint_names)) != len(self.config.joint_names):
            raise ValueError("joint_names must not contain duplicates")
        if any(not name.strip() for name in self.config.policies):
            raise ValueError("policy names must not be empty")
        self._lock = RLock()
        self._backends: dict[str, PolicyBackend] = {}
        self._latest_image: tuple[NDArray[np.uint8], float] | None = None
        self._latest_joint_state: JointState | None = None
        self._stop_event = Event()
        self._thread: Thread | None = None
        self._commands_sent = 0
        self._last_error: str | None = None
        self._active_policy_name: str | None = None
        self._active_task = ""
        self._active_tool_name: str | None = None

    @rpc
    def build(self) -> None:
        """Build the module; checkpoints are loaded when first invoked."""

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

    @skill(uses=[CAP_MOVEMENT], lifecycle="background")
    def execute_learned_policy(
        self,
        policy_name: str,
        duration: float | None = None,
    ) -> str:
        """Execute a configured learned policy against live camera and robot state.

        Args:
            policy_name: Name of the policy in this module's policy catalog.
            duration: Optional maximum execution time; uses the policy default when omitted.
        """
        return self.start_configured_policy(
            policy_name,
            tool_name=_TOOL_NAME,
            duration=duration,
        )

    def start_configured_policy(
        self,
        policy_name: str,
        *,
        tool_name: str,
        duration: float | None = None,
    ) -> str:
        """Start a catalog policy from a user-defined background ``@skill`` method.

        This method must be called by a method decorated with
        ``@skill(..., lifecycle="background")`` so DimOS can associate the
        execution updates with the user-facing tool name.
        """
        self.start_tool(tool_name)
        background_launched = False
        try:
            policy = self.config.policies.get(policy_name)
            if policy is None:
                available = ", ".join(sorted(self.config.policies))
                return f"Unknown learned policy {policy_name!r}. Available policies: {available}."
            execution_duration = policy.default_duration if duration is None else duration
            if execution_duration <= 0:
                return "Duration must be greater than zero."
            with self._lock:
                if self._thread is not None and self._thread.is_alive():
                    if self._active_tool_name == tool_name:
                        background_launched = True
                    return f"Learned policy {self._active_policy_name!r} is already running."
                self._snapshot_observation(time.time())
                backend = self._backends.get(policy_name)

            # Loading can take seconds. Keep observation callbacks flowing so
            # inference starts from a fresh camera frame and joint state.
            if backend is None:
                loaded_backend = _load_policy_backend(self.config, policy)
                with self._lock:
                    backend = self._backends.setdefault(policy_name, loaded_backend)
                logger.info("Loaded LeRobot policy %s from %s", policy_name, policy.policy_path)

            with self._lock:
                # Re-check after loading in case another direct caller started
                # a policy while this checkpoint was being initialized.
                if self._thread is not None and self._thread.is_alive():
                    if self._active_tool_name == tool_name:
                        background_launched = True
                    return f"Learned policy {self._active_policy_name!r} is already running."
                self._snapshot_observation(time.time())
                backend.reset()
                self._stop_event.clear()
                self._commands_sent = 0
                self._last_error = None
                self._active_policy_name = policy_name
                self._active_task = policy.task
                self._active_tool_name = tool_name
                self._thread = Thread(
                    target=self._run_policy,
                    args=(backend, execution_duration, policy.task, tool_name),
                    name=f"lerobot-policy-{policy_name}",
                    daemon=True,
                )
                self._thread.start()
                background_launched = True
            return (
                f"Learned policy {policy_name!r} started for up to {execution_duration:.1f}s. "
                "Use stop_learned_policy to stop early."
            )
        except Exception as exc:
            with self._lock:
                self._last_error = str(exc)
            return f"Learned policy did not start: {exc}"
        finally:
            if not background_launched:
                self.stop_tool(tool_name)

    @skill
    def stop_learned_policy(self) -> str:
        """Stop the running learned policy and hold the last commanded pose."""
        was_running = self._stop_policy()
        return "Learned policy stopped." if was_running else "Learned policy was not running."

    @rpc
    def policy_status(self) -> dict[str, Any]:
        """Return live execution status for CLIs and monitoring."""
        with self._lock:
            running = self._thread is not None and self._thread.is_alive()
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
            logger.warning("Ignoring non-uint8 RGB policy image: %s", image)
            return
        if rgb.data.ndim != 3 or rgb.data.shape[2] != 3:
            logger.warning("Ignoring policy image with unexpected shape: %s", rgb.data.shape)
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

    def _run_policy(
        self,
        backend: PolicyBackend,
        duration: float,
        task: str,
        tool_name: str,
    ) -> None:
        period = 1.0 / self.config.fps
        deadline = time.monotonic() + duration
        next_progress = time.monotonic() + 1.0
        try:
            while not self._stop_event.is_set() and time.monotonic() < deadline:
                tick_started = time.monotonic()
                with self._lock:
                    image, state = self._snapshot_observation(time.time())
                action = np.asarray(
                    backend.predict(
                        image,
                        state,
                        task=task,
                        robot_type=self.config.robot_type,
                    ),
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
                    commands_sent = self._commands_sent
                now = time.monotonic()
                if now >= next_progress:
                    self.tool_update(tool_name, f"Executed {commands_sent} policy steps")
                    next_progress = now + 1.0
                self._stop_event.wait(max(0.0, period - (time.monotonic() - tick_started)))
            if not self._stop_event.is_set():
                self.tool_update(tool_name, f"Policy completed after {self._commands_sent} steps")
        except Exception as exc:
            with self._lock:
                self._last_error = str(exc)
            logger.exception("LeRobot policy execution stopped: %s", exc)
            self.tool_update(tool_name, f"Policy stopped: {exc}")
        finally:
            self._stop_event.set()
            self.stop_tool(tool_name)

    def _stop_policy(self) -> bool:
        with self._lock:
            thread = self._thread
            was_running = thread is not None and thread.is_alive()
            tool_name = self._active_tool_name
            self._stop_event.set()
        if thread is not None and thread is not current_thread():
            thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        if tool_name is not None:
            self.stop_tool(tool_name)
        return was_running
