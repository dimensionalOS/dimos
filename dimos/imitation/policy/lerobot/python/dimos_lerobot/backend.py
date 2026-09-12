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

"""LeRobot inference adapter; lifecycle and execution live in the shared runtime."""

from __future__ import annotations

from contextlib import nullcontext
from typing import Any

from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.factory import get_policy_class, make_pre_post_processors
from lerobot.policies.utils import prepare_observation_for_inference
from lerobot.processor import PolicyProcessorPipeline
from lerobot.types import PolicyAction
from lerobot.utils.import_utils import register_third_party_plugins
import numpy as np
from numpy.typing import NDArray
import torch

from dimos.imitation.policy.backend import Images, joint_permutations
from dimos.imitation.policy.module import PolicyModuleConfig

_STATE_FEATURE = "observation.state"
_ACTION_FEATURE = "action"


class Backend:
    def __init__(self, config: PolicyModuleConfig) -> None:
        self.config = config
        self.to_policy, self.to_hardware = joint_permutations(
            config.joint_names, config.policy_joint_names or config.joint_names
        )
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
        action_lower, action_upper = _checkpoint_action_bounds(
            postprocessor,
            len(self.config.joint_names),
        )
        self.policy = loaded_policy
        self.device = device
        self.preprocessor = preprocessor
        self.postprocessor = postprocessor
        self.use_amp = bool(policy_config.use_amp)
        self.chunk_size = _optional_int_attribute(policy_config, "chunk_size")
        self.n_action_steps = config.execution_steps or _positive_int_attribute(
            policy_config, "n_action_steps"
        )
        self.fps = config.fps or 30.0
        self.action_lower = action_lower[self.to_hardware]
        self.action_upper = action_upper[self.to_hardware]

    def _validate_features(self, policy_config: PreTrainedConfig) -> None:
        inputs = policy_config.input_features or {}
        outputs = policy_config.output_features or {}
        missing = {*self.config.image_mapping.values(), _STATE_FEATURE} - set(inputs)
        if missing:
            raise ValueError(
                "Policy is incompatible with the configured DimOS runtime; "
                f"missing input features: {sorted(missing)}"
            )
        if _ACTION_FEATURE not in outputs:
            raise ValueError(f"Policy has no {_ACTION_FEATURE!r} output feature")
        if getattr(policy_config, "temporal_ensemble_coeff", None) is not None:
            raise ValueError("Policies using temporal ensembling are not supported")

        state_shape = tuple(inputs[_STATE_FEATURE].shape)
        action_shape = tuple(outputs[_ACTION_FEATURE].shape)
        joint_count = len(self.config.joint_names)
        self.image_shapes = {
            name: tuple(feature.shape)
            for name, feature in inputs.items()
            if name in self.config.image_mapping.values()
        }
        extra_images = {name for name in inputs if name.startswith("observation.images.")} - set(
            self.config.image_mapping.values()
        )
        if extra_images:
            raise ValueError(
                f"Missing camera bindings for checkpoint inputs: {sorted(extra_images)}"
            )
        if not state_shape or state_shape[0] != joint_count:
            raise ValueError(
                f"Policy state dimension {state_shape} does not match {joint_count} configured joints"
            )
        if not action_shape or action_shape[0] != joint_count:
            raise ValueError(
                f"Policy action dimension {action_shape} does not match {joint_count} configured joints"
            )

    def predict(
        self, images: Images, state: NDArray[np.float32], *, task: str
    ) -> NDArray[np.float32]:
        for name, image in images.items():
            if (3, *image.shape[:2]) != self.image_shapes[name]:
                raise ValueError(
                    f"Policy image shape for {name} does not match {self.image_shapes[name]}"
                )
        observation: dict[str, Any] = {**images, _STATE_FEATURE: state[self.to_policy]}
        with (
            torch.inference_mode(),
            torch.autocast(device_type="cuda")
            if self.device.type == "cuda" and self.use_amp
            else nullcontext(),
        ):
            prepared = prepare_observation_for_inference(
                observation,
                self.device,
                task=task,
                robot_type=self.config.robot_type,
            )
            prepared = self.preprocessor(prepared)
            predict = getattr(self.policy, "predict_action_chunk", None)
            if not callable(predict):
                raise TypeError("Policy does not provide predict_action_chunk()")
            action_chunk = self.postprocessor(predict(prepared))
        result = np.asarray(action_chunk.to("cpu").numpy(), dtype=np.float32)
        if result.ndim != 3 or result.shape[0] != 1:
            raise ValueError(f"Policy returned invalid action chunk shape {result.shape}")
        if result.shape[2] != len(self.config.joint_names):
            raise ValueError(f"Policy returned invalid action width {result.shape[2]}")
        return np.asarray(result[0][:, self.to_hardware], dtype=np.float32)

    def reset(self) -> None:
        _reset(self.policy)
        _reset(self.preprocessor)
        _reset(self.postprocessor)

    def close(self) -> None:
        self.reset()


def _checkpoint_action_bounds(
    postprocessor: PolicyProcessorPipeline[PolicyAction, PolicyAction],
    expected_width: int,
) -> tuple[NDArray[np.float32], NDArray[np.float32]]:
    lower_tensor: torch.Tensor | None = None
    upper_tensor: torch.Tensor | None = None
    for step in postprocessor.steps:
        state = step.state_dict()
        if "action.min" in state and "action.max" in state:
            lower_tensor = state["action.min"]
            upper_tensor = state["action.max"]
            break
    if lower_tensor is None or upper_tensor is None:
        raise ValueError("Policy postprocessor has no action min/max statistics")

    lower = np.asarray(lower_tensor.detach().cpu().numpy(), dtype=np.float32)
    upper = np.asarray(upper_tensor.detach().cpu().numpy(), dtype=np.float32)
    expected_shape = (expected_width,)
    if lower.shape != expected_shape or upper.shape != expected_shape:
        raise ValueError(
            "Policy action range shape does not match configured joints: "
            f"min={lower.shape}, max={upper.shape}, expected={expected_shape}"
        )
    if not np.all(np.isfinite(lower)) or not np.all(np.isfinite(upper)):
        raise ValueError("Policy action range contains non-finite values")
    if np.any(lower > upper):
        raise ValueError("Policy action range has min greater than max")
    return lower, upper


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
