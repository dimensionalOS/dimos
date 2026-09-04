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

"""LeRobot adapter for the shared DimOS policy rollout runtime."""

from __future__ import annotations

from collections.abc import Mapping
from contextlib import nullcontext
from dataclasses import dataclass
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
import torch

from dimos.imitation.policy.backend import PolicyBackendInfo
from dimos.imitation.policy.lerobot.module import (
    LeRobotPolicyConfig,
    OpenYamLeRobotPolicy,
)
from dimos.imitation.policy.runtime import declare_policy_runtime
from dimos.imitation.profile import ImageSource, PolicyIOProfile


@dataclass(frozen=True)
class _LoadedPolicy:
    policy: PreTrainedPolicy
    device: torch.device
    preprocessor: PolicyProcessorPipeline[RobotObservation, RobotObservation]
    postprocessor: PolicyProcessorPipeline[PolicyAction, PolicyAction]
    use_amp: bool


class LeRobotBackend:
    """Translate the generic profile-keyed arrays to a LeRobot checkpoint."""

    def __init__(self, config: LeRobotPolicyConfig) -> None:
        self.config = config
        self._loaded: _LoadedPolicy | None = None
        self._profile: PolicyIOProfile | None = None

    def load(self, profile: PolicyIOProfile) -> PolicyBackendInfo:
        register_third_party_plugins()
        policy_config = PreTrainedConfig.from_pretrained(self.config.artifact)
        if self.config.device is not None:
            policy_config.device = self.config.device
        if policy_config.device is None:
            raise RuntimeError("LeRobot did not resolve an inference device")
        _validate_features(policy_config, profile)

        device = torch.device(policy_config.device)
        if device.type == "cuda" and not torch.cuda.is_available():
            raise RuntimeError(
                f"Policy requested device {policy_config.device!r}, but CUDA is not available"
            )
        policy_class = get_policy_class(policy_config.type)
        policy = policy_class.from_pretrained(self.config.artifact, config=policy_config)
        preprocessor, postprocessor = make_pre_post_processors(
            policy_cfg=policy_config,
            pretrained_path=self.config.artifact,
            preprocessor_overrides={"device_processor": {"device": str(device)}},
        )
        width = len(profile.action.demonstration.joints)
        lower, upper = _checkpoint_action_bounds(postprocessor, width)
        n_action_steps = _positive_int_attribute(policy_config, "n_action_steps")
        chunk_length = _optional_int_attribute(policy_config, "chunk_size") or n_action_steps
        self._loaded = _LoadedPolicy(
            policy=policy,
            device=device,
            preprocessor=preprocessor,
            postprocessor=postprocessor,
            use_amp=bool(policy_config.use_amp),
        )
        self._profile = profile
        return PolicyBackendInfo(
            name="lerobot",
            chunk_length=chunk_length,
            preferred_execution_steps=n_action_steps,
            action_lower=lower,
            action_upper=upper,
        )

    def reset(self) -> None:
        loaded = self._require_loaded()
        _reset(loaded.policy)
        _reset(loaded.preprocessor)
        _reset(loaded.postprocessor)

    def predict(
        self,
        observations: Mapping[str, NDArray[Any]],
        task: str,
    ) -> NDArray[np.float32]:
        loaded = self._require_loaded()
        assert self._profile is not None
        with (
            torch.inference_mode(),
            torch.autocast(device_type="cuda")
            if loaded.device.type == "cuda" and loaded.use_amp
            else nullcontext(),
        ):
            prepared = prepare_observation_for_inference(
                dict(observations),
                loaded.device,
                task=task,
                robot_type=self._profile.robot_type,
            )
            prepared = loaded.preprocessor(prepared)
            predict = getattr(loaded.policy, "predict_action_chunk", None)
            if not callable(predict):
                raise TypeError("Policy does not provide predict_action_chunk()")
            action_chunk = loaded.postprocessor(predict(prepared))
        result = np.asarray(action_chunk.to("cpu").numpy(), dtype=np.float32)
        if result.ndim != 3 or result.shape[0] != 1:
            raise RuntimeError(f"LeRobot returned invalid batched action shape {result.shape}")
        return np.asarray(result[0], dtype=np.float32)

    def _require_loaded(self) -> _LoadedPolicy:
        if self._loaded is None:
            raise RuntimeError("LeRobot backend is not loaded")
        return self._loaded


LeRobotPolicyRuntime = declare_policy_runtime(
    "LeRobotPolicyRuntime",
    __name__,
    OpenYamLeRobotPolicy,
    LeRobotBackend,
)


def _validate_features(policy_config: PreTrainedConfig, profile: PolicyIOProfile) -> None:
    inputs = policy_config.input_features or {}
    outputs = policy_config.output_features or {}
    missing = set(profile.observations) - set(inputs)
    if missing:
        raise ValueError(f"LeRobot checkpoint is missing input features: {sorted(missing)}")
    if profile.action.key not in outputs:
        raise ValueError(f"LeRobot checkpoint has no {profile.action.key!r} output feature")
    if getattr(policy_config, "temporal_ensemble_coeff", None) is not None:
        raise ValueError("Policies using temporal ensembling are not supported")

    for key, source in profile.observations.items():
        actual = tuple(inputs[key].shape)
        expected = (
            (source.shape[2], source.shape[0], source.shape[1])
            if isinstance(source, ImageSource)
            else (len(source.joints),)
        )
        if actual != expected:
            raise ValueError(f"LeRobot feature {key!r} shape {actual} does not match {expected}")
    action_shape = tuple(outputs[profile.action.key].shape)
    expected_action = (len(profile.action.demonstration.joints),)
    if action_shape != expected_action:
        raise ValueError(f"LeRobot action shape {action_shape} does not match {expected_action}")


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
    shape = (expected_width,)
    if lower.shape != shape or upper.shape != shape:
        raise ValueError(f"Policy action range shape must be {shape}")
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
