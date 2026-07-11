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

from __future__ import annotations

from collections.abc import Callable, Mapping
from contextlib import AbstractContextManager, nullcontext
from dataclasses import dataclass
from importlib import import_module
from typing import cast

import numpy as np

from dimos.robot_learning.policy_rollout.models import (
    BackendBatch,
    BackendOutputEnvelope,
    PolicyBackendDescription,
)

_lerobot_make_pre_post_processors: object | None
_LEROBOT_PROCESSOR_IMPORT_ERROR: ImportError | None

try:
    from lerobot.policies import (  # type: ignore[import-not-found]
        make_pre_post_processors as _lerobot_make_pre_post_processors,
    )

    _LEROBOT_PROCESSOR_IMPORT_ERROR = None
except ImportError as exc:
    try:
        from lerobot.policies.factory import (  # type: ignore[import-not-found]
            make_pre_post_processors as _lerobot_make_pre_post_processors,
        )

        _LEROBOT_PROCESSOR_IMPORT_ERROR = None
    except ImportError:
        _lerobot_make_pre_post_processors = None
        _LEROBOT_PROCESSOR_IMPORT_ERROR = exc

_DEFAULT_POLICY_FAMILY = "vla_jepa"


@dataclass(frozen=True)
class _LeRobotPolicyFamilySpec:
    family: str
    default_checkpoint: str
    module: str
    class_name: str
    install_extra: str

    @property
    def class_path(self) -> str:
        return f"{self.module}.{self.class_name}"


_POLICY_FAMILIES = {
    "vla_jepa": _LeRobotPolicyFamilySpec(
        family="vla_jepa",
        default_checkpoint="lerobot/VLA-JEPA-LIBERO",
        module="lerobot.policies.vla_jepa.modeling_vla_jepa",
        class_name="VLAJEPAPolicy",
        install_extra="vla_jepa",
    ),
    "xvla": _LeRobotPolicyFamilySpec(
        family="xvla",
        default_checkpoint="lerobot/xvla-libero",
        module="lerobot.policies.xvla.modeling_xvla",
        class_name="XVLAPolicy",
        install_extra="xvla",
    ),
}
_DEFAULT_CHECKPOINT = _POLICY_FAMILIES[_DEFAULT_POLICY_FAMILY].default_checkpoint


class LeRobotBackend:
    """Optional in-process LeRobot policy backend for LIBERO policy families."""

    def __init__(
        self,
        *,
        checkpoint_id: str | None = None,
        policy_family: str = _DEFAULT_POLICY_FAMILY,
        device: str | None = None,
        use_action_chunk: bool = False,
    ) -> None:
        self._family_spec = _policy_family_spec(policy_family)
        self._checkpoint_id = checkpoint_id or self._family_spec.default_checkpoint
        self._device = device
        self._use_action_chunk = use_action_chunk
        self._policy: object | None = None
        self._preprocessor: Callable[[Mapping[str, object]], Mapping[str, object]] | None = None
        self._postprocessor: Callable[[object], object] | None = None
        self._policy_class_name: str | None = None
        self._processor_source: str | None = None
        self._supports_action_chunk_inference: bool | None = None

    def initialize(self) -> None:
        if self._policy is not None:
            return
        policy_cls = _load_policy_class(self._family_spec)
        from_pretrained = getattr(policy_cls, "from_pretrained", None)
        if not callable(from_pretrained):
            raise RuntimeError(
                f"LeRobot {self._family_spec.class_name}.from_pretrained was not found"
            )
        policy = from_pretrained(self._checkpoint_id)
        self._policy_class_name = _qualified_name(policy)
        self._supports_action_chunk_inference = callable(
            getattr(policy, "predict_action_chunk", None)
        )
        self._configure_device(policy)
        eval_policy = getattr(policy, "eval", None)
        if callable(eval_policy):
            eval_policy()
        self._policy = policy
        self._prepare_processors(policy)

    def reset_episode(self) -> None:
        policy = self._require_policy()
        reset_policy = getattr(policy, "reset", None)
        if callable(reset_policy):
            reset_policy()

    def infer_batch(self, batch: BackendBatch) -> BackendOutputEnvelope:
        policy = self._require_policy()
        backend_batch: Mapping[str, object] = _tensorized_preprocessor_input(batch.payload)
        if self._preprocessor is not None:
            backend_batch = self._preprocessor(backend_batch)
        with _torch_no_grad():
            output = self._infer(policy, backend_batch)
        if self._postprocessor is not None:
            output = self._postprocessor(output)
        action = _action_output_tuple(output, preserve_rank=self._use_action_chunk)
        return BackendOutputEnvelope(
            output=action,
            metadata={
                "backend_type": "lerobot",
                "policy_family": self._family_spec.family,
                "checkpoint_id": self._checkpoint_id,
                "inference_method": "predict_action_chunk"
                if self._use_action_chunk
                else "select_action",
                "output_shape": _shape_of(output),
                "batch_metadata": dict(batch.metadata),
            },
        )

    def close(self) -> None:
        self._policy = None
        self._preprocessor = None
        self._postprocessor = None

    def describe(self) -> PolicyBackendDescription:
        return PolicyBackendDescription(
            backend_type="lerobot",
            checkpoint_id=self._checkpoint_id,
            device=self._resolved_device(),
            policy_class=self._policy_class_name,
            supports_episode_reset=True,
            metadata={
                "policy_family": self._family_spec.family,
                "use_action_chunk": self._use_action_chunk,
                "inference_method": "predict_action_chunk"
                if self._use_action_chunk
                else "select_action",
                "configured_policy_class": self._family_spec.class_path,
                "supports_action_chunk_inference": self._supports_action_chunk_inference,
                "processor_source": self._processor_source,
            },
        )

    def _infer(self, policy: object, batch: Mapping[str, object]) -> object:
        method_name = "predict_action_chunk" if self._use_action_chunk else "select_action"
        infer = getattr(policy, method_name, None)
        if not callable(infer):
            raise RuntimeError(
                f"LeRobot {self._family_spec.family} policy does not expose {method_name}"
            )
        return infer(batch)

    def _configure_device(self, policy: object) -> None:
        if self._device is None:
            return
        config = getattr(policy, "config", None)
        if config is not None:
            config.device = self._device  # type: ignore[attr-defined]
        to_device = getattr(policy, "to", None)
        if callable(to_device):
            to_device(self._device)

    def _prepare_processors(self, policy: object) -> None:
        config = getattr(policy, "config", None)
        if config is None:
            raise RuntimeError("LeRobot policy does not expose config for processors")
        device = self._resolved_device()
        processors = _make_pre_post_processors(
            self._family_spec,
            policy_cfg=config,
            pretrained_path=self._checkpoint_id,
            preprocessor_overrides={"device_processor": {"device": str(device)}}
            if device is not None
            else None,
        )
        self._install_processors(processors)
        self._processor_source = "checkpoint"

    def _install_processors(self, processors: object) -> None:
        preprocessor, postprocessor = cast("tuple[object, object]", processors)
        if callable(preprocessor):
            self._preprocessor = cast(
                "Callable[[Mapping[str, object]], Mapping[str, object]]", preprocessor
            )
        if callable(postprocessor):
            self._postprocessor = postprocessor

    def _require_policy(self) -> object:
        self.initialize()
        if self._policy is None:
            raise RuntimeError("LeRobot policy did not initialize")
        return self._policy

    def _resolved_device(self) -> str | None:
        if self._device is not None:
            return self._device
        if self._policy is None:
            return None
        config = getattr(self._policy, "config", None)
        device = getattr(config, "device", None)
        return str(device) if device is not None else None


def _policy_family_spec(policy_family: str) -> _LeRobotPolicyFamilySpec:
    try:
        return _POLICY_FAMILIES[policy_family]
    except KeyError as exc:
        families = ", ".join(sorted(_POLICY_FAMILIES))
        raise ValueError(
            f"unsupported LeRobot policy family {policy_family!r}; expected one of {families}"
        ) from exc


def _load_policy_class(spec: _LeRobotPolicyFamilySpec) -> type[object]:
    try:
        module = import_module(spec.module)
    except ImportError as exc:
        raise RuntimeError(_family_install_hint(spec)) from exc
    policy_cls = getattr(module, spec.class_name, None)
    if policy_cls is None:
        raise RuntimeError(f"LeRobot {spec.class_path} was not found")
    return cast("type[object]", policy_cls)


def _family_install_hint(spec: _LeRobotPolicyFamilySpec) -> str:
    return (
        f"Install LeRobot from GitHub main with the {spec.install_extra} extra to use "
        f"LeRobotBackend policy_family={spec.family!r} for {spec.default_checkpoint}. "
        "Example: uv run --with "
        f'"lerobot[{spec.install_extra}] @ git+https://github.com/huggingface/lerobot.git" ...'
    )


def _make_pre_post_processors(
    spec: _LeRobotPolicyFamilySpec,
    **kwargs: object,
) -> tuple[object, object]:
    if not callable(_lerobot_make_pre_post_processors):
        raise RuntimeError(
            "LeRobot processor factory was not found; " + _family_install_hint(spec)
        ) from _LEROBOT_PROCESSOR_IMPORT_ERROR
    return cast("tuple[object, object]", _lerobot_make_pre_post_processors(**kwargs))


def _torch_no_grad() -> AbstractContextManager[object]:
    try:
        import torch  # type: ignore[import-not-found]
    except ImportError:
        return nullcontext()
    return cast("AbstractContextManager[object]", torch.no_grad())


def _qualified_name(value: object) -> str:
    cls = value.__class__
    return f"{cls.__module__}.{cls.__qualname__}"


def _tensorized_preprocessor_input(payload: Mapping[str, object]) -> dict[str, object]:
    prepared: dict[str, object] = {}
    for key, value in payload.items():
        if isinstance(value, np.ndarray):
            prepared[key] = _to_torch_tensor(key, value)
        else:
            prepared[key] = value
    return prepared


def _to_torch_tensor(key: str, value: np.ndarray) -> object:
    try:
        import torch  # type: ignore[import-not-found]
    except ImportError as exc:
        raise RuntimeError("Install torch to tensorize LeRobot backend inputs") from exc
    array = value
    if key.startswith("observation.images.") and array.ndim == 3:
        if array.shape[-1] in (1, 3):
            array = np.transpose(array, (2, 0, 1))
        if array.dtype == np.uint8:
            array = array.astype(np.float32) / 255.0
    return torch.as_tensor(array, dtype=torch.float32)


def _flat_float_tuple(value: object) -> tuple[float, ...]:
    array = _as_numpy_array(value).reshape(-1)
    return tuple(float(item) for item in array)


def _action_output_tuple(
    value: object,
    *,
    preserve_rank: bool,
) -> tuple[float, ...] | tuple[tuple[float, ...], ...]:
    if not preserve_rank:
        return _flat_float_tuple(value)
    array = _as_numpy_array(value)
    if array.ndim == 3 and array.shape[0] == 1:
        array = array[0]
    if array.ndim != 2:
        raise ValueError(f"LeRobot action chunk output must have rank 2, got shape {array.shape}")
    return tuple(tuple(float(item) for item in row) for row in array)


def _as_numpy_array(value: object) -> np.ndarray:
    if isinstance(value, np.ndarray):
        return value.astype(np.float32, copy=False)
    detach = getattr(value, "detach", None)
    if callable(detach):
        value = detach()
    cpu = getattr(value, "cpu", None)
    if callable(cpu):
        value = cpu()
    numpy = getattr(value, "numpy", None)
    if callable(numpy):
        return cast("np.ndarray", numpy()).astype(np.float32, copy=False)
    return np.asarray(value, dtype=np.float32)


def _shape_of(value: object) -> list[int]:
    shape = getattr(value, "shape", None)
    if shape is not None:
        return [int(dim) for dim in shape]
    if isinstance(value, (list, tuple)):
        if value and isinstance(value[0], (list, tuple)):
            return [len(value), len(value[0])]
        return [len(value)]
    return []


def create_backend(**params: object) -> LeRobotBackend:
    checkpoint_id = params.get("checkpoint_id")
    policy_family = params.get("policy_family", _DEFAULT_POLICY_FAMILY)
    device = params.get("device")
    use_action_chunk = params.get("use_action_chunk", False)
    return LeRobotBackend(
        checkpoint_id=str(checkpoint_id) if checkpoint_id is not None else None,
        policy_family=str(policy_family),
        device=cast("str | None", device),
        use_action_chunk=bool(use_action_chunk),
    )
