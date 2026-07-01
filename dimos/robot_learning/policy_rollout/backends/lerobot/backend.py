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
from typing import cast

import numpy as np

from dimos.robot_learning.policy_rollout.models import (
    BackendBatch,
    BackendOutputEnvelope,
    PolicyBackendDescription,
)

_vla_jepa_policy_class: object | None
_lerobot_make_pre_post_processors: object | None
_LEROBOT_IMPORT_ERROR: ImportError | None

try:
    from lerobot.policies import (  # type: ignore[import-not-found]
        make_pre_post_processors as _lerobot_make_pre_post_processors,
    )
    from lerobot.policies.vla_jepa.modeling_vla_jepa import (  # type: ignore[import-not-found]
        VLAJEPAPolicy,
    )

    _vla_jepa_policy_class = VLAJEPAPolicy
    _LEROBOT_IMPORT_ERROR = None
except ImportError as exc:
    _vla_jepa_policy_class = None
    _lerobot_make_pre_post_processors = None
    _LEROBOT_IMPORT_ERROR = exc

_DEFAULT_CHECKPOINT = "lerobot/VLA-JEPA-LIBERO"


class LeRobotBackend:
    """Optional in-process LeRobot policy backend for VLA-JEPA LIBERO."""

    def __init__(
        self,
        *,
        checkpoint_id: str = _DEFAULT_CHECKPOINT,
        device: str | None = None,
        use_action_chunk: bool = False,
    ) -> None:
        self._checkpoint_id = checkpoint_id
        self._device = device
        self._use_action_chunk = use_action_chunk
        self._policy: object | None = None
        self._preprocessor: Callable[[Mapping[str, object]], Mapping[str, object]] | None = None
        self._postprocessor: Callable[[object], object] | None = None
        self._policy_class_name: str | None = None
        self._processor_source: str | None = None

    def initialize(self) -> None:
        if self._policy is not None:
            return
        policy_cls = _load_vla_jepa_policy_class()
        from_pretrained = getattr(policy_cls, "from_pretrained", None)
        if not callable(from_pretrained):
            raise RuntimeError("LeRobot VLAJEPAPolicy.from_pretrained was not found")
        policy = from_pretrained(self._checkpoint_id)
        self._policy_class_name = _qualified_name(policy)
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
        action = _flat_float_tuple(output)
        return BackendOutputEnvelope(
            output=action,
            metadata={
                "backend_type": "lerobot",
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
                "policy_family": "vla_jepa",
                "use_action_chunk": self._use_action_chunk,
                "processor_source": self._processor_source,
            },
        )

    def _infer(self, policy: object, batch: Mapping[str, object]) -> object:
        method_name = "predict_action_chunk" if self._use_action_chunk else "select_action"
        infer = getattr(policy, method_name, None)
        if not callable(infer):
            raise RuntimeError(f"LeRobot policy does not expose {method_name}")
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


def _load_vla_jepa_policy_class() -> type[object]:
    if _vla_jepa_policy_class is None:
        raise RuntimeError(
            "Install LeRobot from GitHub main with the vla_jepa extra to use "
            "LeRobotBackend for lerobot/VLA-JEPA-LIBERO. Example: uv run --with "
            '"lerobot[vla_jepa] @ git+https://github.com/huggingface/lerobot.git" ...'
        ) from _LEROBOT_IMPORT_ERROR
    return cast("type[object]", _vla_jepa_policy_class)


def _make_pre_post_processors(**kwargs: object) -> tuple[object, object]:
    if not callable(_lerobot_make_pre_post_processors):
        raise RuntimeError(
            "LeRobot processor factory was not found; install LeRobot with the vla_jepa extra"
        ) from _LEROBOT_IMPORT_ERROR
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
    checkpoint_id = params.get("checkpoint_id", _DEFAULT_CHECKPOINT)
    device = params.get("device")
    use_action_chunk = params.get("use_action_chunk", False)
    return LeRobotBackend(
        checkpoint_id=str(checkpoint_id),
        device=cast("str | None", device),
        use_action_chunk=bool(use_action_chunk),
    )
