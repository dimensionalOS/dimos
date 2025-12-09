# Copyright 2025 Dimensional Inc.
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

"""Base classes for local GPU models."""

from __future__ import annotations

from abc import ABC
from dataclasses import dataclass
from functools import cached_property
from typing import Any

import torch

from dimos.protocol.service import Configurable  # type: ignore[attr-defined]

type DeviceType = str  # e.g., 'cuda', 'cpu', 'cuda:0'


@dataclass
class LocalModelConfig:
    device: DeviceType = "cuda" if torch.cuda.is_available() else "cpu"
    dtype: torch.dtype = torch.float32
    warmup: bool = False


class LocalModel(ABC, Configurable[LocalModelConfig]):
    """Base class for all local GPU/CPU models.

    Provides common infrastructure for device management, dtype handling,
    lazy model loading, and warmup functionality.

    Subclasses MUST override:
        - _model: @cached_property that loads and returns the model

    Subclasses MAY override:
        - warmup() for custom warmup logic
        - _default_dtype for different default dtype
    """

    default_config = LocalModelConfig
    config: LocalModelConfig

    def __init__(self, **kwargs: object) -> None:
        """Initialize local model with device and dtype configuration.

        Args:
            device: Device to run on ('cuda', 'cpu', 'cuda:0', etc.).
                    Auto-detects CUDA availability if None.
            dtype: Model dtype (torch.float16, torch.bfloat16, etc.).
                   Uses class _default_dtype if None.
            warmup: If True, immediately load and warmup the model.
                    If False (default), model loads lazily on first use.
        """
        super().__init__(**kwargs)
        if self.config.warmup:
            self.warmup()

    @property
    def device(self) -> str:
        """The device this model runs on."""
        return self.config.device

    @property
    def dtype(self) -> torch.dtype:
        """The dtype used by this model."""
        return self.config.dtype

    @cached_property
    def _model(self) -> Any:
        """Lazily loaded model. Subclasses must override this property."""
        raise NotImplementedError(f"{self.__class__.__name__} must override _model property")

    def warmup(self) -> None:
        """Warmup the model by triggering lazy loading.

        Subclasses should override to add actual inference warmup.
        """
        _ = self._model

    def _ensure_cuda_initialized(self) -> None:
        """Initialize CUDA context to prevent cuBLAS allocation failures.

        Some models (CLIP, TorchReID) fail if they are the first to use CUDA.
        Call this before model loading if needed.
        """
        if self.config.device.startswith("cuda") and torch.cuda.is_available():
            try:
                _ = torch.zeros(1, 1, device="cuda") @ torch.zeros(1, 1, device="cuda")
                torch.cuda.synchronize()
            except Exception:
                pass


@dataclass
class HuggingFaceModelConfig(LocalModelConfig):
    model_name: str = ""
    trust_remote_code: bool = True
    dtype: torch.dtype = torch.float16


class HuggingFaceModel(LocalModel):
    """Base class for HuggingFace transformers-based models.

    Provides common patterns for loading models from the HuggingFace Hub
    using from_pretrained().

    Subclasses SHOULD set:
        - _model_class: The AutoModel class to use (e.g., AutoModelForCausalLM)

    Subclasses MAY override:
        - _model: @cached_property for custom model loading
    """

    default_config = HuggingFaceModelConfig
    config: HuggingFaceModelConfig
    _model_class: Any = None  # e.g., AutoModelForCausalLM

    @property
    def model_name(self) -> str:
        """The HuggingFace model identifier."""
        return self.config.model_name

    @cached_property
    def _model(self) -> Any:
        """Load the HuggingFace model using _model_class.

        Override this property for custom loading logic.
        """
        if self._model_class is None:
            raise NotImplementedError(
                f"{self.__class__.__name__} must set _model_class or override _model property"
            )
        model = self._model_class.from_pretrained(
            self.config.model_name,
            trust_remote_code=self.config.trust_remote_code,
            torch_dtype=self.config.dtype,
        )
        return model.to(self.config.device)

    def _move_inputs_to_device(
        self,
        inputs: dict[str, torch.Tensor],
        apply_dtype: bool = True,
    ) -> dict[str, torch.Tensor]:
        """Move input tensors to model device with appropriate dtype.

        Args:
            inputs: Dictionary of input tensors
            apply_dtype: Whether to apply model dtype to floating point tensors

        Returns:
            Dictionary with tensors moved to device
        """
        result = {}
        for k, v in inputs.items():
            if isinstance(v, torch.Tensor):
                if apply_dtype and v.is_floating_point():
                    result[k] = v.to(self.config.device, dtype=self.config.dtype)
                else:
                    result[k] = v.to(self.config.device)
            else:
                result[k] = v
        return result

    def warmup(self) -> None:
        """Warmup by loading model."""
        _ = self._model
