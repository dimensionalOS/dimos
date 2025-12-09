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

from dataclasses import dataclass
from functools import cached_property
from typing import Annotated, Any

import torch

from dimos.core.resource import Resource
from dimos.protocol.service import Configurable  # type: ignore[attr-defined]

# Device string type - 'cuda', 'cpu', 'cuda:0', 'cuda:1', etc.
DeviceType = Annotated[str, "Device identifier (e.g., 'cuda', 'cpu', 'cuda:0')"]


@dataclass
class LocalModelConfig:
    device: DeviceType = "cuda" if torch.cuda.is_available() else "cpu"
    dtype: torch.dtype = torch.float32
    warmup: bool = False
    autostart: bool = False


class LocalModel(Resource, Configurable[LocalModelConfig]):
    """Base class for all local GPU/CPU models.

    Implements Resource interface for lifecycle management.

    Subclasses MUST override:
        - _model: @cached_property that loads and returns the model

    Subclasses MAY override:
        - start() for custom initialization logic
        - stop() for custom cleanup logic
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
            autostart: If True, immediately load the model.
                       If False (default), model loads lazily on first use.
        """
        super().__init__(**kwargs)
        if self.config.warmup or self.config.autostart:
            self.start()

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

    def start(self) -> None:
        """Load the model (Resource interface).

        Subclasses should override to add custom initialization.
        """
        _ = self._model

    def stop(self) -> None:
        """Release model and free GPU memory (Resource interface).

        Subclasses should override and call super().stop() for custom cleanup.
        """
        import gc

        if "_model" in self.__dict__:
            del self.__dict__["_model"]

        # Reset torch.compile caches to free memory from compiled models
        # See: https://github.com/pytorch/pytorch/issues/105181
        try:
            import torch._dynamo

            torch._dynamo.reset()
        except (ImportError, AttributeError):
            pass

        gc.collect()
        if self.config.device.startswith("cuda") and torch.cuda.is_available():
            torch.cuda.empty_cache()

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
