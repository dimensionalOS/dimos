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

from abc import ABC, abstractmethod
from functools import cached_property
from typing import TYPE_CHECKING, Any

import torch

if TYPE_CHECKING:
    from transformers import ProcessorMixin


class LocalModel(ABC):
    """Base class for all local GPU/CPU models.

    Provides common infrastructure for device management, dtype handling,
    lazy model loading, and warmup functionality.

    Subclasses MUST implement:
        - _load_model() -> Any: Return the loaded model

    Subclasses MAY override:
        - warmup() for custom warmup logic
        - _default_dtype for different default dtype
    """

    _device: str
    _dtype: torch.dtype
    _default_dtype: torch.dtype = torch.float32

    def __init__(
        self,
        device: str | None = None,
        dtype: torch.dtype | None = None,
        warmup: bool = False,
    ) -> None:
        """Initialize local model with device and dtype configuration.

        Args:
            device: Device to run on ('cuda', 'cpu', 'cuda:0', etc.).
                    Auto-detects CUDA availability if None.
            dtype: Model dtype (torch.float16, torch.bfloat16, etc.).
                   Uses class _default_dtype if None.
            warmup: If True, immediately load and warmup the model.
                    If False (default), model loads lazily on first use.
        """
        self._device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        self._dtype = dtype if dtype is not None else self._default_dtype
        if warmup:
            self.warmup()

    @property
    def device(self) -> str:
        """The device this model runs on."""
        return self._device

    @property
    def dtype(self) -> torch.dtype:
        """The dtype used by this model."""
        return self._dtype

    @cached_property
    def _model(self) -> Any:
        """Lazily loaded model. Access triggers loading."""
        return self._load_model()

    @abstractmethod
    def _load_model(self) -> Any:
        """Load and return the model. Called once on first access to _model.

        Implementations should:
        1. Load the model from disk/hub
        2. Move to self._device
        3. Apply self._dtype if applicable
        4. Set to eval mode

        Returns:
            The loaded model object
        """
        ...

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
        if self._device.startswith("cuda") and torch.cuda.is_available():
            try:
                _ = torch.zeros(1, 1, device="cuda") @ torch.zeros(1, 1, device="cuda")
                torch.cuda.synchronize()
            except Exception:
                pass


class HuggingFaceModel(LocalModel):
    """Base class for HuggingFace transformers-based models.

    Provides common patterns for loading models and processors from
    the HuggingFace Hub using from_pretrained().

    Subclasses SHOULD set:
        - _model_class: The AutoModel class to use (e.g., AutoModelForCausalLM)

    Subclasses MAY override:
        - _load_model() for custom loading logic
        - _load_processor() for models with processors
    """

    _model_name: str
    _trust_remote_code: bool
    _default_dtype: torch.dtype = torch.float16
    _model_class: type | None = None  # e.g., AutoModelForCausalLM

    def __init__(
        self,
        model_name: str,
        device: str | None = None,
        dtype: torch.dtype | None = None,
        trust_remote_code: bool = True,
        warmup: bool = False,
    ) -> None:
        """Initialize HuggingFace model.

        Args:
            model_name: HuggingFace model identifier (e.g., "microsoft/Florence-2-large")
            device: Device to run on. Auto-detects if None.
            dtype: Model dtype. Defaults to float16 for GPU efficiency.
            trust_remote_code: Whether to trust remote code (needed for many vision models)
            warmup: If True, immediately load and warmup the model.
        """
        self._model_name = model_name
        self._trust_remote_code = trust_remote_code
        super().__init__(device=device, dtype=dtype, warmup=warmup)

    @property
    def model_name(self) -> str:
        """The HuggingFace model identifier."""
        return self._model_name

    @cached_property
    def _processor(self) -> ProcessorMixin | None:
        """Lazily loaded processor (tokenizer, image processor, etc.).

        Returns None if the model doesn't use a separate processor.
        """
        return self._load_processor()

    def _load_processor(self) -> ProcessorMixin | None:
        """Load and return the processor. Override in subclasses that need one.

        Default implementation returns None.
        """
        return None

    def _load_model(self) -> Any:
        """Load the HuggingFace model using _model_class.

        Override this method for custom loading logic (e.g., model.compile()).
        """
        if self._model_class is None:
            raise NotImplementedError(
                f"{self.__class__.__name__} must set _model_class or override _load_model()"
            )
        model = self._model_class.from_pretrained(
            self._model_name,
            trust_remote_code=self._trust_remote_code,
            torch_dtype=self._dtype,
        )
        return model.to(self._device)

    def _move_inputs_to_device(
        self,
        inputs: dict[str, torch.Tensor],
        apply_dtype: bool = True,
    ) -> dict[str, torch.Tensor]:
        """Move input tensors to model device with appropriate dtype.

        Args:
            inputs: Dictionary of input tensors
            apply_dtype: Whether to apply self._dtype to floating point tensors

        Returns:
            Dictionary with tensors moved to device
        """
        result = {}
        for k, v in inputs.items():
            if isinstance(v, torch.Tensor):
                if apply_dtype and v.is_floating_point():
                    result[k] = v.to(self._device, dtype=self._dtype)
                else:
                    result[k] = v.to(self._device)
            else:
                result[k] = v
        return result

    def warmup(self) -> None:
        """Warmup by loading model and processor."""
        _ = self._model
        _ = self._processor
