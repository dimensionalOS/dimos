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

"""Tests for LocalModel and HuggingFaceModel base classes."""

from functools import cached_property

import torch

from dimos.models.base import HuggingFaceModel, LocalModel


class ConcreteLocalModel(LocalModel):
    """Concrete implementation for testing."""

    @cached_property
    def _model(self) -> str:
        return "loaded_model"


class ConcreteHuggingFaceModel(HuggingFaceModel):
    """Concrete implementation for testing."""

    @cached_property
    def _model(self) -> str:
        return f"hf_model:{self.model_name}"


def test_local_model_device_auto_detection() -> None:
    """Test that device is auto-detected based on CUDA availability."""
    model = ConcreteLocalModel()
    expected = "cuda" if torch.cuda.is_available() else "cpu"
    assert model.device == expected


def test_local_model_explicit_device() -> None:
    """Test that explicit device is respected."""
    model = ConcreteLocalModel(device="cpu")
    assert model.device == "cpu"


def test_local_model_default_dtype() -> None:
    """Test that default dtype is float32 for LocalModel."""
    model = ConcreteLocalModel()
    assert model.dtype == torch.float32


def test_local_model_explicit_dtype() -> None:
    """Test that explicit dtype is respected."""
    model = ConcreteLocalModel(dtype=torch.float16)
    assert model.dtype == torch.float16


def test_local_model_lazy_loading() -> None:
    """Test that model is lazily loaded."""
    model = ConcreteLocalModel()
    # Model not loaded yet
    assert "_model" not in model.__dict__
    # Access triggers loading
    _ = model._model
    # Now it's cached
    assert "_model" in model.__dict__
    assert model._model == "loaded_model"


def test_local_model_start_triggers_loading() -> None:
    """Test that start() triggers model loading."""
    model = ConcreteLocalModel()
    assert "_model" not in model.__dict__
    model.start()
    assert "_model" in model.__dict__


def test_huggingface_model_inherits_local_model() -> None:
    """Test that HuggingFaceModel inherits from LocalModel."""
    assert issubclass(HuggingFaceModel, LocalModel)


def test_huggingface_model_default_dtype() -> None:
    """Test that default dtype is float16 for HuggingFaceModel."""
    model = ConcreteHuggingFaceModel(model_name="test/model")
    assert model.dtype == torch.float16


def test_huggingface_model_name() -> None:
    """Test model_name property."""
    model = ConcreteHuggingFaceModel(model_name="microsoft/Florence-2-large")
    assert model.model_name == "microsoft/Florence-2-large"


def test_huggingface_model_trust_remote_code() -> None:
    """Test trust_remote_code defaults to True."""
    model = ConcreteHuggingFaceModel(model_name="test/model")
    assert model.config.trust_remote_code is True

    model2 = ConcreteHuggingFaceModel(model_name="test/model", trust_remote_code=False)
    assert model2.config.trust_remote_code is False


def test_huggingface_start_loads_model() -> None:
    """Test that start() loads model."""
    model = ConcreteHuggingFaceModel(model_name="test/model")
    assert "_model" not in model.__dict__
    model.start()
    assert "_model" in model.__dict__


def test_move_inputs_to_device() -> None:
    """Test _move_inputs_to_device helper."""
    model = ConcreteHuggingFaceModel(model_name="test/model", device="cpu")

    inputs = {
        "input_ids": torch.tensor([1, 2, 3]),
        "attention_mask": torch.tensor([1, 1, 1]),
        "pixel_values": torch.randn(1, 3, 224, 224),
        "labels": "not_a_tensor",
    }

    moved = model._move_inputs_to_device(inputs)

    assert moved["input_ids"].device.type == "cpu"
    assert moved["attention_mask"].device.type == "cpu"
    assert moved["pixel_values"].device.type == "cpu"
    assert moved["pixel_values"].dtype == torch.float16  # dtype applied
    assert moved["labels"] == "not_a_tensor"  # non-tensor unchanged
