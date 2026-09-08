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

import sys
import types

import pytest

from dimos.utils.gpu_utils import is_cuda_available


def _fake_torch(is_available) -> types.ModuleType:
    torch = types.ModuleType("torch")
    torch.cuda = types.SimpleNamespace(is_available=is_available)
    return torch


def test_returns_false_when_torch_is_missing(monkeypatch: pytest.MonkeyPatch) -> None:
    # None in sys.modules makes `import torch` raise ImportError.
    monkeypatch.setitem(sys.modules, "torch", None)
    assert is_cuda_available() is False


@pytest.mark.parametrize("available", [True, False])
def test_passes_through_torch_cuda_is_available(
    monkeypatch: pytest.MonkeyPatch, available: bool
) -> None:
    monkeypatch.setitem(sys.modules, "torch", _fake_torch(lambda: available))
    assert is_cuda_available() is available


def test_returns_false_when_torch_cuda_raises(monkeypatch: pytest.MonkeyPatch) -> None:
    def boom() -> bool:
        raise RuntimeError("no CUDA driver")

    monkeypatch.setitem(sys.modules, "torch", _fake_torch(boom))
    assert is_cuda_available() is False
