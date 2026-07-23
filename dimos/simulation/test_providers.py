# Copyright 2025-2026 Dimensional Inc.
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

from typing import Any

import pytest

from dimos.simulation import providers
from dimos.simulation.providers import SimulationBinding, SimulationRequest


class _Provider:
    def build(self, request: SimulationRequest) -> SimulationBinding:
        raise NotImplementedError


class _EntryPoint:
    name = "test"

    def load(self) -> Any:
        return _Provider()


def test_load_external_simulation_provider(monkeypatch: pytest.MonkeyPatch) -> None:
    def entry_points(*, group: str, name: str | None = None) -> list[_EntryPoint]:
        assert group == providers.ENTRY_POINT_GROUP
        return [_EntryPoint()] if name in (None, "test") else []

    monkeypatch.setattr(providers.importlib_metadata, "entry_points", entry_points)

    assert isinstance(providers.load_simulation_provider("test"), _Provider)
