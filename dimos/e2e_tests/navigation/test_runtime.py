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

import pytest

from dimos.e2e_tests.navigation.runtime import resolve_navigation_provider


def test_navigation_provider_defaults_to_dimsim() -> None:
    provider = resolve_navigation_provider({})

    assert provider.name == "dimsim"
    assert provider.simulator == "dimsim"
    assert provider.transport == "lcm"


def test_pimsim_provider_maps_to_mujoco_and_zenoh() -> None:
    provider = resolve_navigation_provider({"DIMOS_E2E_SIMULATOR": "pimsim"})

    assert provider.name == "pimsim"
    assert provider.simulator == "mujoco"
    assert provider.transport == "zenoh"
    assert "--simulation-provider" in provider.global_args


def test_navigation_provider_rejects_unknown_name() -> None:
    with pytest.raises(ValueError, match="Unknown DIMOS_E2E_SIMULATOR='unknown'"):
        resolve_navigation_provider({"DIMOS_E2E_SIMULATOR": "unknown"})
