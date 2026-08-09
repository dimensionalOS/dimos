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

from dimos.control.components import HardwareComponent, HardwareType
from dimos.core.coordination.blueprints import Blueprint
from dimos.core.global_config import global_config
from dimos.simulation import providers
from dimos.simulation.providers import (
    SimulationBinding,
    SimulationFeature,
    SimulationRequest,
    resolve_robot,
)


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


def test_simulation_binding_carries_ordered_composite_hardware() -> None:
    hardware = (
        HardwareComponent(
            hardware_id="base",
            hardware_type=HardwareType.BASE,
            joints=["base/vx", "base/vy", "base/wz"],
            adapter_type="sim_twist",
            address="run/robot/base",
        ),
        HardwareComponent(
            hardware_id="left_arm",
            hardware_type=HardwareType.MANIPULATOR,
            joints=["left_arm/joint1", "left_arm/joint2"],
            adapter_type="sim_mujoco",
            address="run/robot/left_arm",
        ),
        HardwareComponent(
            hardware_id="right_arm",
            hardware_type=HardwareType.MANIPULATOR,
            joints=["right_arm/joint1", "right_arm/joint2"],
            adapter_type="sim_mujoco",
            address="run/robot/right_arm",
        ),
    )

    binding = SimulationBinding(
        backend=Blueprint(()),
        hardware=hardware,
    )

    assert tuple(component.hardware_id for component in binding.hardware) == (
        "base",
        "left_arm",
        "right_arm",
    )


def test_simulation_binding_rejects_duplicate_hardware_ids() -> None:
    hardware = tuple(
        HardwareComponent(
            hardware_id="arm",
            hardware_type=HardwareType.MANIPULATOR,
            adapter_type="sim_mujoco",
            address=f"run/robot/arm-{index}",
        )
        for index in range(2)
    )

    with pytest.raises(ValueError, match="hardware IDs must be unique"):
        SimulationBinding(
            backend=Blueprint(()),
            hardware=hardware,
        )


def test_simulation_request_validates_feature_names() -> None:
    request = SimulationRequest(
        robot_model="arm",
        features=frozenset(("episode_control", "sensors")),  # type: ignore[arg-type]
    )

    assert request.features == frozenset(
        (SimulationFeature.EPISODE_CONTROL, SimulationFeature.SENSORS)
    )

    with pytest.raises(ValueError, match="not a valid SimulationFeature"):
        SimulationRequest(
            robot_model="arm",
            features=frozenset(("unknown",)),  # type: ignore[arg-type]
        )


def test_resolve_robot_keeps_real_hardware_without_a_provider(
    monkeypatch: pytest.MonkeyPatch,
    mocker,
) -> None:
    hardware = HardwareComponent(
        hardware_id="arm",
        hardware_type=HardwareType.MANIPULATOR,
        joints=["arm/joint1"],
        adapter_type="mock",
    )
    backend = Blueprint(())
    load_provider = mocker.patch.object(providers, "load_simulation_provider")
    monkeypatch.setattr(global_config, "simulation_provider", "")

    binding = resolve_robot(
        real_hardware=(hardware,),
        simulation=SimulationRequest(robot_model="arm"),
        default_backend=backend,
    )

    assert binding.backend is backend
    assert binding.hardware == (hardware,)
    load_provider.assert_not_called()


def test_resolve_robot_uses_provider_hardware_when_configured(
    monkeypatch: pytest.MonkeyPatch,
    mocker,
) -> None:
    request = SimulationRequest(robot_model="arm")
    simulated_hardware = HardwareComponent(
        hardware_id="arm",
        hardware_type=HardwareType.MANIPULATOR,
        joints=["arm/joint1"],
        adapter_type="sim_mujoco",
        address="run/arm",
    )
    expected = SimulationBinding(
        backend=Blueprint(()),
        hardware=(simulated_hardware,),
    )
    provider = mocker.Mock()
    provider.build.return_value = expected
    load_provider = mocker.patch.object(
        providers,
        "load_simulation_provider",
        return_value=provider,
    )
    monkeypatch.setattr(global_config, "simulation_provider", "pimsim")

    binding = resolve_robot(real_hardware=(), simulation=request)

    assert binding is expected
    load_provider.assert_called_once_with("pimsim")
    provider.build.assert_called_once_with(request)
