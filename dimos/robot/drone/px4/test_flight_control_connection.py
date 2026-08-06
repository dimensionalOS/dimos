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

import asyncio
import json
from types import ModuleType
from typing import Protocol

import pytest

from dimos.core.global_config import GlobalConfig, global_config
from dimos.core.module import Module
from dimos.robot.drone.px4 import mavsdk_runtime
from dimos.robot.drone.px4.config import MavsdkConfig
from dimos.robot.drone.px4.errors import MavsdkConnectionTimeoutError, MavsdkUnavailableError
from dimos.robot.drone.px4.flight_control import FlightController
from dimos.robot.drone.px4.mavsdk_runtime import (
    MavsdkOdometryValues,
    MavsdkRuntime,
    _load_mavsdk_runtime,
)


class _FakeTelemetry(Protocol):
    position_received: asyncio.Event
    yaw_received: asyncio.Event


class _FakeOffboard(Protocol):
    velocity_setpoints: list[tuple[float, float, float, float]]
    position_setpoints: list[tuple[float, float, float, float]]
    start_calls: int
    stop_calls: int


class _FakeSystem(Protocol):
    connection_urls: list[str | None]
    telemetry: _FakeTelemetry
    offboard: _FakeOffboard


def _skip_module_initialization(module: Module, *, g: GlobalConfig) -> None:
    del module, g


def _skip_module_lifecycle(module: Module) -> None:
    del module


def test_flight_control_imports_without_mavsdk() -> None:
    assert FlightController.__module__ == "dimos.robot.drone.px4.flight_control"


def test_vision_estimate_factory_receives_reset_counter() -> None:
    class VisionPositionEstimate:
        def __init__(
            self,
            time_usec: int,
            position_body: object,
            angle_body: object,
            pose_covariance: object,
            reset_counter: int,
        ) -> None:
            del time_usec, position_body, angle_body, pose_covariance
            self.reset_counter = reset_counter

    runtime = mavsdk_runtime._LoadedMavsdkRuntime(
        mavsdk_runtime._MavsdkBindings(
            system_factory=object,
            action_error=RuntimeError,
            mocap_error=RuntimeError,
            offboard_error=RuntimeError,
            velocity_factory=lambda *_args: object(),
            position_factory=lambda *_args: object(),
            position_body_factory=lambda *_args: object(),
            angle_body_factory=lambda *_args: object(),
            covariance_factory=lambda _values: object(),
            estimate_factory=VisionPositionEstimate,
        )
    )
    estimate = runtime.vision_position_estimate(
        MavsdkOdometryValues(
            time_usec=1,
            position_body=(1.0, 2.0, 3.0),
            quaternion=(1.0, 0.0, 0.0, 0.0),
            speed_body=(0.0, 0.0, 0.0),
            angular_velocity_body=(0.0, 0.0, 0.0),
            pose_covariance=(0.0,) * 21,
            velocity_covariance=(0.0,) * 21,
        )
    )

    assert isinstance(estimate, VisionPositionEstimate)
    assert estimate.reset_counter == 0


def test_flight_controller_forwards_worker_global_config(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    forwarded: list[GlobalConfig] = []

    def module_initialization(module: Module, *, g: GlobalConfig) -> None:
        del module
        forwarded.append(g)

    monkeypatch.setattr(Module, "__init__", module_initialization)

    FlightController(g=global_config)

    assert forwarded == [global_config]


def test_flight_controller_registers_all_command_skills(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(Module, "__init__", _skip_module_initialization)
    flight_control = FlightController()

    assert {skill.func_name for skill in flight_control.get_skills()} >= {
        "arm",
        "disarm",
        "takeoff",
        "land",
        "move",
        "goto",
        "enter_offboard",
        "exit_offboard",
        "hold",
        "hover",
    }


@pytest.mark.asyncio
async def test_connection_raises_clear_error_when_mavsdk_is_unavailable(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def import_module(module_name: str) -> ModuleType:
        del module_name
        raise ModuleNotFoundError(name="mavsdk")

    monkeypatch.setattr(Module, "__init__", _skip_module_initialization)
    monkeypatch.setattr(mavsdk_runtime, "import_module", import_module)
    flight_control = FlightController()

    with pytest.raises(MavsdkUnavailableError, match="MAVSDK"):
        await flight_control._start_mavsdk()


@pytest.mark.asyncio
async def test_connection_times_out_when_the_autopilot_never_connects(
    monkeypatch: pytest.MonkeyPatch,
    fake_system: _FakeSystem,
    fake_runtime: MavsdkRuntime,
) -> None:
    class NeverConnectedCore:
        async def connection_state(self):
            _ = await asyncio.Event().wait()
            yield None

    monkeypatch.setattr(Module, "__init__", _skip_module_initialization)
    monkeypatch.setattr(fake_system, "_core", NeverConnectedCore())
    flight_control = FlightController(
        mavsdk_config=MavsdkConfig(
            connection_url="udpin://0.0.0.0:14540",
            connection_timeout_s=0.01,
        ),
        mavsdk_runtime=fake_runtime,
    )

    with pytest.raises(MavsdkConnectionTimeoutError, match="14540"):
        await flight_control._start_mavsdk()


@pytest.mark.parametrize("module_name", ("mavsdk.action", "mavsdk.mocap", "mavsdk.offboard"))
def test_runtime_raises_clear_error_when_mavsdk_submodule_is_unavailable(
    monkeypatch: pytest.MonkeyPatch,
    module_name: str,
) -> None:
    def import_module(name: str) -> ModuleType:
        if name == module_name:
            raise ModuleNotFoundError(name=name)
        return ModuleType(name)

    monkeypatch.setattr(mavsdk_runtime, "import_module", import_module)

    with pytest.raises(MavsdkUnavailableError, match="MAVSDK"):
        _load_mavsdk_runtime()


def test_runtime_propagates_unrelated_missing_module(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def import_module(name: str) -> ModuleType:
        del name
        raise ModuleNotFoundError(name="unrelated_dependency")

    monkeypatch.setattr(mavsdk_runtime, "import_module", import_module)

    with pytest.raises(ModuleNotFoundError) as error:
        _load_mavsdk_runtime()

    assert error.value.name == "unrelated_dependency"


@pytest.mark.asyncio
async def test_connect_starts_embedded_server_with_the_configured_mavlink_url(
    monkeypatch: pytest.MonkeyPatch,
    fake_system: _FakeSystem,
    fake_runtime: MavsdkRuntime,
) -> None:
    # Given: a MAVSDK runtime fake and a configured MAVLink endpoint.
    monkeypatch.setattr(Module, "__init__", _skip_module_initialization)
    monkeypatch.setattr(Module, "start", _skip_module_lifecycle)
    monkeypatch.setattr(Module, "stop", _skip_module_lifecycle)
    monkeypatch.setattr(
        "dimos.robot.drone.px4.mavsdk_runtime._load_mavsdk_runtime",
        lambda: fake_runtime,
    )
    flight_control = FlightController(
        mavsdk_config=MavsdkConfig(connection_url="udpin://0.0.0.0:14540"),
    )
    monkeypatch.setattr(flight_control, "_loop", asyncio.get_running_loop())

    # When: the flight-control lifecycle starts.
    await asyncio.to_thread(flight_control.start)

    # Then: System owns its embedded server and receives the MAVLink URL directly.
    assert fake_system.connection_urls == ["udpin://0.0.0.0:14540"]
    _ = await fake_system.telemetry.position_received.wait()
    _ = await fake_system.telemetry.yaw_received.wait()
    assert json.loads(await flight_control.get_status()) == {
        "armed": None,
        "connected": True,
        "flight_mode": None,
        "in_air": None,
    }
    assert await flight_control.enter_offboard() == "offboard mode entered"
    assert await flight_control.move(forward=1.0) == "move command sent"
    assert await flight_control.goto(4.0, 5.0, -6.0, 90.0) == "goto command sent"
    assert await flight_control.hover() == "hover command sent"
    assert await flight_control.hold() == "hold command sent"
    assert await flight_control.exit_offboard() == "offboard mode exited"
    assert fake_system.offboard.velocity_setpoints == [
        (0.0, 0.0, 0.0, 0.0),
        (1.0, -0.0, -0.0, -0.0),
    ]
    assert fake_system.offboard.position_setpoints == [
        (4.0, 5.0, -6.0, 90.0),
        (1.0, 2.0, -3.0, 30.0),
        (1.0, 2.0, -3.0, 30.0),
    ]
    assert fake_system.offboard.start_calls == 1
    assert fake_system.offboard.stop_calls == 1

    await asyncio.to_thread(flight_control.stop)

    assert json.loads(await flight_control.get_status())["connected"] is False
    assert not hasattr(flight_control, "_system")
