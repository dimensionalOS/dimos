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

from typing_extensions import override

from dimos.hardware.manipulators.a750.adapter import A750Adapter
from dimos.hardware.manipulators.openarm.adapter import OpenArmAdapter
from dimos.hardware.manipulators.piper.adapter import PiperAdapter


class _PiperSdk:
    def __init__(self) -> None:
        self.actions: list[str] = []

    def EnablePiper(self) -> bool:
        self.actions.append("enable")
        return True

    def MotionCtrl_2(self, **_: object) -> None:
        self.actions.append("position_mode")

    def EmergencyStop(self) -> None:
        self.actions.append("stop")

    def DisablePiper(self) -> None:
        self.actions.append("disable")


class _LifecyclePiperAdapter(PiperAdapter):
    def use_sdk(self, sdk: _PiperSdk) -> None:
        self._sdk: _PiperSdk | None
        self._sdk = sdk


def test_piper_lifecycle_enables_then_stops_and_disables() -> None:
    sdk = _PiperSdk()
    adapter = _LifecyclePiperAdapter()
    adapter.use_sdk(sdk)

    assert adapter.activate()
    assert adapter.deactivate()
    assert sdk.actions == ["enable", "position_mode", "stop", "disable"]


class _OpenArmLifecycle:
    def __init__(self) -> None:
        self.actions: list[str] = []

    def enable_all(self) -> None:
        self.actions.append("enable")

    def disable_all(self) -> None:
        self.actions.append("disable")


class _LifecycleOpenArmAdapter(OpenArmAdapter):
    def __init__(self, lifecycle: _OpenArmLifecycle) -> None:
        super().__init__()
        self._lifecycle: _OpenArmLifecycle
        self._lifecycle = lifecycle

    @override
    def read_joint_positions(self) -> list[float]:
        return [0.0] * 7

    @override
    def _compute_gravity_torques(self, q: list[float]) -> list[float]:
        return [0.0] * len(q)

    @override
    def write_enable(self, enable: bool) -> bool:
        if enable:
            self._lifecycle.enable_all()
        else:
            self._lifecycle.disable_all()
        return True

    @override
    def write_stop(self) -> bool:
        self._lifecycle.actions.append("hold")
        return True


def test_openarm_lifecycle_enables_then_holds_and_disables() -> None:
    lifecycle = _OpenArmLifecycle()
    adapter = _LifecycleOpenArmAdapter(lifecycle)

    assert adapter.activate()
    assert adapter.deactivate()
    assert lifecycle.actions == ["enable", "hold", "disable"]


class _A750Robot:
    def __init__(self) -> None:
        self.actions: list[str] = []

    def start_control_loop(self) -> None:
        self.actions.append("start")

    def stop_control_loop(self) -> None:
        self.actions.append("stop")


class _LifecycleA750Adapter(A750Adapter):
    def use_robot(self, robot: _A750Robot) -> None:
        self._robot: _A750Robot | None
        self._connected: bool
        self._robot = robot
        self._connected = True


def test_a750_lifecycle_starts_then_stops_control_loop() -> None:
    robot = _A750Robot()
    adapter = _LifecycleA750Adapter()
    adapter.use_robot(robot)

    assert adapter.activate()
    assert adapter.deactivate()
    assert robot.actions == ["start", "stop"]
