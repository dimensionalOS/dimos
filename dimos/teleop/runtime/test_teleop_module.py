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

from collections.abc import Sequence
from typing import Any

import pytest

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.teleop.openarm_mini.config import OpenArmMiniTeleopConfig
from dimos.teleop.runtime.teleop_module import TeleopModule
from dimos.teleop.runtime.types import TeleopCommand


class _PublishedCommands:
    def __init__(self) -> None:
        self.messages: list[object] = []

    def publish(self, msg: object) -> None:
        self.messages.append(msg)


class _Adapter:
    def __init__(self, commands: Sequence[TeleopCommand | None] | None = None) -> None:
        self.commands = list(commands) if commands is not None else []
        self.connected = False
        self.disconnected = False

    def connect(self) -> None:
        self.connected = True

    def disconnect(self) -> None:
        self.disconnected = True

    def get_current_command(self) -> TeleopCommand | None:
        if not self.commands:
            return None
        return self.commands.pop(0)


def _module(adapter: _Adapter) -> TeleopModule:
    module = TeleopModule(
        runtime_adapter=adapter,
        max_publish_rate_hz=100.0,
        stale_command_timeout_s=1.0,
    )
    module.joint_command = _PublishedCommands()  # type: ignore[assignment]
    module.coordinator_cartesian_command = _PublishedCommands()  # type: ignore[assignment]
    module.twist_command = _PublishedCommands()  # type: ignore[assignment]
    return module


def test_command_envelope_requires_payload_unless_stopping() -> None:
    joint = JointState({"name": ["j0"], "position": [1.0]})

    assert TeleopCommand(joint).payload is joint
    assert TeleopCommand(stop=True).stop
    with pytest.raises(ValueError, match="payload"):
        TeleopCommand()
    with pytest.raises(ValueError, match="payload"):
        TeleopCommand(joint, stop=True)


def test_module_config_creates_adapter_from_discriminated_backend() -> None:
    module = TeleopModule(adapter={"backend": "openarm_mini", "enabled_sides": ("right",)})

    try:
        assert isinstance(module.teleop_config.adapter, OpenArmMiniTeleopConfig)
        assert module.teleop_config.adapter.enabled_sides == ("right",)
    finally:
        module.stop()


def test_explicit_stop_command_is_not_published(mocker: Any) -> None:
    adapter = _Adapter([TeleopCommand(timestamp=1.0, stop=True)])
    module = _module(adapter)
    mocker.patch.object(module, "_now", return_value=1.0)

    try:
        module.tick()

        assert module.joint_command.messages == []  # type: ignore[attr-defined]
    finally:
        module.stop()


def test_tick_routes_only_active_primary_output(mocker: Any) -> None:
    joint = JointState({"name": ["j0"], "position": [1.0]})
    cartesian = PoseStamped(position=[1.0, 2.0, 3.0])
    twist = Twist(linear=[1.0, 0.0, 0.0], angular=[0.0, 0.0, 0.0])
    modules = [
        _module(_Adapter([TeleopCommand(joint, timestamp=1.0)])),
        _module(_Adapter([TeleopCommand(cartesian, timestamp=1.0)])),
        _module(_Adapter([TeleopCommand(twist, timestamp=1.0)])),
    ]
    for module in modules:
        mocker.patch.object(module, "_now", return_value=1.0)

    try:
        for module in modules:
            module.tick()

        assert modules[0].joint_command.messages == [joint]  # type: ignore[attr-defined]
        assert modules[1].coordinator_cartesian_command.messages == [cartesian]  # type: ignore[attr-defined]
        assert modules[2].twist_command.messages == [twist]  # type: ignore[attr-defined]
    finally:
        for module in modules:
            module.stop()


def test_stale_commands_are_not_published(mocker: Any) -> None:
    joint = JointState({"name": ["j0"], "position": [1.0]})
    adapter = _Adapter([TeleopCommand(joint, timestamp=1.0)])
    module = _module(adapter)
    mocker.patch.object(module, "_now", return_value=2.01)

    try:
        module.tick()

        assert module.joint_command.messages == []  # type: ignore[attr-defined]
    finally:
        module.stop()


def test_rate_limiting_skips_commands(mocker: Any) -> None:
    first = JointState({"name": ["j0"], "position": [1.0]})
    second = JointState({"name": ["j0"], "position": [2.0]})
    adapter = _Adapter(
        [
            TeleopCommand(first, timestamp=1.0),
            TeleopCommand(second, timestamp=1.0),
        ]
    )
    module = TeleopModule(
        runtime_adapter=adapter,
        max_publish_rate_hz=10.0,
        stale_command_timeout_s=1.0,
    )
    module.joint_command = _PublishedCommands()  # type: ignore[assignment]
    module.coordinator_cartesian_command = _PublishedCommands()  # type: ignore[assignment]
    module.twist_command = _PublishedCommands()  # type: ignore[assignment]
    mocker.patch.object(module, "_now", side_effect=[1.0, 1.0, 1.0, 1.05, 1.05])

    try:
        module.tick()
        module.tick()

        assert module.joint_command.messages == [first]  # type: ignore[attr-defined]
    finally:
        module.stop()


def test_start_stop_connect_disconnect_and_no_publish_after_stop(
    mocker: Any,
) -> None:
    joint = JointState({"name": ["j0"], "position": [1.0]})
    adapter = _Adapter([TeleopCommand(joint, timestamp=1.0)])
    module = _module(adapter)
    mocker.patch("dimos.teleop.runtime.teleop_module.threading.Thread.start")
    mocker.patch("dimos.teleop.runtime.teleop_module.threading.Thread.join")
    mocker.patch.object(module, "_now", return_value=1.0)

    try:
        module.start()
        module.stop()
        module.tick()

        assert adapter.connected
        assert adapter.disconnected
        assert module.joint_command.messages == []  # type: ignore[attr-defined]
    finally:
        if not adapter.disconnected:
            module.stop()
