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

from collections.abc import Callable, Iterator, Sequence
from typing import Any

import pytest

from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.teleop.runtime.teleop_module import TeleopModule
from dimos.teleop.runtime.types import TeleopCommand


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


class _TestTeleopModule(TeleopModule):
    def __init__(self, runtime_adapter: _Adapter, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._runtime_adapter = runtime_adapter
        self.published_payloads: list[object] = []

    def connect_teleop(self) -> None:
        self._runtime_adapter.connect()

    def disconnect_teleop(self) -> None:
        self._runtime_adapter.disconnect()

    def get_current_command(self) -> TeleopCommand | None:
        return self._runtime_adapter.get_current_command()

    def publish_command_payload(self, payload: object) -> None:
        self.published_payloads.append(payload)


@pytest.fixture
def module_factory() -> Iterator[Callable[..., _TestTeleopModule]]:
    modules: list[_TestTeleopModule] = []

    def make(runtime_adapter: _Adapter, **kwargs: Any) -> _TestTeleopModule:
        config = {"max_publish_rate_hz": 100.0, "stale_command_timeout_s": 1.0, **kwargs}
        module = _TestTeleopModule(runtime_adapter=runtime_adapter, **config)
        modules.append(module)
        return module

    yield make

    for module in modules:
        module.stop()


def test_command_envelope_requires_payload_unless_stopping() -> None:
    joint = JointState({"name": ["j0"], "position": [1.0]})

    assert TeleopCommand(joint).payload is joint
    assert TeleopCommand(stop=True).stop
    with pytest.raises(ValueError, match="payload"):
        TeleopCommand()
    with pytest.raises(ValueError, match="payload"):
        TeleopCommand(joint, stop=True)


def test_explicit_stop_command_is_not_published(
    mocker: Any, module_factory: Callable[..., _TestTeleopModule]
) -> None:
    module = module_factory(_Adapter([TeleopCommand(timestamp=1.0, stop=True)]))
    mocker.patch.object(module, "_now", return_value=1.0)

    module.tick()

    assert module.published_payloads == []


@pytest.mark.parametrize(
    "payload",
    [
        JointState({"name": ["j0"], "position": [1.0]}),
        "cartesian-command",
        {"twist": [1.0, 0.0, 0.0]},
    ],
)
def test_tick_publishes_payload_via_concrete_hook(
    payload: object,
    mocker: Any,
    module_factory: Callable[..., _TestTeleopModule],
) -> None:
    module = module_factory(_Adapter([TeleopCommand(payload, timestamp=1.0)]))
    mocker.patch.object(module, "_now", return_value=1.0)

    module.tick()

    assert module.published_payloads == [payload]


def test_stale_commands_are_not_published(
    mocker: Any, module_factory: Callable[..., _TestTeleopModule]
) -> None:
    joint = JointState({"name": ["j0"], "position": [1.0]})
    module = module_factory(_Adapter([TeleopCommand(joint, timestamp=1.0)]))
    mocker.patch.object(module, "_now", return_value=2.01)

    module.tick()

    assert module.published_payloads == []


def test_rate_limiting_skips_commands(
    mocker: Any, module_factory: Callable[..., _TestTeleopModule]
) -> None:
    first = JointState({"name": ["j0"], "position": [1.0]})
    second = JointState({"name": ["j0"], "position": [2.0]})
    module = module_factory(
        _Adapter([TeleopCommand(first, timestamp=1.0), TeleopCommand(second, timestamp=1.0)]),
        max_publish_rate_hz=10.0,
    )
    mocker.patch.object(module, "_now", side_effect=[1.0, 1.0, 1.0, 1.05, 1.05])

    module.tick()
    module.tick()

    assert module.published_payloads == [first]


def test_start_stop_connect_disconnect_and_no_publish_after_stop(
    mocker: Any,
    module_factory: Callable[..., _TestTeleopModule],
) -> None:
    joint = JointState({"name": ["j0"], "position": [1.0]})
    adapter = _Adapter([TeleopCommand(joint, timestamp=1.0)])
    module = module_factory(adapter)
    mocker.patch("dimos.teleop.runtime.teleop_module.threading.Thread.start")
    mocker.patch("dimos.teleop.runtime.teleop_module.threading.Thread.join")
    mocker.patch.object(module, "_now", return_value=1.0)

    module.start()
    module.stop()
    module.tick()

    assert adapter.connected
    assert adapter.disconnected
    assert module.published_payloads == []
