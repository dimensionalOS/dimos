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

"""dock() / undock() block and raise like M20Connection's, on the controller's charger state."""

from collections.abc import Callable
import json
import threading
from typing import Any

import pytest

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.msgs.std_msgs.String import String
from dimos.robot.deeprobotics.m20.controller.module import M20ControllerApi
from dimos.robot.deeprobotics.m20.controller.service import M20ControllerClientConfig


class FakeController(M20ControllerApi, Module):
    """The ports in memory; `on_dock_cmd` scripts the charger states the controller would report."""

    config: M20ControllerClientConfig

    @rpc
    def start(self) -> None:
        self.watchers: dict[str, list[Callable[[Any], None]]] = {}
        self.sent: list[tuple[str, Any]] = []
        self.goals: list[Any] = []
        self.on_dock_cmd: Callable[[str], list[int]] = lambda _: []
        super().start()
        self.goal.publish = self.goals.append  # type: ignore[method-assign]

    def _can_sequence(self, what: str) -> bool:
        return True

    def _send(self, port: str, msg: Any) -> None:
        self.sent.append((port, msg))
        if port == "dock_cmd":
            states = self.on_dock_cmd(msg.data)
            threading.Thread(target=self.play, args=(states,), daemon=True).start()

    def _watch(self, port: str, callback: Callable[[Any], None]) -> Callable[[], None]:
        self.watchers.setdefault(port, []).append(callback)
        return lambda: None

    def report(self, charger: int, state: str = "RlControl") -> None:
        for cb in self.watchers.get("controller_report", []):
            cb(String(json.dumps({"state": state, "charger_state": charger, "gait": "0x3003"})))

    def play(self, states: list[int]) -> None:
        import time

        for s in states:
            time.sleep(0.2)
            self.report(s)


@pytest.fixture
def fake(mocker):
    with FakeController(dock_settle_s=0.0, dock_start_timeout_s=1.0, undock_timeout_s=3.0) as f:
        f.start()
        mocker.patch.object(FakeController, "standup", return_value=True)
        yield f


def test_dock_returns_once_charging_and_cancels_the_goal(fake) -> None:
    fake.report(0)
    fake.on_dock_cmd = lambda cmd: [1, 1, 2] if cmd == "dock" else []
    fake.dock()
    assert ("dock_cmd", "dock") in [(p, m.data) for p, m in fake.sent]
    assert len(fake.goals) == 1


def test_a_dock_that_returns_to_idle_raises_and_resumes_navigation(fake) -> None:
    fake.report(0)
    fake.on_dock_cmd = lambda cmd: [1, 0] if cmd == "dock" else []
    with pytest.raises(RuntimeError, match="returned to idle.*standing ready to navigate"):
        fake.dock()


def test_dock_is_refused_unless_idle_and_standing(fake) -> None:
    with pytest.raises(RuntimeError, match="not reporting its charge status"):
        fake.dock()
    fake.report(2)
    with pytest.raises(RuntimeError, match="charge status is charging"):
        fake.dock()
    fake.report(0, state="Passive")
    with pytest.raises(RuntimeError, match="not standing"):
        fake.dock()


def test_undock_waits_until_idle_then_stands(fake) -> None:
    fake.report(2)
    fake.on_dock_cmd = lambda cmd: [3, 3, 3, 3, 3, 0] if cmd == "undock" else []
    fake.undock()
    assert ("dock_cmd", "undock") in [(p, m.data) for p, m in fake.sent]
    FakeController.standup.assert_called_once()  # type: ignore[attr-defined]


def test_undock_off_the_dock_returns_at_once(fake) -> None:
    fake.report(0)
    fake.undock()
    assert fake.sent == []


def test_velocity_while_on_the_dock_cancels_the_goal(fake) -> None:
    fake.report(2)
    for cb in fake.watchers["cmd_vel"]:
        cb(None)
        cb(None)
    assert len(fake.goals) == 1


def test_stop_movement_cancels_a_dock_under_way(fake) -> None:
    fake.report(1)
    fake.stop_movement()
    assert [(p, m.data) for p, m in fake.sent] == [("dock_cmd", "undock")]


def test_stop_movement_off_the_dock_is_a_zero_velocity(fake) -> None:
    fake.report(0)
    fake.stop_movement()
    assert [p for p, _ in fake.sent] == ["cmd_vel"]


def test_undock_from_the_nochargeplug_error_leaves_the_dock(fake) -> None:
    fake.report(4)
    fake.on_dock_cmd = lambda cmd: [3, 3, 0] if cmd == "undock" else []
    fake.undock()
    assert ("dock_cmd", "undock") in [(p, m.data) for p, m in fake.sent]
    FakeController.standup.assert_called_once()  # type: ignore[attr-defined]
