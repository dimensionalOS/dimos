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

"""reset_joint_zero() asks with motion_cmd and returns the code controller_report's joint_zero ends with."""

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
    """The ports in memory; a reset_joint_zero motion_cmd reports `code` as the finished zeroing."""

    config: M20ControllerClientConfig

    @rpc
    def start(self) -> None:
        self.watchers: dict[str, list[Callable[[Any], None]]] = {}
        self.sent: list[tuple[str, str]] = []
        self.code: int | None = 0
        super().start()

    def _can_sequence(self, what: str) -> bool:
        return True

    def _send(self, port: str, msg: Any) -> None:
        self.sent.append((port, msg.data))
        if port == "motion_cmd" and msg.data == "reset_joint_zero" and self.code is not None:
            threading.Timer(0.1, self.report, args=(1, self.code)).start()

    def _watch(self, port: str, callback: Callable[[Any], None]) -> Callable[[], None]:
        self.watchers.setdefault(port, []).append(callback)
        return lambda: None

    def report(self, finished: int, code: int) -> None:
        zero = {"running": False, "finished": finished, "last_code": code}
        for cb in self.watchers.get("controller_report", []):
            cb(String(json.dumps({"state": "LieDown", "joint_zero": zero})))


@pytest.fixture
def fake():
    with FakeController(joint_zero_timeout_s=1.0) as f:
        f.start()
        yield f


def test_reset_joint_zero_returns_the_zeroing_s_code(fake) -> None:
    fake.report(0, 0)
    fake.code = 6
    assert fake.reset_joint_zero() == 6
    assert fake.sent == [("motion_cmd", "reset_joint_zero")]


def test_reset_joint_zero_needs_a_report_and_a_result(fake) -> None:
    with pytest.raises(RuntimeError, match="not reporting"):
        fake.reset_joint_zero()
    assert fake.sent == []
    fake.report(3, 0)
    fake.code = None
    with pytest.raises(RuntimeError, match="no result after 1 s"):
        fake.reset_joint_zero()
