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

from typing import Any

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.robot.limx.tron1.skill_container import TRON1SkillContainer


class _Conn:
    def __init__(self) -> None:
        self.moves: list[tuple[Twist, float]] = []
        self.requests: list[tuple[str, dict[str, Any]]] = []

    def move(self, twist: Twist, duration: float = 0.0) -> None:
        self.moves.append((twist, duration))

    def publish_request(self, name: str, data: dict[str, Any]) -> dict[str, Any]:
        self.requests.append((name, data))
        return {}


def test_skill_move_calls_connection() -> None:
    c = TRON1SkillContainer()
    conn = _Conn()
    c._connection = conn  # type: ignore[assignment]
    c.move(x=0.1, y=0.2, yaw=0.3, duration=1.0)
    assert len(conn.moves) == 1
    twist, duration = conn.moves[0]
    assert twist.linear.x == 0.1
    assert twist.linear.y == 0.2
    assert twist.angular.z == 0.3
    assert duration == 1.0


def test_skill_stand_calls_publish_request() -> None:
    c = TRON1SkillContainer()
    conn = _Conn()
    c._connection = conn  # type: ignore[assignment]
    c.stand()
    assert conn.requests
    assert conn.requests[0][0] == "request_stand_mode"

