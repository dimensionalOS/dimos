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

from dimos.navigation.route_follower.messages import (
    Command,
    CommandOp,
    build_state_document,
    is_valid_click,
    parse_command,
)
from dimos.navigation.route_follower.route import Route


def test_parses_each_command_op() -> None:
    assert parse_command('{"op": "set_mode", "enabled": true}') == Command(
        CommandOp.SET_MODE, enabled=True
    )
    assert parse_command('{"op": "set_mode", "enabled": false}') == Command(
        CommandOp.SET_MODE, enabled=False
    )
    assert parse_command('{"op": "remove", "index": 2}') == Command(CommandOp.REMOVE, index=2)
    assert parse_command('{"op": "clear"}') == Command(CommandOp.CLEAR)
    assert parse_command('{"op": "start"}') == Command(CommandOp.START)
    assert parse_command('{"op": "stop"}') == Command(CommandOp.STOP)
    assert parse_command('{"op": "keepalive"}') == Command(CommandOp.KEEPALIVE)


def test_rejects_unknown_or_missing_op() -> None:
    assert parse_command('{"op": "explode"}') is None
    assert parse_command('{"op": 3}') is None
    assert parse_command("{}") is None
    assert parse_command('["start"]') is None
    assert parse_command("not json") is None


def test_rejects_non_boolean_enabled() -> None:
    assert parse_command('{"op": "set_mode", "enabled": 1}') is None
    assert parse_command('{"op": "set_mode", "enabled": "true"}') is None
    assert parse_command('{"op": "set_mode"}') is None


def test_rejects_non_integer_index() -> None:
    assert parse_command('{"op": "remove", "index": "2"}') is None
    assert parse_command('{"op": "remove", "index": 2.0}') is None
    assert parse_command('{"op": "remove", "index": true}') is None
    assert parse_command('{"op": "remove"}') is None


def test_ignores_extra_fields() -> None:
    assert parse_command('{"op": "start", "extra": 1}') == Command(CommandOp.START)
    assert parse_command('{"op": "remove", "index": 2, "note": "x"}') == Command(
        CommandOp.REMOVE, index=2
    )


def test_click_coordinate_validation() -> None:
    assert is_valid_click(1.0, 2.0, 0.0) is True
    assert is_valid_click(500.0, -500.0, 50.0) is True
    assert is_valid_click(float("nan"), 0.0, 0.0) is False
    assert is_valid_click(0.0, float("inf"), 0.0) is False
    assert is_valid_click(501.0, 0.0, 0.0) is False
    assert is_valid_click(0.0, -501.0, 0.0) is False
    assert is_valid_click(0.0, 0.0, 51.0) is False


def test_state_document_matches_contract_example() -> None:
    route = Route()
    route.add_waypoint(1.5, -0.5, 0.0)
    route.add_waypoint(3.0, 0.0, 0.0)
    route.add_waypoint(4.5, 1.0, 0.0)
    route.start()
    route.mark_current_reached()
    route.remove_waypoint(99)
    route.set_add_mode(True)
    route.set_add_mode(False)
    route.set_add_mode(True)
    route.set_add_mode(False)
    route.set_add_mode(True)
    route.set_add_mode(False)

    assert build_state_document(route, robot_position_known=True) == {
        "revision": 12,
        "status": "running",
        "add_mode": False,
        "current_index": 2,
        "message": None,
        "waypoints": [
            {"index": 1, "x": 1.5, "y": -0.5, "z": 0.0, "state": "reached"},
            {"index": 2, "x": 3.0, "y": 0.0, "z": 0.0, "state": "current"},
            {"index": 3, "x": 4.5, "y": 1.0, "z": 0.0, "state": "pending"},
        ],
        "robot_position_known": True,
    }
