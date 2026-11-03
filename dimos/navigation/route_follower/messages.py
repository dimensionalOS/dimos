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

"""Command and state documents exchanged on the route streams.

Contract: specs/001-multi-waypoint-route/contracts/route-streams.md.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import json
import math
from typing import TYPE_CHECKING, Any

from dimos.navigation.movement_manager.movement_manager import (
    MAX_CLICK_HORIZONTAL_M,
    MAX_CLICK_VERTICAL_M,
)

if TYPE_CHECKING:
    from dimos.navigation.route_follower.route import Route


class CommandOp(str, Enum):
    SET_MODE = "set_mode"
    REMOVE = "remove"
    CLEAR = "clear"
    START = "start"
    STOP = "stop"
    KEEPALIVE = "keepalive"


@dataclass(frozen=True)
class Command:
    op: CommandOp
    enabled: bool | None = None
    index: int | None = None


def parse_command(text: str) -> Command | None:
    """Parse one route command document; None when it must be rejected."""
    try:
        doc = json.loads(text)
    except ValueError:
        return None
    if not isinstance(doc, dict):
        return None
    op_value = doc.get("op")
    if not isinstance(op_value, str):
        return None
    try:
        op = CommandOp(op_value)
    except ValueError:
        return None
    if op is CommandOp.SET_MODE:
        enabled = doc.get("enabled")
        if not isinstance(enabled, bool):
            return None
        return Command(op=op, enabled=enabled)
    if op is CommandOp.REMOVE:
        index = doc.get("index")
        if isinstance(index, bool) or not isinstance(index, int):
            return None
        return Command(op=op, index=index)
    return Command(op=op)


def is_valid_click(x: float, y: float, z: float) -> bool:
    """True when click coordinates are finite and within the click range limits."""
    return (
        math.isfinite(x)
        and math.isfinite(y)
        and math.isfinite(z)
        and abs(x) <= MAX_CLICK_HORIZONTAL_M
        and abs(y) <= MAX_CLICK_HORIZONTAL_M
        and abs(z) <= MAX_CLICK_VERTICAL_M
    )


def build_state_document(route: Route, robot_position_known: bool) -> dict[str, Any]:
    """Build the `route_state` document for *route*."""
    return {
        "revision": route.revision,
        "status": route.status.value,
        "add_mode": route.add_mode,
        "current_index": route.current_index,
        "message": route.message,
        "waypoints": [
            {"index": wp.index, "x": wp.x, "y": wp.y, "z": wp.z, "state": wp.state.value}
            for wp in route.waypoints
        ],
        "robot_position_known": robot_position_known,
    }
