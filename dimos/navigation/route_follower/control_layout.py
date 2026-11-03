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

"""Window layout, hit-testing and control enablement for the Route window.

Pure logic, no pygame. Rectangles are in window pixel coordinates with the
origin at the top left.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from enum import Enum
from typing import Any

from dimos.navigation.route_follower.route import RouteStatus

WINDOW_WIDTH = 420
WINDOW_HEIGHT = 720
PADDING = 10
BUTTON_HEIGHT = 36
BUTTON_GAP = 8
ROW_HEIGHT = 24
REMOVE_WIDTH = 48
MAX_ROWS = 20


@dataclass(frozen=True)
class Rect:
    x: int
    y: int
    w: int
    h: int

    def contains(self, px: int, py: int) -> bool:
        return self.x <= px < self.x + self.w and self.y <= py < self.y + self.h


class ControlKind(str, Enum):
    NONE = "none"
    TOGGLE_ADD_MODE = "toggle_add_mode"
    START = "start"
    STOP = "stop"
    CLEAR = "clear"
    REMOVE = "remove"


@dataclass(frozen=True)
class Control:
    kind: ControlKind
    index: int | None = None


@dataclass(frozen=True)
class Layout:
    toggle_add_mode: Rect
    start: Rect
    stop: Rect
    clear: Rect
    list_area: Rect

    @classmethod
    def default(cls) -> Layout:
        button_w = (WINDOW_WIDTH - 2 * PADDING - 2 * BUTTON_GAP) // 3
        buttons_y = PADDING + BUTTON_HEIGHT + PADDING
        list_y = buttons_y + BUTTON_HEIGHT + PADDING
        return cls(
            toggle_add_mode=Rect(PADDING, PADDING, WINDOW_WIDTH - 2 * PADDING, BUTTON_HEIGHT),
            start=Rect(PADDING, buttons_y, button_w, BUTTON_HEIGHT),
            stop=Rect(PADDING + button_w + BUTTON_GAP, buttons_y, button_w, BUTTON_HEIGHT),
            clear=Rect(PADDING + 2 * (button_w + BUTTON_GAP), buttons_y, button_w, BUTTON_HEIGHT),
            list_area=Rect(PADDING, list_y, WINDOW_WIDTH - 2 * PADDING, MAX_ROWS * ROW_HEIGHT),
        )

    def row_rect(self, index: int) -> Rect:
        if not 1 <= index <= MAX_ROWS:
            raise ValueError(f"row index {index} outside 1..{MAX_ROWS}")
        y = self.list_area.y + (index - 1) * ROW_HEIGHT
        return Rect(self.list_area.x, y, self.list_area.w, ROW_HEIGHT)

    def remove_rect(self, index: int) -> Rect:
        row = self.row_rect(index)
        return Rect(row.x + row.w - REMOVE_WIDTH, row.y, REMOVE_WIDTH, row.h)

    def hit_test(self, x: int, y: int) -> Control:
        if self.toggle_add_mode.contains(x, y):
            return Control(ControlKind.TOGGLE_ADD_MODE)
        if self.start.contains(x, y):
            return Control(ControlKind.START)
        if self.stop.contains(x, y):
            return Control(ControlKind.STOP)
        if self.clear.contains(x, y):
            return Control(ControlKind.CLEAR)
        for index in range(1, MAX_ROWS + 1):
            if self.row_rect(index).contains(x, y):
                if self.remove_rect(index).contains(x, y):
                    return Control(ControlKind.REMOVE, index)
                return Control(ControlKind.NONE)
        return Control(ControlKind.NONE)


def start_enabled(doc: Mapping[str, Any]) -> bool:
    return (
        bool(doc.get("waypoints"))
        and bool(doc.get("robot_position_known"))
        and doc.get("status") in (RouteStatus.PLANNING.value, RouteStatus.HELD.value)
    )


def stop_enabled(doc: Mapping[str, Any]) -> bool:
    return doc.get("status") in (RouteStatus.RUNNING.value, RouteStatus.HELD.value)


def edit_enabled(doc: Mapping[str, Any]) -> bool:
    return doc.get("status") != RouteStatus.RUNNING.value


def status_text(doc: Mapping[str, Any]) -> str:
    return f"status: {doc.get('status') or 'planning'}"


def target_text(doc: Mapping[str, Any]) -> str:
    current = doc.get("current_index")
    if not current:
        return "waypoint -"
    total = len(doc.get("waypoints") or [])
    return f"waypoint {current} of {total}"


def message_text(doc: Mapping[str, Any], notice: str | None = None) -> str:
    return notice if notice is not None else (doc.get("message") or "")
