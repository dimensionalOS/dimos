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

from dimos.navigation.route_follower.control_layout import (
    WINDOW_HEIGHT,
    WINDOW_WIDTH,
    Control,
    ControlKind,
    Layout,
    Rect,
    edit_enabled,
    message_text,
    start_enabled,
    status_text,
    stop_enabled,
    target_text,
)
from dimos.navigation.route_follower.messages import build_state_document
from dimos.navigation.route_follower.route import Route


def _overlap(a: Rect, b: Rect) -> bool:
    return a.x < b.x + b.w and b.x < a.x + a.w and a.y < b.y + b.h and b.y < a.y + a.h


def _center(rect: Rect) -> tuple[int, int]:
    return rect.x + rect.w // 2, rect.y + rect.h // 2


def test_button_rects_do_not_overlap_and_fit_the_window() -> None:
    layout = Layout.default()
    buttons = [layout.toggle_add_mode, layout.start, layout.stop, layout.clear]
    for i, a in enumerate(buttons):
        assert 0 <= a.x and a.x + a.w <= WINDOW_WIDTH
        assert 0 <= a.y and a.y + a.h <= WINDOW_HEIGHT
        for b in buttons[i + 1 :]:
            assert not _overlap(a, b)


def test_hit_testing_returns_the_right_control() -> None:
    layout = Layout.default()

    assert layout.hit_test(*_center(layout.toggle_add_mode)) == Control(ControlKind.TOGGLE_ADD_MODE)
    assert layout.hit_test(*_center(layout.start)) == Control(ControlKind.START)
    assert layout.hit_test(*_center(layout.stop)) == Control(ControlKind.STOP)
    assert layout.hit_test(*_center(layout.clear)) == Control(ControlKind.CLEAR)
    assert layout.hit_test(0, 0) == Control(ControlKind.NONE)
    assert layout.hit_test(WINDOW_WIDTH - 1, WINDOW_HEIGHT - 1) == Control(ControlKind.NONE)


def test_twenty_list_rows_fit_the_window() -> None:
    layout = Layout.default()
    for i in range(1, 21):
        row = layout.row_rect(i)
        assert 0 <= row.x and row.x + row.w <= WINDOW_WIDTH
        assert row.y >= layout.list_area.y
        assert row.y + row.h <= layout.list_area.y + layout.list_area.h
        assert row.y + row.h <= WINDOW_HEIGHT
    assert not _overlap(layout.row_rect(1), layout.row_rect(20))


def test_row_remove_control_maps_to_that_row_index() -> None:
    layout = Layout.default()
    for i in (1, 2, 20):
        remove = layout.remove_rect(i)
        assert remove.y == layout.row_rect(i).y
        assert layout.hit_test(*_center(remove)) == Control(ControlKind.REMOVE, i)
        row = layout.row_rect(i)
        assert layout.hit_test(row.x + 2, row.y + 2) == Control(ControlKind.NONE)


def test_start_disabled_without_waypoints_or_position_or_while_running() -> None:
    empty = build_state_document(Route(), robot_position_known=True)
    assert start_enabled(empty) is False

    route = Route()
    route.add_waypoint(1.0, 0.0, 0.0)
    assert start_enabled(build_state_document(route, robot_position_known=False)) is False

    assert start_enabled(build_state_document(route, robot_position_known=True)) is True

    route.start()
    assert start_enabled(build_state_document(route, robot_position_known=True)) is False


def test_stop_and_edit_enablement_follow_the_status() -> None:
    route = Route()
    route.add_waypoint(1.0, 0.0, 0.0)
    planning = build_state_document(route, robot_position_known=True)
    assert stop_enabled(planning) is False
    assert edit_enabled(planning) is True

    route.start()
    running = build_state_document(route, robot_position_known=True)
    assert stop_enabled(running) is True
    assert edit_enabled(running) is False

    route.hold()
    held = build_state_document(route, robot_position_known=True)
    assert stop_enabled(held) is True
    assert edit_enabled(held) is True
    assert start_enabled(held) is True


def test_status_target_and_message_texts() -> None:
    route = Route()
    route.add_waypoint(1.0, 0.0, 0.0)
    route.add_waypoint(2.0, 0.0, 0.0)
    route.add_waypoint(3.0, 0.0, 0.0)
    route.start()
    route.mark_current_reached()
    doc = build_state_document(route, robot_position_known=True)

    assert status_text(doc) == "status: running"
    assert target_text(doc) == "waypoint 2 of 3"
    assert message_text(doc) == ""

    route.set_message("stop the route first")
    doc = build_state_document(route, robot_position_known=True)
    assert message_text(doc) == "stop the route first"
    assert message_text(doc, "local notice") == "local notice"

    empty = build_state_document(Route(), robot_position_known=False)
    assert status_text(empty) == "status: planning"
    assert target_text(empty) == "waypoint -"


def test_edit_controls_follow_the_running_status_and_clear_disables_start() -> None:
    route = Route()
    route.add_waypoint(1.0, 0.0, 0.0)
    planning = build_state_document(route, robot_position_known=True)
    assert edit_enabled(planning) is True
    assert start_enabled(planning) is True

    route.start()
    running = build_state_document(route, robot_position_known=True)
    assert edit_enabled(running) is False

    after_clear = build_state_document(Route(), robot_position_known=True)
    assert start_enabled(after_clear) is False
