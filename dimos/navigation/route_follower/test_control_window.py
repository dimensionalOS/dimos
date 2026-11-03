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

from collections.abc import Callable
import json
import time
from typing import Any

from dimos_lcm.std_msgs import String
import pygame
import pytest

from dimos.navigation.route_follower.control_layout import Layout
from dimos.navigation.route_follower.control_window import RouteControl
from dimos.navigation.route_follower.messages import build_state_document
from dimos.navigation.route_follower.route import Route


class FakeTransport:
    def __init__(self) -> None:
        self._callbacks: list[Callable[[Any], Any]] = []

    def subscribe(
        self, callback: Callable[[Any], Any], selfstream: Any = None
    ) -> Callable[[], None]:
        self._callbacks.append(callback)

        def unsubscribe() -> None:
            self._callbacks.remove(callback)

        return unsubscribe

    def broadcast(self, stream: Any, msg: Any) -> None:
        for callback in list(self._callbacks):
            callback(msg)

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass


def _make_window() -> tuple[RouteControl, list[String]]:
    module = RouteControl()
    commands: list[String] = []
    module.route_state.transport = FakeTransport()
    module.route_cmd.subscribe(commands.append)
    return module, commands


def _feed(module: RouteControl, route: Route) -> int:
    doc = build_state_document(route, robot_position_known=True)
    module.route_state.transport.broadcast(None, String(data=json.dumps(doc)))
    return int(doc["revision"])


def _wait_for(predicate: Callable[[], bool], timeout: float = 5.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    assert predicate()


def _keepalives(commands: list[String]) -> list[String]:
    return [c for c in commands if json.loads(c.data)["op"] == "keepalive"]


def _set_mode_off_commands(commands: list[String]) -> list[String]:
    return [c for c in commands if json.loads(c.data) == {"op": "set_mode", "enabled": False}]


@pytest.fixture()
def window(monkeypatch: pytest.MonkeyPatch) -> Any:
    monkeypatch.setenv("SDL_VIDEODRIVER", "dummy")
    module, commands = _make_window()
    module.start()
    try:
        yield module, commands
    finally:
        module.stop()


def test_window_starts_and_redraws_on_new_state(window: Any) -> None:
    module, commands = window
    assert module.visible is True

    route = Route()
    route.add_waypoint(1.0, 0.0, 0.0)
    revision = _feed(module, route)
    _wait_for(lambda: module.last_drawn_revision == revision)

    route.add_waypoint(2.0, 0.0, 0.0)
    revision = _feed(module, route)
    _wait_for(lambda: module.last_drawn_revision == revision)


def test_stop_closes_the_window_cleanly(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("SDL_VIDEODRIVER", "dummy")
    module, commands = _make_window()
    try:
        module.start()
    finally:
        module.stop()
    assert module._thread is not None
    assert not module._thread.is_alive()


def test_close_request_hides_the_window_and_sends_set_mode_off(window: Any) -> None:
    module, commands = window
    route = Route()
    route.add_waypoint(1.0, 0.0, 0.0)
    revision = _feed(module, route)
    _wait_for(lambda: module.last_drawn_revision == revision)

    pygame.event.post(pygame.event.Event(pygame.QUIT))

    _wait_for(lambda: not module.visible)
    _wait_for(lambda: _set_mode_off_commands(commands))


def test_close_request_while_a_route_runs_is_ignored(window: Any) -> None:
    module, commands = window
    route = Route()
    route.add_waypoint(1.0, 0.0, 0.0)
    route.start()
    revision = _feed(module, route)
    _wait_for(lambda: module.last_drawn_revision == revision)

    pygame.event.post(pygame.event.Event(pygame.QUIT))

    _wait_for(lambda: module.notice == "stop the route first")
    assert module.visible is True
    assert not _set_mode_off_commands(commands)


def test_show_window_reopens_the_hidden_window(window: Any) -> None:
    module, commands = window
    route = Route()
    revision = _feed(module, route)
    _wait_for(lambda: module.last_drawn_revision == revision)

    pygame.event.post(pygame.event.Event(pygame.QUIT))
    _wait_for(lambda: not module.visible)

    module.show_window()
    assert module.visible is True

    route.add_waypoint(1.0, 0.0, 0.0)
    revision = _feed(module, route)
    _wait_for(lambda: module.last_drawn_revision == revision)


def test_pause_keepalive_stops_and_resumes_the_keepalive(
    window: Any, monkeypatch: pytest.MonkeyPatch
) -> None:
    module, commands = window
    now = [100.0]
    monkeypatch.setattr(module, "_clock", lambda: now[0])

    module.pause_keepalive(True)
    baseline = len(_keepalives(commands))
    assert module.visible is True

    now[0] = 200.0
    module._maybe_send_keepalive()
    assert len(_keepalives(commands)) == baseline

    module.pause_keepalive(False)
    now[0] = 200.5
    module._maybe_send_keepalive()
    assert len(_keepalives(commands)) == baseline

    now[0] = 201.5
    module._maybe_send_keepalive()
    _wait_for(lambda: len(_keepalives(commands)) == baseline + 1)
    assert module.visible is True


def test_missing_pygame_names_the_extra_to_install(monkeypatch: pytest.MonkeyPatch) -> None:
    import dimos.navigation.route_follower.control_window as control_window

    monkeypatch.setattr(control_window, "pygame", None)
    monkeypatch.setenv("SDL_VIDEODRIVER", "dummy")
    module = RouteControl()
    try:
        with pytest.raises(ImportError, match="unitree"):
            module.start()
    finally:
        module.stop()


def _click_at(rect: Any) -> None:
    pygame.event.post(
        pygame.event.Event(pygame.MOUSEBUTTONDOWN, pos=(rect.x + rect.w // 2, rect.y + rect.h // 2))
    )


def _ops(commands: list[String]) -> list[dict[str, Any]]:
    return [json.loads(c.data) for c in commands]


def test_toggle_emits_set_mode(window: Any) -> None:
    module, commands = window
    route = Route()
    revision = _feed(module, route)
    _wait_for(lambda: module.last_drawn_revision == revision)

    _click_at(Layout.default().toggle_add_mode)
    _wait_for(lambda: {"op": "set_mode", "enabled": True} in _ops(commands))

    route.set_add_mode(True)
    revision = _feed(module, route)
    _wait_for(lambda: module.last_drawn_revision == revision)

    _click_at(Layout.default().toggle_add_mode)
    _wait_for(lambda: {"op": "set_mode", "enabled": False} in _ops(commands))


def test_start_emits_start_only_when_enabled(window: Any) -> None:
    module, commands = window
    route = Route()
    revision = _feed(module, route)
    _wait_for(lambda: module.last_drawn_revision == revision)

    _click_at(Layout.default().start)
    _click_at(Layout.default().toggle_add_mode)
    _wait_for(lambda: {"op": "set_mode", "enabled": True} in _ops(commands))
    assert {"op": "start"} not in _ops(commands)

    route.add_waypoint(5.0, 5.0, 0.0)
    revision = _feed(module, route)
    _wait_for(lambda: module.last_drawn_revision == revision)

    _click_at(Layout.default().start)
    _wait_for(lambda: {"op": "start"} in _ops(commands))


def test_window_shows_status_target_and_message(window: Any) -> None:
    module, commands = window
    route = Route()
    route.add_waypoint(1.0, 0.0, 0.0)
    route.add_waypoint(2.0, 0.0, 0.0)
    route.add_waypoint(3.0, 0.0, 0.0)
    route.start()
    route.mark_current_reached()
    route.set_message("stop the route first")
    revision = _feed(module, route)

    _wait_for(lambda: module.last_drawn_revision == revision)
    assert module.drawn_lines == ("status: running", "waypoint 2 of 3", "stop the route first")


def test_row_remove_emits_remove_with_that_index(window: Any) -> None:
    module, commands = window
    route = Route()
    route.add_waypoint(1.0, 0.0, 0.0)
    route.add_waypoint(2.0, 0.0, 0.0)
    route.add_waypoint(3.0, 0.0, 0.0)
    revision = _feed(module, route)
    _wait_for(lambda: module.last_drawn_revision == revision)

    _click_at(Layout.default().remove_rect(2))
    _wait_for(lambda: {"op": "remove", "index": 2} in _ops(commands))


def test_clear_emits_clear(window: Any) -> None:
    module, commands = window
    route = Route()
    route.add_waypoint(1.0, 0.0, 0.0)
    revision = _feed(module, route)
    _wait_for(lambda: module.last_drawn_revision == revision)

    _click_at(Layout.default().clear)
    _wait_for(lambda: {"op": "clear"} in _ops(commands))


def test_remove_and_clear_are_disabled_while_running(window: Any) -> None:
    module, commands = window
    route = Route()
    route.add_waypoint(1.0, 0.0, 0.0)
    route.add_waypoint(2.0, 0.0, 0.0)
    route.start()
    revision = _feed(module, route)
    _wait_for(lambda: module.last_drawn_revision == revision)

    _click_at(Layout.default().remove_rect(1))
    _click_at(Layout.default().clear)
    _click_at(Layout.default().toggle_add_mode)
    _wait_for(lambda: {"op": "set_mode", "enabled": True} in _ops(commands))
    assert {"op": "clear"} not in _ops(commands)
    assert not any(op.get("op") == "remove" for op in _ops(commands))


def test_start_is_disabled_after_clear(window: Any) -> None:
    module, commands = window
    route = Route()
    route.add_waypoint(1.0, 0.0, 0.0)
    revision = _feed(module, route)
    _wait_for(lambda: module.last_drawn_revision == revision)
    _click_at(Layout.default().clear)
    _wait_for(lambda: {"op": "clear"} in _ops(commands))

    revision = _feed(module, Route())
    _wait_for(lambda: module.last_drawn_revision == revision)
    _click_at(Layout.default().start)
    _click_at(Layout.default().toggle_add_mode)
    _wait_for(lambda: {"op": "set_mode", "enabled": True} in _ops(commands))
    assert {"op": "start"} not in _ops(commands)


def test_stop_emits_stop_and_is_enabled_while_running_or_held(window: Any) -> None:
    module, commands = window
    route = Route()
    route.add_waypoint(1.0, 0.0, 0.0)
    route.start()
    revision = _feed(module, route)
    _wait_for(lambda: module.last_drawn_revision == revision)

    _click_at(Layout.default().stop)
    _wait_for(lambda: {"op": "stop"} in _ops(commands))

    route.hold()
    revision = _feed(module, route)
    _wait_for(lambda: module.last_drawn_revision == revision)

    _click_at(Layout.default().stop)
    _wait_for(lambda: _ops(commands).count({"op": "stop"}) == 2)


def test_stop_is_disabled_without_a_running_route(window: Any) -> None:
    module, commands = window
    route = Route()
    route.add_waypoint(1.0, 0.0, 0.0)
    revision = _feed(module, route)
    _wait_for(lambda: module.last_drawn_revision == revision)

    _click_at(Layout.default().stop)
    _click_at(Layout.default().toggle_add_mode)
    _wait_for(lambda: {"op": "set_mode", "enabled": True} in _ops(commands))
    assert {"op": "stop"} not in _ops(commands)


def test_keepalive_emits_about_once_per_second_only_while_open(
    window: Any, monkeypatch: pytest.MonkeyPatch
) -> None:
    module, commands = window
    now = [100.0]
    monkeypatch.setattr(module, "_clock", lambda: now[0])
    module.pause_keepalive(False)
    route = Route()
    revision = _feed(module, route)
    _wait_for(lambda: module.last_drawn_revision == revision)

    baseline = len(_keepalives(commands))
    now[0] = 101.2
    _wait_for(lambda: len(_keepalives(commands)) == baseline + 1)
    module._maybe_send_keepalive()
    assert len(_keepalives(commands)) == baseline + 1

    pygame.event.post(pygame.event.Event(pygame.QUIT))
    _wait_for(lambda: not module.visible)
    now[0] = 103.0
    module._maybe_send_keepalive()
    assert len(_keepalives(commands)) == baseline + 1


def test_status_text_shows_end_states_distinctly(window: Any) -> None:
    module, commands = window

    stopped = Route()
    stopped.add_waypoint(1.0, 0.0, 0.0)
    stopped.start()
    stopped.stop()
    held = Route()
    held.add_waypoint(1.0, 0.0, 0.0)
    held.add_waypoint(2.0, 0.0, 0.0)
    held.start()
    held.hold()
    interrupted = Route()
    interrupted.add_waypoint(1.0, 0.0, 0.0)
    interrupted.add_waypoint(2.0, 0.0, 0.0)
    interrupted.add_waypoint(3.0, 0.0, 0.0)
    interrupted.start()
    interrupted.interrupt()
    completed = Route()
    completed.add_waypoint(1.0, 0.0, 0.0)
    completed.add_waypoint(2.0, 0.0, 0.0)
    completed.add_waypoint(3.0, 0.0, 0.0)
    completed.add_waypoint(4.0, 0.0, 0.0)
    completed.start()
    completed.mark_current_reached()
    completed.mark_current_reached()
    completed.mark_current_reached()
    completed.mark_current_reached()

    seen = []
    for route in (stopped, held, interrupted, completed):
        revision = _feed(module, route)
        _wait_for(lambda revision=revision: module.last_drawn_revision == revision)
        seen.append(module.drawn_lines[0])

    assert seen == ["status: stopped", "status: held", "status: interrupted", "status: completed"]
    assert len(set(seen)) == 4


def test_failed_state_shows_the_failed_waypoint_and_the_message(window: Any) -> None:
    module, commands = window
    route = Route()
    route.add_waypoint(1.0, 0.0, 0.0)
    route.add_waypoint(2.0, 0.0, 0.0)
    route.start()
    route.fail_current()
    revision = _feed(module, route)

    _wait_for(lambda: module.last_drawn_revision == revision)
    assert module.drawn_lines == ("status: failed", "waypoint -", "waypoint 1 could not be reached")
    assert any("failed" in row for row in module.drawn_rows)


def test_start_is_available_again_after_the_next_edit(window: Any) -> None:
    module, commands = window
    route = Route()
    route.add_waypoint(1.0, 0.0, 0.0)
    route.start()
    route.fail_current()
    revision = _feed(module, route)
    _wait_for(lambda: module.last_drawn_revision == revision)

    _click_at(Layout.default().start)
    _click_at(Layout.default().toggle_add_mode)
    _wait_for(lambda: {"op": "set_mode", "enabled": True} in _ops(commands))
    assert {"op": "start"} not in _ops(commands)

    route.clear()
    route.add_waypoint(1.0, 0.0, 0.0)
    revision = _feed(module, route)
    _wait_for(lambda: module.last_drawn_revision == revision)

    _click_at(Layout.default().start)
    _wait_for(lambda: {"op": "start"} in _ops(commands))
