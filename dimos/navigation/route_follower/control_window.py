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

"""RouteControl: the companion Route window for the multi-waypoint route.

A pygame window drawn from the `route_state` document that sends route
commands on `route_cmd`, following the KeyboardTeleopModule window pattern.
"""

from __future__ import annotations

import json
import os
import threading
import time
from typing import Any

from dimos_lcm.std_msgs import String
from reactivex.disposable import Disposable

try:
    import pygame
except ImportError:
    pygame = None  # type: ignore[assignment]

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.navigation.route_follower.control_layout import (
    MAX_ROWS,
    PADDING,
    WINDOW_HEIGHT,
    WINDOW_WIDTH,
    ControlKind,
    Layout,
    edit_enabled,
    message_text,
    start_enabled,
    status_text,
    stop_enabled,
    target_text,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Force X11 driver to avoid OpenGL threading issues
os.environ["SDL_VIDEODRIVER"] = "x11"

KEEPALIVE_INTERVAL_S = 1.0
PYGAME_INSTALL_HINT = (
    "pygame is required for the Route window. "
    "Install the 'unitree' extra: pip install 'dimos[unitree]'"
)
NOTICE_STOP_ROUTE_FIRST = "stop the route first"

_BACKGROUND = (30, 30, 30)
_TEXT = (220, 220, 220)
_BUTTON = (70, 70, 90)
_BUTTON_OFF = (50, 50, 50)
_ACCENT_ON = (60, 160, 60)
_REMOVE = (140, 60, 60)


class RouteControl(Module):
    """Edit and drive the route from a desktop window."""

    route_state: In[String]
    route_cmd: Out[String]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._layout = Layout.default()
        self._state_doc: dict[str, Any] = {}
        self._notice: str | None = None
        self._visible = False
        self._drawn_revision: int | None = None
        self._keepalive_paused = False
        self._last_keepalive: float | None = None
        self._clock = time.monotonic
        self._drawn_lines: tuple[str, str, str] = ("", "", "")
        self._drawn_rows: tuple[str, ...] = ()

    @property
    def visible(self) -> bool:
        with self._lock:
            return self._visible

    @property
    def notice(self) -> str | None:
        with self._lock:
            return self._notice

    @property
    def last_drawn_revision(self) -> int | None:
        return self._drawn_revision

    @property
    def drawn_lines(self) -> tuple[str, str, str]:
        return self._drawn_lines

    @property
    def drawn_rows(self) -> tuple[str, ...]:
        return self._drawn_rows

    @rpc
    def start(self) -> None:
        if pygame is None:
            raise ImportError(PYGAME_INSTALL_HINT)
        super().start()
        self.register_disposable(Disposable(self.route_state.subscribe(self._on_route_state)))
        self._stop_event.clear()
        with self._lock:
            self._visible = True
        self._thread = threading.Thread(target=self._ui_loop, daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(DEFAULT_THREAD_JOIN_TIMEOUT)
        super().stop()

    @rpc
    def show_window(self) -> None:
        with self._lock:
            self._visible = True
            self._notice = None

    @rpc
    def pause_keepalive(self, paused: bool) -> None:
        with self._lock:
            self._keepalive_paused = paused
            self._last_keepalive = self._clock()

    def _on_route_state(self, msg: String) -> None:
        with self._lock:
            self._state_doc = json.loads(msg.data)

    def _publish_command(self, payload: dict[str, Any]) -> None:
        self.route_cmd.publish(String(data=json.dumps(payload)))

    def _maybe_send_keepalive(self) -> None:
        with self._lock:
            if self._keepalive_paused or not self._visible:
                return
            now = self._clock()
            if (
                self._last_keepalive is not None
                and now - self._last_keepalive < KEEPALIVE_INTERVAL_S
            ):
                return
            self._last_keepalive = now
        self._publish_command({"op": "keepalive"})

    def _request_close(self) -> None:
        with self._lock:
            if stop_enabled(self._state_doc):
                self._notice = NOTICE_STOP_ROUTE_FIRST
                return
            self._visible = False
            self._notice = None
        self._publish_command({"op": "set_mode", "enabled": False})

    def _handle_click(self, pos: tuple[int, int]) -> None:
        with self._lock:
            doc = dict(self._state_doc)
        control = self._layout.hit_test(*pos)
        if control.kind is ControlKind.TOGGLE_ADD_MODE:
            self._publish_command({"op": "set_mode", "enabled": not bool(doc.get("add_mode"))})
        elif control.kind is ControlKind.START and start_enabled(doc):
            self._publish_command({"op": "start"})
        elif control.kind is ControlKind.STOP and stop_enabled(doc):
            self._publish_command({"op": "stop"})
        elif control.kind is ControlKind.CLEAR and edit_enabled(doc):
            self._publish_command({"op": "clear"})
        elif control.kind is ControlKind.REMOVE and edit_enabled(doc) and control.index is not None:
            self._publish_command({"op": "remove", "index": control.index})

    def _ui_loop(self) -> None:
        if pygame is None:
            return
        pygame.init()
        screen: Any = None
        font = pygame.font.Font(None, 22)
        clock = pygame.time.Clock()
        try:
            while not self._stop_event.is_set():
                with self._lock:
                    visible = self._visible
                    doc = dict(self._state_doc)
                    notice = self._notice
                if visible:
                    if screen is None:
                        screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
                        pygame.display.set_caption("Route")
                    for event in pygame.event.get():
                        if event.type == pygame.QUIT:
                            self._request_close()
                        elif event.type == pygame.MOUSEBUTTONDOWN:
                            self._handle_click(event.pos)
                    self._draw(screen, font, doc, notice)
                    self._drawn_revision = doc.get("revision")
                    pygame.display.flip()
                elif screen is not None:
                    pygame.display.quit()
                    screen = None
                self._maybe_send_keepalive()
                clock.tick(30)
        finally:
            pygame.quit()

    def _draw(self, screen: Any, font: Any, doc: dict[str, Any], notice: str | None) -> None:
        screen.fill(_BACKGROUND)
        layout = self._layout

        add_mode = bool(doc.get("add_mode"))
        self._draw_button(
            screen,
            font,
            layout.toggle_add_mode,
            f"Add-waypoints: {'ON' if add_mode else 'OFF'}",
            _ACCENT_ON if add_mode else _BUTTON,
            enabled=True,
        )

        self._draw_button(
            screen,
            font,
            layout.start,
            "Start",
            _BUTTON,
            enabled=start_enabled(doc),
        )
        self._draw_button(
            screen,
            font,
            layout.stop,
            "Stop",
            _BUTTON,
            enabled=stop_enabled(doc),
        )
        self._draw_button(
            screen,
            font,
            layout.clear,
            "Clear",
            _BUTTON,
            enabled=edit_enabled(doc),
        )

        waypoints = doc.get("waypoints") or []
        labels = tuple(
            f"{w['index']}. ({w['x']:.2f}, {w['y']:.2f}) [{w['state']}]"
            for w in waypoints[:MAX_ROWS]
        )
        self._drawn_rows = labels
        for position, label in enumerate(labels, start=1):
            row = layout.row_rect(position)
            screen.blit(font.render(label, True, _TEXT), (row.x + 4, row.y + 4))
            self._draw_button(
                screen,
                font,
                layout.remove_rect(position),
                "X",
                _REMOVE,
                enabled=edit_enabled(doc),
            )

        lines = (status_text(doc), target_text(doc), message_text(doc, notice))
        self._drawn_lines = lines
        line_y = layout.list_area.y + layout.list_area.h + PADDING
        for line in lines:
            screen.blit(font.render(line, True, _TEXT), (PADDING, line_y))
            line_y += 32

    def _draw_button(
        self,
        screen: Any,
        font: Any,
        rect: Any,
        label: str,
        color: tuple[int, int, int],
        *,
        enabled: bool,
    ) -> None:
        if not pygame:
            return
        fill = color if enabled else _BUTTON_OFF
        pygame.draw.rect(screen, fill, (rect.x, rect.y, rect.w, rect.h))
        text = font.render(label, True, _TEXT if enabled else _BUTTON_OFF)
        screen.blit(text, (rect.x + 8, rect.y + rect.h // 2 - text.get_height() // 2))


route_control = RouteControl.blueprint
