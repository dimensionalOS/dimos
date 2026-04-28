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

"""Dtop sub-app — embedded resource monitor."""

from __future__ import annotations

from collections import deque
import threading
import time
from typing import TYPE_CHECKING, Any

from rich.console import Group, RenderableType
from rich.panel import Panel
from rich.rule import Rule
from rich.text import Text
from textual.containers import VerticalScroll
from textual.widgets import Static

from dimos.utils.cli import theme
from dimos.utils.cli.dio.sub_app import SubApp
from dimos.utils.cli.dtop import (
    _SPARK_WIDTH,
    ResourceSpyApp,
    _compute_ranges,
)

if TYPE_CHECKING:
    from textual.app import ComposeResult
    from textual.widget import Widget


class DtopSubApp(SubApp):
    TITLE = "dtop"

    DEFAULT_CSS = """
    DtopSubApp {
        layout: vertical;
        height: 1fr;
        background: $dio-bg;
    }
    DtopSubApp VerticalScroll {
        height: 1fr;
        scrollbar-size: 0 0;
    }
    DtopSubApp VerticalScroll.waiting {
        align: center middle;
    }
    DtopSubApp .waiting #dtop-panels {
        width: auto;
    }
    DtopSubApp #dtop-panels {
        background: transparent;
    }
    """

    def __init__(self) -> None:
        super().__init__()
        self._lcm: Any = None
        self._lock = threading.Lock()
        self._latest: dict[str, Any] | None = None
        self._last_msg_time: float = 0.0
        self._cpu_history: dict[str, deque[float]] = {}
        self._reconnecting = False

    def _debug(self, msg: str) -> None:
        try:
            self.app._dlog(f"[{theme.BTN_MUTED}]DTOP:[/{theme.BTN_MUTED}] {msg}")  # type: ignore[attr-defined]
        except Exception:
            pass

    # How long without a message before we consider the connection stale
    _STALE_TIMEOUT = 5.0
    # How long without a message before we attempt to reconnect LCM
    _RECONNECT_TIMEOUT = 15.0

    def _waiting_panel(self) -> Panel:
        msg = Text(justify="center")
        msg.append("Waiting for resource stats...\n\n", style=theme.FOREGROUND)
        msg.append("Blueprint must be launched with ", style=theme.DIM)
        msg.append("--dtop", style=f"bold {theme.CYAN}")
        msg.append(" to emit stats.\n", style=theme.DIM)
        msg.append("Enable it in the ", style=theme.DIM)
        msg.append("config", style=f"bold {theme.CYAN}")
        msg.append(" tab before launching.", style=theme.DIM)
        return Panel(msg, border_style=theme.CYAN, expand=False)

    def compose(self) -> ComposeResult:
        with VerticalScroll(id="dtop-scroll", classes="waiting"):
            yield Static(self._waiting_panel(), id="dtop-panels")

    def get_focus_target(self) -> Widget | None:
        """Return the VerticalScroll for focus."""
        try:
            return self.query_one("#dtop-scroll")
        except Exception:
            return super().get_focus_target()

    def on_mount_subapp(self) -> None:
        self.run_worker(self._init_lcm, exclusive=True, thread=True)
        self._start_refresh_timer()

    def on_resume_subapp(self) -> None:
        self._start_refresh_timer()
        # Reinitialize LCM if it was lost (e.g. after unmount/remount)
        if self._lcm is None:
            self.run_worker(self._init_lcm, exclusive=True, thread=True)

    def _start_refresh_timer(self) -> None:
        self.set_interval(0.5, self._refresh)

    def _init_lcm(self) -> None:
        """Blocking LCM init — runs in a worker thread."""
        # Stop any existing LCM instance first
        if self._lcm is not None:
            try:
                self._lcm.stop()
            except Exception:
                pass
            self._lcm = None

        try:
            from dimos.protocol.pubsub.impl.lcmpubsub import PickleLCM, Topic

            plcm = PickleLCM()
            plcm.subscribe(Topic("/dimos/resource_stats"), self._on_msg)
            plcm.start()
            self._lcm = plcm
        except Exception as e:
            self._debug(f"_init_lcm FAILED: {e}")
            self._lcm = None

    def on_unmount_subapp(self) -> None:
        if self._lcm:
            try:
                self._lcm.stop()
            except Exception:
                pass
            self._lcm = None

    def reinit_lcm(self) -> None:
        self._debug("reinit_lcm called (autoconf changed network config)")
        self._latest = None
        self._reconnecting = False
        self.run_worker(self._init_lcm, exclusive=True, thread=True)

    def _reconnect_lcm(self) -> None:
        """Tear down and re-create the LCM subscription."""
        try:
            self._init_lcm()
        except Exception:
            pass
        finally:
            self._reconnecting = False

    def _on_msg(self, msg: dict[str, Any], _topic: str) -> None:
        with self._lock:
            self._latest = msg
            self._last_msg_time = time.monotonic()

    def _refresh(self) -> None:
        with self._lock:
            data = self._latest
            last_msg = self._last_msg_time

        try:
            scroll = self.query_one(VerticalScroll)
        except Exception:
            return

        now = time.monotonic()

        if data is None:
            scroll.add_class("waiting")
            self.query_one("#dtop-panels", Static).update(self._waiting_panel())
            return
        scroll.remove_class("waiting")

        stale = (now - last_msg) > self._STALE_TIMEOUT

        # Auto-reconnect if we haven't received data in a while
        if (now - last_msg) > self._RECONNECT_TIMEOUT and not self._reconnecting:
            self._reconnecting = True
            self._debug(f"No data for {now - last_msg:.0f}s, reconnecting LCM...")
            self.run_worker(self._reconnect_lcm, exclusive=True, thread=True)
        dim = theme.STALE_COLOR
        border_style = dim if stale else theme.PID_COLOR

        entries: list[tuple[str, str, dict[str, Any], str, str]] = []
        coord = data.get("coordinator", {})
        entries.append(("coordinator", theme.BRIGHT_CYAN, coord, "", str(coord.get("pid", ""))))

        for w in data.get("workers", []):
            alive = w.get("alive", False)
            wid = w.get("worker_id", "?")
            role_style = theme.BRIGHT_GREEN if alive else theme.BRIGHT_RED
            modules = ", ".join(w.get("modules", [])) or ""
            entries.append((f"worker {wid}", role_style, w, modules, str(w.get("pid", ""))))

        ranges = _compute_ranges([d for _, _, d, _, _ in entries])

        parts: list[RenderableType] = []
        for i, (role, rs, d, mods, pid) in enumerate(entries):
            if role not in self._cpu_history:
                self._cpu_history[role] = deque(maxlen=_SPARK_WIDTH * 2)
            if not stale:
                self._cpu_history[role].append(d.get("cpu_percent", 0))
            if i > 0:
                title = Text(" ")
                title.append(role, style=dim if stale else theme.LABEL_COLOR)
                if mods:
                    title.append(": ", style=dim if stale else theme.LABEL_COLOR)
                    title.append(mods, style=dim if stale else rs)
                if pid:
                    title.append(f" [{pid}]", style=dim if stale else theme.PID_COLOR)
                title.append(" ")
                parts.append(Rule(title=title, style=border_style))
            parts.extend(ResourceSpyApp._make_lines(d, stale, ranges, self._cpu_history[role]))

        first_role, first_rs, _, first_mods, first_pid = entries[0]
        panel_title = Text(" ")
        panel_title.append(first_role, style=dim if stale else theme.LABEL_COLOR)
        if first_mods:
            panel_title.append(": ", style=dim if stale else theme.LABEL_COLOR)
            panel_title.append(first_mods, style=dim if stale else first_rs)
        if first_pid:
            panel_title.append(f" [{first_pid}]", style=dim if stale else theme.PID_COLOR)
        panel_title.append(" ")

        panel = Panel(Group(*parts), title=panel_title, border_style=border_style)
        self.query_one("#dtop-panels", Static).update(panel)
