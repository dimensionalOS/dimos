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

"""Live TUI for per-worker resource stats over LCM.

Usage:
    uv run python -m dimos.utils.cli.dtop [--topic /dimos/resource_stats]
"""

from __future__ import annotations

import threading
from typing import Any

from rich.text import Text
from textual.app import App, ComposeResult
from textual.color import Color
from textual.widgets import DataTable

from dimos.protocol.pubsub.impl.lcmpubsub import PickleLCM, Topic
from dimos.utils.cli import theme


def _heat(ratio: float) -> str:
    """Map 0..1 ratio to a cyan → yellow → red gradient."""
    cyan = Color.parse(theme.CYAN)
    yellow = Color.parse(theme.YELLOW)
    red = Color.parse(theme.RED)
    if ratio <= 0.5:
        return cyan.blend(yellow, ratio * 2).hex
    return yellow.blend(red, (ratio - 0.5) * 2).hex


def _bar(value: float, max_val: float, width: int = 12) -> Text:
    """Render a tiny colored bar."""
    ratio = min(value / max_val, 1.0) if max_val > 0 else 0.0
    filled = int(ratio * width)
    return Text("█" * filled + "░" * (width - filled), style=_heat(ratio))


def _fmt_mb(val: float) -> Text:
    if val >= 1024:
        return Text(f"{val / 1024:.1f} GB", style=theme.BRIGHT_YELLOW)
    return Text(f"{val:.1f} MB", style=theme.WHITE)


def _fmt_pct(val: float) -> Text:
    return Text(f"{val:.0f}%", style=_heat(min(val / 100.0, 1.0)))


def _fmt_time(seconds: float) -> Text:
    if seconds >= 3600:
        return Text(f"{seconds / 3600:.1f}h", style=theme.WHITE)
    if seconds >= 60:
        return Text(f"{seconds / 60:.1f}m", style=theme.WHITE)
    return Text(f"{seconds:.1f}s", style=theme.WHITE)


class ResourceSpyApp(App):  # type: ignore[type-arg]
    CSS_PATH = "dimos.tcss"

    TITLE = ""
    SHOW_TREE = False

    CSS = f"""
    Screen {{
        layout: vertical;
        background: {theme.BACKGROUND};
    }}
    DataTable {{
        height: 1fr;
        border: solid {theme.BORDER};
        background: {theme.BG};
        scrollbar-size: 0 0;
    }}
    DataTable > .datatable--header {{
        color: {theme.ACCENT};
        background: transparent;
    }}
    """

    BINDINGS = [("q", "quit"), ("ctrl+c", "quit")]

    def __init__(self, topic_name: str = "/dimos/resource_stats") -> None:
        super().__init__()
        self._topic_name = topic_name
        self._lcm = PickleLCM(autoconf=True)
        self._lock = threading.Lock()
        self._latest: dict[str, Any] | None = None

    def compose(self) -> ComposeResult:
        table: DataTable = DataTable(zebra_stripes=True, cursor_type=None)  # type: ignore[type-arg, arg-type]
        table.add_column("Role", width=14)
        table.add_column("PID", width=8)
        table.add_column("CPU %", width=8)
        table.add_column("CPU bar", width=14)
        table.add_column("User", width=8)
        table.add_column("Sys", width=8)
        table.add_column("IOw", width=8)
        table.add_column("PSS", width=10)
        table.add_column("USS", width=10)
        table.add_column("RSS", width=10)
        table.add_column("VMS", width=10)
        table.add_column("Thr", width=5)
        table.add_column("Ch", width=5)
        table.add_column("FDs", width=5)
        table.add_column("IO R", width=10)
        table.add_column("IO W", width=10)
        table.add_column("Modules", width=30)
        yield table

    def on_mount(self) -> None:
        self._lcm.subscribe(Topic(self._topic_name), self._on_msg)
        self._lcm.start()
        self.set_interval(1.0, self._refresh)

    async def on_unmount(self) -> None:
        self._lcm.stop()

    def _on_msg(self, msg: dict[str, Any], _topic: str) -> None:
        with self._lock:
            self._latest = msg

    def _refresh(self) -> None:
        with self._lock:
            data = self._latest

        if data is None:
            return

        table = self.query_one(DataTable)
        table.clear(columns=False)

        coord = data.get("coordinator", {})
        self._add_row(table, "coordinator", theme.BRIGHT_CYAN, coord, "—")

        for w in data.get("workers", []):
            alive = w.get("alive", False)
            wid = w.get("worker_id", "?")
            role_style = theme.BRIGHT_GREEN if alive else theme.BRIGHT_RED
            modules = ", ".join(w.get("modules", [])) or "—"
            self._add_row(table, f"worker {wid}", role_style, w, modules)

    @staticmethod
    def _add_row(
        table: DataTable,  # type: ignore[type-arg]
        role: str,
        role_style: str,
        d: dict[str, Any],
        modules: str,
    ) -> None:
        table.add_row(
            Text(role, style=role_style),
            Text(str(d.get("pid", "?")), style=theme.DIM),
            _fmt_pct(d.get("cpu_percent", 0)),
            _bar(d.get("cpu_percent", 0), 100),
            _fmt_time(d.get("cpu_time_user", 0)),
            _fmt_time(d.get("cpu_time_system", 0)),
            _fmt_time(d.get("cpu_time_iowait", 0)),
            _fmt_mb(d.get("pss_mb", 0)),
            _fmt_mb(d.get("uss_mb", 0)),
            _fmt_mb(d.get("rss_mb", 0)),
            _fmt_mb(d.get("vms_mb", 0)),
            Text(str(d.get("num_threads", 0)), style=theme.WHITE),
            Text(str(d.get("num_children", 0)), style=theme.WHITE),
            Text(str(d.get("num_fds", 0)), style=theme.WHITE),
            _fmt_mb(d.get("io_read_mb", 0)),
            _fmt_mb(d.get("io_write_mb", 0)),
            Text(modules, style=theme.BRIGHT_BLUE),
        )


def main() -> None:
    import sys

    topic = "/dimos/resource_stats"
    if len(sys.argv) > 1 and sys.argv[1] == "--topic" and len(sys.argv) > 2:
        topic = sys.argv[2]

    ResourceSpyApp(topic_name=topic).run()


if __name__ == "__main__":
    main()
