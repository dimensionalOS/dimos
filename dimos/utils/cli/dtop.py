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
import time
from typing import TYPE_CHECKING, Any

from rich.console import Group, RenderableType
from rich.panel import Panel
from rich.rule import Rule
from rich.text import Text
from textual.app import App, ComposeResult
from textual.color import Color
from textual.containers import VerticalScroll
from textual.widgets import Static

from dimos.protocol.pubsub.impl.lcmpubsub import PickleLCM, Topic
from dimos.utils.cli import theme

if TYPE_CHECKING:
    from collections.abc import Callable

# ---------------------------------------------------------------------------
# Color helpers
# ---------------------------------------------------------------------------


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


def _rel_style(value: float, lo: float, hi: float) -> str:
    """Color a value by where it sits in the observed [lo, hi] range."""
    if hi <= lo:
        return _heat(0.0)
    return _heat(min((value - lo) / (hi - lo), 1.0))


# ---------------------------------------------------------------------------
# Metric formatters (plain strings — color applied separately via _rel_style)
# ---------------------------------------------------------------------------


def _fmt_pct(v: float) -> str:
    return f"{v:.0f}%"


def _fmt_mem(v: float) -> str:
    mb = v / 1048576
    if mb >= 1024:
        return f"{mb / 1024:.1f} GB"
    return f"{mb:.1f} MB"


def _fmt_int(v: float) -> str:
    return str(int(v))


def _fmt_secs(v: float) -> str:
    if v >= 3600:
        return f"{v / 3600:.1f}h"
    if v >= 60:
        return f"{v / 60:.1f}m"
    return f"{v:.1f}s"


def _fmt_io(v: float) -> str:
    return f"{v / 1048576:.0f} MB"


# ---------------------------------------------------------------------------
# Metric definitions — add a tuple here to add a new field
# (label, dict_key, format_fn)
# ---------------------------------------------------------------------------

_LINE1: list[tuple[str, str, Callable[[float], str]]] = [
    ("CPU", "cpu_percent", _fmt_pct),
    ("PSS", "pss", _fmt_mem),
    ("Thr", "num_threads", _fmt_int),
    ("Child", "num_children", _fmt_int),
    ("FDs", "num_fds", _fmt_int),
]

_LINE2: list[tuple[str, str, Callable[[float], str]]] = [
    ("UserT", "cpu_time_user", _fmt_secs),
    ("SysT", "cpu_time_system", _fmt_secs),
    ("ioT", "cpu_time_iowait", _fmt_secs),
]

# IO r/w is a compound field handled specially in _make_lines
_IO_KEYS = ("io_read_bytes", "io_write_bytes")

_ALL_KEYS = {key for _, key, _ in _LINE1 + _LINE2} | set(_IO_KEYS)


def _compute_ranges(data_dicts: list[dict[str, Any]]) -> dict[str, tuple[float, float]]:
    """(min, max) per metric across all processes (for relative coloring)."""
    ranges: dict[str, tuple[float, float]] = {}
    for key in _ALL_KEYS:
        vals = [d.get(key, 0) for d in data_dicts]
        ranges[key] = (min(vals), max(vals))
    return ranges


# ---------------------------------------------------------------------------
# App
# ---------------------------------------------------------------------------


class ResourceSpyApp(App):  # type: ignore[type-arg]
    CSS_PATH = "dimos.tcss"

    TITLE = ""
    SHOW_TREE = False

    CSS = f"""
    Screen {{
        layout: vertical;
        background: {theme.BACKGROUND};
    }}
    VerticalScroll {{
        height: 1fr;
        scrollbar-size: 0 0;
    }}
    #panels {{
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
        self._last_msg_time: float = 0.0

    def compose(self) -> ComposeResult:
        with VerticalScroll():
            yield Static(id="panels")

    def on_mount(self) -> None:
        self._lcm.subscribe(Topic(self._topic_name), self._on_msg)
        self._lcm.start()
        self.set_interval(1.0, self._refresh)

    async def on_unmount(self) -> None:
        self._lcm.stop()

    def _on_msg(self, msg: dict[str, Any], _topic: str) -> None:
        with self._lock:
            self._latest = msg
            self._last_msg_time = time.monotonic()

    def _refresh(self) -> None:
        with self._lock:
            data = self._latest
            last_msg = self._last_msg_time

        if data is None:
            return

        stale = (time.monotonic() - last_msg) > 2.0
        dim = "#606060"
        border_style = dim if stale else "#777777"

        # Collect (role, role_style, data_dict, modules) entries
        entries: list[tuple[str, str, dict[str, Any], str]] = []

        coord = data.get("coordinator", {})
        entries.append(("coordinator", theme.BRIGHT_CYAN, coord, ""))

        for w in data.get("workers", []):
            alive = w.get("alive", False)
            wid = w.get("worker_id", "?")
            role_style = theme.BRIGHT_GREEN if alive else theme.BRIGHT_RED
            modules = ", ".join(w.get("modules", [])) or ""
            entries.append((f"worker {wid}", role_style, w, modules))

        # Per-metric max for relative coloring
        ranges = _compute_ranges([d for _, _, d, _ in entries])

        # Build inner content: sections separated by Rules
        parts: list[RenderableType] = []
        for i, (role, rs, d, mods) in enumerate(entries):
            if i > 0:
                title = Text(
                    f" {role}: {mods} " if mods else f" {role} ",
                    style=dim if stale else rs,
                )
                parts.append(Rule(title=title, style=border_style))
            parts.extend(self._make_lines(d, stale, ranges))

        # First entry title goes on the Panel itself
        first_role, first_rs, _, first_mods = entries[0]
        panel_title = Text(
            f" {first_role}: {first_mods} " if first_mods else f" {first_role} ",
            style=dim if stale else first_rs,
        )

        panel = Panel(
            Group(*parts),
            title=panel_title,
            border_style=border_style,
        )
        self.query_one("#panels", Static).update(panel)

    @staticmethod
    def _make_lines(
        d: dict[str, Any],
        stale: bool,
        ranges: dict[str, tuple[float, float]],
    ) -> list[Text]:
        dim = "#606060"
        label1_style = dim if stale else theme.WHITE
        label2_style = label1_style

        # Line 1
        line1 = Text()
        for label, key, fmt in _LINE1:
            val = d.get(key, 0)
            lo, hi = ranges[key]
            # CPU% uses absolute 0-100 scale; everything else is relative
            if key == "cpu_percent":
                val_style = dim if stale else _heat(min(val / 100.0, 1.0))
            else:
                val_style = dim if stale else _rel_style(val, lo, hi)
            line1.append(f"{label} ", style=label1_style)
            line1.append(fmt(val), style=val_style)
            # CPU bar right after CPU%
            if key == "cpu_percent":
                line1.append(" ")
                if stale:
                    line1.append("░" * 12, style=dim)
                else:
                    line1.append_text(_bar(val, 100))
            line1.append("  ")

        # Line 2
        line2 = Text()
        for _i, (label, key, fmt) in enumerate(_LINE2):
            val = d.get(key, 0)
            lo, hi = ranges[key]
            val_style = dim if stale else _rel_style(val, lo, hi)
            line2.append(f"{label} ", style=label2_style)
            line2.append(fmt(val), style=val_style)
            line2.append("  ")

        # IO r/w — compound field
        io_r = d.get(_IO_KEYS[0], 0)
        io_w = d.get(_IO_KEYS[1], 0)
        lo_r, hi_r = ranges[_IO_KEYS[0]]
        lo_w, hi_w = ranges[_IO_KEYS[1]]
        line2.append("IO r/w ", style=label2_style)
        line2.append(_fmt_io(io_r), style=dim if stale else _rel_style(io_r, lo_r, hi_r))
        line2.append("/", style=label2_style)
        line2.append(_fmt_io(io_w), style=dim if stale else _rel_style(io_w, lo_w, hi_w))

        return [line1, line2]


# ---------------------------------------------------------------------------
# Preview
# ---------------------------------------------------------------------------

_PREVIEW_DATA: dict[str, Any] = {
    "coordinator": {
        "cpu_percent": 12.3,
        "pss": 47_400_000,
        "num_threads": 4,
        "num_children": 0,
        "num_fds": 32,
        "cpu_time_user": 1.2,
        "cpu_time_system": 0.3,
        "cpu_time_iowait": 0.0,
        "io_read_bytes": 12_582_912,
        "io_write_bytes": 4_194_304,
        "pid": 1234,
    },
    "workers": [
        {
            "worker_id": 0,
            "alive": True,
            "modules": ["nav", "lidar"],
            "cpu_percent": 34.0,
            "pss": 125_829_120,
            "num_threads": 8,
            "num_children": 2,
            "num_fds": 64,
            "cpu_time_user": 5.1,
            "cpu_time_system": 1.0,
            "cpu_time_iowait": 0.2,
            "io_read_bytes": 47_185_920,
            "io_write_bytes": 12_582_912,
            "pid": 1235,
        },
        {
            "worker_id": 1,
            "alive": False,
            "modules": ["vision"],
            "cpu_percent": 87.0,
            "pss": 536_870_912,
            "num_threads": 16,
            "num_children": 1,
            "num_fds": 128,
            "cpu_time_user": 42.5,
            "cpu_time_system": 8.3,
            "cpu_time_iowait": 1.1,
            "io_read_bytes": 1_073_741_824,
            "io_write_bytes": 536_870_912,
            "pid": 1236,
        },
    ],
}


def _preview() -> None:
    """Print a static preview with fake data (no LCM needed)."""
    from rich.console import Console

    data = _PREVIEW_DATA
    border_style = "#555555"

    entries: list[tuple[str, str, dict[str, Any], str]] = []
    entries.append(("coordinator", theme.BRIGHT_CYAN, data["coordinator"], ""))
    for w in data["workers"]:
        rs = theme.BRIGHT_GREEN if w.get("alive") else theme.BRIGHT_RED
        mods = ", ".join(w.get("modules", []))
        entries.append((f"worker {w['worker_id']}", rs, w, mods))

    ranges = _compute_ranges([d for _, _, d, _ in entries])

    parts: list[RenderableType] = []
    for i, (role, rs, d, mods) in enumerate(entries):
        if i > 0:
            title = Text(f" {role}: {mods} " if mods else f" {role} ", style=rs)
            parts.append(Rule(title=title, style=border_style))
        parts.extend(ResourceSpyApp._make_lines(d, stale=False, ranges=ranges))

    first_role, first_rs, _, first_mods = entries[0]
    panel_title = Text(f" {first_role} ", style=first_rs)
    Console().print(Panel(Group(*parts), title=panel_title, border_style=border_style))


def main() -> None:
    import sys

    if "--preview" in sys.argv:
        _preview()
        return

    topic = "/dimos/resource_stats"
    if len(sys.argv) > 1 and sys.argv[1] == "--topic" and len(sys.argv) > 2:
        topic = sys.argv[2]

    ResourceSpyApp(topic_name=topic).run()


if __name__ == "__main__":
    main()
