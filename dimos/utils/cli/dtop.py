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

"""dtop — Live TUI for per-worker resource stats over LCM.

Usage:
    uv run python -m dimos.utils.cli.dtop [--topic /dimos/resource_stats]
"""

from __future__ import annotations

from collections import deque
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


# Braille sparkline — each cell packs two samples (left / right column)
_BRAILLE_BASE = 0x2800
_LDOTS = (0x00, 0x40, 0x44, 0x46, 0x47)  # left col: 0‥4 filled rows
_RDOTS = (0x00, 0x80, 0xA0, 0xB0, 0xB8)  # right col: 0‥4 filled rows
_SPARK_WIDTH = 12  # characters (×2 = 24 samples of history)
_LABEL_COLOR = "#cccccc"  # metric label color (CPU, PSS, Thr, etc.)


def _spark(history: deque[float], width: int = _SPARK_WIDTH) -> Text:
    """Render a braille sparkline from CPU% history (0‥100 values)."""
    n = width * 2
    vals = list(history)
    if len(vals) < n:
        vals = [0.0] * (n - len(vals)) + vals
    else:
        vals = vals[-n:]
    result = Text()
    for i in range(0, n, 2):
        lv = min(vals[i] / 100.0, 1.0)
        rv = min(vals[i + 1] / 100.0, 1.0)
        li = min(int(lv * 4 + 0.5), 4)
        ri = min(int(rv * 4 + 0.5), 4)
        ch = chr(_BRAILLE_BASE | _LDOTS[li] | _RDOTS[ri])
        result.append(ch, style=_heat(max(lv, rv)))
    return result


def _rel_style(value: float, lo: float, hi: float) -> str:
    """Color a value by where it sits in the observed [lo, hi] range."""
    if hi <= lo:
        return _heat(0.0)
    return _heat(min((value - lo) / (hi - lo), 1.0))


def _fmt_pct(v: float) -> str:
    return f"{v:3.0f}%"


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


class ResourceSpyApp(App[None]):
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
    VerticalScroll.waiting {{
        align: center middle;
    }}
    .waiting #panels {{
        width: auto;
    }}
    #panels {{
        background: transparent;
    }}
    """

    BINDINGS = [("q", "quit"), ("ctrl+c", "quit")]

    def __init__(self, topic_name: str = "/dimos/resource_stats") -> None:
        super().__init__()
        self._topic_name = topic_name
        # Warn about missing system config before entering TUI raw mode.
        from dimos.protocol.service.lcmservice import autoconf

        autoconf(check_only=True)

        self._lcm = PickleLCM()
        self._lcm.subscribe(Topic(self._topic_name), self._on_msg)
        self._lcm.start()
        self._lock = threading.Lock()
        self._latest: dict[str, Any] | None = None
        self._last_msg_time: float = 0.0
        self._cpu_history: dict[str, deque[float]] = {}

    def compose(self) -> ComposeResult:
        with VerticalScroll():
            yield Static(id="panels")

    def on_mount(self) -> None:
        self.set_interval(0.5, self._refresh)

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

        scroll = self.query_one(VerticalScroll)
        if data is None:
            scroll.add_class("waiting")
            waiting = Panel(
                Text(
                    "use `dimos --dtop ...` to emit stats",
                    style=theme.FOREGROUND,
                    justify="center",
                ),
                border_style=theme.CYAN,
                expand=False,
            )
            self.query_one("#panels", Static).update(waiting)
            return
        scroll.remove_class("waiting")

        stale = (time.monotonic() - last_msg) > 2.0
        dim = "#606060"
        border_style = dim if stale else "#777777"

        # Collect (role, role_style, data_dict, modules, pid) entries
        entries: list[tuple[str, str, dict[str, Any], str, str]] = []

        coord = data.get("coordinator", {})
        entries.append(("coordinator", theme.BRIGHT_CYAN, coord, "", str(coord.get("pid", ""))))

        for w in data.get("workers", []):
            alive = w.get("alive", False)
            wid = w.get("worker_id", "?")
            role_style = theme.BRIGHT_GREEN if alive else theme.BRIGHT_RED
            modules = ", ".join(w.get("modules", [])) or ""
            entries.append((f"worker {wid}", role_style, w, modules, str(w.get("pid", ""))))

        # Per-metric max for relative coloring
        ranges = _compute_ranges([d for _, _, d, _, _ in entries])

        # Build inner content: sections separated by Rules
        parts: list[RenderableType] = []
        for i, (role, rs, d, mods, pid) in enumerate(entries):
            if role not in self._cpu_history:
                self._cpu_history[role] = deque(maxlen=_SPARK_WIDTH * 2)
            if not stale:
                self._cpu_history[role].append(d.get("cpu_percent", 0))
            if i > 0:
                title = Text(" ")
                title.append(role, style=dim if stale else _LABEL_COLOR)
                if mods:
                    title.append(": ", style=dim if stale else _LABEL_COLOR)
                    title.append(mods, style=dim if stale else rs)
                if pid:
                    title.append(f" [{pid}]", style=dim if stale else "#777777")
                title.append(" ")
                parts.append(Rule(title=title, style=border_style))
            parts.extend(self._make_lines(d, stale, ranges, self._cpu_history[role]))

        # First entry title goes on the Panel itself
        first_role, first_rs, _, first_mods, first_pid = entries[0]
        panel_title = Text(" ")
        panel_title.append(first_role, style=dim if stale else _LABEL_COLOR)
        if first_mods:
            panel_title.append(": ", style=dim if stale else _LABEL_COLOR)
            panel_title.append(first_mods, style=dim if stale else first_rs)
        if first_pid:
            panel_title.append(f" [{first_pid}]", style=dim if stale else "#777777")
        panel_title.append(" ")

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
        cpu_hist: deque[float] | None = None,
    ) -> list[Text]:
        dim = "#606060"
        label1_style = dim if stale else _LABEL_COLOR
        label2_style = label1_style

        sep = " · "
        sep_style = dim if stale else "#555555"

        # Line 1
        line1 = Text()
        for idx, (label, key, fmt) in enumerate(_LINE1):
            val = d.get(key, 0)
            lo, hi = ranges[key]
            # CPU% uses absolute 0-100 scale; everything else is relative
            if key == "cpu_percent":
                val_style = dim if stale else _heat(min(val / 100.0, 1.0))
            else:
                val_style = dim if stale else _rel_style(val, lo, hi)
            if idx > 0:
                line1.append(sep, style=sep_style)
            line1.append(f"{label} ", style=label1_style)
            line1.append(fmt(val), style=val_style)
            # CPU bar right after CPU%
            if key == "cpu_percent":
                line1.append(" ")
                if stale:
                    line1.append("░" * _SPARK_WIDTH, style=dim)
                elif cpu_hist is not None and len(cpu_hist) > 0:
                    line1.append_text(_spark(cpu_hist))
                else:
                    line1.append_text(_bar(val, 100))

        # Line 2
        line2 = Text()
        for idx, (label, key, fmt) in enumerate(_LINE2):
            val = d.get(key, 0)
            lo, hi = ranges[key]
            val_style = dim if stale else _rel_style(val, lo, hi)
            if idx > 0:
                line2.append(sep, style=sep_style)
            line2.append(f"{label} ", style=label2_style)
            line2.append(fmt(val), style=val_style)

        # IO r/w — compound field
        io_r = d.get(_IO_KEYS[0], 0)
        io_w = d.get(_IO_KEYS[1], 0)
        lo_r, hi_r = ranges[_IO_KEYS[0]]
        lo_w, hi_w = ranges[_IO_KEYS[1]]
        line2.append(sep, style=sep_style)
        line2.append("IO r/w ", style=label2_style)
        line2.append(_fmt_io(io_r), style=dim if stale else _rel_style(io_r, lo_r, hi_r))
        line2.append("/", style=label2_style)
        line2.append(_fmt_io(io_w), style=dim if stale else _rel_style(io_w, lo_w, hi_w))

        return [line1, line2]


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
    import math

    from rich.console import Console

    data = _PREVIEW_DATA
    border_style = "#555555"

    entries: list[tuple[str, str, dict[str, Any], str, str]] = []
    entries.append(
        (
            "coordinator",
            theme.BRIGHT_CYAN,
            data["coordinator"],
            "",
            str(data["coordinator"].get("pid", "")),
        )
    )
    for w in data["workers"]:
        rs = theme.BRIGHT_GREEN if w.get("alive") else theme.BRIGHT_RED
        mods = ", ".join(w.get("modules", []))
        entries.append((f"worker {w['worker_id']}", rs, w, mods, str(w.get("pid", ""))))

    ranges = _compute_ranges([d for _, _, d, _, _ in entries])

    parts: list[RenderableType] = []
    for i, (role, rs, d, mods, pid) in enumerate(entries):
        cpu = d.get("cpu_percent", 0)
        hist: deque[float] = deque(maxlen=_SPARK_WIDTH * 2)
        for j in range(_SPARK_WIDTH * 2):
            hist.append(max(0, min(100, cpu + 20 * math.sin(j * 0.6))))
        if i > 0:
            title = Text(" ")
            title.append(role, style=_LABEL_COLOR)
            if mods:
                title.append(": ", style=_LABEL_COLOR)
                title.append(mods, style=rs)
            if pid:
                title.append(f" [{pid}]", style="#777777")
            title.append(" ")
            parts.append(Rule(title=title, style=border_style))
        parts.extend(ResourceSpyApp._make_lines(d, stale=False, ranges=ranges, cpu_hist=hist))

    first_role, first_rs, _, first_mods, first_pid = entries[0]
    panel_title = Text(" ")
    panel_title.append(first_role, style=_LABEL_COLOR)
    if first_mods:
        panel_title.append(": ", style=_LABEL_COLOR)
        panel_title.append(first_mods, style=first_rs)
    if first_pid:
        panel_title.append(f" [{first_pid}]", style="#777777")
    panel_title.append(" ")
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
