# Copyright 2025-2026 Dimensional Inc.
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

"""lcmflow — a real-time LCM packet highway.

Each topic is a lane; every packet drives across it as a vehicle.
Small fast packets (cmd_vel, tf) are zippy dots, images and point
clouds are long slow trucks. Run alongside any DimOS stack:

    lcmflow            # native TUI
    lcmflow web        # serve the TUI in a browser
    lcmflow dashboard  # module-graph "constellation" web dashboard (deno)
    dimos lcmflow      # same, via the dimos CLI
"""

from __future__ import annotations

from rich.segment import Segment
from rich.style import Style
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.color import Color
from textual.geometry import Size
from textual.scroll_view import ScrollView
from textual.strip import Strip
from textual.widgets import Static

from dimos.utils.cli import theme
from dimos.utils.cli.lcmflow.lcmflow import TRAIL_LEN, Highway, Lane

FPS = 20.0
LABEL_W = 38  # left column: topic names + stats
ROWS_PER_LANE = 2  # vehicle row + separator/stats row

BG = Color.parse(theme.BACKGROUND)
ROAD_DOT_STYLE = Style(color=Color.parse(theme.DIM).blend(BG, 0.55).hex, bgcolor=theme.BACKGROUND)
SEPARATOR_STYLE = Style(color=Color.parse(theme.DIM).blend(BG, 0.35).hex, bgcolor=theme.BACKGROUND)
BLANK_STYLE = Style(bgcolor=theme.BACKGROUND)

BODY_CHAR = "▰"
HEAD_CHAR = "▶"
NANO_CHAR = "●"
TRAIL_CHARS = ("╸", "╌", "·")  # brightest first, fading behind the vehicle


def freq_gradient(freq: float, max_freq: float = 30.0) -> str:
    """Cyan (idle) to yellow (hot) for frequency readouts."""
    ratio = min(freq / max_freq, 1.0)
    return Color.parse(theme.CYAN).blend(Color.parse(theme.YELLOW), ratio).hex


class HighwayView(ScrollView):
    """Scrollable, line-rendered view of all lanes."""

    can_focus = True

    def __init__(self, highway: Highway) -> None:
        super().__init__()
        self.highway = highway
        self.order: list[str] = []  # lane display order (arrival)
        self.sort_mode = "arrival"  # arrival | traffic | name
        self._lanes_view: list[Lane] = []  # sorted snapshot, rebuilt per tick

    def on_mount(self) -> None:
        self.set_interval(1 / FPS, self.tick)

    def tick(self) -> None:
        road_width = max(40, self.size.width - LABEL_W)
        self.highway.tick(road_width)
        for channel in self.highway.lanes:
            if channel not in self.order:
                self.order.append(channel)
        self._lanes_view = self._sorted_lanes()
        self.virtual_size = Size(self.size.width, len(self.order) * ROWS_PER_LANE)
        self.refresh()

    def cycle_sort(self) -> str:
        modes = ["arrival", "traffic", "name"]
        self.sort_mode = modes[(modes.index(self.sort_mode) + 1) % len(modes)]
        return self.sort_mode

    def _sorted_lanes(self) -> list[Lane]:
        lanes = [self.highway.lanes[c] for c in self.order if c in self.highway.lanes]
        if self.sort_mode == "traffic":
            spy_topics = self.highway.spy.topics()
            lanes.sort(
                key=lambda lane: spy_topics[lane.channel].total_traffic()
                if lane.channel in spy_topics
                else 0,
                reverse=True,
            )
        elif self.sort_mode == "name":
            lanes.sort(key=lambda lane: lane.topic)
        return lanes

    def render_line(self, y: int) -> Strip:
        y += int(self.scroll_offset.y)
        lanes = self._lanes_view
        lane_idx, row = divmod(y, ROWS_PER_LANE)
        if lane_idx >= len(lanes):
            return Strip([Segment(" " * self.size.width, BLANK_STYLE)])
        lane = lanes[lane_idx]
        road_width = max(40, self.size.width - LABEL_W)
        if row == 0:
            segments = self._label_segments(lane) + self._road_segments(lane, road_width)
        else:
            segments = self._stats_segments(lane) + self._separator_segments(road_width)
        return Strip(segments, self.size.width)

    def _label_segments(self, lane: Lane) -> list[Segment]:
        """Topic name, with the message type in the lane color."""
        topic = lane.topic
        msg_name = lane.type_name.rsplit(".", 1)[-1] if lane.type_name else ""
        room = LABEL_W - 2  # leading space + trailing space
        if msg_name and len(topic) + 1 + len(msg_name) > room:
            topic = topic[: max(1, room - len(msg_name) - 2)] + "…"
        text = f" {topic}"
        segments = [Segment(text, Style(color=theme.BRIGHT_WHITE, bgcolor=theme.BACKGROUND))]
        used = len(text)
        if msg_name:
            type_text = f"·{msg_name}"[: max(0, LABEL_W - used - 1)]
            segments.append(
                Segment(
                    type_text,
                    Style(
                        color=Color.parse(lane.color).blend(BG, 0.25).hex, bgcolor=theme.BACKGROUND
                    ),
                )
            )
            used += len(type_text)
        segments.append(Segment(" " * max(0, LABEL_W - used), BLANK_STYLE))
        return segments

    def _stats_segments(self, lane: Lane) -> list[Segment]:
        spy_topic = self.highway.spy.topics().get(lane.channel)
        if spy_topic is None:
            return [Segment(" " * LABEL_W, BLANK_STYLE)]
        freq = spy_topic.freq(5.0)
        parts = [
            ("   ", BLANK_STYLE),
            (f"{freq:6.1f} Hz ", Style(color=freq_gradient(freq), bgcolor=theme.BACKGROUND)),
            (
                f"{spy_topic.kbps_hr(5.0):>12} ",
                Style(color=theme.WHITE, bgcolor=theme.BACKGROUND, dim=True),
            ),
            (
                f"Σ{spy_topic.total_traffic_hr():>10}",
                Style(color=theme.DIM, bgcolor=theme.BACKGROUND),
            ),
        ]
        segments: list[Segment] = []
        used = 0
        for text, style in parts:
            text = text[: max(0, LABEL_W - used)]
            if text:
                segments.append(Segment(text, style))
                used += len(text)
        segments.append(Segment(" " * max(0, LABEL_W - used), BLANK_STYLE))
        return segments

    def _road_segments(self, lane: Lane, road_width: int) -> list[Segment]:
        """Render the vehicles of one lane into styled segments."""
        chars = [" "] * road_width
        styles: list[Style] = [BLANK_STYLE] * road_width
        # Faint distance markers so empty road still reads as a road.
        for x in range(0, road_width, 8):
            chars[x] = "·"
            styles[x] = ROAD_DOT_STYLE

        now = self.highway.clock
        color = Color.parse(lane.color)
        body_style = Style(color=lane.color, bgcolor=theme.BACKGROUND, bold=True)
        head_style = Style(
            color=color.blend(Color.parse(theme.BRIGHT_WHITE), 0.65).hex,
            bgcolor=theme.BACKGROUND,
            bold=True,
        )
        trail_styles = [
            Style(color=color.blend(BG, 1 - alpha).hex, bgcolor=theme.BACKGROUND)
            for alpha in (0.55, 0.3, 0.15)
        ]

        for vehicle in lane.vehicles:
            head = int(vehicle.head(now))
            length = vehicle.length
            tail = head - length + 1
            if tail >= road_width or head < -TRAIL_LEN:
                continue
            # Fading trail behind the tail.
            for i in range(TRAIL_LEN):
                x = tail - 1 - i
                if 0 <= x < road_width:
                    chars[x] = TRAIL_CHARS[i]
                    styles[x] = trail_styles[i]
            # Body.
            for x in range(max(0, tail), min(road_width, head + 1)):
                chars[x] = BODY_CHAR if length > 1 else NANO_CHAR
                styles[x] = body_style
            # Bright nose on anything truck-sized.
            if length >= 3 and 0 <= head < road_width:
                chars[head] = HEAD_CHAR
                styles[head] = head_style
            # Passenger count for coalesced bursts, etched into the body.
            if vehicle.count > 1 and length >= 4:
                badge = f"×{min(vehicle.count, 999)}"  # noqa: RUF001 — intentional UI glyph
                badge_style = Style(color=theme.BACKGROUND, bgcolor=lane.color, bold=True)
                start = tail + max(0, (length - 1 - len(badge)) // 2)
                if start >= 0 and start + len(badge) < head:
                    for i, ch in enumerate(badge):
                        if 0 <= start + i < road_width:
                            chars[start + i] = ch
                            styles[start + i] = badge_style

        # Group identical styles into few segments.
        segments: list[Segment] = []
        run_start = 0
        for x in range(1, road_width + 1):
            if x == road_width or styles[x] is not styles[run_start]:
                segments.append(Segment("".join(chars[run_start:x]), styles[run_start]))
                run_start = x
        return segments

    def _separator_segments(self, road_width: int) -> list[Segment]:
        pattern = ("╌" * 5 + "   ") * (road_width // 8 + 1)
        return [Segment(pattern[:road_width], SEPARATOR_STYLE)]


class HeaderBar(Static):
    """One-line global stats strip."""

    def __init__(self, view: HighwayView) -> None:
        super().__init__()
        self.view = view

    def on_mount(self) -> None:
        self.set_interval(0.5, self.refresh_stats)
        self.refresh_stats()

    def refresh_stats(self) -> None:
        highway = self.view.highway
        spy = highway.spy
        n_topics = len(highway.lanes)
        state = "  ⏸ PAUSED" if highway.paused else ""
        self.update(
            f"[bold {theme.CYAN}]◢◤ LCM FLOW[/]"
            f"[{theme.DIM}] · packet highway[/]"
            f"[bold {theme.YELLOW}]{state}[/]"
            f"   [{theme.WHITE}]{n_topics} lanes[/]"
            f"   [{freq_gradient(spy.freq(5.0), 100)}]{spy.freq(5.0):.1f} Hz[/]"
            f"   [{theme.WHITE}]{spy.kbps_hr(5.0)}[/]"
            f"   [{theme.DIM}]Σ {spy.total_traffic_hr()}[/]"
            f"   [{theme.DIM}]sort: {self.view.sort_mode}[/]"
        )


class LCMFlowApp(App):  # type: ignore[type-arg]
    """Real-time LCM packet highway visualization."""

    CSS = f"""
    Screen {{
        layout: vertical;
        background: {theme.BACKGROUND};
    }}
    HeaderBar {{
        height: 2;
        padding: 0 1;
        background: {theme.BACKGROUND};
        border-bottom: solid {theme.BORDER};
    }}
    HighwayView {{
        background: {theme.BACKGROUND};
        scrollbar-size: 1 1;
    }}
    Footer {{
        background: {theme.BACKGROUND};
    }}
    """

    BINDINGS = [
        Binding("q", "quit", "quit"),
        Binding("ctrl+c", "quit", show=False),
        Binding("space", "pause", "pause"),
        Binding("s", "sort", "sort"),
    ]

    def __init__(self) -> None:
        super().__init__()
        # Warn about missing system config before entering TUI raw mode.
        from dimos.protocol.service.lcmservice import autoconf

        autoconf(check_only=True)

        self.highway = Highway()
        self.highway.start()
        self.view = HighwayView(self.highway)

    def compose(self) -> ComposeResult:
        from textual.widgets import Footer

        yield HeaderBar(self.view)
        yield self.view
        yield Footer()

    async def on_unmount(self) -> None:
        self.highway.stop()

    def action_pause(self) -> None:
        self.highway.toggle_pause()

    def action_sort(self) -> None:
        self.view.cycle_sort()


def _stop_dashboard() -> None:
    """Terminate any running lcmflow dashboard servers."""
    import psutil

    stopped = 0
    for proc in psutil.process_iter(["cmdline"]):
        try:
            cmdline = " ".join(proc.info["cmdline"] or [])
            if "web/lcmflow/server.ts" in cmdline:
                proc.terminate()
                stopped += 1
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    print(f"stopped {stopped} dashboard server(s)" if stopped else "no dashboard running")


def _run_dashboard(extra_args: list[str]) -> None:
    """Exec the deno-based constellation dashboard (dimos/web/lcmflow)."""
    import os
    import shutil
    import sys

    import dimos

    if extra_args and extra_args[0] in ("stop", "--stop"):
        _stop_dashboard()
        return

    deno = shutil.which("deno")
    if deno is None:
        print("lcmflow dashboard requires deno: https://docs.deno.com/runtime/")
        print("  curl -fsSL https://deno.land/install.sh | sh")
        raise SystemExit(1)
    server_ts = os.path.join(os.path.dirname(dimos.__file__), "web", "lcmflow", "server.ts")
    # Pass this venv's python so the server can shell out to the direction
    # sidecar (dimos.utils.cli.lcmflow.topology) — that's how arrow direction
    # is recovered without changing the coordinator. --allow-run is unscoped
    # because Deno rejects path-scoped run permission once LD_LIBRARY_PATH is
    # inherited; the only thing spawned is this same interpreter.
    python = sys.executable
    os.execv(
        deno,
        [
            deno,
            "run",
            "--allow-net",
            "--allow-read",
            "--allow-env",
            "--allow-run",
            "--unstable-net",
            server_ts,
            "--python",
            python,
            *extra_args,
        ],
    )


def main() -> None:
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == "dashboard":
        _run_dashboard(sys.argv[2:])
    elif len(sys.argv) > 1 and sys.argv[1] == "web":
        import os

        from textual_serve.server import Server  # type: ignore[import-not-found]

        server = Server(f"python {os.path.abspath(__file__)}")
        server.serve()
    else:
        LCMFlowApp().run()


if __name__ == "__main__":
    main()
