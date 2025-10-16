# Copyright 2025 Dimensional Inc.
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

import math
import random
import threading
from typing import List

from rich.text import Text
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.color import Color
from textual.containers import Container
from textual.reactive import reactive
from textual.renderables.sparkline import Sparkline as SparklineRenderable
from textual.widgets import DataTable, Header, Label, Sparkline

from dimos.utils.cli import theme
from dimos.utils.cli.lcmspy.lcmspy import GraphLCMSpy
from dimos.utils.cli.lcmspy.lcmspy import GraphTopic as SpyTopic


def gradient(max_value: float, value: float) -> str:
    """Gradient from cyan (low) to yellow (high) using DimOS theme colors"""
    ratio = min(value / max_value, 1.0)
    # Parse hex colors from theme
    cyan = Color.parse(theme.CYAN)
    yellow = Color.parse(theme.YELLOW)
    color = cyan.blend(yellow, ratio)

    return color.hex


def topic_text(topic_name: str) -> Text:
    """Format topic name with DimOS theme colors"""
    if "#" in topic_name:
        parts = topic_name.split("#", 1)
        return Text(parts[0], style=theme.BRIGHT_WHITE) + Text("#" + parts[1], style=theme.BLUE)

    if topic_name[:4] == "/rpc":
        return Text(topic_name[:4], style=theme.BLUE) + Text(
            topic_name[4:], style=theme.BRIGHT_WHITE
        )

    return Text(topic_name, style=theme.BRIGHT_WHITE)


class LCMSpyApp(App):
    """A real-time CLI dashboard for LCM traffic statistics using Textual."""

    CSS_PATH = "../dimos.tcss"

    CSS = f"""
    Screen {{
        layout: vertical;
        background: {theme.BACKGROUND};
    }}
    DataTable {{
        height: 2fr;
        width: 1fr;
        border: solid {theme.BORDER};
        background: {theme.BG};
        scrollbar-size: 0 0;
    }}
    DataTable > .datatable--header {{
        color: {theme.ACCENT};
        background: transparent;
    }}
    """

    refresh_interval: float = 0.5  # seconds

    BINDINGS = [
        ("q", "quit"),
        ("ctrl+c", "quit"),
    ]

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.spy = GraphLCMSpy(autoconf=True, graph_log_window=0.5)
        self.table: DataTable | None = None

    def compose(self) -> ComposeResult:
        self.table = DataTable(zebra_stripes=False, cursor_type=None)
        self.table.add_column("Topic")
        self.table.add_column("Freq (Hz)")
        self.table.add_column("Bandwidth")
        self.table.add_column("Total Traffic")
        yield self.table

    def on_mount(self):
        self.spy.start()
        self.set_interval(self.refresh_interval, self.refresh_table)

    async def on_unmount(self):
        self.spy.stop()

    def refresh_table(self):
        topics: List[SpyTopic] = list(self.spy.topic.values())
        topics.sort(key=lambda t: t.total_traffic(), reverse=True)
        self.table.clear(columns=False)

        for t in topics:
            freq = t.freq(5.0)
            kbps = t.kbps(5.0)
            bw_val, bw_unit = t.kbps_hr(5.0)
            total_val, total_unit = t.total_traffic_hr()

            self.table.add_row(
                topic_text(t.name),
                Text(f"{freq:.1f}", style=gradient(10, freq)),
                Text(f"{bw_val} {bw_unit.value}/s", style=gradient(1024 * 3, kbps)),
                Text(f"{total_val} {total_unit.value}"),
            )


def main():
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == "web":
        import os

        from textual_serve.server import Server

        server = Server(f"python {os.path.abspath(__file__)}")
        server.serve()
    else:
        LCMSpyApp().run()


if __name__ == "__main__":
    main()
