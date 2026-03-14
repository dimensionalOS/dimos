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

"""LCM Spy sub-app — embedded LCM traffic monitor."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from rich.text import Text
from textual.widgets import DataTable

from dimos.utils.cli.dio.sub_app import SubApp

if TYPE_CHECKING:
    from textual.app import ComposeResult


class LCMSpySubApp(SubApp):
    TITLE = "lcmspy"

    DEFAULT_CSS = """
    LCMSpySubApp {
        layout: vertical;
        background: $dio-bg;
    }
    LCMSpySubApp DataTable {
        height: 1fr;
        width: 1fr;
        border: solid $dio-dim;
        background: $dio-bg;
        scrollbar-size: 0 0;
    }
    LCMSpySubApp DataTable > .datatable--header {
        color: $dio-text;
        background: transparent;
    }
    """

    def __init__(self) -> None:
        super().__init__()
        self._spy: Any = None

    def _debug(self, msg: str) -> None:
        try:
            self.app._dlog(f"[dim]LCMSPY:[/dim] {msg}")  # type: ignore[attr-defined]
        except Exception:
            pass

    def compose(self) -> ComposeResult:
        table: DataTable[str] = DataTable(zebra_stripes=False, cursor_type=None)  # type: ignore[arg-type]
        table.add_column("Topic")
        table.add_column("Freq (Hz)")
        table.add_column("Bandwidth")
        table.add_column("Total Traffic")
        yield table

    def on_mount_subapp(self) -> None:
        self.run_worker(self._init_lcm, exclusive=True, thread=True)
        self._start_refresh_timer()

    def on_resume_subapp(self) -> None:
        self._start_refresh_timer()

    def _start_refresh_timer(self) -> None:
        self.set_interval(0.5, self._refresh_table)

    def _init_lcm(self) -> None:
        """Blocking LCM init — runs in a worker thread."""
        try:
            from dimos.utils.cli.lcmspy.lcmspy import GraphLCMSpy

            self._spy = GraphLCMSpy(graph_log_window=0.5)
            self._spy.start()
        except Exception:
            import traceback

            self._debug(traceback.format_exc())

    def on_unmount_subapp(self) -> None:
        if self._spy:
            try:
                self._spy.stop()
            except Exception:
                pass
            self._spy = None

    def reinit_lcm(self) -> None:
        self._debug("reinit_lcm called (autoconf changed network config)")
        # Stop existing spy and start fresh
        if self._spy:
            try:
                self._spy.stop()
            except Exception:
                pass
            self._spy = None
        self.run_worker(self._init_lcm, exclusive=True, thread=True)

    def _refresh_table(self) -> None:
        if not self._spy:
            return

        from dimos.utils.cli.lcmspy.run_lcmspy import gradient, topic_text

        try:
            table = self.query_one(DataTable)
        except Exception:
            return
        topics = list(self._spy.topic.values())
        topics.sort(key=lambda t: t.total_traffic(), reverse=True)
        table.clear(columns=False)

        for t in topics:
            freq = t.freq(5.0)
            kbps = t.kbps(5.0)
            table.add_row(
                topic_text(t.name),
                Text(f"{freq:.1f}", style=gradient(10, freq)),
                Text(t.kbps_hr(5.0), style=gradient(1024 * 3, kbps)),
                Text(t.total_traffic_hr()),
            )
