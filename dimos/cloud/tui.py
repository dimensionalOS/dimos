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

"""Interactive browser for `dimos data ls -i`.

Arrow keys move, enter/space expands the topic list of the selected row,
`d` opens the full record, `p` pulls, `r` refreshes. Columns share the
terminal width by weight and re-flow on resize; wide values wrap inside
their column (auto-height rows) instead of truncating."""

from __future__ import annotations

import threading
import time
from typing import Any, Literal

from rich.pretty import Pretty
from rich.text import Text
from textual import events
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal, VerticalScroll
from textual.screen import ModalScreen
from textual.widgets import DataTable, Footer, Label, ProgressBar, Static

from dimos.cloud.cli import local_time, tz_label
from dimos.cloud.data import CloudData


def _topics(u: dict[str, Any]) -> list[str]:
    return [s.get("name", "?") for s in (u.get("manifest") or {}).get("streams") or []]


class _PullCancelledError(Exception):
    """Raised inside the progress callback to abort a pull the user cancelled."""


class DetailScreen(ModalScreen[None]):
    CSS = """
    DetailScreen { align: center middle; }
    #detail {
        width: 80%;
        height: 80%;
        border: heavy $accent;
        background: $surface;
        padding: 1 2;
    }
    """
    BINDINGS = [Binding("escape,q,enter", "dismiss", "close")]

    def __init__(self, row: dict[str, Any]) -> None:
        super().__init__()
        self._row = row

    def compose(self) -> ComposeResult:
        with VerticalScroll(id="detail"):
            yield Static(Pretty(self._row, expand_all=True))


class DataBrowser(App[None]):
    TITLE = "dimos data"
    BINDINGS = [
        Binding("q", "quit", "quit"),
        Binding("r", "refresh", "refresh"),
        Binding("space", "toggle_topics", "topics", key_display="enter"),
        Binding("d", "detail", "detail"),
        Binding("p", "pull", "pull"),
        Binding("x", "cancel_pull", "cancel pull"),
    ]

    FIXED = {"id": 12, "uploaded": 16, "kind": 9, "size": 9, "state": 9}
    FLEX = {"file": 3, "uploader": 2, "blueprint": 2, "robot": 2, "topics": 3}

    def __init__(self, cloud: CloudData | None = None) -> None:
        super().__init__()
        self._cloud = cloud or CloudData()
        self._rows: list[dict[str, Any]] = []
        self._expanded: set[str] = set()
        self._pull_cancel: threading.Event | None = None
        self._rate: tuple[float, int] | None = None  # (monotonic, done) of last tick
        self._speed = 0.0  # EMA bytes/s

    CSS = """
    #pull-status { height: 1; padding: 0 1; background: $boost; }
    #pull-status Label { margin-right: 2; }
    #pull-bar { width: 1fr; }
    #pull-bar Bar { width: 1fr; }
    """

    def compose(self) -> ComposeResult:
        yield DataTable[Text](cursor_type="row", zebra_stripes=True)
        with Horizontal(id="pull-status"):
            yield Label(id="pull-label")
            yield ProgressBar(id="pull-bar")
        yield Static(id="quota")
        yield Footer()

    def on_mount(self) -> None:
        self.query_one("#pull-status").display = False
        self.action_refresh()

    def on_resize(self, _: events.Resize) -> None:
        self._render()

    def _selected(self) -> dict[str, Any] | None:
        table = self.query_one(DataTable)
        if not self._rows or table.cursor_row is None:
            return None
        return self._rows[table.cursor_row]

    def action_refresh(self) -> None:
        self.run_worker(self._load, thread=True, exclusive=True)

    def _load(self) -> None:
        rows = self._cloud.ls()
        quota = self._cloud.quota()
        self.call_from_thread(self._set_data, rows, quota)

    def _set_data(self, rows: list[dict[str, Any]], quota: dict[str, Any]) -> None:
        self._rows = rows
        limits = quota.get("limits") or {}
        self.query_one("#quota", Static).update(
            f" {quota.get('pct', '?')}% of {limits.get('total_gb', '?')} GB used"
            f" ({quota.get('state', '?')})"
        )
        self._render()

    def _widths(self) -> dict[str, int]:
        """Fixed columns keep their width; the rest split the remaining terminal
        width by weight. Cells wrap rather than truncate, so a too-narrow column
        costs height, not data."""
        pad = 2 * (len(self.FIXED) + len(self.FLEX)) + 2  # cell padding + scrollbar
        spare = max(0, self.size.width - sum(self.FIXED.values()) - pad)
        total = sum(self.FLEX.values())
        flex = {k: max(8, spare * w // total) for k, w in self.FLEX.items()}
        return {**self.FIXED, **flex}

    def _render(self) -> None:
        table = self.query_one(DataTable)
        selected = self._selected()
        table.clear(columns=True)
        widths = self._widths()
        order = ["id", "file", "uploaded", "kind", "uploader", "blueprint", "robot"]
        order += ["topics", "size", "state"]
        for name in order:
            label = f"uploaded ({tz_label()})" if name == "uploaded" else name
            table.add_column(label, width=widths[name])
        from rich.filesize import decimal

        for u in self._rows:
            topics = _topics(u)
            state = u["state"]
            table.add_row(
                Text(u["id"][:12], style="cyan"),
                Text(u["filename"], style="bold"),
                Text(local_time(str(u.get("created_at") or "")) or "—", style="dim"),
                Text(u.get("kind", "")),
                Text(u.get("uploader_email") or "—", style="dim"),
                Text((u.get("manifest") or {}).get("blueprint") or "—", style="magenta"),
                Text(u.get("robot_id") or "—", style="magenta"),
                Text("\n".join(topics))
                if topics and u["id"] in self._expanded
                else Text(f"{len(topics)} topic" + "s" * (len(topics) != 1), style="dim"),
                Text(decimal(u["size"]), justify="right"),
                Text(state, style="green" if state == "complete" else "yellow"),
                height=None,
            )
        # Measure auto-height rows now instead of on idle: the deferred pass
        # repaints via the virtual_size reactive, which skips when total height
        # lands unchanged (0/1-topic toggles) and leaves the post-clear blank
        # frame on screen; it also lets the cursor restore scroll against
        # zero-height rows.
        table._require_update_dimensions = False
        table._new_rows.clear()
        table._update_dimensions(set(table.rows.keys()))
        if selected:
            for i, u in enumerate(self._rows):
                if u["id"] == selected["id"]:
                    table.move_cursor(row=i)
                    break
        table.refresh()

    def on_data_table_row_selected(self, _: DataTable.RowSelected) -> None:
        self.action_toggle_topics()

    def action_toggle_topics(self) -> None:
        row = self._selected()
        if not row or not _topics(row):  # nothing to expand: don't rebuild at all
            return
        self._expanded ^= {row["id"]}
        self._render()

    def action_detail(self) -> None:
        if row := self._selected():
            self.push_screen(DetailScreen(row))

    def action_pull(self) -> None:
        row = self._selected()
        if not row:
            return
        if row["state"] != "complete":
            self.notify(f"{row['filename']} is {row['state']} — not pullable", severity="warning")
            return
        if self._pull_cancel is not None:
            self.notify("a pull is already running — x cancels it", severity="warning")
            return
        cancel = threading.Event()
        self._pull_cancel = cancel
        self.run_worker(lambda: self._pull(row, cancel), thread=True)

    def action_cancel_pull(self) -> None:
        if self._pull_cancel is not None:
            self._pull_cancel.set()

    def _pull(self, row: dict[str, Any], cancel: threading.Event) -> None:
        last = 0.0

        def tick(phase: str, done: int, total: int) -> None:
            nonlocal last
            if cancel.is_set():
                raise _PullCancelledError
            if phase == "download" and done != total and time.monotonic() - last < 0.1:
                return  # a 40GB pull ticks per MB; don't flood the UI thread
            last = time.monotonic()
            self.call_from_thread(self._pull_progress, row["filename"], phase, done, total)

        try:
            out = self._cloud.pull(str(row["id"]), progress=tick)
            self.call_from_thread(self._pull_finished, f"pulled to {out}", "information")
        except _PullCancelledError:
            self.call_from_thread(
                self._pull_finished, f"pull of {row['filename']} cancelled", "warning"
            )
        except (RuntimeError, OSError) as e:
            self.call_from_thread(self._pull_finished, str(e), "error")
        finally:
            self._pull_cancel = None

    def _pull_progress(self, filename: str, phase: str, done: int, total: int) -> None:
        from rich.filesize import decimal

        now = time.monotonic()
        if phase == "download" and self._rate is not None:
            dt, db = now - self._rate[0], done - self._rate[1]
            if dt > 0 and db >= 0:
                inst = db / dt
                self._speed = inst if not self._speed else 0.8 * self._speed + 0.2 * inst
        self._rate = (now, done)
        self.query_one("#pull-status").display = True
        if phase == "download":
            of = f"  {decimal(done)} / {decimal(total)}" if total else ""
            speed = f"  [cyan]↓ {decimal(int(self._speed))}/s[/]" if self._speed else ""
            label = f"[b]{filename}[/b]{of}{speed}  [dim](x cancels)[/]"
        else:
            label = f"[b]{filename}[/b]  {phase}…"
        self.query_one("#pull-label", Label).update(label)
        self.query_one("#pull-bar", ProgressBar).update(total=total or None, progress=done)

    def _pull_finished(
        self, message: str, severity: Literal["information", "warning", "error"]
    ) -> None:
        self.query_one("#pull-status").display = False
        self._rate, self._speed = None, 0.0
        self.notify(message, severity=severity)
