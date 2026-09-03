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

from typing import Any

from rich.pretty import Pretty
from rich.text import Text
from textual import events
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import VerticalScroll
from textual.screen import ModalScreen
from textual.widgets import DataTable, Footer, Static

from dimos.cloud.data import CloudData


def _topics(u: dict[str, Any]) -> list[str]:
    return [s.get("name", "?") for s in (u.get("manifest") or {}).get("streams") or []]


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
    ]

    FIXED = {"id": 12, "uploaded": 16, "kind": 9, "size": 9, "state": 9}
    FLEX = {"file": 3, "uploader": 2, "blueprint": 2, "robot": 2, "topics": 3}

    def __init__(self, cloud: CloudData | None = None) -> None:
        super().__init__()
        self._cloud = cloud or CloudData()
        self._rows: list[dict[str, Any]] = []
        self._expanded: set[str] = set()

    def compose(self) -> ComposeResult:
        yield DataTable[Text](cursor_type="row", zebra_stripes=True)
        yield Static(id="quota")
        yield Footer()

    def on_mount(self) -> None:
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
            table.add_column(name, width=widths[name])
        from rich.filesize import decimal

        for u in self._rows:
            topics = _topics(u)
            state = u["state"]
            table.add_row(
                Text(u["id"][:12], style="cyan"),
                Text(u["filename"], style="bold"),
                Text(str(u.get("created_at") or "")[:16].replace("T", " ") or "—", style="dim"),
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
        self.notify(f"pulling {row['filename']}…")
        self.run_worker(lambda: self._pull(row), thread=True)

    def _pull(self, row: dict[str, Any]) -> None:
        try:
            out = self._cloud.pull(str(row["id"]))
            self.call_from_thread(self.notify, f"pulled to {out}")
        except (RuntimeError, OSError) as e:
            self.call_from_thread(self.notify, str(e), severity="error")
