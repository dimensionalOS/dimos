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

import threading
from typing import List

from rich.text import Text
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Container, Horizontal, Vertical
from textual.coordinate import Coordinate
from textual.reactive import reactive
from textual.widgets import DataTable, Footer, Header, Label, Static

from dimos.utils.cli.recorder.recorder import RecorderService, TopicInfo


def gradient(max_value: float, value: float) -> str:
    """Get gradient color based on value."""
    from textual.color import Color

    ratio = min(value / max_value, 1.0)
    green = Color(0, 255, 0)
    red = Color(255, 0, 0)
    color = green.blend(red, ratio)
    return color.hex


def topic_text(topic_name: str) -> Text:
    """Format topic name with colors."""
    if "#" in topic_name:
        parts = topic_name.split("#", 1)
        return Text(parts[0], style="white") + Text("#" + parts[1], style="blue")

    if topic_name[:4] == "/rpc":
        return Text(topic_name[:4], style="red") + Text(topic_name[4:], style="white")

    return Text(topic_name, style="white")


def format_duration(duration: float) -> str:
    """Format duration in human readable format."""
    hours = int(duration // 3600)
    minutes = int((duration % 3600) // 60)
    seconds = int(duration % 60)

    if hours > 0:
        return f"{hours:02d}:{minutes:02d}:{seconds:02d}"
    else:
        return f"{minutes:02d}:{seconds:02d}"


class LCMRecorderApp(App):
    """Interactive LCM topic recorder using Textual."""

    CSS = """
    Screen {
        layout: vertical;
    }
    
    #status-bar {
        height: 3;
        background: $surface;
        border: solid $primary;
        padding: 0 1;
    }
    
    #status-text {
        padding: 0;
        margin: 0;
    }
    
    DataTable {
        height: 1fr;
        width: 1fr;
        border: none;
        background: black;
    }
    
    Footer {
        dock: bottom;
    }
    """

    refresh_interval: float = 0.5
    show_command_palette = reactive(True)

    BINDINGS = [
        Binding("q", "quit", "Quit"),
        Binding("ctrl+c", "quit", "Quit"),
        Binding("r", "toggle_record", "Record", priority=True),
        Binding("space", "toggle_selection", "Select", priority=True),
        Binding("a", "select_all", "Select All", priority=True),
        Binding("n", "select_none", "Select None", priority=True),
        Binding("up", "cursor_up", "Up", show=False),
        Binding("down", "cursor_down", "Down", show=False),
    ]

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.recorder = RecorderService(autoconf=True)
        self.table: DataTable | None = None
        self.status_label: Label | None = None
        self._recording_path: str = ""

    def compose(self) -> ComposeResult:
        # Status bar
        with Container(id="status-bar"):
            self.status_label = Label("Status: Idle", id="status-text")
            yield self.status_label

        # Topic table
        self.table = DataTable(zebra_stripes=False, cursor_type="row")
        self.table.add_column("", width=3)  # No header for selected column
        self.table.add_column("Topic")
        self.table.add_column("Freq (Hz)", width=12)
        self.table.add_column("Bandwidth", width=15)
        self.table.add_column("Total Traffic", width=15)

        yield self.table
        yield Footer()

    def on_mount(self):
        """Initialize the app when mounted."""
        self.theme = "flexoki"
        self.recorder.start()
        self.set_interval(self.refresh_interval, self.refresh_display)

    async def on_unmount(self):
        """Clean up when unmounting."""
        self.recorder.stop()

    def refresh_display(self):
        """Refresh the table and status display."""
        self.refresh_table()
        self.refresh_status()

    def refresh_table(self):
        """Update the topic table."""
        topics: List[TopicInfo] = list(self.recorder.topics.values())
        topics.sort(key=lambda t: t.total_traffic(), reverse=True)

        # Remember current cursor row index
        current_row = None
        if self.table.cursor_coordinate:
            current_row = self.table.cursor_coordinate.row

        self.table.clear(columns=False)

        for t in topics:
            freq = t.freq(5.0)
            kbps = t.kbps(5.0)
            bw_val, bw_unit = t.kbps_hr(5.0)
            total_val, total_unit = t.total_traffic_hr()

            # Selection indicator
            selected = Text("✓", style="green bold") if t.selected else Text(" ")

            self.table.add_row(
                selected,
                topic_text(t.name),
                Text(f"{freq:.1f}", style=gradient(10, freq)),
                Text(f"{bw_val} {bw_unit.value}/s", style=gradient(1024 * 3, kbps)),
                Text(f"{total_val} {total_unit.value}"),
                key=t.name,  # Use topic name as row key for cursor tracking
            )

        # Restore cursor position if possible
        if current_row is not None and current_row < len(topics):
            self.table.move_cursor(row=current_row, column=0)

    def refresh_status(self):
        """Update the status display."""
        if self.recorder.recording:
            duration = self.recorder.get_recording_duration()
            selected_count = len(self.recorder.get_selected_topics())
            status = f"Status: RECORDING ({selected_count} topics) - Duration: {format_duration(duration)}"
            self.status_label.update(Text(status, style="red bold"))

            # Show recording path notification
            if self._recording_path != str(self.recorder.recording_dir):
                self._recording_path = str(self.recorder.recording_dir)
                self.notify(f"Recording to: {self._recording_path}", severity="information")
        else:
            selected_count = len(self.recorder.get_selected_topics())
            total_count = len(self.recorder.topics)
            status = f"Status: Idle - {selected_count}/{total_count} topics selected"
            self.status_label.update(Text(status, style="green"))

    def action_toggle_selection(self):
        """Toggle selection of current topic."""
        cursor_coord = self.table.cursor_coordinate
        if cursor_coord is not None:
            # Get the row key which is the topic name using coordinate_to_cell_key
            try:
                row_key, _ = self.table.coordinate_to_cell_key(cursor_coord)
                if row_key is not None:
                    # The row_key is a RowKey object, get its value
                    topic_name = row_key.value if hasattr(row_key, "value") else str(row_key)
                    self.recorder.toggle_topic_selection(topic_name)
                    self.refresh_display()
            except Exception as e:
                self.notify(f"Error: {e}", severity="error")

    def action_select_all(self):
        """Select all topics."""
        self.recorder.select_all_topics(True)
        self.refresh_display()

    def action_select_none(self):
        """Deselect all topics."""
        self.recorder.select_all_topics(False)
        self.refresh_display()

    def action_toggle_record(self):
        """Toggle recording state."""
        if self.recorder.recording:
            # Stop recording
            recording_dir = self.recorder.stop_recording()
            if recording_dir:
                self.notify(
                    f"Recording saved to: {recording_dir}", severity="information", timeout=5.0
                )
        else:
            # Start recording
            if self.recorder.start_recording():
                self.refresh_display()
            else:
                self.notify("No topics selected for recording!", severity="error")

    def action_cursor_up(self):
        """Move cursor up."""
        cursor_coord = self.table.cursor_coordinate
        if cursor_coord is not None and cursor_coord.row > 0:
            self.table.move_cursor(row=cursor_coord.row - 1)

    def action_cursor_down(self):
        """Move cursor down."""
        cursor_coord = self.table.cursor_coordinate
        if cursor_coord is not None:
            max_row = len(self.recorder.topics) - 1
            if cursor_coord.row < max_row:
                self.table.move_cursor(row=cursor_coord.row + 1)


def main():
    """Entry point for the LCM recorder."""
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == "web":
        import os
        from textual_serve.server import Server

        server = Server(f"python {os.path.abspath(__file__)}")
        server.serve()
    else:
        app = LCMRecorderApp()
        app.run()


if __name__ == "__main__":
    main()
