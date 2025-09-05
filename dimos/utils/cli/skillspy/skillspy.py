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

import logging
import threading
import time
from typing import Callable, Dict, Optional

from rich.text import Text
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Vertical
from textual.reactive import reactive
from textual.widgets import DataTable, Footer, RichLog

from dimos.protocol.skill.comms import SkillMsg
from dimos.protocol.skill.coordinator import SkillCoordinator, SkillState, SkillStateEnum
from dimos.protocol.skill.type import MsgType


class AgentSpy:
    """Spy on agent skill executions via LCM messages."""

    def __init__(self):
        self.agent_interface = SkillCoordinator()
        self.message_callbacks: list[Callable[[Dict[str, SkillState]], None]] = []
        self._lock = threading.Lock()
        self._latest_state: Dict[str, SkillState] = {}

    def start(self):
        """Start spying on agent messages."""
        # Start the agent interface
        self.agent_interface.start()

        # Subscribe to the agent interface's comms
        self.agent_interface.skill_transport.subscribe(self._handle_message)

    def stop(self):
        """Stop spying."""
        self.agent_interface.stop()

    def _handle_message(self, msg: SkillMsg):
        """Handle incoming skill messages."""

        # Small delay to ensure agent_interface has processed the message
        def delayed_update():
            time.sleep(0.1)
            with self._lock:
                self._latest_state = self.agent_interface.generate_snapshot(clear=False)
                for callback in self.message_callbacks:
                    callback(self._latest_state)

        # Run in separate thread to not block LCM
        threading.Thread(target=delayed_update, daemon=True).start()

    def subscribe(self, callback: Callable[[Dict[str, SkillState]], None]):
        """Subscribe to state updates."""
        self.message_callbacks.append(callback)

    def get_state(self) -> Dict[str, SkillState]:
        """Get current state snapshot."""
        with self._lock:
            return self._latest_state.copy()


def state_color(state: SkillStateEnum) -> str:
    """Get color for skill state."""
    if state == SkillStateEnum.pending:
        return "yellow"
    elif state == SkillStateEnum.running:
        return "green"
    elif state == SkillStateEnum.completed:
        return "cyan"
    elif state == SkillStateEnum.error:
        return "red"
    return "white"


def format_duration(duration: float) -> str:
    """Format duration in human readable format."""
    if duration < 1:
        return f"{duration * 1000:.0f}ms"
    elif duration < 60:
        return f"{duration:.1f}s"
    elif duration < 3600:
        return f"{duration / 60:.1f}m"
    else:
        return f"{duration / 3600:.1f}h"


class AgentSpyLogFilter(logging.Filter):
    """Filter to suppress specific log messages in agentspy."""

    def filter(self, record):
        # Suppress the "Skill state not found" warning as it's expected in agentspy
        if (
            record.levelname == "WARNING"
            and "Skill state for" in record.getMessage()
            and "not found" in record.getMessage()
        ):
            return False
        return True


class TextualLogHandler(logging.Handler):
    """Custom log handler that sends logs to a Textual RichLog widget."""

    def __init__(self, log_widget: RichLog):
        super().__init__()
        self.log_widget = log_widget
        # Add filter to suppress expected warnings
        self.addFilter(AgentSpyLogFilter())

    def emit(self, record):
        """Emit a log record to the RichLog widget."""
        try:
            msg = self.format(record)
            # Color based on level
            if record.levelno >= logging.ERROR:
                style = "bold red"
            elif record.levelno >= logging.WARNING:
                style = "yellow"
            elif record.levelno >= logging.INFO:
                style = "green"
            else:
                style = "dim"

            self.log_widget.write(Text(msg, style=style))
        except Exception:
            self.handleError(record)


class AgentSpyApp(App):
    """A real-time CLI dashboard for agent skill monitoring using Textual."""

    CSS = """
    Screen {
        layout: vertical;
    }
    Vertical {
        height: 100%;
    }
    DataTable {
        height: 70%;
        border: none;
        background: black;
    }
    RichLog {
        height: 30%;
        border: none;
        background: black;
        border-top: solid $primary;
    }
    """

    BINDINGS = [
        Binding("q", "quit", "Quit"),
        Binding("c", "clear", "Clear History"),
        Binding("l", "toggle_logs", "Toggle Logs"),
        Binding("ctrl+c", "quit", "Quit", show=False),
    ]

    show_logs = reactive(True)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.spy = AgentSpy()
        self.table: Optional[DataTable] = None
        self.log_view: Optional[RichLog] = None
        self.skill_history: list[tuple[str, SkillState, float]] = []  # (call_id, state, start_time)
        self.log_handler: Optional[TextualLogHandler] = None

    def compose(self) -> ComposeResult:
        self.table = DataTable(zebra_stripes=False, cursor_type=None)
        self.table.add_column("Call ID")
        self.table.add_column("Skill Name")
        self.table.add_column("State")
        self.table.add_column("Duration")
        self.table.add_column("Messages")
        self.table.add_column("Details")

        self.log_view = RichLog(markup=True, wrap=True)

        with Vertical():
            yield self.table
            yield self.log_view

        yield Footer()

    def on_mount(self):
        """Start the spy when app mounts."""
        self.theme = "flexoki"

        # Remove ALL existing handlers from ALL loggers to prevent console output
        # This is needed because setup_logger creates loggers with propagate=False
        for name in logging.root.manager.loggerDict:
            logger = logging.getLogger(name)
            logger.handlers.clear()
            logger.propagate = True

        # Clear root logger handlers too
        logging.root.handlers.clear()

        # Set up custom log handler to show logs in the UI
        if self.log_view:
            self.log_handler = TextualLogHandler(self.log_view)

            # Custom formatter that shortens the logger name and highlights call_ids
            class ShortNameFormatter(logging.Formatter):
                def format(self, record):
                    # Remove the common prefix from logger names
                    if record.name.startswith("dimos.protocol.skill."):
                        record.name = record.name.replace("dimos.protocol.skill.", "")

                    # Highlight call_ids in the message
                    msg = record.getMessage()
                    if "call_id=" in msg:
                        # Extract and colorize call_id
                        import re

                        msg = re.sub(r"call_id=([^\s\)]+)", r"call_id=\033[94m\1\033[0m", msg)
                        record.msg = msg
                        record.args = ()

                    return super().format(record)

            self.log_handler.setFormatter(
                ShortNameFormatter(
                    "%(asctime)s - %(name)s - %(levelname)s - %(message)s", datefmt="%H:%M:%S"
                )
            )
            # Add handler to root logger
            root_logger = logging.getLogger()
            root_logger.addHandler(self.log_handler)
            root_logger.setLevel(logging.INFO)

        # Set initial visibility
        if not self.show_logs:
            self.log_view.visible = False
            self.table.styles.height = "100%"

        self.spy.subscribe(self.update_state)
        self.spy.start()

        # Also set up periodic refresh to update durations
        self.set_interval(1.0, self.refresh_table)

    def on_unmount(self):
        """Stop the spy when app unmounts."""
        self.spy.stop()
        # Remove log handler to prevent errors on shutdown
        if self.log_handler:
            root_logger = logging.getLogger()
            root_logger.removeHandler(self.log_handler)

    def update_state(self, state: Dict[str, SkillState]):
        """Update state from spy callback. State dict is keyed by call_id."""
        # Update history with current state
        current_time = time.time()

        # Add new skills or update existing ones
        for call_id, skill_state in state.items():
            # Find if this call_id already in history
            found = False
            for i, (existing_call_id, old_state, start_time) in enumerate(self.skill_history):
                if existing_call_id == call_id:
                    # Update existing entry
                    self.skill_history[i] = (call_id, skill_state, start_time)
                    found = True
                    break

            if not found:
                # Add new entry with current time as start
                start_time = current_time
                if skill_state.start_msg:
                    # Use start message timestamp if available
                    start_time = skill_state.start_msg.ts
                self.skill_history.append((call_id, skill_state, start_time))

        # Schedule UI update
        self.call_from_thread(self.refresh_table)

    def refresh_table(self):
        """Refresh the table display."""
        if not self.table:
            return

        # Clear table
        self.table.clear(columns=False)

        # Sort by start time (newest first)
        sorted_history = sorted(self.skill_history, key=lambda x: x[2], reverse=True)

        # Get terminal height and calculate how many rows we can show
        height = self.size.height - 6  # Account for header, footer, column headers
        max_rows = max(1, height)

        # Show only top N entries
        for call_id, skill_state, start_time in sorted_history[:max_rows]:
            # Calculate how long ago it started (for progress indicator)
            time_ago = time.time() - start_time

            # Duration
            duration_str = format_duration(skill_state.duration())

            # Message count
            msg_count = len(skill_state)

            # Details based on state and last message
            details = ""
            if skill_state.state == SkillStateEnum.error and skill_state.error_msg:
                # Show error message
                error_content = skill_state.error_msg.content
                if isinstance(error_content, dict):
                    details = error_content.get("msg", "Error")[:40]
                else:
                    details = str(error_content)[:40]
            elif skill_state.state == SkillStateEnum.completed and skill_state.ret_msg:
                # Show return value
                details = f"→ {str(skill_state.ret_msg.content)[:37]}"
            elif skill_state.state == SkillStateEnum.running:
                # Show progress indicator
                details = "⋯ " + "▸" * min(int(time_ago), 20)

            # Format call_id for display (truncate if too long)
            display_call_id = call_id
            if len(call_id) > 16:
                display_call_id = call_id[:13] + "..."

            # Add row with colored state
            self.table.add_row(
                Text(display_call_id, style="bright_blue"),
                Text(skill_state.name, style="white"),
                Text(skill_state.state.name, style=state_color(skill_state.state)),
                Text(duration_str, style="dim"),
                Text(str(msg_count), style="dim"),
                Text(details, style="dim white"),
            )

    def action_clear(self):
        """Clear the skill history."""
        self.skill_history.clear()
        self.refresh_table()

    def action_toggle_logs(self):
        """Toggle the log view visibility."""
        self.show_logs = not self.show_logs
        if self.show_logs:
            self.table.styles.height = "70%"
        else:
            self.table.styles.height = "100%"
        self.log_view.visible = self.show_logs


def main():
    """Main entry point for agentspy CLI."""
    import sys

    # Check if running in web mode
    if len(sys.argv) > 1 and sys.argv[1] == "web":
        import os

        from textual_serve.server import Server

        server = Server(f"python {os.path.abspath(__file__)}")
        server.serve()
    else:
        app = AgentSpyApp()
        app.run()


if __name__ == "__main__":
    main()
