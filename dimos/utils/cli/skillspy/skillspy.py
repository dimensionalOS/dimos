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
import time
from typing import TYPE_CHECKING

from rich.text import Text
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.widgets import DataTable, Footer

from dimos.protocol.skill.coordinator import SkillCoordinator, SkillState, SkillStateEnum
from dimos.utils.cli import theme

if TYPE_CHECKING:
    from collections.abc import Callable

    from dimos.protocol.skill.comms import SkillMsg  # type: ignore[attr-defined]


class AgentSpy:
    """Spy on agent skill executions via LCM messages."""

    def __init__(self) -> None:
        self.agent_interface = SkillCoordinator()
        self.message_callbacks: list[Callable[[dict[str, SkillState]], None]] = []
        self._lock = threading.Lock()
        self._latest_state: dict[str, SkillState] = {}
        self._running = False

    def start(self) -> None:
        """Start spying on agent messages."""
        self._running = True
        # Start the agent interface
        self.agent_interface.start()

        # Subscribe to the agent interface's comms
        self.agent_interface.skill_transport.subscribe(self._handle_message)

    def stop(self) -> None:
        """Stop spying."""
        self._running = False
        # Give threads a moment to finish processing
        time.sleep(0.2)
        self.agent_interface.stop()

    def _handle_message(self, msg: SkillMsg) -> None:  # type: ignore[type-arg]
        """Handle incoming skill messages."""
        if not self._running:
            return

        # Small delay to ensure agent_interface has processed the message
        def delayed_update() -> None:
            time.sleep(0.1)
            if not self._running:
                return
            with self._lock:
                self._latest_state = self.agent_interface.generate_snapshot(clear=False)
                for callback in self.message_callbacks:
                    callback(self._latest_state)

        # Run in separate thread to not block LCM
        threading.Thread(target=delayed_update, daemon=True).start()

    def subscribe(self, callback: Callable[[dict[str, SkillState]], None]) -> None:
        """Subscribe to state updates."""
        self.message_callbacks.append(callback)

    def get_state(self) -> dict[str, SkillState]:
        """Get current state snapshot."""
        with self._lock:
            return self._latest_state.copy()


def state_color(state: SkillStateEnum) -> str:
    """Get color for skill state."""
    if state == SkillStateEnum.pending:
        return theme.WARNING
    elif state == SkillStateEnum.running:
        return theme.AGENT
    elif state == SkillStateEnum.completed:
        return theme.SUCCESS
    elif state == SkillStateEnum.error:
        return theme.ERROR
    return theme.FOREGROUND


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


class AgentSpyApp(App):  # type: ignore[type-arg]
    """A real-time CLI dashboard for agent skill monitoring using Textual."""

    CSS_PATH = theme.CSS_PATH

    CSS = f"""
    Screen {{
        layout: vertical;
        background: {theme.BACKGROUND};
    }}
    DataTable {{
        height: 100%;
        border: solid $border;
        background: {theme.BACKGROUND};
    }}
    DataTable > .datatable--header {{
        background: transparent;
    }}
    Footer {{
        background: transparent;
    }}
    """

    BINDINGS = [
        Binding("q", "quit", "Quit"),
        Binding("c", "clear", "Clear History"),
        Binding("ctrl+c", "quit", "Quit", show=False),
    ]

    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(*args, **kwargs)
        self.spy = AgentSpy()
        self.table: DataTable | None = None  # type: ignore[type-arg]
        self.skill_history: list[tuple[str, SkillState, float]] = []  # (call_id, state, start_time)

    def compose(self) -> ComposeResult:
        self.table = DataTable(zebra_stripes=False, cursor_type=None)  # type: ignore[arg-type]
        self.table.add_column("Call ID")
        self.table.add_column("Skill Name")
        self.table.add_column("State")
        self.table.add_column("Duration")
        self.table.add_column("Messages")
        self.table.add_column("Details")

        yield self.table
        yield Footer()

    def on_mount(self) -> None:
        """Start the spy when app mounts."""
        self.spy.subscribe(self.update_state)
        self.spy.start()

        # Set up periodic refresh to update durations
        self.set_interval(1.0, self.refresh_table)

    def on_unmount(self) -> None:
        """Stop the spy when app unmounts."""
        self.spy.stop()

    def update_state(self, state: dict[str, SkillState]) -> None:
        """Update state from spy callback. State dict is keyed by call_id."""
        # Update history with current state
        current_time = time.time()

        # Add new skills or update existing ones
        for call_id, skill_state in state.items():
            # Find if this call_id already in history
            found = False
            for i, (existing_call_id, _old_state, start_time) in enumerate(self.skill_history):
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

    def refresh_table(self) -> None:
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
                Text(display_call_id, style=theme.BRIGHT_BLUE),
                Text(skill_state.name, style=theme.YELLOW),
                Text(skill_state.state.name, style=state_color(skill_state.state)),
                Text(duration_str, style=theme.WHITE),
                Text(str(msg_count), style=theme.YELLOW),
                Text(details, style=theme.FOREGROUND),
            )

    def action_clear(self) -> None:
        """Clear the skill history."""
        self.skill_history.clear()
        self.refresh_table()


def main() -> None:
    """Main entry point for agentspy CLI."""
    import sys

    # Check if running in web mode
    if len(sys.argv) > 1 and sys.argv[1] == "web":
        import os

        from textual_serve.server import Server  # type: ignore[import-not-found]

        server = Server(f"python {os.path.abspath(__file__)}")
        server.serve()
    else:
        app = AgentSpyApp()
        app.run()


if __name__ == "__main__":
    main()
