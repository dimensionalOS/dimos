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

from __future__ import annotations

from datetime import datetime
import json
import textwrap
import threading
from typing import TYPE_CHECKING, Any

from langchain_core.messages import AIMessage, HumanMessage, SystemMessage, ToolCall, ToolMessage
from rich.highlighter import JSONHighlighter
from rich.theme import Theme
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Container
from textual.geometry import Size
from textual.widgets import Input, RichLog

from dimos.core import pLCMTransport
from dimos.utils.cli import theme
from dimos.utils.generic import truncate_display_string

if TYPE_CHECKING:
    from collections.abc import Callable

    from textual.events import Key

# Custom theme for JSON highlighting
JSON_THEME = Theme(
    {
        "json.key": theme.CYAN,
        "json.str": theme.ACCENT,
        "json.number": theme.ACCENT,
        "json.bool_true": theme.ACCENT,
        "json.bool_false": theme.ACCENT,
        "json.null": theme.DIM,
        "json.brace": theme.BRIGHT_WHITE,
    }
)


class ThinkingIndicator:
    """Manages a throbbing 'thinking...' chat message in a RichLog."""

    def __init__(
        self,
        app: App[Any],
        chat_log: RichLog,
        add_message_fn: Callable[[str, str, str, str], None],
    ) -> None:
        self._app: App[Any] = app
        self._chat_log = chat_log
        self._add_message = add_message_fn
        self._timer: Any = None
        self._strips: list[Any] = []
        self.visible = False
        self._throb_dim = False

    def show(self) -> None:
        if self.visible:
            return
        self.visible = True
        self._throb_dim = False
        self._write_line()
        self._timer = self._app.set_interval(0.6, self._toggle_throb)

    def hide(self) -> None:
        if not self.visible:
            return
        self.visible = False
        if self._timer is not None:
            self._timer.stop()
            self._timer = None
        self._remove_lines()

    def detach_if_needed(self) -> bool:
        if self.visible and self._strips:
            self._remove_lines()
            return True
        return False

    def reattach(self) -> None:
        self._write_line()

    def _write_line(self) -> None:
        before_count = len(self._chat_log.lines)
        color = theme.DIM if self._throb_dim else theme.ACCENT
        timestamp = datetime.now().strftime("%H:%M:%S")
        self._add_message(timestamp, "", "[italic]thinking...[/italic]", color)
        self._strips = list(self._chat_log.lines[before_count:])

    def _remove_lines(self) -> None:
        if not self._strips:
            return
        strip_ids = {id(s) for s in self._strips}
        self._chat_log.lines = [line for line in self._chat_log.lines if id(line) not in strip_ids]
        self._strips = []
        self._chat_log._line_cache.clear()
        self._chat_log.virtual_size = Size(
            self._chat_log.virtual_size.width, len(self._chat_log.lines)
        )
        self._chat_log.refresh()

    def _toggle_throb(self) -> None:
        if not self.visible:
            return
        self._remove_lines()
        self._throb_dim = not self._throb_dim
        self._write_line()


class HumanCLIApp(App):  # type: ignore[type-arg]
    """IRC-like interface for interacting with DimOS agents."""

    CSS_PATH = theme.CSS_PATH

    CSS = f"""
    Screen {{
        background: {theme.BACKGROUND};
    }}

    #chat-container {{
        height: 1fr;
    }}

    RichLog {{
        scrollbar-size: 0 0;
    }}

    Input {{
        dock: bottom;
    }}

    """

    BINDINGS = [
        Binding("q", "quit", "Quit", show=False),
        Binding("ctrl+c", "quit", "Quit"),
        Binding("ctrl+l", "clear", "Clear chat"),
    ]

    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(*args, **kwargs)
        self._human_transport = pLCMTransport("/human_input")  # type: ignore[var-annotated]
        self._agent_transport = pLCMTransport("/agent")  # type: ignore[var-annotated]
        self._agent_idle = pLCMTransport("/agent_idle")  # type: ignore[var-annotated]
        self.chat_log: RichLog | None = None
        self.input_widget: Input | None = None
        self._subscription_thread: threading.Thread | None = None
        self._idle_subscription_thread: threading.Thread | None = None
        self._thinking: ThinkingIndicator | None = None
        self._running = False

    def compose(self) -> ComposeResult:
        """Compose the IRC-like interface."""
        with Container(id="chat-container"):
            self.chat_log = RichLog(highlight=True, markup=True, wrap=False)
            yield self.chat_log

        self.input_widget = Input(placeholder="Type a message...")
        yield self.input_widget

    def on_mount(self) -> None:
        """Initialize the app when mounted."""
        self._running = True

        # Apply custom JSON theme to app console
        self.console.push_theme(JSON_THEME)

        # Set custom highlighter for RichLog
        self.chat_log.highlighter = JSONHighlighter()  # type: ignore[union-attr]

        assert self.chat_log is not None
        self._thinking = ThinkingIndicator(self, self.chat_log, self._add_message)

        # Start subscription threads
        self._subscription_thread = threading.Thread(target=self._subscribe_to_agent, daemon=True)
        self._subscription_thread.start()
        self._idle_subscription_thread = threading.Thread(
            target=self._subscribe_to_idle, daemon=True
        )
        self._idle_subscription_thread.start()

        # Focus on input
        self.input_widget.focus()  # type: ignore[union-attr]

        self.chat_log.write(f"[{theme.ACCENT}]{theme.ascii_logo}[/{theme.ACCENT}]")  # type: ignore[union-attr]

        self._add_system_message("Connected to DimOS Agent Interface")

    def on_unmount(self) -> None:
        """Clean up when unmounting."""
        self._running = False

    def _subscribe_to_agent(self) -> None:
        """Subscribe to agent messages in a separate thread."""

        def receive_msg(msg) -> None:  # type: ignore[no-untyped-def]
            if not self._running:
                return

            timestamp = datetime.now().strftime("%H:%M:%S")

            if isinstance(msg, SystemMessage):
                self.call_from_thread(
                    self._add_message,
                    timestamp,
                    "system",
                    truncate_display_string(msg.content, 1000),
                    theme.YELLOW,
                )
            elif isinstance(msg, AIMessage):
                content = msg.content or ""
                tool_calls = getattr(msg, "tool_calls", None) or msg.additional_kwargs.get(
                    "tool_calls", []
                )

                # Display the main content first
                if content:
                    self.call_from_thread(
                        self._add_message, timestamp, "agent", content, theme.AGENT
                    )

                # Display tool calls separately with different formatting
                if tool_calls:
                    for tc in tool_calls:
                        tool_info = self._format_tool_call(tc)
                        self.call_from_thread(
                            self._add_message, timestamp, "tool", tool_info, theme.TOOL
                        )

                # If neither content nor tool calls, show a placeholder
                if not content and not tool_calls:
                    self.call_from_thread(
                        self._add_message, timestamp, "agent", "<no response>", theme.DIM
                    )
            elif isinstance(msg, ToolMessage):
                self.call_from_thread(
                    self._add_message, timestamp, "tool", msg.content, theme.TOOL_RESULT
                )
            elif isinstance(msg, HumanMessage):
                self.call_from_thread(
                    self._add_message, timestamp, "human", msg.content, theme.HUMAN
                )

        self._agent_transport.subscribe(receive_msg)

    def _subscribe_to_idle(self) -> None:
        def receive_idle(is_idle: bool) -> None:
            assert self._thinking is not None

            if not self._running:
                return

            self.call_from_thread(self._thinking.hide if is_idle else self._thinking.show)

        self._agent_idle.subscribe(receive_idle)

    def _format_tool_call(self, tool_call: ToolCall) -> str:
        """Format a tool call for display."""
        name = tool_call.get("name", "unknown")
        args = tool_call.get("args", {})
        args_str = json.dumps(args, separators=(",", ":"))
        return f"▶ {name}({args_str})"

    def _add_message(self, timestamp: str, sender: str, content: str, color: str) -> None:
        assert self._thinking is not None
        reattach = self._thinking.detach_if_needed()

        # Strip leading/trailing whitespace from content
        content = content.strip() if content else ""

        # Format timestamp with nicer colors - split into hours, minutes, seconds
        time_parts = timestamp.split(":")
        if len(time_parts) == 3:
            # Format as HH:MM:SS with colored colons
            timestamp_formatted = f" [{theme.TIMESTAMP}]{time_parts[0]}:{time_parts[1]}:{time_parts[2]}[/{theme.TIMESTAMP}]"
        else:
            timestamp_formatted = f" [{theme.TIMESTAMP}]{timestamp}[/{theme.TIMESTAMP}]"

        # Format sender with consistent width
        sender_formatted = f"[{color}]{sender:>8}[/{color}]"

        # Calculate the prefix length for proper indentation
        # space (1) + timestamp (8) + space (1) + sender (8) + space (1) + separator (1) + space (1) = 21
        prefix = f"{timestamp_formatted} {sender_formatted} │ "
        indent = " " * 19  # Spaces to align with the content after the separator

        # Get the width of the chat area (accounting for borders and padding)
        width = self.chat_log.size.width - 4 if self.chat_log.size else 76  # type: ignore[union-attr]

        # Calculate the available width for text (subtract prefix length)
        text_width = max(width - 20, 40)  # Minimum 40 chars for text

        # Split content into lines first (respecting explicit newlines)
        lines = content.split("\n")

        for line_idx, line in enumerate(lines):
            # Wrap each line to fit the available width
            if line_idx == 0:
                # First line includes the full prefix
                wrapped = textwrap.wrap(
                    line, width=text_width, initial_indent="", subsequent_indent=""
                )
                if wrapped:
                    self.chat_log.write(prefix + f"[{color}]{wrapped[0]}[/{color}]")  # type: ignore[union-attr]
                    for wrapped_line in wrapped[1:]:
                        self.chat_log.write(indent + f"│ [{color}]{wrapped_line}[/{color}]")  # type: ignore[union-attr]
                else:
                    # Empty line
                    self.chat_log.write(prefix)  # type: ignore[union-attr]
            else:
                # Subsequent lines from explicit newlines
                wrapped = textwrap.wrap(
                    line, width=text_width, initial_indent="", subsequent_indent=""
                )
                if wrapped:
                    for wrapped_line in wrapped:
                        self.chat_log.write(indent + f"│ [{color}]{wrapped_line}[/{color}]")  # type: ignore[union-attr]
                else:
                    # Empty line
                    self.chat_log.write(indent + "│")  # type: ignore[union-attr]

        if reattach:
            self._thinking.reattach()

    def _add_system_message(self, content: str) -> None:
        """Add a system message to the chat."""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self._add_message(timestamp, "system", content, theme.YELLOW)

    def on_key(self, event: Key) -> None:
        """Handle key events."""
        if event.key == "ctrl+c":
            self.exit()
            event.prevent_default()

    def on_input_submitted(self, event: Input.Submitted) -> None:
        """Handle input submission."""
        message = event.value.strip()
        if not message:
            return

        # Clear input
        self.input_widget.value = ""  # type: ignore[union-attr]

        # Check for commands
        if message.lower() in ["/exit", "/quit"]:
            self.exit()
            return
        elif message.lower() == "/clear":
            self.action_clear()
            return
        elif message.lower() == "/help":
            help_text = """Commands:
  /clear - Clear the chat log
  /help  - Show this help message
  /exit  - Exit the application
  /quit  - Exit the application

Tool calls are displayed in cyan with ▶ prefix"""
            self._add_system_message(help_text)
            return

        # Send to agent (message will be displayed when received back)
        self._human_transport.publish(message)

    def action_clear(self) -> None:
        """Clear the chat log."""
        self.chat_log.clear()  # type: ignore[union-attr]

    def action_quit(self) -> None:  # type: ignore[override]
        """Quit the application."""
        self._running = False
        self.exit()


def main() -> None:
    """Main entry point for the human CLI."""
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == "web":
        # Support for textual-serve web mode
        import os

        from textual_serve.server import Server  # type: ignore[import-not-found]

        server = Server(f"python {os.path.abspath(__file__)}")
        server.serve()
    else:
        app = HumanCLIApp()
        app.run()


if __name__ == "__main__":
    main()
