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

from datetime import datetime
import textwrap
import threading
from typing import TYPE_CHECKING

from langchain_core.messages import AIMessage, HumanMessage, SystemMessage, ToolCall, ToolMessage
from rich.highlighter import JSONHighlighter
from rich.theme import Theme
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Container
from textual.widgets import Input, RichLog

from dimos.core import pLCMTransport
from dimos.utils.cli import theme
from dimos.utils.generic import truncate_display_string

if TYPE_CHECKING:
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
        self.human_transport = pLCMTransport("/human_input")  # type: ignore[var-annotated]
        self.agent_transport = pLCMTransport("/agent")  # type: ignore[var-annotated]
        self.chat_log: RichLog | None = None
        self.input_widget: Input | None = None
        self._subscription_thread: threading.Thread | None = None
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

        # Start subscription thread
        self._subscription_thread = threading.Thread(target=self._subscribe_to_agent, daemon=True)
        self._subscription_thread.start()

        # Focus on input
        self.input_widget.focus()  # type: ignore[union-attr]

        self.chat_log.write(f"[{theme.ACCENT}]{theme.ascii_logo}[/{theme.ACCENT}]")  # type: ignore[union-attr]

        # Welcome message
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
                tool_calls = msg.additional_kwargs.get("tool_calls", [])

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

        self.agent_transport.subscribe(receive_msg)

    def _format_tool_call(self, tool_call: ToolCall) -> str:
        """Format a tool call for display."""
        f = tool_call.get("function", {})
        name = f.get("name", "unknown")  # type: ignore[attr-defined]
        return f"▶ {name}({f.get('arguments', '')})"  # type: ignore[attr-defined]

    def _add_message(self, timestamp: str, sender: str, content: str, color: str) -> None:
        """Add a message to the chat log."""
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
        self.human_transport.publish(message)

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
