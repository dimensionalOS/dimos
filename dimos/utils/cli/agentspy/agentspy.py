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

from collections import deque
from dataclasses import dataclass
import time
from typing import Any, Union

from langchain_core.messages import (
    AIMessage,
    HumanMessage,
    SystemMessage,
    ToolMessage,
)
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.widgets import Footer, RichLog

from dimos.protocol.pubsub.lcmpubsub import PickleLCM
from dimos.utils.cli import theme

# Type alias for all message types we might receive
AnyMessage = Union[SystemMessage, ToolMessage, AIMessage, HumanMessage]


@dataclass
class MessageEntry:
    """Store a single message with metadata."""

    timestamp: float
    message: AnyMessage

    def __post_init__(self) -> None:
        """Initialize timestamp if not provided."""
        if self.timestamp is None:
            self.timestamp = time.time()


class AgentMessageMonitor:
    """Monitor agent messages published via LCM."""

    def __init__(self, topic: str = "/agent", max_messages: int = 1000) -> None:
        self.topic = topic
        self.max_messages = max_messages
        self.messages: deque[MessageEntry] = deque(maxlen=max_messages)
        self.transport = PickleLCM()
        self.transport.start()
        self.callbacks: list[callable] = []  # type: ignore[valid-type]
        pass

    def start(self) -> None:
        """Start monitoring messages."""
        self.transport.subscribe(self.topic, self._handle_message)

    def stop(self) -> None:
        """Stop monitoring."""
        # PickleLCM doesn't have explicit stop method
        pass

    def _handle_message(self, msg: Any, topic: str) -> None:
        """Handle incoming messages."""
        # Check if it's one of the message types we care about
        if isinstance(msg, SystemMessage | ToolMessage | AIMessage | HumanMessage):
            entry = MessageEntry(timestamp=time.time(), message=msg)
            self.messages.append(entry)

            # Notify callbacks
            for callback in self.callbacks:
                callback(entry)  # type: ignore[misc]
        else:
            pass

    def subscribe(self, callback: callable) -> None:  # type: ignore[valid-type]
        """Subscribe to new messages."""
        self.callbacks.append(callback)

    def get_messages(self) -> list[MessageEntry]:
        """Get all stored messages."""
        return list(self.messages)


def format_timestamp(timestamp: float) -> str:
    """Format timestamp as HH:MM:SS.mmm."""
    return (
        time.strftime("%H:%M:%S", time.localtime(timestamp)) + f".{int((timestamp % 1) * 1000):03d}"
    )


def get_message_type_and_style(msg: AnyMessage) -> tuple[str, str]:
    """Get message type name and style color."""
    if isinstance(msg, HumanMessage):
        return "Human ", "green"
    elif isinstance(msg, AIMessage):
        if hasattr(msg, "metadata") and msg.metadata.get("state"):
            return "State ", "blue"
        return "Agent ", "yellow"
    elif isinstance(msg, ToolMessage):
        return "Tool  ", "red"
    elif isinstance(msg, SystemMessage):
        return "System", "red"
    else:
        return "Unkn  ", "white"


def format_message_content(msg: AnyMessage) -> str:
    """Format message content for display."""
    if isinstance(msg, ToolMessage):
        return f"{msg.name}() -> {msg.content}"
    elif isinstance(msg, AIMessage) and msg.tool_calls:
        # Include tool calls in content
        tool_info = []
        for tc in msg.tool_calls:
            args_str = str(tc.get("args", {}))
            tool_info.append(f"{tc.get('name')}({args_str})")
        content = msg.content or ""
        if content and tool_info:
            return f"{content}\n[Tool Calls: {', '.join(tool_info)}]"
        elif tool_info:
            return f"[Tool Calls: {', '.join(tool_info)}]"
        return content  # type: ignore[return-value]
    else:
        return str(msg.content) if hasattr(msg, "content") else str(msg)


class AgentSpyApp(App):  # type: ignore[type-arg]
    """TUI application for monitoring agent messages."""

    CSS_PATH = theme.CSS_PATH

    CSS = f"""
    Screen {{
        layout: vertical;
        background: {theme.BACKGROUND};
    }}

    RichLog {{
        height: 1fr;
        border: none;
        background: {theme.BACKGROUND};
        padding: 0 1;
    }}

    Footer {{
        dock: bottom;
        height: 1;
    }}
    """

    BINDINGS = [
        Binding("q", "quit", "Quit"),
        Binding("c", "clear", "Clear"),
        Binding("ctrl+c", "quit", show=False),
    ]

    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(*args, **kwargs)
        self.monitor = AgentMessageMonitor()
        self.message_log: RichLog | None = None

    def compose(self) -> ComposeResult:
        """Compose the UI."""
        self.message_log = RichLog(wrap=True, highlight=True, markup=True)
        yield self.message_log
        yield Footer()

    def on_mount(self) -> None:
        """Start monitoring when app mounts."""
        self.theme = "flexoki"

        # Subscribe to new messages
        self.monitor.subscribe(self.on_new_message)
        self.monitor.start()

        # Write existing messages to the log
        for entry in self.monitor.get_messages():
            self.on_new_message(entry)

    def on_unmount(self) -> None:
        """Stop monitoring when app unmounts."""
        self.monitor.stop()

    def on_new_message(self, entry: MessageEntry) -> None:
        """Handle new messages."""
        if self.message_log:
            msg = entry.message
            msg_type, style = get_message_type_and_style(msg)
            content = format_message_content(msg)

            # Format the message for the log
            timestamp = format_timestamp(entry.timestamp)
            self.message_log.write(
                f"[dim white]{timestamp}[/dim white] | "
                f"[bold {style}]{msg_type}[/bold {style}] | "
                f"[{style}]{content}[/{style}]"
            )

    def refresh_display(self) -> None:
        """Refresh the message display."""
        # Not needed anymore as messages are written directly to the log

    def action_clear(self) -> None:
        """Clear message history."""
        self.monitor.messages.clear()
        if self.message_log:
            self.message_log.clear()


def main() -> None:
    """Main entry point for agentspy."""
    import sys

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
