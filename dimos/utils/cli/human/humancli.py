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

from collections import deque
from datetime import datetime, timedelta
import json
import os
import sys
import textwrap
import threading
from typing import TYPE_CHECKING, Any

from langchain_core.messages import AIMessage, HumanMessage, SystemMessage, ToolCall, ToolMessage
from rich.highlighter import JSONHighlighter
from rich.panel import Panel
from rich.text import Text
from rich.theme import Theme
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Container
from textual.geometry import Size
from textual.strip import Strip
from textual.widgets import Input, RichLog, Static

from dimos.agents.mcp import tool_stream
from dimos.core.transport import pLCMTransport
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

# How many of a tool's most recent stream lines to show inside its box.
RECENT_LINES = 5
# Prefix `McpClient` puts on tool-stream updates it re-emits to `/agent`.
TOOL_MSG_PREFIX = "[tool:"
# Markers pairing a tool call with its result in the scrollback.
TOOL_CALL_MARKER = "▶"
TOOL_RESULT_MARKER = "↳"
# Backstop: finalize a stopped tool's box this long after the stop if no agent
# activity follows to trigger an idle transition.
STOP_FINALIZE_DELAY = 1.0


def _format_elapsed(delta: timedelta) -> str:
    total = int(delta.total_seconds())
    return f"{total // 60:02d}:{total % 60:02d}"


def _split_tool_message(content: Any) -> tuple[str, str] | None:
    """Parse a `[tool:NAME] <text>` tool-stream message into (name, text)."""
    if not isinstance(content, str) or not content.startswith(TOOL_MSG_PREFIX):
        return None
    end = content.find("]")
    if end == -1:
        return None
    return content[len(TOOL_MSG_PREFIX) : end], content[end + 1 :].lstrip()


class ToolPanel:
    """Live state for one streaming tool's box.

    ``entries`` holds the most recent ``(timestamp, kind, text)`` tuples where
    ``kind`` is ``"tool"`` (a stream update) or ``"agent"`` (the agent's reply
    to one), so each box shows the stream and the agent's annotations together.
    """

    def __init__(self, tool_name: str, start: datetime) -> None:
        self.tool_name = tool_name
        self.start = start
        self.entries: deque[tuple[str, str, str]] = deque(maxlen=RECENT_LINES)
        self.count = 0
        self.static: Static | None = None


class ToolPanelRegion:
    """Docked region of live per-tool boxes.

    All methods run on the Textual main thread; the tool-stream subscription
    hands updates over via ``call_from_thread``.
    """

    def __init__(
        self,
        container: Container,
        flush_fn: Callable[[Any], None],
    ) -> None:
        self._container = container
        self._flush = flush_fn
        self._panels: dict[str, ToolPanel] = {}

    def update(self, tool_name: str, text: str, kind: str, timestamp: str) -> None:
        panel = self._panels.get(tool_name)
        if panel is None:
            panel = ToolPanel(tool_name, datetime.now())
            self._panels[tool_name] = panel
            panel.entries.append((timestamp, kind, text))
            panel.count = 1
            panel.static = Static(self._render(panel))
            self._container.mount(panel.static)
        else:
            panel.entries.append((timestamp, kind, text))
            panel.count += 1
            assert panel.static is not None
            panel.static.update(self._render(panel))

    def finalize(self, tool_name: str) -> None:
        panel = self._panels.pop(tool_name, None)
        if panel is None:
            return
        self._flush(self._render(panel, done=True))
        if panel.static is not None:
            panel.static.remove()

    def _render(self, panel: ToolPanel, done: bool = False) -> Panel:
        elapsed = _format_elapsed(datetime.now() - panel.start)
        body = Text()
        hidden = panel.count - len(panel.entries)
        if hidden > 0:
            body.append(f"(+{hidden} earlier)\n", style=theme.DIM)
        for i, (timestamp, kind, text) in enumerate(panel.entries):
            if i:
                body.append("\n")
            body.append(f"{timestamp} ", style=theme.TIMESTAMP)
            if kind == "agent":
                body.append("> ", style=theme.AGENT)
                body.append(text, style=theme.AGENT)
            else:
                body.append(text)
        status = f"{panel.count} updates" if done else "running"
        title = f"{panel.tool_name}  {status}  {elapsed}"
        return Panel(body, title=title, title_align="left", border_style=theme.PURPLE)


class ThinkingIndicator:
    """Manages a throbbing 'thinking...' chat message in a RichLog."""

    def __init__(
        self,
        app: App[Any],
        chat_log: RichLog,
        add_message_fn: Callable[[str, str, str, str], object],
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

    #tool-panels {{
        height: auto;
        max-height: 50%;
        overflow-y: auto;
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
        self._tool_panels: ToolPanelRegion | None = None
        self._tool_stream_cleanup: Callable[[], None] | None = None
        # The tool whose box the next agent reply belongs to (None -> inline).
        self._reply_target: str | None = None
        # tool_call_id -> the scrollback strips of that call, so its result can
        # be spliced in directly beneath it (calls/results can arrive in any order).
        self._tool_call_anchors: dict[str, list[Strip]] = {}
        # Tools that have stopped but whose box waits for the agent to catch up.
        self._pending_stops: set[str] = set()
        self._agent_is_idle = True
        self._running = False

    def compose(self) -> ComposeResult:
        """Compose the IRC-like interface."""
        with Container(id="chat-container"):
            self.chat_log = RichLog(highlight=True, markup=True, wrap=False)
            yield self.chat_log

        yield Container(id="tool-panels")

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

        # Live boxes for streaming tools, fed straight off the tool-stream topic.
        self._tool_panels = ToolPanelRegion(
            self.query_one("#tool-panels", Container), self._flush_tool_panel
        )
        self._tool_stream_cleanup = tool_stream.subscribe(self._on_tool_stream)

        # Start subscription threads
        self._subscription_thread = threading.Thread(target=self._subscribe_to_agent, daemon=True)
        self._subscription_thread.start()
        self._idle_subscription_thread = threading.Thread(
            target=self._subscribe_to_idle, daemon=True
        )
        self._idle_subscription_thread.start()

        # Focus on input
        self.input_widget.focus()  # type: ignore[union-attr]

        self.chat_log.write(f"[{theme.ACCENT}]{theme.ascii_logo}[/{theme.ACCENT}]")

        self._add_system_message("Connected to DimOS Agent Interface")

    def on_unmount(self) -> None:
        """Clean up when unmounting."""
        self._running = False
        if self._tool_stream_cleanup is not None:
            self._tool_stream_cleanup()
            self._tool_stream_cleanup = None

    def _subscribe_to_agent(self) -> None:
        """Subscribe to agent messages in a separate thread."""

        def receive_msg(msg) -> None:  # type: ignore[no-untyped-def]
            if not self._running:
                return
            assert self._tool_panels is not None
            assert self._thinking is not None

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

                # A reply to a tool-stream update goes inside that tool's box so
                # it reads as an annotation of the stream, not the agent talking
                # to itself. Replies to a typed message stay inline.
                if content and self._reply_target is not None and isinstance(content, str):
                    self.call_from_thread(
                        self._tool_panels.update, self._reply_target, content, "agent", timestamp
                    )
                elif content:
                    self.call_from_thread(
                        self._add_message, timestamp, "agent", content, theme.AGENT
                    )

                # Tool calls are real actions; always show them inline, and
                # remember each one so its result can be grouped under it.
                if tool_calls:
                    for tc in tool_calls:
                        tool_info = self._format_tool_call(tc)
                        self.call_from_thread(
                            self._write_tool_call, timestamp, tool_info, tc.get("id")
                        )

                # If neither content nor tool calls, show a placeholder (but not
                # for the silent step that can follow a tool-stream update).
                if not content and not tool_calls and self._reply_target is None:
                    self.call_from_thread(
                        self._add_message, timestamp, "agent", "<no response>", theme.DIM
                    )
            elif isinstance(msg, ToolMessage):
                self.call_from_thread(
                    self._write_tool_result,
                    timestamp,
                    msg.content,
                    getattr(msg, "tool_call_id", None),
                )
            elif isinstance(msg, HumanMessage):
                # Tool-stream updates arrive here as `[tool:NAME] <text>`. Route
                # the update into the tool's box and remember the tool so the
                # following agent reply lands in the same box. A real typed
                # message clears the target and renders inline.
                parsed = _split_tool_message(msg.content)
                if parsed is not None:
                    name, text = parsed
                    self._reply_target = name
                    if text:
                        self.call_from_thread(
                            self._tool_panels.update, name, text, "tool", timestamp
                        )
                    return
                self._reply_target = None
                self.call_from_thread(
                    self._add_message, timestamp, "human", msg.content, theme.HUMAN
                )
                # Keep the spinner up while this turn is processed (also re-shows
                # it in the rare case a stale idle signal hid it post-submit).
                self.call_from_thread(self._thinking.show)

        self._agent_transport.subscribe(receive_msg)

    def _subscribe_to_idle(self) -> None:
        def receive_idle(is_idle: bool) -> None:
            if not self._running:
                return
            self.call_from_thread(self._set_agent_idle, is_idle)

        self._agent_idle.subscribe(receive_idle)

    def _set_agent_idle(self, is_idle: bool) -> None:
        assert self._thinking is not None
        self._agent_is_idle = is_idle
        if is_idle:
            # "thinking..." is only shown for human-initiated turns (on submit),
            # so just hide here. Showing it on every busy signal would flash it
            # for each tool-stream update, which the agent also runs through the
            # graph.
            self._thinking.hide()
            # The queue is drained, so every stopped tool's trailing replies
            # have now rendered -> finalize their boxes. A short delay lets the
            # last reply (delivered on a separate topic) settle first.
            if self._pending_stops:
                self.set_timer(0.15, self._flush_pending_stops)

    def _flush_pending_stops(self) -> None:
        assert self._tool_panels is not None
        # Only finalize while the agent is idle. A stop can arrive while the
        # tool's last update/reply is still queued (the real-time stop signal
        # outruns the LLM-paced /agent stream); finalizing then would strand the
        # trailing reply in a new, never-closing box. A later idle retries.
        if not self._agent_is_idle or not self._pending_stops:
            return
        for name in self._pending_stops:
            self._tool_panels.finalize(name)
        self._pending_stops.clear()

    def _on_tool_stream(self, msg: dict[str, Any]) -> None:
        """Watch the tool-stream topic for stop signals.

        Box content (updates and the agent's replies) is driven off the ordered
        `/agent` stream so the two stay paired; this subscription only needs the
        stop signal, which is the one event `/agent` does not carry (a
        self-terminating background tool produces no message there).
        """
        if not self._running:
            return
        if msg.get("method") != tool_stream.TOOL_STREAM_STOPPED_METHOD:
            return
        tool_name = (msg.get("params") or {}).get("tool_name") or "tool"
        self.call_from_thread(self._record_stop, tool_name)

    def _record_stop(self, tool_name: str) -> None:
        self._pending_stops.add(tool_name)
        # Don't finalize off the current (possibly stale) idle flag: the tool's
        # last update may have just been queued and not yet rendered. The idle
        # handler finalizes once the agent has caught up; this timer is only a
        # backstop for a tool that stops with no further agent activity.
        self.set_timer(STOP_FINALIZE_DELAY, self._flush_pending_stops)

    def _flush_tool_panel(self, renderable: Any) -> None:
        """Write a finalized tool box into the scrollback, keeping the
        thinking indicator pinned to the bottom."""
        assert self._thinking is not None
        reattach = self._thinking.detach_if_needed()
        self.chat_log.write(renderable)  # type: ignore[union-attr]
        if reattach:
            self._thinking.reattach()

    def _format_tool_call(self, tool_call: ToolCall) -> str:
        """Format a tool call for display."""
        name = tool_call.get("name", "unknown")
        args = tool_call.get("args", {})
        args_str = json.dumps(args, separators=(",", ":"))
        return f"{TOOL_CALL_MARKER} {name}({args_str})"

    def _write_tool_call(self, timestamp: str, tool_info: str, call_id: str | None) -> None:
        strips = self._add_message(timestamp, "tool", tool_info, theme.TOOL)
        if call_id and strips:
            self._tool_call_anchors[call_id] = strips

    def _write_tool_result(self, timestamp: str, content: str, call_id: str | None) -> None:
        text = f"{TOOL_RESULT_MARKER} {content}"
        anchor = self._tool_call_anchors.pop(call_id, None) if call_id else None
        if anchor is None:
            # No matching call on screen (e.g. a lookout continuation) -> append.
            self._add_message(timestamp, "tool", text, theme.TOOL_RESULT)
            return
        self._insert_after(anchor, timestamp, text)

    def _insert_after(self, anchor: list[Strip], timestamp: str, text: str) -> None:
        """Render a tool result and splice it into the log right below the
        matching tool call, even when lines were written in between."""
        log = self.chat_log
        assert log is not None
        new_strips = self._add_message(timestamp, "tool", text, theme.TOOL_RESULT)
        if not new_strips:
            return
        # Pull the just-appended result back out so it can be re-homed.
        new_ids = {id(s) for s in new_strips}
        log.lines = [line for line in log.lines if id(line) not in new_ids]
        # Locate the call's last line by identity (indices shift as the log
        # grows) and splice the result in directly after it.
        anchor_id = id(anchor[-1])
        insert_at = len(log.lines)
        for i, line in enumerate(log.lines):
            if id(line) == anchor_id:
                insert_at = i + 1
                break
        log.lines[insert_at:insert_at] = new_strips
        log._line_cache.clear()
        log.virtual_size = Size(log.virtual_size.width, len(log.lines))
        log.refresh()
        log.scroll_end(animate=False)

    def _add_message(self, timestamp: str, sender: str, content: str, color: str) -> list[Strip]:
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

        before = len(self.chat_log.lines)  # type: ignore[union-attr]
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

        written = list(self.chat_log.lines[before:])  # type: ignore[union-attr]
        if reattach:
            self._thinking.reattach()
        return written

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

        # Send to agent (message will be displayed when received back). Show
        # "thinking..." now so a human turn always has a spinner, even while the
        # agent first drains any queued tool-stream updates.
        if self._thinking is not None:
            self._thinking.show()
        self._human_transport.publish(message)

    def action_clear(self) -> None:
        """Clear the chat log."""
        self._tool_call_anchors.clear()
        self.chat_log.clear()  # type: ignore[union-attr]

    def action_quit(self) -> None:  # type: ignore[override]
        """Quit the application."""
        self._running = False
        self.exit()


def main() -> None:
    if len(sys.argv) > 1 and sys.argv[1] == "web":
        from textual_serve.server import Server  # type: ignore[import-not-found]

        server = Server(f"python {os.path.abspath(__file__)}")
        server.serve()
    else:
        app = HumanCLIApp()
        app.run()


if __name__ == "__main__":
    main()
