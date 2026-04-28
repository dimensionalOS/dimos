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

"""HumanCLI sub-app — embedded agent chat interface."""

from __future__ import annotations

from collections import deque
from datetime import datetime
from enum import Enum, auto
import json
import textwrap
import threading
from typing import TYPE_CHECKING, Any

from textual.containers import Container
from textual.geometry import Size
from textual.widgets import Input, RichLog, Static

from dimos.utils.cli import theme
from dimos.utils.cli.dio.sub_app import SubApp

if TYPE_CHECKING:
    from textual.app import ComposeResult


class _ConnState(Enum):
    DISCONNECTED = auto()
    CONNECTING = auto()  # LCM transports initializing
    WAITING_FOR_AGENT = auto()  # transports ready, no agent response yet
    CONNECTED = auto()  # agent confirmed present
    NO_AGENT = auto()  # timed out waiting for agent
    ERROR = auto()


def _status_style(state: _ConnState) -> tuple[str, str]:
    """Return (label, color) for a connection state, reading theme at call time."""
    return {
        _ConnState.DISCONNECTED: ("disconnected", theme.DIM),
        _ConnState.CONNECTING: ("connecting…", theme.YELLOW),
        _ConnState.WAITING_FOR_AGENT: ("waiting for agent…", theme.YELLOW),
        _ConnState.CONNECTED: ("connected", theme.GREEN),
        _ConnState.NO_AGENT: ("no agent — blueprint may not include one", theme.DIM),
        _ConnState.ERROR: ("error", theme.RED),
    }[state]


# Seconds to wait for an agent response before showing "no agent"
_AGENT_DETECT_TIMEOUT = 8.0


class _ThinkingIndicator:
    """Animated 'thinking…' line inside a RichLog."""

    def __init__(self, app: Any, chat_log: RichLog, add_message_fn: Any) -> None:
        self._app = app
        self._log = chat_log
        self._add = add_message_fn
        self._timer: Any = None
        self._strips: list[Any] = []
        self.visible = False
        self._dim = False

    def show(self) -> None:
        if self.visible:
            return
        self.visible = True
        self._dim = False
        self._write()
        self._timer = self._app.set_interval(0.6, self._toggle)

    def hide(self) -> None:
        if not self.visible:
            return
        self.visible = False
        if self._timer is not None:
            self._timer.stop()
            self._timer = None
        self._remove()

    def detach_if_needed(self) -> bool:
        if self.visible and self._strips:
            self._remove()
            return True
        return False

    def reattach(self) -> None:
        self._write()

    def _write(self) -> None:
        before = len(self._log.lines)
        color = theme.DIM if self._dim else theme.ACCENT
        ts = datetime.now().strftime("%H:%M:%S")
        self._add(ts, "", "[italic]thinking…[/italic]", color)
        self._strips = list(self._log.lines[before:])

    def _remove(self) -> None:
        if not self._strips:
            return
        ids = {id(s) for s in self._strips}
        self._log.lines = [l for l in self._log.lines if id(l) not in ids]
        self._strips = []
        self._log._line_cache.clear()
        self._log.virtual_size = Size(self._log.virtual_size.width, len(self._log.lines))
        self._log.refresh()

    def _toggle(self) -> None:
        if not self.visible:
            return
        self._remove()
        self._dim = not self._dim
        self._write()


class HumanCLISubApp(SubApp):
    TITLE = "chat"

    DEFAULT_CSS = """
    HumanCLISubApp {
        layout: vertical;
        background: $dio-bg;
    }
    HumanCLISubApp #hcli-status-bar {
        height: 1;
        dock: top;
        padding: 0 1;
        background: $dio-bg;
        color: $dio-dim;
    }
    HumanCLISubApp #hcli-chat {
        height: 1fr;
    }
    HumanCLISubApp RichLog {
        height: 1fr;
        scrollbar-size: 0 0;
        border: solid $dio-dim;
    }
    HumanCLISubApp Input {
        dock: bottom;
    }
    """

    def __init__(self) -> None:
        super().__init__()
        self._human_transport: Any = None
        self._agent_transport: Any = None
        self._idle_transport: Any = None
        self._running = False
        self._conn_state = _ConnState.DISCONNECTED
        self._conn_error: str = ""
        self._send_queue: deque[str] = deque()
        self._queue_lock = threading.Lock()
        self._thinking: _ThinkingIndicator | None = None
        self._agent_seen = False
        self._agent_timeout_timer: Any = None

    def compose(self) -> ComposeResult:
        # Show default disconnected status immediately (before on_mount_subapp)
        label, color = _status_style(_ConnState.DISCONNECTED)
        yield Static(f"[{color}]● {label}[/{color}]", id="hcli-status-bar")
        with Container(id="hcli-chat"):
            yield RichLog(id="hcli-log", highlight=True, markup=True, wrap=False)
        yield Input(placeholder="Type a message…", id="hcli-input")

    def get_focus_target(self) -> Any:
        return self.query_one("#hcli-input", Input)

    def on_mount_subapp(self) -> None:
        self._running = True
        log = self.query_one("#hcli-log", RichLog)
        self._thinking = _ThinkingIndicator(self.app, log, self._add_message_raw)
        self._set_conn_state(_ConnState.CONNECTING)
        self.run_worker(self._init_transports, exclusive=True, thread=True)

    def on_unmount_subapp(self) -> None:
        self._running = False
        if self._agent_timeout_timer is not None:
            self._agent_timeout_timer.stop()
            self._agent_timeout_timer = None
        if self._thinking:
            self._thinking.hide()

    def reinit_lcm(self) -> None:
        """Recreate LCM transports after autoconf changed network config."""
        self._human_transport = None
        self._agent_transport = None
        self._idle_transport = None
        self._agent_seen = False
        if self._agent_timeout_timer is not None:
            self._agent_timeout_timer.stop()
            self._agent_timeout_timer = None
        self._set_conn_state(_ConnState.CONNECTING)
        self.run_worker(self._init_transports, exclusive=True, thread=True)

    # ── connection state ──────────────────────────────────────────

    def _set_conn_state(self, state: _ConnState, error: str = "") -> None:
        self._conn_state = state
        self._conn_error = error
        label, color = _status_style(state)
        if state == _ConnState.ERROR and error:
            label = f"error: {error}"
        try:
            bar = self.query_one("#hcli-status-bar", Static)
            bar.update(f"[{color}]● {label}[/{color}]")
        except Exception:
            pass

    # ── transport init (worker thread) ────────────────────────────

    def _init_transports(self) -> None:
        try:
            from dimos.core.transport import pLCMTransport

            self._human_transport = pLCMTransport("/human_input")
            self._agent_transport = pLCMTransport("/agent")
            self._idle_transport = pLCMTransport("/agent_idle")
        except Exception as exc:
            self.app.call_from_thread(self._set_conn_state, _ConnState.ERROR, str(exc))
            return

        self._subscribe_to_agent()
        self._subscribe_to_idle()
        self.app.call_from_thread(self._on_transport_ready)

    def _on_transport_ready(self) -> None:
        self._set_conn_state(_ConnState.WAITING_FOR_AGENT)
        # Messages can be sent over LCM even while waiting — flush the queue
        self._flush_queue()
        # Start a timer: if no agent responds within the timeout, show "no agent"
        self._agent_timeout_timer = self.set_timer(
            _AGENT_DETECT_TIMEOUT, self._on_agent_detect_timeout
        )

    def _on_agent_detected(self) -> None:
        """Called (on main thread) the first time we hear from an agent."""
        if self._agent_seen:
            return
        self._agent_seen = True
        if self._agent_timeout_timer is not None:
            self._agent_timeout_timer.stop()
            self._agent_timeout_timer = None
        self._set_conn_state(_ConnState.CONNECTED)
        log = self.query_one("#hcli-log", RichLog)
        self._add_system_message(log, "Agent connected")

    def _on_agent_detect_timeout(self) -> None:
        """Fired if no agent message arrived within the timeout window."""
        self._agent_timeout_timer = None
        if not self._agent_seen:
            self._set_conn_state(_ConnState.NO_AGENT)
            log = self.query_one("#hcli-log", RichLog)
            self._add_system_message(log, "No agent detected — this blueprint may not include one")

    # ── message queue ─────────────────────────────────────────────

    def _enqueue(self, message: str) -> None:
        with self._queue_lock:
            self._send_queue.append(message)

    def _flush_queue(self) -> None:
        with self._queue_lock:
            queued = list(self._send_queue)
            self._send_queue.clear()
        for msg in queued:
            self._do_send(msg)

    def _do_send(self, message: str) -> None:
        """Actually publish a message. Expects to be called on main thread."""
        if self._human_transport:
            self._human_transport.publish(message)

    # ── subscriptions ─────────────────────────────────────────────

    def _subscribe_to_agent(self) -> None:
        from langchain_core.messages import AIMessage, HumanMessage, SystemMessage, ToolMessage

        def receive_msg(msg: Any) -> None:
            if not self._running:
                return
            # Any message from the agent transport proves an agent exists
            if not self._agent_seen:
                self.app.call_from_thread(self._on_agent_detected)
            try:
                log = self.query_one("#hcli-log", RichLog)
            except Exception:
                return
            timestamp = datetime.now().strftime("%H:%M:%S")

            if isinstance(msg, SystemMessage):
                self.app.call_from_thread(
                    self._add_message,
                    log,
                    timestamp,
                    "system",
                    str(msg.content)[:1000],
                    theme.YELLOW,
                )
            elif isinstance(msg, AIMessage):
                content = msg.content or ""
                tool_calls = getattr(msg, "tool_calls", None) or msg.additional_kwargs.get(
                    "tool_calls", []
                )
                if content:
                    # Check for common API key errors
                    content_lower = str(content).lower()
                    if any(
                        phrase in content_lower
                        for phrase in ["api key", "authentication", "unauthorized", "invalid key"]
                    ):
                        self.app.call_from_thread(
                            self._set_conn_state,
                            _ConnState.ERROR,
                            "API key issue — check your config",
                        )
                    self.app.call_from_thread(
                        self._add_message, log, timestamp, "agent", content, theme.AGENT
                    )
                if tool_calls:
                    for tc in tool_calls:
                        name = tc.get("name", "unknown")
                        args = tc.get("args", {})
                        info = f"▶ {name}({json.dumps(args, separators=(',', ':'))})"
                        self.app.call_from_thread(
                            self._add_message, log, timestamp, "tool", info, theme.TOOL
                        )
            elif isinstance(msg, ToolMessage):
                self.app.call_from_thread(
                    self._add_message, log, timestamp, "tool", str(msg.content), theme.TOOL_RESULT
                )
            elif isinstance(msg, HumanMessage):
                pass  # We already display user messages locally

        self._agent_transport.subscribe(receive_msg)

    def _subscribe_to_idle(self) -> None:
        def receive_idle(is_idle: bool) -> None:
            if not self._running:
                return
            # Any idle signal proves an agent exists
            if not self._agent_seen:
                self.app.call_from_thread(self._on_agent_detected)
            if self._thinking:
                self.app.call_from_thread(self._thinking.hide if is_idle else self._thinking.show)

        self._idle_transport.subscribe(receive_idle)

    # ── display helpers ───────────────────────────────────────────

    def _add_message(
        self, log: RichLog, timestamp: str, sender: str, content: str, color: str
    ) -> None:
        if self._thinking:
            reattach = self._thinking.detach_if_needed()
        else:
            reattach = False
        self._add_message_raw(timestamp, sender, content, color)
        if reattach and self._thinking:
            self._thinking.reattach()

    def _add_message_raw(self, timestamp: str, sender: str, content: str, color: str) -> None:
        """Write a formatted message line to the log (no thinking-indicator management)."""
        try:
            log = self.query_one("#hcli-log", RichLog)
        except Exception:
            return
        content = content.strip() if content else ""
        prefix = (
            f" [{theme.TIMESTAMP}]{timestamp}[/{theme.TIMESTAMP}] [{color}]{sender:>8}[/{color}] │ "
        )
        indent = " " * 19 + "│ "
        width = max(log.size.width - 24, 40) if log.size else 60

        for i, line in enumerate(content.split("\n")):
            wrapped = textwrap.wrap(line, width=width) or [""]
            if i == 0:
                log.write(prefix + f"[{color}]{wrapped[0]}[/{color}]")
                for wl in wrapped[1:]:
                    log.write(indent + f"[{color}]{wl}[/{color}]")
            else:
                for wl in wrapped:
                    log.write(indent + f"[{color}]{wl}[/{color}]")

    def _add_system_message(self, log: RichLog, content: str) -> None:
        timestamp = datetime.now().strftime("%H:%M:%S")
        self._add_message(log, timestamp, "system", content, theme.YELLOW)

    def _add_user_message(self, log: RichLog, content: str, queued: bool = False) -> None:
        """Show user's own message immediately."""
        timestamp = datetime.now().strftime("%H:%M:%S")
        suffix = f"  [{theme.DIM}](queued)[/{theme.DIM}]" if queued else ""
        self._add_message(log, timestamp, "you", content + suffix, theme.HUMAN)

    # ── input handling ────────────────────────────────────────────

    def on_input_submitted(self, event: Input.Submitted) -> None:
        if event.input.id != "hcli-input":
            return
        message = event.value.strip()
        if not message:
            return
        event.input.value = ""

        if message.lower() in ("/exit", "/quit"):
            return
        if message.lower() == "/clear":
            self.query_one("#hcli-log", RichLog).clear()
            return

        log = self.query_one("#hcli-log", RichLog)
        transport_ready = self._conn_state in (
            _ConnState.WAITING_FOR_AGENT,
            _ConnState.CONNECTED,
            _ConnState.NO_AGENT,
        )

        if transport_ready and self._human_transport:
            self._add_user_message(log, message)
            self._human_transport.publish(message)
        else:
            # Transport not ready yet — queue the message and tell the user
            self._add_user_message(log, message, queued=True)
            self._enqueue(message)
            n = len(self._send_queue)
            bar = self.query_one("#hcli-status-bar", Static)
            label, color = _status_style(self._conn_state)
            bar.update(
                f"[{color}]● {label}[/{color}]"
                f"  [{theme.DIM}]({n} message{'s' if n != 1 else ''} queued — will send when connected)[/{theme.DIM}]"
            )
