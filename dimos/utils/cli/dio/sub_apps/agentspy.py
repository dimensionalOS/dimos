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

"""AgentSpy sub-app — embedded agent message monitor."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from textual.widgets import RichLog

from dimos.utils.cli.dio.sub_app import SubApp

if TYPE_CHECKING:
    from textual.app import ComposeResult


class AgentSpySubApp(SubApp):
    TITLE = "agentspy"

    DEFAULT_CSS = """
    AgentSpySubApp {
        layout: vertical;
        background: $dio-bg;
    }
    AgentSpySubApp RichLog {
        height: 1fr;
        border: none;
        background: $dio-bg;
        padding: 0 1;
    }
    """

    def __init__(self) -> None:
        super().__init__()
        self._monitor: Any = None

    def compose(self) -> ComposeResult:
        yield RichLog(id="aspy-log", wrap=True, highlight=True, markup=True)

    def on_mount_subapp(self) -> None:
        self.run_worker(self._init_monitor, exclusive=True, thread=True)

    def _init_monitor(self) -> None:
        """Blocking monitor init — runs in a worker thread."""
        try:
            from dimos.utils.cli.agentspy.agentspy import AgentMessageMonitor

            self._monitor = AgentMessageMonitor()
            self._monitor.subscribe(self._on_new_message)
            self._monitor.start()

            # Write existing messages
            for entry in self._monitor.get_messages():
                self.app.call_from_thread(self._write_entry_safe, entry)
        except Exception:
            pass

    def on_unmount_subapp(self) -> None:
        if self._monitor:
            try:
                self._monitor.stop()
            except Exception:
                pass
            self._monitor = None

    def _on_new_message(self, entry: Any) -> None:
        try:
            self.app.call_from_thread(self._write_entry_safe, entry)
        except Exception:
            pass

    def _write_entry_safe(self, entry: Any) -> None:
        try:
            log = self.query_one("#aspy-log", RichLog)
            self._write_entry(log, entry)
        except Exception:
            pass

    def _write_entry(self, log: RichLog, entry: Any) -> None:
        from dimos.utils.cli.agentspy.agentspy import (
            format_message_content,
            format_timestamp,
            get_message_type_and_style,
        )

        msg = entry.message
        msg_type, style = get_message_type_and_style(msg)
        content = format_message_content(msg)
        timestamp = format_timestamp(entry.timestamp)
        log.write(
            f"[dim white]{timestamp}[/dim white] | "
            f"[bold {style}]{msg_type}[/bold {style}] | "
            f"[{style}]{content}[/{style}]"
        )
