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

"""Status sub-app — log viewer and blueprint lifecycle controls.

Supports multiple concurrent running blueprints via an instance picker.
"""

from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys
import threading
import time
from typing import TYPE_CHECKING, Any

from rich.panel import Panel
from rich.text import Text
from textual.containers import Horizontal, VerticalScroll
from textual.widgets import Button, RichLog, Static

from dimos.utils.cli import theme
from dimos.utils.cli.dio.sub_app import SubApp

if TYPE_CHECKING:
    from textual.app import ComposeResult


def _get_all_running() -> list[Any]:
    """Return all running InstanceInfo objects."""
    try:
        from dimos.core.instance_registry import list_running

        return list_running()
    except Exception:
        return []


def _stdout_log_path(info: Any) -> Path | None:
    """Return the stdout.log path for a running instance."""
    if info and getattr(info, "run_dir", None):
        p = Path(info.run_dir) / "stdout.log"
        if p.exists():
            return p
    return None


class StatusSubApp(SubApp):
    TITLE = "status"

    DEFAULT_CSS = """
    StatusSubApp {
        layout: vertical;
        height: 1fr;
        background: $dio-bg;
    }
    StatusSubApp .subapp-header {
        width: 100%;
        height: auto;
        color: $dio-accent2;
        padding: 1 2;
        text-style: bold;
    }
    StatusSubApp RichLog {
        height: 1fr;
        background: $dio-bg;
        border: solid $dio-dim;
        scrollbar-size-vertical: 0;
        scrollbar-size-horizontal: 1;
    }
    StatusSubApp #idle-container {
        height: 1fr;
        align: center middle;
    }
    StatusSubApp #idle-panel {
        width: auto;
        background: transparent;
    }
    StatusSubApp #instance-picker {
        height: auto;
        padding: 0 1;
        background: $dio-bg;
    }
    StatusSubApp #instance-picker Button {
        margin: 0 1 0 0;
        min-width: 8;
        background: transparent;
        border: solid $dio-dim;
        color: $dio-dim;
    }
    StatusSubApp #instance-picker Button.--selected {
        border: solid $dio-accent;
        color: $dio-accent;
        text-style: bold;
    }
    StatusSubApp #instance-picker Button:focus {
        background: $dio-panel-bg;
    }
    StatusSubApp #run-controls {
        height: auto;
        padding: 0 1;
        background: $dio-bg;
    }
    StatusSubApp #run-controls Button {
        margin: 0 1 0 0;
        min-width: 12;
        background: transparent;
        border: solid $dio-dim;
        color: $dio-text;
    }
    StatusSubApp .status-bar {
        height: 1;
        dock: bottom;
        background: $dio-hint-bg;
        color: $dio-dim;
        padding: 0 1;
    }
    StatusSubApp #btn-stop {
        border: solid $dio-btn-danger-bg;
        color: $dio-btn-danger;
    }
    StatusSubApp #btn-stop:hover {
        border: solid $dio-btn-danger;
    }
    StatusSubApp #btn-stop:focus {
        background: $dio-btn-danger-bg;
        color: $dio-white;
        border: solid $dio-btn-danger;
    }
    StatusSubApp #btn-sudo-kill {
        border: solid $dio-btn-kill-bg;
        color: $dio-btn-kill;
    }
    StatusSubApp #btn-sudo-kill:hover {
        border: solid $dio-btn-kill;
    }
    StatusSubApp #btn-sudo-kill:focus {
        background: $dio-btn-kill-bg;
        color: $dio-white;
        border: solid $dio-btn-kill;
    }
    StatusSubApp #btn-restart {
        border: solid $dio-btn-warn-bg;
        color: $dio-btn-warn;
    }
    StatusSubApp #btn-restart:hover {
        border: solid $dio-btn-warn;
    }
    StatusSubApp #btn-restart:focus {
        background: $dio-btn-warn-bg;
        color: $dio-white;
        border: solid $dio-btn-warn;
    }
    StatusSubApp #btn-open-log {
        border: solid $dio-btn-muted-bg;
        color: $dio-btn-muted;
    }
    StatusSubApp #btn-open-log:hover {
        border: solid $dio-btn-muted;
    }
    StatusSubApp #btn-open-log:focus {
        background: $dio-btn-muted-bg;
        color: $dio-white;
        border: solid $dio-btn-muted;
    }
    """

    def __init__(self) -> None:
        super().__init__()
        # Multi-instance tracking
        self._running_entries: list[Any] = []
        self._selected_name: str | None = None  # name of the selected instance
        # Launching state (pre-build)
        self._launching_name: str | None = None
        self._launching_run_dir: Path | None = None
        self._stopping: bool = False
        self._log_thread: threading.Thread | None = None
        self._stop_log = False
        self._failed_stop_pid: int | None = None
        self._poll_count = 0
        self._last_click_time: float = 0.0
        self._last_click_y: int = -1
        self._saved_status: str = ""

    @property
    def _selected_entry(self) -> Any:
        """Return the currently selected running entry, or None."""
        if self._selected_name:
            for e in self._running_entries:
                if getattr(e, "name", None) == self._selected_name:
                    return e
        return None

    def _debug(self, msg: str) -> None:
        """Log to the DIO debug panel if available."""
        try:
            self.app._log(f"[{theme.BTN_MUTED}]STATUS:[/{theme.BTN_MUTED}] {msg}")  # type: ignore[attr-defined]
        except Exception:
            pass

    def compose(self) -> ComposeResult:
        yield Static("Blueprint Status", classes="subapp-header")
        with VerticalScroll(id="idle-container"):
            yield Static(self._idle_panel(), id="idle-panel")
        with Horizontal(id="instance-picker"):
            pass  # populated dynamically
        yield RichLog(id="runner-log", markup=True, wrap=False, auto_scroll=True, min_width=600)
        with Horizontal(id="run-controls"):
            yield Button("Stop", id="btn-stop", variant="error")
            yield Button("Force Kill (sudo)", id="btn-sudo-kill")
            yield Button("Restart", id="btn-restart", variant="warning")
            yield Button("Open Log File", id="btn-open-log")
        yield Static("", id="runner-status", classes="status-bar")

    def _idle_panel(self) -> Panel:
        msg = Text(justify="center")
        msg.append("No Blueprint Running\n\n", style=f"bold {theme.BTN_DANGER}")
        msg.append("Use the ", style=theme.DIM)
        msg.append("launch", style=f"bold {theme.CYAN}")
        msg.append(" tab to start a blueprint", style=theme.DIM)
        return Panel(msg, border_style=theme.DIM, expand=False)

    def on_mount_subapp(self) -> None:
        self._debug("on_mount_subapp called")
        self._refresh_entries()
        if self._running_entries:
            self._selected_name = self._running_entries[0].name
            self._debug(f"-> _show_running (initial: {self._selected_name})")
            self._show_running()
        else:
            self._debug("-> _show_idle")
            self._show_idle()
        self._start_poll_timer()

    def on_resume_subapp(self) -> None:
        self._debug("on_resume_subapp: restarting timer after remount")
        self._start_poll_timer()
        self._refresh_entries()
        if self._running_entries:
            self._show_running()

    def _start_poll_timer(self) -> None:
        self.set_interval(1.0, self._poll_running)
        self._debug("timer started")

    def get_focus_target(self) -> object | None:
        if self._running_entries or self._launching_name is not None:
            try:
                return self.query_one("#runner-log", RichLog)
            except Exception:
                pass
        return super().get_focus_target()

    # ------------------------------------------------------------------
    # Instance picker
    # ------------------------------------------------------------------

    def _rebuild_picker(self) -> None:
        """Rebuild the instance picker buttons from current entries."""
        picker = self.query_one("#instance-picker", Horizontal)
        picker.remove_children()

        # Collect names: running entries + launching entry if not yet registered
        names: list[str] = [getattr(e, "name", "?") for e in self._running_entries]
        if self._launching_name and self._launching_name not in names:
            names.append(self._launching_name)

        if len(names) <= 1:
            # No need for picker with 0 or 1 instance
            picker.styles.display = "none"
            return

        picker.styles.display = "block"
        for name in names:
            btn = Button(name, id=f"pick-{name}")
            if name == self._selected_name or name == self._launching_name:
                btn.add_class("--selected")
            picker.mount(btn)

    def _on_picker_pressed(self, name: str) -> None:
        """Handle an instance picker button press."""
        if name == self._selected_name:
            return

        self._debug(f"picker: switching to {name}")
        self._selected_name = name
        self._stop_log = True

        # Update picker button styles
        picker = self.query_one("#instance-picker", Horizontal)
        for child in picker.children:
            if isinstance(child, Button):
                if getattr(child, "id", "") == f"pick-{name}":
                    child.add_class("--selected")
                else:
                    child.remove_class("--selected")

        # Switch log and status to selected entry
        entry = self._selected_entry
        if entry:
            self._show_running_for_entry(entry)
        elif name == self._launching_name and self._launching_run_dir:
            # Still in launching phase
            log_widget = self.query_one("#runner-log", RichLog)
            log_widget.clear()
            self._start_log_follow_from_path(self._launching_run_dir / "stdout.log")
            status = self.query_one("#runner-status", Static)
            status.update(f"Launching: {name}...")

    # ------------------------------------------------------------------
    # Launcher notification (immediate feedback)
    # ------------------------------------------------------------------

    def on_launch_started(self, instance_name: str, run_dir: Path) -> None:
        """Called by launcher immediately after launch_blueprint() returns.

        Shows UI controls before the daemon has finished building.
        """
        self._debug(f"on_launch_started: {instance_name} at {run_dir}")
        self._launching_name = instance_name
        self._launching_run_dir = run_dir
        self._selected_name = instance_name
        self._stopping = False

        # Show controls for the launching state
        self.query_one("#idle-container").styles.display = "none"
        self.query_one("#runner-log").styles.display = "block"
        self.query_one("#run-controls").styles.display = "block"
        self.query_one("#btn-stop").styles.display = "block"
        self.query_one("#btn-sudo-kill").styles.display = "none"
        self.query_one("#btn-restart").styles.display = "none"  # no restart during build
        self.query_one("#btn-open-log").styles.display = "block"
        self._failed_stop_pid = None

        status = self.query_one("#runner-status", Static)
        status.update(f"Launching: {instance_name}...")

        # Clear log and start tailing the new run_dir's stdout.log
        log_widget = self.query_one("#runner-log", RichLog)
        self._stop_log = True  # stop any existing tail
        log_widget.clear()
        self._start_log_follow_from_path(run_dir / "stdout.log")

        # Rebuild picker (may now show multiple entries)
        self._rebuild_picker()

    # ------------------------------------------------------------------
    # State management
    # ------------------------------------------------------------------

    def _refresh_entries(self) -> None:
        """Update the list of running entries from the registry."""
        try:
            self._running_entries = _get_all_running()
        except Exception as e:
            self._debug(f"_refresh_entries exception: {e}")
            self._running_entries = []

    def _poll_running(self) -> None:
        self._poll_count += 1
        old_entries = self._running_entries
        old_names = {getattr(e, "name", None) for e in old_entries}
        self._refresh_entries()
        new_names = {getattr(e, "name", None) for e in self._running_entries}

        # If we're in launching state and the matching instance appeared -> transition
        if self._launching_name is not None and self._launching_name in new_names:
            self._debug(f"launch completed: {self._launching_name} -> running")
            completed_name = self._launching_name
            self._launching_name = None
            self._launching_run_dir = None
            # If the completed instance is selected, show restart button
            if self._selected_name == completed_name:
                self.query_one("#btn-restart").styles.display = "block"
                entry = self._selected_entry
                if entry:
                    status = self.query_one("#runner-status", Static)
                    status.update(self._format_status_line(entry))
            self._rebuild_picker()
            return

        changed = old_names != new_names
        if changed or self._poll_count % 10 == 1:
            self._debug(
                f"poll #{self._poll_count}: old={old_names} new={new_names} changed={changed}"
            )

        if changed:
            self._rebuild_picker()

            # If nothing is running anymore
            if not self._running_entries and self._launching_name is None:
                self._debug("-> all instances gone")
                self._selected_name = None
                self._stop_log = True
                self._show_stopped("All processes ended")
                return

            # If selected instance disappeared
            if self._selected_name and self._selected_name not in new_names:
                # The selected instance died — if launching, keep showing that
                if self._launching_name == self._selected_name:
                    return
                self._debug(f"selected instance {self._selected_name} gone")
                self._stop_log = True
                if self._running_entries:
                    # Switch to another running instance
                    self._selected_name = self._running_entries[0].name
                    self._show_running()
                else:
                    self._selected_name = None
                    self._show_stopped("Process ended")
                return

            # New instance appeared (maybe launched externally)
            added = new_names - old_names
            if added and not self._selected_name:
                # Auto-select the new one if nothing selected
                self._selected_name = self._running_entries[0].name
                self._show_running()

    def _show_running(self) -> None:
        """Show controls for the selected running blueprint."""
        entry = self._selected_entry
        if entry:
            self._show_running_for_entry(entry)
        self._rebuild_picker()

    def _show_running_for_entry(self, entry: Any) -> None:
        """Show controls for a specific running entry."""
        self._debug(f"_show_running_for_entry: {getattr(entry, 'name', '?')}")
        try:
            self.query_one("#idle-container").styles.display = "none"
            self.query_one("#runner-log").styles.display = "block"
            self.query_one("#run-controls").styles.display = "block"
            self.query_one("#btn-stop").styles.display = "block"
            self.query_one("#btn-sudo-kill").styles.display = "none"
            self.query_one("#btn-restart").styles.display = "block"
            self.query_one("#btn-open-log").styles.display = "block"
            self._failed_stop_pid = None

            status = self.query_one("#runner-status", Static)
            status.update(self._format_status_line(entry))
            self._debug(f"starting log follow for {entry.name}")
            self._start_log_follow(entry)
        except Exception as e:
            self._debug(f"_show_running_for_entry CRASHED: {e}")

    def _show_stopped(self, message: str = "Stopped") -> None:
        """Show controls for a stopped state with logs still visible."""
        self._launching_name = None
        self._launching_run_dir = None
        self._stopping = False
        self.query_one("#idle-container").styles.display = "none"
        self.query_one("#runner-log").styles.display = "block"
        self.query_one("#run-controls").styles.display = "block"
        self.query_one("#btn-stop").styles.display = "none"
        self.query_one("#btn-sudo-kill").styles.display = "none"
        self.query_one("#btn-restart").styles.display = "none"
        self.query_one("#btn-open-log").styles.display = "block"
        self._failed_stop_pid = None
        status = self.query_one("#runner-status", Static)
        status.update(message)
        self._rebuild_picker()

    def _show_idle(self) -> None:
        """Show big idle message — no blueprint running."""
        self._debug("_show_idle called")
        self.query_one("#idle-container").styles.display = "block"
        self.query_one("#runner-log").styles.display = "none"
        self.query_one("#run-controls").styles.display = "none"
        self.query_one("#instance-picker").styles.display = "none"
        self._failed_stop_pid = None
        self._launching_name = None
        self._launching_run_dir = None
        self._selected_name = None

        # Check if there are past runs
        try:
            from dimos.core.instance_registry import _instances_dir
            base = _instances_dir()
            if base.exists():
                for child in sorted(base.iterdir(), reverse=True):
                    runs = child / "runs"
                    if runs.exists() and any(runs.iterdir()):
                        status = self.query_one("#runner-status", Static)
                        status.update(f"Last run: {child.name}")
                        return
        except Exception:
            pass

        status = self.query_one("#runner-status", Static)
        status.update("No blueprint running")

    # ------------------------------------------------------------------
    # Entry info formatting
    # ------------------------------------------------------------------

    @staticmethod
    def _format_status_line(entry: Any) -> str:
        """One-line status bar summary including config overrides."""
        overrides = getattr(entry, "config_overrides", None) or {}
        name = getattr(entry, "name", getattr(entry, "blueprint", "?"))
        pid = getattr(entry, "pid", "?")
        parts = [f"Running: {name} (PID {pid}) — double-click log to open"]
        if overrides:
            flags = " ".join(
                f"--{k.replace('_', '-')}"
                if isinstance(v, bool) and v
                else f"--no-{k.replace('_', '-')}"
                if isinstance(v, bool)
                else f"--{k.replace('_', '-')}={v}"
                for k, v in overrides.items()
            )
            parts.append(flags)
        return " | ".join(parts)

    @staticmethod
    def _format_launch_header(entry: Any) -> list[str]:
        """Rich-markup lines summarising how the blueprint was launched."""
        lines: list[str] = []
        argv = getattr(entry, "original_argv", None) or []
        overrides = getattr(entry, "config_overrides", None) or {}
        if argv:
            lines.append(f"[dim]$ {' '.join(argv)}[/dim]")
        if overrides:
            items = "  ".join(f"[{theme.CYAN}]{k}[/{theme.CYAN}]={v}" for k, v in overrides.items())
            lines.append(f"[dim]config overrides:[/dim] {items}")
        lines.append("")  # blank separator
        return lines

    # ------------------------------------------------------------------
    # Log streaming
    # ------------------------------------------------------------------

    _LEVEL_STYLES: dict[str, str] = {
        "dbg": "bold cyan",
        "deb": "bold cyan",
        "inf": "bold green",
        "war": "bold yellow",
        "err": "bold red",
        "cri": "bold red",
    }

    @staticmethod
    def _format_jsonl_line(raw: str) -> Text:
        """Parse a JSONL log line and return a colorized Rich Text object."""
        import json
        from pathlib import Path as P

        _STANDARD_KEYS = {"timestamp", "level", "logger", "event", "func_name", "lineno"}

        try:
            rec: dict[str, object] = json.loads(raw)
        except (json.JSONDecodeError, ValueError):
            return Text(raw.rstrip())

        ts = str(rec.get("timestamp", ""))
        hms = ts[11:19] if len(ts) >= 19 else ts
        level = str(rec.get("level", "?"))[:3].lower()
        logger_name = P(str(rec.get("logger", "?"))).name
        event = str(rec.get("event", ""))

        line = Text()
        line.append(hms, style="dim")
        lvl_style = StatusSubApp._LEVEL_STYLES.get(level, "")
        line.append(f"[{level}]", style=lvl_style)
        line.append(f"[{logger_name:17}] ", style="dim")
        line.append(event, style="blue")

        extras = {k: v for k, v in rec.items() if k not in _STANDARD_KEYS}
        if extras:
            line.append(" ")
            for k, v in sorted(extras.items()):
                line.append(f"{k}", style="cyan")
                line.append("=", style="white")
                line.append(f"{v}", style="magenta")
                line.append(" ")

        return line

    def _write_log_line(self, log_widget: RichLog, rendered: Text | str) -> None:
        """Write a line to the log widget."""
        log_widget.write(rendered)

    def _start_log_follow(self, entry: Any) -> None:
        """Tail stdout.log from the instance's run directory."""
        self._stop_log = False
        log_widget = self.query_one("#runner-log", RichLog)

        # If a tail is already running, stop it
        if self._log_thread is not None and self._log_thread.is_alive():
            self._stop_log = True
            self._log_thread.join(timeout=1.0)
            self._stop_log = False

        log_widget.clear()
        # Print launch info header
        for line in self._format_launch_header(entry):
            self._write_log_line(log_widget, line)

        # Use stdout.log from the instance's run directory
        log_path = _stdout_log_path(entry)
        if log_path:
            self._start_log_follow_from_path(log_path)

    def _start_log_follow_from_path(self, log_path: Path) -> None:
        """Tail a log file path, waiting for it to appear if needed."""
        self._stop_log = False
        log_widget = self.query_one("#runner-log", RichLog)

        # If a tail is already running, stop it first
        if self._log_thread is not None and self._log_thread.is_alive():
            self._stop_log = True
            self._log_thread.join(timeout=1.0)
            self._stop_log = False

        def _follow() -> None:
            try:
                # Wait for log file to appear
                if not log_path.exists():
                    self.app.call_from_thread(
                        self._write_log_line, log_widget, "[dim]Waiting for log...[/dim]"
                    )
                    for _ in range(150):  # ~30s
                        if self._stop_log:
                            return
                        if log_path.exists():
                            break
                        time.sleep(0.2)
                    else:
                        self.app.call_from_thread(
                            self._write_log_line, log_widget, "[dim]Log file did not appear[/dim]"
                        )
                        return

                with open(log_path) as f:
                    while not self._stop_log:
                        line = f.readline()
                        if line:
                            rendered = Text.from_ansi(line.rstrip("\n"))
                            self.app.call_from_thread(
                                self._write_log_line, log_widget, rendered
                            )
                        else:
                            time.sleep(0.2)
            except Exception as e:
                self.app.call_from_thread(
                    self._write_log_line, log_widget, f"[red]Error: {e}[/red]"
                )
                self.app.call_from_thread(
                    self.app.notify, f"Log follow error: {e}", severity="error", timeout=8,
                )

        self._log_thread = threading.Thread(target=_follow, daemon=True)
        self._log_thread.start()

    # ------------------------------------------------------------------
    # Button handling
    # ------------------------------------------------------------------

    def _is_click_on_log(self, event: Any) -> bool:
        """Return True if the click event is inside the runner-log RichLog."""
        try:
            node = event.widget
            while node is not None:
                if getattr(node, "id", None) == "runner-log":
                    return True
                node = node.parent
        except Exception:
            pass
        return False

    def on_click(self, event: Any) -> None:
        """Single click: show hint. Double click: open source file."""
        if not self._is_click_on_log(event):
            return

        now = time.monotonic()
        click_y = getattr(event, "screen_y", -1)
        is_double = (now - self._last_click_time) < 0.4 and abs(click_y - self._last_click_y) <= 1
        self._last_click_time = now
        self._last_click_y = click_y

        if is_double:
            self._handle_double_click(event)
        else:
            status = self.query_one("#runner-status", Static)
            current = status.renderable
            if not isinstance(current, str) or "double-click" not in current:
                self._saved_status = str(current)
            status.update("double-click to open log file")
            self.set_timer(2.0, self._restore_status)

    def _restore_status(self) -> None:
        """Restore the status bar after the hint."""
        try:
            status = self.query_one("#runner-status", Static)
            current = str(status.renderable)
            if "double-click" in current and self._saved_status:
                status.update(self._saved_status)
        except Exception:
            pass

    def _get_clicked_line_number(self, event: Any) -> int:
        """Map a click event to a 1-based line number in the log file."""
        try:
            log_widget = self.query_one("#runner-log", RichLog)
            local_y = event.screen_y - log_widget.region.y
            line_idx = int(log_widget.scroll_y) + local_y
            return max(1, line_idx + 1)
        except Exception:
            return 1

    def _handle_double_click(self, event: Any) -> None:
        """Open the stdout.log in the user's editor."""
        lineno = self._get_clicked_line_number(event)

        entry = self._selected_entry
        log_path = _stdout_log_path(entry) if entry else None

        # Fallback to launching run_dir during build phase
        if log_path is None and self._launching_run_dir is not None:
            candidate = self._launching_run_dir / "stdout.log"
            if candidate.exists():
                log_path = candidate

        if log_path and log_path.exists():
            self._open_source_file(str(log_path), lineno)
        else:
            self.app.notify("No log found", severity="warning")

    def _open_source_file(self, file_path: str, lineno: int) -> None:
        """Open a source file in the user's preferred GUI editor."""
        import shutil

        full_path = Path(file_path)
        if not full_path.is_absolute():
            for base in [Path.cwd(), Path(__file__).resolve().parents[5]]:
                candidate = base / file_path
                if candidate.exists():
                    full_path = candidate
                    break

        loc = f"{full_path}:{lineno}" if lineno else str(full_path)
        loc_short = f"{full_path.name}:{lineno}" if lineno else full_path.name

        if not full_path.exists():
            self.app.copy_to_clipboard(loc)
            self.app.notify(f"File not found, copied path: {loc}", severity="warning")
            return

        _GUI_EDITORS: list[tuple[str, list[str]]] = []

        for env_var in ("VISUAL", "EDITOR"):
            cmd = os.environ.get(env_var, "")
            if not cmd or not shutil.which(cmd):
                continue
            cmd_name = Path(cmd).name
            if cmd_name in ("code", "code-insiders"):
                _GUI_EDITORS.append((cmd, ["-g", loc]))
            elif cmd_name in ("subl", "sublime", "subl3"):
                _GUI_EDITORS.append((cmd, [loc]))
            elif cmd_name in ("atom", "zed", "fleet"):
                _GUI_EDITORS.append((cmd, [loc]))
            elif cmd_name in ("idea", "pycharm", "goland", "webstorm", "clion"):
                _GUI_EDITORS.append((cmd, ["--line", str(lineno), str(full_path)]))

        for cmd, args in [
            ("code", ["-g", loc]),
            ("subl", [loc]),
            ("zed", [loc]),
        ]:
            if shutil.which(cmd):
                _GUI_EDITORS.append((cmd, args))

        for cmd, args in _GUI_EDITORS:
            try:
                subprocess.Popen(
                    [cmd, *args],
                    stdin=subprocess.DEVNULL,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    start_new_session=True,
                )
                self.app.notify(f"Opened {loc_short}")
                return
            except Exception:
                continue

        self.app.copy_to_clipboard(loc)
        self.app.notify(f"Copied to clipboard: {loc}")

    def on_button_pressed(self, event: Button.Pressed) -> None:
        btn_id = event.button.id or ""
        if btn_id == "btn-stop":
            self._stop_running()
        elif btn_id == "btn-sudo-kill":
            self._sudo_kill()
        elif btn_id == "btn-restart":
            self._restart_running()
        elif btn_id == "btn-open-log":
            self._open_log_in_editor()
        elif btn_id.startswith("pick-"):
            self._on_picker_pressed(btn_id[5:])

    def on_key(self, event: Any) -> None:
        key = getattr(event, "key", "")
        if key in ("left", "right"):
            self._cycle_button_focus(1 if key == "right" else -1)
            event.prevent_default()
            event.stop()
        elif key == "enter":
            focused = self.app.focused
            if isinstance(focused, Button):
                focused.press()
                event.prevent_default()
                event.stop()

    def _get_visible_buttons(self) -> list[Button]:
        buttons: list[Button] = []
        for bid in ("btn-stop", "btn-sudo-kill", "btn-restart", "btn-open-log"):
            try:
                btn = self.query_one(f"#{bid}", Button)
                if btn.styles.display != "none":
                    buttons.append(btn)
            except Exception:
                pass
        return buttons

    def _cycle_button_focus(self, delta: int) -> None:
        buttons = self._get_visible_buttons()
        if not buttons:
            return
        focused = self.app.focused
        try:
            idx = buttons.index(focused)  # type: ignore[arg-type]
            idx = (idx + delta) % len(buttons)
        except ValueError:
            idx = 0
        buttons[idx].focus()

    # ------------------------------------------------------------------
    # Stop / restart / kill
    # ------------------------------------------------------------------

    def _stop_running(self) -> None:
        if self._stopping:
            return
        self._stopping = True
        self._stop_log = True
        log_widget = self.query_one("#runner-log", RichLog)
        status = self.query_one("#runner-status", Static)

        entry = self._selected_entry
        stop_name = getattr(entry, "name", None) or self._launching_name
        status.update(f"Stopping {stop_name or 'blueprint'}...")

        for bid in ("btn-stop", "btn-restart"):
            try:
                self.query_one(f"#{bid}", Button).disabled = True
            except Exception:
                pass

        def _do_stop() -> None:
            permission_error = False
            if stop_name:
                try:
                    from dimos.core.instance_registry import stop as registry_stop

                    msg, _ = registry_stop(stop_name)
                    self.app.call_from_thread(
                        log_widget.write, f"[{theme.YELLOW}]{msg}[/{theme.YELLOW}]"
                    )
                except PermissionError:
                    permission_error = True
                    pid = getattr(entry, "pid", "?")
                    self.app.call_from_thread(
                        log_widget.write,
                        f"[red]Permission denied — cannot stop PID {pid}[/red]",
                    )
                    self.app.call_from_thread(
                        self.app.notify, f"Permission denied stopping PID {pid}", severity="error", timeout=8,
                    )
                except Exception as e:
                    if (
                        "permission" in str(e).lower()
                        or "operation not permitted" in str(e).lower()
                    ):
                        permission_error = True
                    self.app.call_from_thread(log_widget.write, f"[red]Stop error: {e}[/red]")
                    self.app.call_from_thread(
                        self.app.notify, f"Stop error: {e}", severity="error", timeout=8,
                    )

            def _after_stop() -> None:
                self._stopping = False
                for bid in ("btn-stop", "btn-restart"):
                    try:
                        self.query_one(f"#{bid}", Button).disabled = False
                    except Exception:
                        pass
                if permission_error and entry:
                    self._failed_stop_pid = entry.pid
                    self.query_one("#btn-sudo-kill").styles.display = "block"
                    self.query_one("#btn-sudo-kill", Button).focus()
                    s = self.query_one("#runner-status", Static)
                    s.update(
                        f"Stop failed (permission denied) — try Force Kill for PID {entry.pid}"
                    )
                else:
                    # Refresh — if other instances still running, switch to one
                    self._refresh_entries()
                    if self._running_entries:
                        self._selected_name = self._running_entries[0].name
                        self._show_running()
                    else:
                        self._selected_name = None
                        self._show_stopped(f"Stopped {stop_name}")

            self.app.call_from_thread(_after_stop)

        threading.Thread(target=_do_stop, daemon=True).start()

    def _restart_running(self) -> None:
        entry = self._selected_entry
        name = getattr(entry, "name", None) or getattr(entry, "blueprint", None)
        if not name:
            return
        self._stop_log = True
        log_widget = self.query_one("#runner-log", RichLog)
        status = self.query_one("#runner-status", Static)
        status.update(f"Restarting {name}...")
        for bid in ("btn-stop", "btn-restart"):
            try:
                self.query_one(f"#{bid}", Button).disabled = True
            except Exception:
                pass

        # Gather config overrides
        config_overrides: dict[str, object] = {}
        try:
            from dimos.utils.cli.dio.sub_apps.config import ConfigSubApp

            for inst in self.app._instances:  # type: ignore[attr-defined]
                if isinstance(inst, ConfigSubApp):
                    config_overrides = inst.get_overrides()
                    break
        except Exception:
            pass

        blueprint = getattr(entry, "blueprint", name)

        def _do_restart() -> None:
            # Stop old
            if entry:
                try:
                    from dimos.core.instance_registry import stop as registry_stop

                    registry_stop(entry.name)
                except Exception:
                    pass

            # Launch new via launch_blueprint
            try:
                from dimos.core.daemon import launch_blueprint

                result = launch_blueprint(
                    robot_types=[blueprint],
                    config_overrides=config_overrides,
                    force_replace=True,
                )

                def _after() -> None:
                    for bid in ("btn-stop", "btn-restart"):
                        try:
                            self.query_one(f"#{bid}", Button).disabled = False
                        except Exception:
                            pass
                    self.on_launch_started(result.instance_name, result.run_dir)

                self.app.call_from_thread(_after)
            except Exception as e:

                def _err() -> None:
                    for bid in ("btn-stop", "btn-restart"):
                        try:
                            self.query_one(f"#{bid}", Button).disabled = False
                        except Exception:
                            pass
                    self.app.call_from_thread(log_widget.write, f"[red]Restart error: {e}[/red]")
                    self.app.notify(f"Restart failed: {e}", severity="error", timeout=10)
                    self._refresh_entries()
                    if self._running_entries:
                        self._selected_name = self._running_entries[0].name
                        self._show_running()
                    else:
                        self._show_stopped("Restart failed")

                self.app.call_from_thread(_err)

        threading.Thread(target=_do_restart, daemon=True).start()

    def _sudo_kill(self) -> None:
        pid = self._failed_stop_pid
        if pid is None:
            return
        log_widget = self.query_one("#runner-log", RichLog)
        self.query_one("#btn-sudo-kill", Button).disabled = True

        def _do_kill() -> None:
            try:
                result = subprocess.run(
                    ["sudo", "-n", "kill", "-9", str(pid)],
                    capture_output=True,
                    text=True,
                )
                if result.returncode == 0:
                    self.app.call_from_thread(
                        log_widget.write,
                        f"[{theme.YELLOW}]Killed PID {pid} with sudo[/{theme.YELLOW}]",
                    )
                    # Clean up registry
                    try:
                        from dimos.core.instance_registry import list_running, unregister

                        for info in list_running():
                            if info.pid == pid:
                                unregister(info.name)
                                break
                    except Exception:
                        pass

                    def _after() -> None:
                        self._failed_stop_pid = None
                        self._refresh_entries()
                        if self._running_entries:
                            self._selected_name = self._running_entries[0].name
                            self._show_running()
                        else:
                            self._selected_name = None
                            self._show_stopped("Killed with sudo")

                    self.app.call_from_thread(_after)
                else:
                    from dimos.utils.prompt import sudo_prompt

                    got_sudo = sudo_prompt("sudo is required to force-kill the process")
                    if got_sudo:
                        result2 = subprocess.run(
                            ["sudo", "-n", "kill", "-9", str(pid)],
                            capture_output=True,
                            text=True,
                        )
                        if result2.returncode == 0:
                            self.app.call_from_thread(
                                log_widget.write,
                                f"[{theme.YELLOW}]Killed PID {pid} with sudo[/{theme.YELLOW}]",
                            )
                            try:
                                from dimos.core.instance_registry import list_running, unregister

                                for info in list_running():
                                    if info.pid == pid:
                                        unregister(info.name)
                                        break
                            except Exception:
                                pass

                            def _after2() -> None:
                                self._failed_stop_pid = None
                                self._refresh_entries()
                                if self._running_entries:
                                    self._selected_name = self._running_entries[0].name
                                    self._show_running()
                                else:
                                    self._selected_name = None
                                    self._show_stopped("Killed with sudo")

                            self.app.call_from_thread(_after2)
                            return

                    self.app.call_from_thread(
                        log_widget.write,
                        "[red]sudo kill failed — could not obtain sudo credentials[/red]",
                    )
                    self.app.call_from_thread(
                        self.app.notify, "sudo kill failed — no credentials", severity="error", timeout=8,
                    )

                    def _reenable() -> None:
                        self.query_one("#btn-sudo-kill", Button).disabled = False

                    self.app.call_from_thread(_reenable)
            except Exception as e:
                self.app.call_from_thread(log_widget.write, f"[red]sudo kill error: {e}[/red]")
                self.app.call_from_thread(
                    self.app.notify, f"sudo kill error: {e}", severity="error", timeout=8,
                )

                def _reenable2() -> None:
                    self.query_one("#btn-sudo-kill", Button).disabled = False

                self.app.call_from_thread(_reenable2)

        threading.Thread(target=_do_kill, daemon=True).start()

    def _open_log_in_editor(self) -> None:
        """Open the stdout.log in the user's editor (non-blocking)."""
        entry = self._selected_entry
        log_path = _stdout_log_path(entry) if entry else None

        # Fallback to launching run_dir during build phase
        if log_path is None and self._launching_run_dir is not None:
            candidate = self._launching_run_dir / "stdout.log"
            if candidate.exists():
                log_path = candidate

        if log_path and log_path.exists():
            self._open_source_file(str(log_path), 0)
        else:
            self.app.notify("No log file found", severity="warning")

    def on_unmount_subapp(self) -> None:
        self._stop_log = True
