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

"""Launcher sub-app — blueprint picker and launcher."""

from __future__ import annotations

import threading
from typing import TYPE_CHECKING, Any

from textual.widgets import Input, Label, ListItem, ListView, Static

from dimos.utils.cli.dio.sub_app import SubApp

if TYPE_CHECKING:
    from pathlib import Path

    from textual.app import ComposeResult


def _list_running_names() -> list[str]:
    """Return names of currently running blueprints."""
    try:
        from dimos.core.instance_registry import list_running

        return [info.blueprint for info in list_running()]
    except Exception:
        return []


def _debug_log(msg: str) -> None:
    """Append to the DIO debug log file."""
    try:
        from dimos.core.instance_registry import dimos_home

        log_path = dimos_home() / "dio-debug.log"
        with open(log_path, "a") as f:
            f.write(f"LAUNCHER: {msg}\n")
    except Exception:
        pass


class LauncherSubApp(SubApp):
    TITLE = "launch"

    DEFAULT_CSS = """
    LauncherSubApp {
        layout: vertical;
        height: 1fr;
        background: $dio-bg;
    }
    LauncherSubApp .subapp-header {
        width: 100%;
        height: auto;
        color: $dio-accent2;
        padding: 1 2;
        text-style: bold;
    }
    LauncherSubApp #launch-filter {
        width: 100%;
        background: $dio-bg;
        border: solid $dio-dim;
        color: $dio-text;
    }
    LauncherSubApp #launch-filter:focus {
        border: solid $dio-accent;
    }
    LauncherSubApp ListView {
        height: 1fr;
        background: $dio-bg;
    }
    LauncherSubApp ListView > ListItem {
        background: $dio-bg;
        color: $dio-text;
        padding: 1 2;
    }
    LauncherSubApp ListView > ListItem.--highlight {
        background: $dio-panel-bg;
    }
    LauncherSubApp.--locked ListView {
        opacity: 0.35;
    }
    LauncherSubApp.--locked #launch-filter {
        opacity: 0.35;
    }
    LauncherSubApp .status-bar {
        height: 1;
        dock: bottom;
        background: $dio-hint-bg;
        color: $dio-dim;
        padding: 0 1;
    }
    """

    def __init__(self) -> None:
        super().__init__()
        self._blueprints: list[str] = []
        self._filtered: list[str] = []
        self._launching = False
        self._launching_name: str | None = None  # name of blueprint currently launching

    def compose(self) -> ComposeResult:
        yield Static("Blueprint Launcher", classes="subapp-header")
        yield Input(placeholder="Type to filter blueprints...", id="launch-filter")
        yield ListView(id="launch-list")
        yield Static("", id="launch-status", classes="status-bar")

    def on_mount_subapp(self) -> None:
        self._populate_blueprints()
        self._sync_status()
        self._start_poll_timer()

    def on_resume_subapp(self) -> None:
        self._start_poll_timer()
        self._sync_status()

    def _start_poll_timer(self) -> None:
        self.set_interval(2.0, self._sync_status)

    def get_focus_target(self) -> object | None:
        try:
            return self.query_one("#launch-filter", Input)
        except Exception:
            return super().get_focus_target()

    def _populate_blueprints(self) -> None:
        try:
            from dimos.robot.all_blueprints import all_blueprints

            self._blueprints = sorted(
                name for name in all_blueprints if not name.startswith("demo-")
            )
        except Exception:
            self._blueprints = []

        self._filtered = list(self._blueprints)
        self._rebuild_list()

    def _rebuild_list(self) -> None:
        lv = self.query_one("#launch-list", ListView)
        lv.clear()
        for name in self._filtered:
            lv.append(ListItem(Label(name)))
        if self._filtered:
            lv.index = 0

    @property
    def _is_locked(self) -> bool:
        """True only while a launch is in progress."""
        return self._launching

    def _sync_status(self) -> None:
        status = self.query_one("#launch-status", Static)
        filter_input = self.query_one("#launch-filter", Input)
        lv = self.query_one("#launch-list", ListView)

        if self._launching:
            self.add_class("--locked")
            filter_input.disabled = True
            lv.disabled = True
            return  # don't overwrite "Launching..." message

        self.remove_class("--locked")
        filter_input.disabled = False
        lv.disabled = False

        running = _list_running_names()
        if running:
            names = ", ".join(running)
            status.update(f"Running: {names} | Enter: launch another")
        else:
            status.update("Up/Down: navigate | Enter: launch | Type to filter")

    def on_input_changed(self, event: Input.Changed) -> None:
        if event.input.id == "launch-filter":
            q = event.value.strip().lower()
            if not q:
                self._filtered = list(self._blueprints)
            else:
                self._filtered = [n for n in self._blueprints if q in n.lower()]
            self._rebuild_list()

    def on_input_submitted(self, event: Input.Submitted) -> None:
        _debug_log(f"on_input_submitted: id={event.input.id} locked={self._is_locked}")
        if event.input.id == "launch-filter":
            if self._is_locked:
                return
            lv = self.query_one("#launch-list", ListView)
            idx = lv.index
            _debug_log(f"on_input_submitted: idx={idx} filtered_len={len(self._filtered)}")
            if idx is not None and 0 <= idx < len(self._filtered):
                _debug_log(f"on_input_submitted: confirming {self._filtered[idx]}")
                self._confirm_and_launch(self._filtered[idx])

    def on_list_view_selected(self, event: ListView.Selected) -> None:
        _debug_log(f"on_list_view_selected: locked={self._is_locked}")
        if self._is_locked:
            return
        lv = self.query_one("#launch-list", ListView)
        idx = lv.index
        if idx is not None and 0 <= idx < len(self._filtered):
            _debug_log(f"on_list_view_selected: confirming {self._filtered[idx]}")
            self._confirm_and_launch(self._filtered[idx])

    def _confirm_and_launch(self, name: str) -> None:
        from dimos.utils.cli.dio.confirm_screen import ConfirmScreen

        running = _list_running_names()
        if self._launching_name and self._launching_name not in running:
            running.append(self._launching_name)
        _debug_log(f"_confirm_and_launch: name={name} running={running}")
        if running:
            names = ", ".join(running)
            message = (
                f"The {names} blueprint{'s are' if len(running) > 1 else ' is'} "
                f"already running, are you sure you want to start {name}?"
            )
            warning = True
        else:
            message = f"Launch {name}?"
            warning = False

        def _on_confirm(result: bool) -> None:
            _debug_log(f"_on_confirm: result={result}")
            if result:
                self._launch(name)

        self.app.push_screen(ConfirmScreen(message, warning=warning), _on_confirm)

    def on_key(self, event: Any) -> None:
        key = getattr(event, "key", "")
        focused = self.app.focused
        filter_input = self.query_one("#launch-filter", Input)
        if focused is filter_input and key in ("up", "down"):
            lv = self.query_one("#launch-list", ListView)
            if self._filtered:
                current = lv.index or 0
                if key == "up":
                    lv.index = max(0, current - 1)
                else:
                    lv.index = min(len(self._filtered) - 1, current + 1)
            event.prevent_default()
            event.stop()

    def _launch(self, name: str) -> None:
        _debug_log(f"_launch called: name={name} locked={self._is_locked}")
        if self._is_locked:
            self._sync_status()
            return

        self._launching = True
        self._launching_name = name
        self._sync_status()  # lock the UI immediately
        status = self.query_one("#launch-status", Static)
        status.update(f"Launching {name}...")

        # Gather config overrides as a Python dict (no CLI arg conversion)
        config_overrides: dict[str, object] = {}
        try:
            from dimos.utils.cli.dio.sub_apps.config import ConfigSubApp

            for inst in self.app._instances:  # type: ignore[attr-defined]
                if isinstance(inst, ConfigSubApp):
                    config_overrides = inst.get_overrides()
                    break
        except Exception:
            pass

        _debug_log(f"_launch: config_overrides={config_overrides}")

        def _do_launch() -> None:
            _debug_log("_do_launch: thread started")
            try:
                from dimos.core.daemon import launch_blueprint

                _debug_log(f"_do_launch: calling launch_blueprint(robot_types=[{name}])")
                result = launch_blueprint(
                    robot_types=[name],
                    config_overrides=config_overrides,
                    force_replace=False,
                )
                _debug_log(
                    f"_do_launch: success! instance={result.instance_name} run_dir={result.run_dir}"
                )

                def _after() -> None:
                    self._launching = False
                    self._launching_name = None
                    # Tell StatusSubApp immediately
                    self._notify_runner(result.instance_name, result.run_dir)
                    self._sync_status()

                self.app.call_from_thread(_after)
            except Exception as e:
                import traceback

                _debug_log(f"_do_launch: EXCEPTION: {e}\n{traceback.format_exc()}")

                def _err() -> None:
                    self._launching = False
                    self._launching_name = None
                    self.query_one("#launch-status", Static).update(f"Launch error: {e}")
                    self.app.notify(f"Launch failed: {e}", severity="error", timeout=10)
                    self._sync_status()

                self.app.call_from_thread(_err)

        threading.Thread(target=_do_launch, daemon=True).start()

    def _notify_runner(self, instance_name: str, run_dir: Path) -> None:
        """Tell the StatusSubApp about the just-launched instance."""
        from dimos.utils.cli.dio.sub_apps.runner import StatusSubApp

        for inst in self.app._instances:  # type: ignore[attr-defined]
            if isinstance(inst, StatusSubApp):
                inst.on_launch_started(instance_name, run_dir)
                break
