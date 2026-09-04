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

"""Interactive controls for an already-running teach collection stack."""

from __future__ import annotations

import time
from typing import Any, cast

from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Container, Horizontal
from textual.widgets import Button, Footer, Static
import typer

from dimos.cli import theme
from dimos.msgs.imitation_msgs.EpisodeStatus import EpisodeStatus
from dimos.porcelain.dimos import Dimos

_MONITOR = "EpisodeMonitorModule"
_COORDINATOR = "ControlCoordinator"
_REQUIRED_TASKS = {"teach_openyam"}


class TeachCollectionSession:
    """RPC client for the operator controls used by the collection panel."""

    def __init__(self, client: Dimos, monitor: Any) -> None:
        self._client = client
        self._monitor = monitor
        self._closed = False

    @classmethod
    def connect(cls) -> TeachCollectionSession:
        """Attach to and validate the canonical teach collection modules."""
        client = Dimos.connect()
        try:
            modules = {info.instance_name: info for info in client.list_modules()}
            for name, rpcs in {
                _MONITOR: {"command", "get_status"},
                _COORDINATOR: {"list_tasks"},
            }.items():
                info = modules.get(name)
                if info is None:
                    raise RuntimeError(f"running stack has no {name!r} module")
                available = {rpc.name for rpc in info.rpcs}
                missing = rpcs - available
                if missing:
                    raise RuntimeError(f"{name!r} is missing RPCs: {sorted(missing)}")

            monitor = cast("Any", client.get_module(_MONITOR))
            coordinator = cast("Any", client.get_module(_COORDINATOR))
            tasks = set(coordinator.list_tasks())
            missing_tasks = _REQUIRED_TASKS - tasks
            if missing_tasks:
                raise RuntimeError(f"ControlCoordinator is missing tasks: {sorted(missing_tasks)}")
            monitor.get_status()
            return cls(client, monitor)
        except Exception:
            client.stop()
            raise

    def get_status(self) -> EpisodeStatus:
        """Read the monitor's latest state."""
        status = self._monitor.get_status()
        if not isinstance(status, EpisodeStatus):
            raise RuntimeError(
                f"EpisodeMonitorModule returned {type(status).__name__}, expected EpisodeStatus"
            )
        return status

    def command(self, event: str) -> EpisodeStatus:
        """Send one episode command."""
        status = self._monitor.command(event)
        if not isinstance(status, EpisodeStatus):
            raise RuntimeError(
                f"EpisodeMonitorModule returned {type(status).__name__}, expected EpisodeStatus"
            )
        return status

    def close(self) -> None:
        """Close only this RPC client; leave the daemon and robot running."""
        if not self._closed:
            self._client.stop()
            self._closed = True


class TeachCollectionApp(App[None]):
    """Operator dashboard for hand-guided teach collection."""

    CSS_PATH = theme.CSS_PATH
    CSS = f"""
    Screen {{
        align: center middle;
        background: {theme.BACKGROUND};
    }}

    #dashboard {{
        width: 82;
        max-width: 95%;
        height: auto;
        padding: 1 2;
        border: double {theme.BORDER};
        background: {theme.BG};
    }}

    #title {{
        height: 1;
        content-align: center middle;
        color: {theme.ACCENT};
        text-style: bold;
    }}

    #task {{
        height: 1;
        text-align: center;
        color: {theme.WHITE};
    }}

    #state {{
        height: 3;
        margin-top: 1;
        border: round {theme.SUCCESS};
        content-align: center middle;
        color: {theme.SUCCESS};
        text-style: bold;
    }}

    #state.recording, #state.disconnected {{
        border: round {theme.ERROR};
        color: {theme.ERROR};
    }}

    #counters {{
        height: 3;
    }}

    .counter {{
        width: 1fr;
        margin: 0 1;
        border: round {theme.DIM};
        content-align: center middle;
        text-align: center;
    }}

    #guidance {{
        height: 3;
        content-align: center middle;
        text-align: center;
        color: {theme.FOREGROUND};
    }}

    #message {{
        height: 2;
        content-align: center middle;
        text-align: center;
        color: {theme.WARNING};
    }}

    #actions {{
        height: 3;
    }}

    #actions Button {{
        width: 1fr;
        margin: 0 1;
    }}
    """

    BINDINGS = [
        Binding("space", "toggle_recording", "Start / save"),
        Binding("d", "discard", "Discard"),
        Binding("q", "quit", "Detach"),
        Binding("ctrl+c", "quit", "Detach", show=False),
    ]

    def __init__(self, session: TeachCollectionSession) -> None:
        super().__init__()
        self._session = session
        self._status = session.get_status()
        self._message = "Reset the scene, then start a take."
        self._detached = False
        self._recording_started_at: float | None = None

    def compose(self) -> ComposeResult:
        with Container(id="dashboard"):
            yield Static("OPENYAM  /  TEACH COLLECTION", id="title")
            yield Static(id="task")
            yield Static(id="state")
            with Horizontal(id="counters"):
                yield Static(id="saved", classes="counter")
                yield Static(id="discarded", classes="counter")
            yield Static(id="guidance")
            yield Static(id="message")
            with Horizontal(id="actions"):
                yield Button("Start recording", id="toggle", variant="success")
                yield Button("Discard", id="discard", variant="error", disabled=True)
                yield Button("Detach", id="detach")
        yield Footer()

    def on_mount(self) -> None:
        self._refresh()
        self.set_interval(0.25, self._poll)

    def on_unmount(self) -> None:
        self._session.close()

    @staticmethod
    def _format_elapsed(seconds: float) -> str:
        minutes, seconds = divmod(max(seconds, 0.0), 60.0)
        return f"{int(minutes):02d}:{seconds:04.1f}"

    def _set_status(self, status: EpisodeStatus) -> None:
        was_recording = self._status.state == "recording"
        self._status = status
        recording = status.state == "recording"
        if recording and not was_recording:
            self._recording_started_at = time.monotonic()
        elif not recording:
            self._recording_started_at = None

    def _state_text(self) -> str:
        if self._detached:
            return "DISCONNECTED"
        recording = self._status.state == "recording"
        if not recording:
            return "READY"
        elapsed = (
            "--:--"
            if self._recording_started_at is None
            else self._format_elapsed(time.monotonic() - self._recording_started_at)
        )
        return f"●  RECORDING   {elapsed}"

    def _refresh(self) -> None:
        recording = self._status.state == "recording"
        state = self.query_one("#state", Static)
        state.set_class(recording and not self._detached, "recording")
        state.set_class(self._detached, "disconnected")
        state.update(self._state_text())

        task = self._status.task_label or "Untitled task"
        self.query_one("#task", Static).update(f"TASK  {task}")
        self.query_one("#saved", Static).update(f"SAVED\n{self._status.episodes_saved}")
        self.query_one("#discarded", Static).update(f"DISCARDED\n{self._status.episodes_discarded}")
        guidance = (
            "Move the gravity-compensated arm and passive gripper by hand.\n"
            "Press Space to save this episode, or D to discard it."
            if recording
            else "Reset the scene and place the arm at the starting pose.\n"
            "Press Space when the demonstration begins."
        )
        if self._detached:
            guidance = "The RPC connection closed. The daemon and arm are still running."
        self.query_one("#guidance", Static).update(guidance)
        self.query_one("#message", Static).update(self._message)

        toggle = self.query_one("#toggle", Button)
        toggle.label = "Save episode" if recording else "Start recording"
        toggle.variant = "error" if recording else "success"
        toggle.disabled = self._detached
        self.query_one("#discard", Button).disabled = self._detached or not recording
        detach = self.query_one("#detach", Button)
        detach.label = "Exit" if self._detached else "Detach"
        detach.disabled = recording and not self._detached

    def _poll(self) -> None:
        if self._detached:
            return
        try:
            self._set_status(self._session.get_status())
            self._refresh()
        except Exception as exc:
            self._fail(exc)

    def _fail(self, exc: Exception) -> None:
        self._message = f"Connection error: {exc}"
        self._detached = True
        self._session.close()
        self._refresh()

    def _episode_command(self, event: str) -> None:
        if self._detached:
            return
        try:
            self._set_status(self._session.command(event))
            self._message = {
                "start": "Recording. Drag the arm through the demonstration.",
                "save": "Episode saved. Reset the scene for the next take.",
                "discard": "Episode discarded. Reset the scene and try again.",
            }.get(self._status.last_event, self._status.last_event)
            self._refresh()
        except Exception as exc:
            self._fail(exc)

    def action_toggle_recording(self) -> None:
        self._episode_command("toggle")

    def action_discard(self) -> None:
        if self._status.state != "recording":
            self._message = "Nothing to discard. Start a take first."
            self._refresh()
            return
        self._episode_command("discard")

    def on_button_pressed(self, event: Button.Pressed) -> None:
        actions = {
            "toggle": self.action_toggle_recording,
            "discard": self.action_discard,
            "detach": self.action_quit,
        }
        action = actions.get(event.button.id or "")
        if action is not None:
            action()

    def action_quit(self) -> None:  # type: ignore[override]
        if not self._detached and self._status.state == "recording":
            self._message = "Save with Space or discard with D before detaching."
            self._refresh()
            return
        self.exit()


def collect() -> None:
    """Control an already-running OpenYAM teach collection stack."""
    try:
        session = TeachCollectionSession.connect()
    except Exception as exc:
        typer.echo(f"Unable to attach collection controls: {exc}", err=True)
        raise typer.Exit(1) from exc
    TeachCollectionApp(session).run()
