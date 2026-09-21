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

"""Attached operator interfaces. The running blueprint owns robot lifecycle."""

from __future__ import annotations

import time

from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Container, Horizontal
from textual.widgets import Button, Footer, Static

from dimos.cli import theme
from dimos.imitation.collection.episode_monitor import EpisodeCommand, EpisodeControlSpec
from dimos.imitation.policy.module import RolloutControlSpec, RolloutStatus
from dimos.msgs.imitation_msgs.EpisodeStatus import EpisodeStatus
from dimos.porcelain.dimos import Dimos


class CollectionSession:
    """Operator RPCs for an attached collection stack."""

    def __init__(self, driver: Dimos) -> None:
        self._driver = driver
        self._monitor = driver.find_module_by_spec(EpisodeControlSpec)
        self._closed = False
        self.get_status()

    def get_status(self) -> EpisodeStatus:
        status = self._monitor.get_status()
        if not isinstance(status, EpisodeStatus):
            raise RuntimeError(f"episode monitor returned {type(status).__name__}")
        return status

    def command(self, event: EpisodeCommand) -> EpisodeStatus:
        status = self._monitor.command(event)
        if not isinstance(status, EpisodeStatus):
            raise RuntimeError(f"episode monitor returned {type(status).__name__}")
        return status

    def close(self) -> None:
        if not self._closed:
            self._driver.stop()
            self._closed = True


class CollectionApp(App[None]):
    """Episode controls for an attached collection session."""

    CSS_PATH = theme.CSS_PATH
    CSS = f"""
    Screen {{ align: center middle; background: {theme.BACKGROUND}; }}
    #dashboard {{ width: 82; max-width: 95%; height: auto; padding: 1 2;
        border: double {theme.BORDER}; background: {theme.BG}; }}
    #title {{ height: 1; content-align: center middle; color: {theme.ACCENT};
        text-style: bold; }}
    #task {{ height: 1; text-align: center; color: {theme.WHITE}; }}
    #state {{ height: 3; margin-top: 1; border: round {theme.SUCCESS};
        content-align: center middle; color: {theme.SUCCESS}; text-style: bold; }}
    #state.recording, #state.disconnected {{ border: round {theme.ERROR}; color: {theme.ERROR}; }}
    #counters, #actions {{ height: 3; }}
    .counter {{ width: 1fr; margin: 0 1; border: round {theme.DIM};
        content-align: center middle; text-align: center; }}
    #guidance {{ height: 3; content-align: center middle; text-align: center;
        color: {theme.FOREGROUND}; }}
    #message {{ height: 2; content-align: center middle; text-align: center;
        color: {theme.WARNING}; }}
    #actions Button {{ width: 1fr; margin: 0 1; }}
    """
    BINDINGS = [
        Binding("space", "toggle_recording", "Start / save"),
        Binding("d", "discard", "Discard"),
        Binding("q", "quit", "Detach"),
        Binding("ctrl+c", "quit", "Detach", show=False),
    ]

    def __init__(self, session: CollectionSession, title: str = "Collection") -> None:
        super().__init__()
        self._session = session
        self._title = title
        self._status = session.get_status()
        self._message = "Reset the scene, then start a take."
        self._disconnected = False
        self._recording_started_at: float | None = None
        self._quit_armed = False

    def compose(self) -> ComposeResult:
        with Container(id="dashboard"):
            yield Static(self._title.upper(), id="title")
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
                yield Button("Detach", id="stop")
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

    def _refresh(self) -> None:
        recording = self._status.state == "recording"
        state = self.query_one("#state", Static)
        state.set_class(recording and not self._disconnected, "recording")
        state.set_class(self._disconnected, "disconnected")
        if self._disconnected:
            state.update("DISCONNECTED")
        elif recording:
            elapsed = (
                "--:--"
                if self._recording_started_at is None
                else self._format_elapsed(time.monotonic() - self._recording_started_at)
            )
            state.update(f"●  RECORDING   {elapsed}")
        else:
            state.update("READY")
        self.query_one("#task", Static).update(f"TASK  {self._status.task_label}")
        self.query_one("#saved", Static).update(f"SAVED\n{self._status.episodes_saved}")
        self.query_one("#discarded", Static).update(f"DISCARDED\n{self._status.episodes_discarded}")
        guidance = (
            "Press Space to save this episode, or D to discard it."
            if recording
            else "Reset the scene. Press Space when the demonstration begins."
        )
        self.query_one("#guidance", Static).update(guidance)
        self.query_one("#message", Static).update(self._message)
        toggle = self.query_one("#toggle", Button)
        toggle.label = "Save episode" if recording else "Start recording"
        toggle.variant = "error" if recording else "success"
        toggle.disabled = self._disconnected
        self.query_one("#discard", Button).disabled = self._disconnected or not recording

    def _poll(self) -> None:
        if self._disconnected:
            return
        try:
            self._set_status(self._session.get_status())
            self._refresh()
        except Exception as exc:
            self._message = f"Connection error: {exc}"
            self._disconnected = True
            self._session.close()
            self._refresh()

    def _episode_command(self, event: EpisodeCommand) -> None:
        if self._disconnected:
            return
        try:
            self._set_status(self._session.command(event))
            self._message = {
                "start": "Recording.",
                "save": "Episode saved. Reset the scene for the next take.",
                "discard": "Episode discarded. Reset the scene and try again.",
            }.get(self._status.last_event, self._status.last_event)
            self._quit_armed = False
            self._refresh()
        except Exception as exc:
            self._message = f"Command failed: {exc}"
            self._refresh()

    def action_toggle_recording(self) -> None:
        self._episode_command("toggle")

    def action_discard(self) -> None:
        if self._status.state == "recording":
            self._episode_command("discard")

    def action_quit(self) -> None:  # type: ignore[override]
        if self._status.state == "recording" and not self._quit_armed:
            self._quit_armed = True
            self._message = "Recording will continue. Press Q again to detach."
            self._refresh()
            return
        self.exit()

    def on_button_pressed(self, event: Button.Pressed) -> None:
        action = {
            "toggle": self.action_toggle_recording,
            "discard": self.action_discard,
            "stop": self.action_quit,
        }.get(event.button.id or "")
        if action is not None:
            action()


class RolloutSession:
    """Operator RPCs for an attached policy rollout stack."""

    def __init__(self, driver: Dimos) -> None:
        self._driver = driver
        self._policy = driver.find_module_by_spec(RolloutControlSpec)
        self._closed = False

    def preflight(self) -> RolloutStatus:
        return self._policy.preflight_rollout()

    def status(self) -> RolloutStatus:
        return self._policy.rollout_status()

    def toggle(self) -> RolloutStatus:
        status = self.status()
        method = self._policy.stop_rollout if status["active"] else self._policy.start_rollout
        if not status["active"]:
            ready = self.preflight()
            if not ready["policy_ready"] or not ready["observations_ready"]:
                return ready
        return method()

    def close(self) -> None:
        if not self._closed:
            self._driver.stop()
            self._closed = True


class RolloutApp(App[None]):
    """Attached policy controls with preflight on an explicit start request."""

    CSS_PATH = theme.CSS_PATH
    CSS = CollectionApp.CSS
    BINDINGS = [
        Binding("space", "toggle_rollout", "Start / stop policy"),
        Binding("q", "quit", "Detach"),
        Binding("ctrl+c", "quit", "Detach", show=False),
    ]

    def __init__(self, session: RolloutSession, title: str = "Rollout") -> None:
        super().__init__()
        self._session = session
        self._title = title
        self._status = session.status()
        self._task_label = self._status["task"]
        self._disconnected = False
        self._message = "Attached. Press Space to start or stop the policy."

    def compose(self) -> ComposeResult:
        with Container(id="dashboard"):
            yield Static(self._title.upper(), id="title")
            yield Static(f"TASK  {self._task_label}", id="task")
            yield Static(id="state")
            yield Static(id="guidance")
            yield Static(id="message")
            with Horizontal(id="actions"):
                yield Button("Start policy", id="toggle", variant="success")
                yield Button("Detach", id="stop")
        yield Footer()

    def on_mount(self) -> None:
        self._refresh()
        self.set_interval(0.25, self._poll)

    def on_unmount(self) -> None:
        self._session.close()

    def _refresh(self) -> None:
        active = self._status["active"]
        state = self.query_one("#state", Static)
        state.set_class(active, "recording")
        state.update(
            "DISCONNECTED" if self._disconnected else ("●  POLICY ACTIVE" if active else "READY")
        )
        state.set_class(self._disconnected, "disconnected")
        self.query_one("#guidance", Static).update(
            "Press Space to stop immediately." if active else "Press Space to start the policy."
        )
        error = self._status["last_error"]
        self.query_one("#message", Static).update(
            self._message if self._disconnected else error or self._message
        )
        toggle = self.query_one("#toggle", Button)
        toggle.label = "Stop policy" if active else "Start policy"
        toggle.variant = "error" if active else "success"
        toggle.disabled = self._disconnected

    def _poll(self) -> None:
        if self._disconnected:
            return
        try:
            self._status = self._session.status()
        except Exception as exc:
            self._message = f"Connection lost; policy may still be running: {exc}"
            self._disconnected = True
            self._session.close()
        self._refresh()

    def action_toggle_rollout(self) -> None:
        if self._disconnected:
            return
        try:
            self._status = self._session.toggle()
        except Exception as exc:
            self._message = f"Command failed: {exc}"
        self._refresh()

    def action_quit(self) -> None:  # type: ignore[override]
        self.exit()

    def on_button_pressed(self, event: Button.Pressed) -> None:
        action = {
            "toggle": self.action_toggle_rollout,
            "stop": self.action_quit,
        }.get(event.button.id or "")
        if action is not None:
            action()
