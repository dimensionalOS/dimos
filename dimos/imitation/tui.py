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

"""Attached controls for the capabilities provided by a running learning stack."""

from __future__ import annotations

from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Container, Horizontal
from textual.widgets import Button, Footer, Static

from dimos.cli import theme
from dimos.core.coordination.coordinator_rpc import CoordinatorRPC
from dimos.imitation.collection.episode import EpisodeStatus
from dimos.imitation.collection.episode_monitor import EpisodeCommand, EpisodeControlSpec
from dimos.imitation.policy.lerobot.module import RolloutControlSpec, RolloutStatus
from dimos.porcelain.dimos import Dimos, NoMatchingModuleError


class ControlSession:
    """Discover operator interfaces and own the attached connections."""

    episodes: EpisodeControlSpec | None
    policy: RolloutControlSpec | None

    def __init__(self, driver: Dimos) -> None:
        self._driver = driver
        self._closed = False
        self._coordinator = CoordinatorRPC.connect(timeout=5.0)
        try:
            self._instance_id = self._coordinator.call("instance_id")
            try:
                self.episodes = driver.find_module_by_spec(EpisodeControlSpec)
            except NoMatchingModuleError:
                self.episodes = None
            try:
                self.policy = driver.find_module_by_spec(RolloutControlSpec)
            except NoMatchingModuleError:
                self.policy = None
            if self.episodes is None and self.policy is None:
                raise LookupError("The running stack provides no episode or rollout controls")
        except BaseException:
            self._coordinator.stop()
            raise

    def episode_status(self) -> EpisodeStatus | None:
        if self.episodes is None:
            return None
        status = self.episodes.get_status()
        if not isinstance(status, EpisodeStatus):
            raise TypeError("Episode controller returned an invalid status")
        return status

    def policy_status(self) -> RolloutStatus | None:
        return None if self.policy is None else self.policy.rollout_status()

    def episode_command(self, event: EpisodeCommand) -> EpisodeStatus:
        if self.episodes is None:
            raise RuntimeError("This stack has no episode controls")
        return self.episodes.command(event)

    def start_policy(self) -> RolloutStatus:
        if self.policy is None:
            raise RuntimeError("This stack has no policy controls")
        ready = self.policy.preflight_rollout()
        if not ready["policy_ready"] or not ready["observations_ready"]:
            return ready
        return self.policy.start_rollout()

    def stop_policy(self) -> RolloutStatus:
        if self.policy is None:
            raise RuntimeError("This stack has no policy controls")
        return self.policy.stop_rollout()

    def stop_stack(self) -> None:
        status = self.episode_status()
        if status is not None and status.state == "recording":
            raise RuntimeError("Save or discard the active episode before stopping the stack")
        self._coordinator.call("shutdown", expected_instance_id=self._instance_id)

    def close(self) -> None:
        if not self._closed:
            self._coordinator.stop()
            self._driver.stop()
            self._closed = True


class ControlApp(App[None]):
    """One operator panel with explicit episode, policy, and runtime actions."""

    CSS_PATH = theme.CSS_PATH
    CSS = f"""
    Screen {{ align: center middle; background: {theme.BACKGROUND}; }}
    #dashboard {{ width: 90; max-width: 100%; height: auto; max-height: 100%;
        overflow-y: auto; padding: 1 2; border: double {theme.BORDER};
        background: {theme.BG}; }}
    #title {{ height: 1; text-align: center; color: {theme.ACCENT}; text-style: bold; }}
    .status {{ height: auto; min-height: 2; margin-top: 1; }}
    .actions {{ height: 3; }}
    Button {{ width: 1fr; min-width: 10; }}
    #message {{ height: auto; min-height: 2; color: {theme.WARNING}; }}
    #confirmation {{ height: auto; }}
    """
    BINDINGS = [
        Binding("q", "quit", "Detach"),
        Binding("ctrl+c", "quit", "Detach", show=False),
    ]

    def __init__(self, session: ControlSession) -> None:
        super().__init__()
        self._session = session
        self._episode = session.episode_status()
        self._policy = session.policy_status()
        self._disconnected = False
        self._message = "Attached. Detaching leaves the stack running."
        self._confirming_stop = False
        self._quit_armed = False

    def compose(self) -> ComposeResult:
        with Container(id="dashboard"):
            yield Static("LEARNING CONTROLS", id="title")
            if self._episode is not None:
                yield Static(id="episode-status", classes="status")
                with Horizontal(classes="actions"):
                    yield Button("Start episode", id="start-episode", variant="success")
                    yield Button("Save episode", id="save-episode")
                    yield Button("Discard", id="discard-episode", variant="error")
            if self._policy is not None:
                yield Static(id="policy-status", classes="status")
                with Horizontal(classes="actions"):
                    yield Button("Start policy", id="start-policy", variant="success")
                    yield Button("Stop policy", id="stop-policy", variant="error")
            yield Static(id="message")
            with Container(id="confirmation"):
                yield Static("Support the arm(s). Shut down the entire stack?")
                with Horizontal(classes="actions"):
                    yield Button("Confirm shutdown", id="confirm-stop", variant="error")
                    yield Button("Cancel", id="cancel-stop")
            with Horizontal(classes="actions"):
                yield Button("Detach", id="detach")
                yield Button("Stop stack…", id="stop-stack", variant="error")
        yield Footer()

    def on_mount(self) -> None:
        self._refresh()
        self.set_interval(0.25, self._poll)

    def on_unmount(self) -> None:
        self._session.close()

    def _refresh(self) -> None:
        blocked = self._disconnected or self._confirming_stop
        if self._episode is not None:
            episode = self._episode
            recording = episode.state == "recording"
            self.query_one("#episode-status", Static).update(
                f"EPISODES | {episode.state.upper()} | {episode.task_label}\n"
                f"Saved: {episode.episodes_saved}   Discarded: {episode.episodes_discarded}"
            )
            self.query_one("#start-episode", Button).disabled = blocked or recording
            self.query_one("#save-episode", Button).disabled = blocked or not recording
            self.query_one("#discard-episode", Button).disabled = blocked or not recording
        if self._policy is not None:
            active = self._policy["active"]
            self.query_one("#policy-status", Static).update(
                f"POLICY | {'ACTIVE' if active else 'STOPPED'} | {self._policy['task']}\n"
                f"{self._policy['last_error'] or ''}"
            )
            self.query_one("#start-policy", Button).disabled = blocked or active
            self.query_one("#stop-policy", Button).disabled = blocked or not active
        self.query_one("#message", Static).update(self._message)
        self.query_one("#confirmation").display = self._confirming_stop
        self.query_one("#stop-stack", Button).disabled = self._disconnected

    def _poll(self) -> None:
        if self._disconnected:
            return
        try:
            self._episode = self._session.episode_status()
            self._policy = self._session.policy_status()
        except Exception as exc:
            self._disconnected = True
            self._confirming_stop = False
            self._message = f"Connection lost; robot and recording may still be running: {exc}"
            self._session.close()
        self._refresh()

    def action_quit(self) -> None:  # type: ignore[override]
        if (
            self._episode is not None
            and self._episode.state == "recording"
            and not self._quit_armed
        ):
            self._quit_armed = True
            self._message = "Recording will continue. Press Q again to detach."
            self._refresh()
            return
        self.exit()

    def on_button_pressed(self, event: Button.Pressed) -> None:
        action = event.button.id
        if action == "detach":
            self.action_quit()
            return
        if action == "cancel-stop":
            self._confirming_stop = False
            self._refresh()
            return
        if self._disconnected:
            return
        try:
            if action in {"start-episode", "save-episode", "discard-episode"}:
                commands: dict[str, EpisodeCommand] = {
                    "start-episode": "start",
                    "save-episode": "save",
                    "discard-episode": "discard",
                }
                self._episode = self._session.episode_command(commands[action])
                self._message = f"Episode: {self._episode.last_event}."
                self._quit_armed = False
            elif action == "start-policy":
                self._policy = self._session.start_policy()
            elif action == "stop-policy":
                self._policy = self._session.stop_policy()
            elif action == "stop-stack":
                self._episode = self._session.episode_status()
                if self._episode is not None and self._episode.state == "recording":
                    self._message = "Save or discard the active episode before stopping the stack."
                else:
                    self._confirming_stop = True
            elif action == "confirm-stop":
                self._session.stop_stack()
                self._confirming_stop = False
                self._disconnected = True
                self._message = "Stack shutdown requested. Detach to close this panel."
                self._session.close()
        except Exception as exc:
            self._confirming_stop = False
            self._message = f"Command failed: {exc}"
        self._refresh()
