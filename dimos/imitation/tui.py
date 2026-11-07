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

import asyncio

from textual import work
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Container, Horizontal
from textual.widgets import Button, Footer, Input, Static

from dimos.cli import theme
from dimos.core.coordination.coordinator_rpc import CoordinatorRPC
from dimos.imitation.collection.episode import EpisodeStatus
from dimos.imitation.collection.episode_monitor import EpisodeCommand, EpisodeControlSpec
from dimos.imitation.collection.prompts import CollectionSpeech
from dimos.imitation.policy.module import RolloutControlSpec, RolloutStatus
from dimos.porcelain.dimos import Dimos, NoMatchingModuleError
from dimos.stream.audio.wav_player import WavPlayer


class ControlSession:
    """Discover operator interfaces and own the attached connections."""

    episodes: EpisodeControlSpec | None
    policy: RolloutControlSpec | None

    def __init__(self, driver: Dimos) -> None:
        self._driver = driver
        self._closed = False
        self._coordinator = CoordinatorRPC.connect(timeout=5.0)
        try:
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

    def start_policy(
        self, dispatch_actions: bool = False, max_chunks: int | None = 1
    ) -> RolloutStatus:
        if self.policy is None:
            raise RuntimeError("This stack has no policy controls")
        ready = self.policy.preflight_rollout()
        if not ready["policy_ready"] or not ready["observations_ready"]:
            return ready
        return self.policy.start_rollout(dispatch_actions=dispatch_actions, max_chunks=max_chunks)

    def configure_policy(self, task: str, policy_path: str) -> RolloutStatus:
        if self.policy is None:
            raise RuntimeError("This stack has no policy controls")
        if not task.strip() or not policy_path.strip():
            raise ValueError("Model and task must not be blank")
        current = self.policy.rollout_status()
        if current["active"] or current.get("busy", False):
            raise RuntimeError("Stop rollout and wait for it to drain before editing")
        if policy_path != current["policy_path"]:
            self.policy.set_policy_path(policy_path)
        if task != current["task"]:
            self.policy.set_task(task)
        return self.policy.rollout_status()

    def stop_policy(self) -> RolloutStatus:
        if self.policy is None:
            raise RuntimeError("This stack has no policy controls")
        return self.policy.stop_rollout()

    def stop_stack(self) -> None:
        status = self.episode_status()
        if status is not None and status.state == "recording":
            raise RuntimeError("Save or discard the active episode before stopping the stack")
        self._coordinator.call("shutdown")

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
        Binding("f8", "stop_rollout", "Stop rollout", priority=True),
        Binding("ctrl+c", "quit", "Detach", show=False, priority=True),
    ]

    def __init__(self, session: ControlSession, *, speech: CollectionSpeech | None = None) -> None:
        super().__init__()
        self._session = session
        self._episode = session.episode_status()
        self._policy = session.policy_status()
        self._speech = speech
        self._player = WavPlayer() if speech is not None else None
        self._audio_error: str | None = None
        if speech is not None and self._episode is not None:
            speech.update(self._episode, snapshot=True)
        self._disconnected = False
        self._message = "Attached. Detaching leaves the stack running."
        self._confirming_stop = False
        self._quit_armed = False
        self._command_pending = False
        self._poll_pending = False
        self._revision = 0

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
                yield Static(id="policy-status", classes="status", markup=False)
                model = Input(
                    value=self._policy.get("policy_path", ""),
                    placeholder="Model checkpoint",
                    id="policy-model",
                )
                model.border_title = "Model checkpoint"
                yield model
                task = Input(
                    value=self._policy["task"], placeholder="Task prompt", id="policy-task"
                )
                task.border_title = "Task prompt"
                yield task
                yield Button("Apply model / task", id="apply-policy")
                yield Static("Preview uploads images/state. Execute moves the robot.", markup=False)
                with Horizontal(classes="actions"):
                    yield Button("Preview once", id="start-policy")
                    yield Button("Execute 1 chunk", id="execute-once", variant="warning")
                    yield Button("Execute continuous", id="execute-continuous", variant="success")
                yield Button("Stop rollout", id="stop-policy", variant="error")
                yield Static(id="preview-summary", classes="status", markup=False)
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
        self.stop_audio()
        self._session.close()

    def stop_audio(self) -> None:
        if self._player is not None:
            self._player.stop()

    def _set_episode_status(self, status: EpisodeStatus | None) -> None:
        self._episode = status
        if self._speech is not None and self._player is not None and status is not None:
            audio = self._speech.update(status)
            if audio is not None:
                try:
                    self._player.play(audio)
                    self._audio_error = None
                except Exception as exc:
                    self._audio_error = f"Audio unavailable: {exc}"

    def _refresh(self) -> None:
        blocked = self._disconnected or self._confirming_stop or self._command_pending
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
            policy = self._policy
            active = policy["active"] or policy.get("busy", False)
            phase = policy.get("phase", "active" if active else "stopped").upper()
            retry = policy.get("retry_in_s")
            retry_text = f" | retry in {retry:.0f}s (preview only)" if retry is not None else ""
            if policy.get("execution_requires_restart"):
                retry_text += " | cloud retry switched to preview; Execute again to move"
            camera_text = "  ".join(
                f"{name}: {'PRESENT' if ready else 'MISSING'}"
                for name, ready in policy.get("cameras", {}).items()
            )
            self.query_one("#policy-status", Static).update(
                f"ROLLOUT | {policy.get('backend', 'policy')} | {phase} | {'EXECUTE' if policy.get('dispatch_actions') else 'PREVIEW'}{retry_text}\n"
                f"Received {policy.get('chunks_received', 0)}  Submitted {policy.get('chunks_accepted', 0)}  Finished {policy.get('chunks_completed', 0)}\n"
                f"{camera_text}\n{policy['last_error'] or ''}"
            )
            dirty = (
                self.query_one("#policy-model", Input).value != policy.get("policy_path", "")
                or self.query_one("#policy-task", Input).value != policy["task"]
            )
            for name in (
                "start-policy",
                "execute-once",
                "execute-continuous",
                "apply-policy",
                "policy-model",
                "policy-task",
            ):
                self.query_one(f"#{name}").disabled = (
                    blocked
                    or active
                    or (dirty and name in {"start-policy", "execute-once", "execute-continuous"})
                )
            self.query_one("#stop-policy", Button).disabled = blocked or not active
            debug = []
            if policy.get("check_observation_timing") is False:
                debug.append("observation timing OFF")
            if policy.get("require_endpoint_convergence") is False:
                debug.append("endpoint convergence OFF; Finished means elapsed, not pose accuracy")
            self.query_one("#preview-summary", Static).update(
                policy.get("preview_summary", "No preview yet.")
                + ("\nDebug: " + "; ".join(debug) if debug else "")
            )
        self.query_one("#message", Static).update(self._audio_error or self._message)
        self.query_one("#confirmation").display = self._confirming_stop
        self.query_one("#stop-stack", Button).disabled = self._disconnected

    def _poll(self) -> None:
        if self._disconnected or self._poll_pending or self._command_pending:
            return
        self._poll_pending = True
        self._poll_worker()

    @work(group="status")
    async def _poll_worker(self) -> None:
        revision = self._revision
        try:
            episode = await asyncio.to_thread(self._session.episode_status)
            policy = await asyncio.to_thread(self._session.policy_status)
            if revision == self._revision and not self._command_pending and not self._disconnected:
                self._set_episode_status(episode)
                self._policy = policy
        except Exception as exc:
            if revision == self._revision and not self._command_pending:
                self._disconnected = True
                self._confirming_stop = False
                self._message = f"Connection lost; robot and recording may still be running: {exc}"
                self.stop_audio()
        finally:
            self._poll_pending = False
            self._refresh()

    def action_stop_rollout(self) -> None:
        if self._policy is not None:
            self.query_one("#stop-policy", Button).press()

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
        if self._disconnected or self._command_pending:
            return
        self._revision += 1
        self._command_pending = True
        self._message = (
            "Checking inputs and policy…"
            if action in {"start-policy", "execute-once", "execute-continuous"}
            else "Working…"
        )
        self._refresh()
        self._command(action)

    def on_input_changed(self, event: Input.Changed) -> None:
        if self.is_mounted(event.input) and event.input.id in {"policy-model", "policy-task"}:
            self._refresh()

    def on_input_submitted(self, event: Input.Submitted) -> None:
        if event.input.id in {"policy-model", "policy-task"}:
            self.query_one("#apply-policy", Button).press()

    @work(group="command")
    async def _command(self, action: str | None) -> None:
        try:
            if action in {"start-episode", "save-episode", "discard-episode"}:
                commands: dict[str, EpisodeCommand] = {
                    "start-episode": "start",
                    "save-episode": "save",
                    "discard-episode": "discard",
                }
                status = await asyncio.to_thread(self._session.episode_command, commands[action])
                self._set_episode_status(status)
                self._message = f"Episode: {status.last_event}."
                self._quit_armed = False
            elif action in {"start-policy", "execute-once", "execute-continuous"}:
                self._policy = await asyncio.to_thread(
                    self._session.start_policy,
                    action != "start-policy",
                    None if action == "execute-continuous" else 1,
                )
                self._message = (
                    "Rollout requested. Stop cancels policy work; it is not an emergency stop."
                )
            elif action == "apply-policy":
                self._policy = await asyncio.to_thread(
                    self._session.configure_policy,
                    self.query_one("#policy-task", Input).value,
                    self.query_one("#policy-model", Input).value,
                )
                self._message = (
                    "Configuration applied. Next start runs preflight and fresh inference."
                )
            elif action == "stop-policy":
                self._policy = await asyncio.to_thread(self._session.stop_policy)
            elif action == "stop-stack":
                self._set_episode_status(await asyncio.to_thread(self._session.episode_status))
                if self._episode is not None and self._episode.state == "recording":
                    self._message = "Save or discard the active episode before stopping the stack."
                else:
                    self._confirming_stop = True
            elif action == "confirm-stop":
                await asyncio.to_thread(self._session.stop_stack)
                self._confirming_stop = False
                self._disconnected = True
                self._message = "Stack shutdown requested. Detach to close this panel."
                self._session.close()
        except Exception as exc:
            self._confirming_stop = False
            self._message = f"Command failed: {exc}"
        self._command_pending = False
        self._refresh()
