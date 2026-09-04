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

"""CLI-first imitation-learning workflow."""

from __future__ import annotations

from datetime import datetime
import json
from pathlib import Path
import subprocess
import time
from typing import Any, cast

from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Container, Horizontal
from textual.widgets import Button, Footer, Static
import typer

from dimos.cli import theme
from dimos.constants import DIMOS_PROJECT_ROOT, STATE_DIR
from dimos.core.run_registry import list_runs
from dimos.imitation.dataprep.build import inspect_dataset, inspect_recording
from dimos.imitation.dataprep.core import DataPrepConfig, OutputConfig
from dimos.imitation.policy.module import POLICY_ROLLOUT_INSTANCE_NAME, RolloutStatus
from dimos.imitation.profile import ImageSource, PolicyIOProfile
from dimos.imitation.workflows import (
    COLLECTION_WORKFLOWS,
    ROLLOUT_WORKFLOWS,
    CollectionWorkflow,
    RolloutWorkflow,
    get_collection_workflow,
    get_rollout_workflow,
)
from dimos.msgs.imitation_msgs.EpisodeStatus import EpisodeStatus
from dimos.porcelain.dimos import Dimos

imitation_app = typer.Typer(help="Collect, prepare, train, and run imitation policies")

_MONITOR = "EpisodeMonitorModule"
_POLICY = POLICY_ROLLOUT_INSTANCE_NAME


def _collection_workflow(value: str) -> CollectionWorkflow:
    try:
        return get_collection_workflow(value)
    except ValueError as exc:
        raise typer.BadParameter(str(exc)) from exc


def _rollout_workflow(value: str) -> RolloutWorkflow:
    try:
        return get_rollout_workflow(value)
    except ValueError as exc:
        raise typer.BadParameter(str(exc)) from exc


def _camera_device(value: str) -> int | str:
    return int(value) if value.isdecimal() else value


def _default_recording(workflow: CollectionWorkflow) -> Path:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    return STATE_DIR / "recordings" / f"{workflow.name}_{timestamp}.mcap"


def _camera_devices(values: list[str], profile: PolicyIOProfile) -> dict[str, int | str]:
    required = {
        source.stream for source in profile.observations.values() if isinstance(source, ImageSource)
    }
    devices: dict[str, int | str] = {}
    for value in values:
        try:
            stream, device = value.split("=", 1)
        except ValueError as exc:
            raise typer.BadParameter("--camera must use STREAM=DEVICE") from exc
        if not stream or not device:
            raise typer.BadParameter("--camera must use non-empty STREAM=DEVICE values")
        if stream in devices:
            raise typer.BadParameter(f"camera stream {stream!r} was declared more than once")
        devices[stream] = _camera_device(device)
    missing = sorted(required - set(devices))
    unknown = sorted(set(devices) - required)
    if missing:
        raise typer.BadParameter(f"missing --camera declarations for: {', '.join(missing)}")
    if unknown:
        raise typer.BadParameter(f"unknown camera streams: {', '.join(unknown)}")
    return devices


def _dual_can_kwargs(
    *,
    enabled: bool,
    left_can_port: str | None,
    right_can_port: str | None,
) -> dict[str, str]:
    configured = left_can_port is not None or right_can_port is not None
    if not enabled:
        if configured:
            raise typer.BadParameter("CAN-pair options apply only to Dual OpenYAM workflows")
        return {}
    if left_can_port is None or right_can_port is None:
        raise typer.BadParameter("Dual OpenYAM requires both --left-can-port and --right-can-port")
    return {"left_can_port": left_can_port, "right_can_port": right_can_port}


def _default_dataset(recording: Path) -> Path:
    return STATE_DIR / "datasets" / recording.stem


def _require_new_path(path: Path, kind: str) -> Path:
    resolved = path.expanduser().resolve()
    if resolved.exists():
        raise typer.BadParameter(f"{kind} already exists: {resolved}")
    return resolved


def _require_idle_coordinator() -> None:
    runs = list_runs()
    if runs:
        names = ", ".join(run.run_id for run in runs)
        raise RuntimeError(f"another DimOS run is active: {names}; stop it before continuing")
    client: Dimos | None = None
    try:
        client = Dimos.connect(timeout=0.25)
        client.list_modules()
    except Exception:
        if client is not None:
            client.stop()
        return
    client.stop()
    raise RuntimeError("another DimOS coordinator is active; stop it before continuing")


class CollectionSession:
    """Operator RPCs for a CLI-owned collection stack."""

    def __init__(self, driver: Dimos) -> None:
        self._driver = driver
        self._monitor = cast("Any", driver.get_module(_MONITOR))
        self._closed = False
        self.get_status()

    def get_status(self) -> EpisodeStatus:
        status = self._monitor.get_status()
        if not isinstance(status, EpisodeStatus):
            raise RuntimeError(f"episode monitor returned {type(status).__name__}")
        return status

    def command(self, event: str) -> EpisodeStatus:
        status = self._monitor.command(event)
        if not isinstance(status, EpisodeStatus):
            raise RuntimeError(f"episode monitor returned {type(status).__name__}")
        return status

    def close(self) -> None:
        if not self._closed:
            self._driver.stop()
            self._closed = True


class CollectionApp(App[None]):
    """Episode controls for a CLI-owned collection session."""

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
        Binding("q", "quit", "Stop"),
        Binding("ctrl+c", "force_quit", "Stop", show=False),
    ]

    def __init__(self, session: CollectionSession, workflow_name: str) -> None:
        super().__init__()
        self._session = session
        self._workflow_name = workflow_name
        self._status = session.get_status()
        self._message = "Reset the scene, then start a take."
        self._disconnected = False
        self._recording_started_at: float | None = None
        self._quit_armed = False

    def compose(self) -> ComposeResult:
        with Container(id="dashboard"):
            yield Static(self._workflow_name.upper(), id="title")
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
                yield Button("Stop", id="stop")
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
        self.query_one("#stop", Button).disabled = recording and not self._disconnected

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

    def _episode_command(self, event: str) -> None:
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
        if self._status.state == "recording":
            self._message = "Save with Space or discard with D before stopping."
            self._refresh()
            return
        if not self._quit_armed:
            self._quit_armed = True
            self._message = "Press Q again to stop; the arm will de-torque."
            self._refresh()
            return
        self.exit()

    def action_force_quit(self) -> None:
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
    """Operator RPCs for a CLI-owned policy rollout stack."""

    def __init__(self, driver: Dimos) -> None:
        self._driver = driver
        self._policy = cast("Any", driver.get_module(_POLICY))
        self._closed = False

    def preflight(self) -> RolloutStatus:
        return cast("RolloutStatus", self._policy.preflight_rollout())

    def status(self) -> RolloutStatus:
        return cast("RolloutStatus", self._policy.rollout_status())

    def toggle(self) -> RolloutStatus:
        status = self.status()
        method = self._policy.stop_rollout if status["active"] else self._policy.start_rollout
        return cast("RolloutStatus", method())

    def close(self) -> None:
        if not self._closed:
            try:
                self._policy.stop_rollout()
            finally:
                self._driver.stop()
                self._closed = True


class RolloutApp(App[None]):
    """Minimal terminal controls for a preflighted policy rollout."""

    CSS_PATH = theme.CSS_PATH
    CSS = CollectionApp.CSS
    BINDINGS = [
        Binding("space", "toggle_rollout", "Start / stop policy"),
        Binding("q", "quit", "Stop stack"),
        Binding("ctrl+c", "force_quit", "Stop stack", show=False),
    ]

    def __init__(self, session: RolloutSession, workflow_name: str, task: str) -> None:
        super().__init__()
        self._session = session
        self._workflow_name = workflow_name
        self._task_label = task
        self._status = session.status()
        self._message = "Preflight passed. Press Space to start the policy."
        self._quit_armed = False

    def compose(self) -> ComposeResult:
        with Container(id="dashboard"):
            yield Static(f"{self._workflow_name.upper()} ROLLOUT", id="title")
            yield Static(f"TASK  {self._task_label}", id="task")
            yield Static(id="state")
            yield Static(id="guidance")
            yield Static(id="message")
            with Horizontal(id="actions"):
                yield Button("Start policy", id="toggle", variant="success")
                yield Button("Stop stack", id="stop")
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
        state.update("●  POLICY ACTIVE" if active else "READY")
        self.query_one("#guidance", Static).update(
            "Press Space to stop immediately." if active else "Press Space to start the policy."
        )
        error = self._status["last_error"]
        self.query_one("#message", Static).update(error or self._message)
        toggle = self.query_one("#toggle", Button)
        toggle.label = "Stop policy" if active else "Start policy"
        toggle.variant = "error" if active else "success"

    def _poll(self) -> None:
        self._status = self._session.status()
        self._refresh()

    def action_toggle_rollout(self) -> None:
        self._status = self._session.toggle()
        self._quit_armed = False
        self._refresh()

    def action_quit(self) -> None:  # type: ignore[override]
        if self._status["active"]:
            self._message = "Stop the policy with Space before stopping the stack."
            self._refresh()
            return
        if not self._quit_armed:
            self._quit_armed = True
            self._message = "Press Q again to stop; the arm will de-torque."
            self._refresh()
            return
        self.exit()

    def action_force_quit(self) -> None:
        self.exit()

    def on_button_pressed(self, event: Button.Pressed) -> None:
        action = {
            "toggle": self.action_toggle_rollout,
            "stop": self.action_quit,
        }.get(event.button.id or "")
        if action is not None:
            action()


@imitation_app.command("list")
def list_imitation_workflows() -> None:
    """List built-in workflows without importing robot hardware modules."""
    typer.echo("Collection workflows:")
    for collection_workflow in COLLECTION_WORKFLOWS.values():
        hardware = ", ".join(collection_workflow.required_hardware)
        typer.echo(
            f"{collection_workflow.name}\n"
            f"  method: {collection_workflow.method}\n"
            f"  hardware: {hardware}"
        )
    typer.echo("Rollout workflows:")
    for rollout_workflow in ROLLOUT_WORKFLOWS.values():
        hardware = ", ".join(rollout_workflow.required_hardware)
        typer.echo(
            f"{rollout_workflow.name}\n"
            f"  backend: {rollout_workflow.backend}\n"
            f"  hardware: {hardware}"
        )


@imitation_app.command()
def collect(
    workflow_name: str = typer.Argument(..., metavar="WORKFLOW"),
    task: str = typer.Option(..., "--task", help="Demonstration task description"),
    recording: Path | None = typer.Option(None, "--recording", help="New MCAP recording path"),
    camera: list[str] = typer.Option(
        [],
        "--camera",
        help="Repeat for every profile camera as STREAM=DEVICE",
    ),
    left_can_port: str | None = typer.Option(None, "--left-can-port"),
    right_can_port: str | None = typer.Option(None, "--right-can-port"),
) -> None:
    """Collect demonstrations and own the robot stack for the full session."""
    workflow = _collection_workflow(workflow_name)
    profile = workflow.load_profile()
    if not isinstance(profile, PolicyIOProfile):
        raise TypeError(f"collection workflow {workflow.name!r} has an invalid profile")
    cameras = _camera_devices(camera, profile)
    can_kwargs = _dual_can_kwargs(
        enabled=workflow.dual_can,
        left_can_port=left_can_port,
        right_can_port=right_can_port,
    )
    path = _require_new_path(recording or _default_recording(workflow), "recording")
    if not task.strip():
        raise typer.BadParameter("--task must not be blank")
    driver: Dimos | None = None
    try:
        _require_idle_coordinator()
        path.parent.mkdir(parents=True, exist_ok=True)
        builder = workflow.load_builder()
        blueprint = builder(
            recording=path,
            task=task.strip(),
            cameras=cameras,
            **can_kwargs,
        )
        typer.echo(f"Recording: {path}")
        typer.echo("Safety: stopping this command de-torques the arm. Keep the robot supported.")
        driver = Dimos()
        driver.run(blueprint)
        session = CollectionSession(driver)
        try:
            CollectionApp(session, workflow.name).run()
        finally:
            session.close()
    except Exception as exc:
        typer.echo(f"collection failed: {exc}", err=True)
        raise typer.Exit(1) from exc
    finally:
        if driver is not None:
            driver.stop()


def _dataprep_config(
    workflow: CollectionWorkflow,
    source: Path,
    output: Path,
) -> DataPrepConfig:
    profile = workflow.load_profile()
    if not isinstance(profile, PolicyIOProfile):
        raise TypeError(f"collection workflow {workflow.name!r} has an invalid profile")
    return profile.dataprep_config(
        source=str(source),
        output=OutputConfig(
            format="lerobot",
            path=output,
            metadata={"repo_id": f"local/{profile.name}", "robot_type": profile.robot_type},
        ),
    )


@imitation_app.command()
def prepare(
    workflow_name: str = typer.Argument(..., metavar="WORKFLOW"),
    recording: Path = typer.Argument(..., metavar="RECORDING"),
    output: Path | None = typer.Option(None, "--output", help="New LeRobot dataset directory"),
) -> None:
    """Convert one recording into a strict LeRobot dataset."""
    workflow = _collection_workflow(workflow_name)
    source = recording.expanduser().resolve()
    target = _require_new_path(output or _default_dataset(source), "dataset")
    typer.echo(f"Recording: {source}")
    typer.echo(f"Dataset: {target}")
    try:
        from dimos.imitation.dataprep.lerobot import run_lerobot_dataprep

        result = run_lerobot_dataprep(_dataprep_config(workflow, source, target))
    except Exception as exc:
        typer.echo(f"prepare failed: {exc}", err=True)
        raise typer.Exit(1) from exc
    typer.echo(f"Wrote dataset: {result}")


@imitation_app.command()
def inspect(
    artifact: Path = typer.Argument(..., metavar="ARTIFACT"),
    workflow_name: str | None = typer.Option(None, "--workflow", help="Validate a recording"),
) -> None:
    """Summarize a recording or prepared dataset."""
    path = artifact.expanduser().resolve()
    try:
        if workflow_name is not None and path.suffix.lower() in {".mcap", ".db"}:
            workflow = _collection_workflow(workflow_name)
            profile = workflow.load_profile()
            if not isinstance(profile, PolicyIOProfile):
                raise TypeError(f"collection workflow {workflow.name!r} has an invalid profile")
            config = profile.dataprep_config(
                source=str(path),
                output=OutputConfig(path=_default_dataset(path)),
            )
            info = inspect_recording(path, config=config)
        else:
            info = inspect_dataset(path)
    except Exception as exc:
        typer.echo(f"inspect failed: {exc}", err=True)
        raise typer.Exit(1) from exc
    typer.echo(json.dumps(info, indent=2, default=str))


@imitation_app.command(
    context_settings={
        "allow_extra_args": True,
        "ignore_unknown_options": True,
        "help_option_names": [],
    }
)
def train(ctx: typer.Context) -> None:
    """Pass all arguments directly to ``lerobot-train``."""
    project = DIMOS_PROJECT_ROOT / "dimos" / "imitation" / "policy" / "lerobot" / "python"
    command = ["uv", "run", "--project", str(project), "--frozen", "lerobot-train", *ctx.args]
    result = subprocess.run(command, check=False)
    if result.returncode:
        raise typer.Exit(result.returncode)


@imitation_app.command("run")
def run_policy(
    workflow_name: str = typer.Argument(..., metavar="WORKFLOW"),
    artifact: Path = typer.Argument(..., metavar="ARTIFACT"),
    task: str = typer.Option(..., "--task", help="Task conditioning text"),
    camera: list[str] = typer.Option(
        [],
        "--camera",
        help="Repeat for every profile camera as STREAM=DEVICE",
    ),
    device: str | None = typer.Option(
        None, "--device", help="Inference device, such as cuda or cpu"
    ),
    quest_control: bool = typer.Option(False, "--quest-control", help="Enable Quest takeover"),
    left_can_port: str | None = typer.Option(None, "--left-can-port"),
    right_can_port: str | None = typer.Option(None, "--right-can-port"),
) -> None:
    """Preflight and run a trained policy with terminal controls."""
    workflow = _rollout_workflow(workflow_name)
    profile = workflow.load_profile()
    if not isinstance(profile, PolicyIOProfile):
        raise TypeError(f"rollout workflow {workflow.name!r} has an invalid profile")
    cameras = _camera_devices(camera, profile)
    can_kwargs = _dual_can_kwargs(
        enabled=workflow.dual_can,
        left_can_port=left_can_port,
        right_can_port=right_can_port,
    )
    if not task.strip():
        raise typer.BadParameter("--task must not be blank")
    driver: Dimos | None = None
    try:
        _require_idle_coordinator()
        builder = workflow.load_builder()
        blueprint = builder(
            artifact=str(artifact.expanduser().resolve()),
            task=task.strip(),
            cameras=cameras,
            device=device,
            quest_control=quest_control,
            **can_kwargs,
        )
        typer.echo("Safety: stopping this command de-torques the arm. Keep the robot supported.")
        typer.echo("Running non-moving policy preflight...")
        driver = Dimos()
        driver.run(blueprint)
        session = RolloutSession(driver)
        status = session.preflight()
        if not status["policy_ready"] or not status["observations_ready"]:
            raise RuntimeError(status["last_error"] or "policy preflight failed")
        typer.echo("Preflight passed. No trajectory has been sent.")
        try:
            RolloutApp(session, workflow.name, task.strip()).run()
        finally:
            session.close()
    except Exception as exc:
        typer.echo(f"rollout failed: {exc}", err=True)
        raise typer.Exit(1) from exc
    finally:
        if driver is not None:
            driver.stop()
