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

from typing import Any, cast

from rich.panel import Panel
from rich.text import Text
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.widgets import Footer, Static
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
        self._client.stop()


class TeachCollectionApp(App[None]):
    """Small keyboard panel for teach collection."""

    CSS = f"""
    Screen {{
        align: center middle;
        background: {theme.BACKGROUND};
    }}
    #status {{
        width: 72;
        height: auto;
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
        self._message = "Move the arm and gripper by hand; press Space when the take begins."
        self._detached = False

    def compose(self) -> ComposeResult:
        yield Static(self._render(), id="status")
        yield Footer()

    def on_mount(self) -> None:
        self.set_interval(0.25, self._poll)

    def on_unmount(self) -> None:
        self._session.close()

    def _render(self) -> Panel:
        recording = self._status.state == "recording"
        state_style = theme.ERROR if recording else theme.SUCCESS
        body = Text()
        body.append("Task       ", style="bold")
        body.append(f"{self._status.task_label}\n")
        body.append("State      ", style="bold")
        body.append(f"{self._status.state.upper()}\n", style=f"bold {state_style}")
        body.append("Episodes   ", style="bold")
        body.append(
            f"{self._status.episodes_saved} saved, {self._status.episodes_discarded} discarded\n"
        )
        body.append("Gripper    ", style="bold")
        body.append("passive — move by hand\n\n")
        body.append(self._message)
        if self._detached:
            body.append(
                "\n\nRPC connection closed; the daemon and arm are still running.",
                style=theme.ERROR,
            )
        return Panel(body, title="OpenYAM teach collection", border_style=theme.BORDER)

    def _refresh(self) -> None:
        self.query_one("#status", Static).update(self._render())

    def _poll(self) -> None:
        if self._detached:
            return
        try:
            self._status = self._session.get_status()
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
            self._status = self._session.command(event)
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
        self._episode_command("discard")

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
