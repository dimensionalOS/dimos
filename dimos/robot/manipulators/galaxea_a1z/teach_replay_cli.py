# Copyright 2025-2026 Dimensional Inc.
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

"""Foreground A1Z hand-teach and replay commands."""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
import time
from typing import Any

import typer

from dimos.constants import STATE_DIR

app = typer.Typer(help="Record and replay Galaxea A1Z hand-taught episodes")


def _default_recording_path() -> Path:
    return STATE_DIR / "recordings" / f"a1z_teach_{datetime.now():%Y%m%d_%H%M%S}.db"


def _press_enter(message: str) -> None:
    typer.prompt(message, default="", show_default=False)


@app.command()
def teach(
    output: Path | None = typer.Argument(
        None,
        help="Memory2 .db output (default: timestamped file under the DimOS state directory)",
    ),
    task: str | None = typer.Option(None, "--task", help="Task label stored with each episode"),
) -> None:
    """Hand-teach episodes into one Memory2 recording."""
    from dimos.core.coordination.module_coordinator import ModuleCoordinator
    from dimos.learning.collection.episode_monitor import EpisodeMonitorModule
    from dimos.robot.manipulators.galaxea_a1z.blueprints.basic import (
        make_a1z_teach_blueprint,
    )

    db_path = (output or _default_recording_path()).expanduser().resolve()
    if db_path.exists():
        typer.echo(f"error: refusing to overwrite existing recording: {db_path}", err=True)
        raise typer.Exit(2)

    typer.echo("A1Z hand-teach mode")
    typer.echo(f"Recording: {db_path}")
    typer.echo("The arm and gripper will become hand-drivable after startup.")
    typer.echo("Keep the arm supported: it has no brakes and can fall when motors disable.\n")

    coordinator: ModuleCoordinator | None = None
    recording = False
    try:
        coordinator = ModuleCoordinator.build(
            make_a1z_teach_blueprint(db_path, task_label=task),
            {},
        )
        monitor: Any = coordinator.get_instance(EpisodeMonitorModule)
        typer.echo("Ready. Move only after starting an episode.")

        while True:
            if not recording:
                command = (
                    typer.prompt(
                        "Press ENTER to start an episode, or q to finish",
                        default="",
                        show_default=False,
                    )
                    .strip()
                    .lower()
                )
                if command == "q":
                    break
                if command:
                    typer.echo("Use ENTER to start or q to finish.")
                    continue
                monitor.start_episode()
                recording = True
                typer.echo("RECORDING — move the arm and gripper by hand.")
                continue

            command = (
                typer.prompt(
                    "Press ENTER to save, d to discard, or q to discard and finish",
                    default="",
                    show_default=False,
                )
                .strip()
                .lower()
            )
            if command == "d":
                monitor.discard_episode()
                recording = False
                typer.echo("Episode discarded.")
            elif command == "q":
                monitor.discard_episode()
                recording = False
                typer.echo("Active episode discarded.")
                break
            elif not command:
                status = monitor.save_episode()
                recording = False
                typer.echo(f"Episode saved ({status.episodes_saved} total).")
            else:
                typer.echo("Use ENTER to save, d to discard, or q to finish.")
    except KeyboardInterrupt:
        if coordinator is not None and recording:
            monitor = coordinator.get_instance(EpisodeMonitorModule)
            monitor.discard_episode()
            typer.echo("\nActive episode discarded.")
    except Exception as exc:
        typer.echo(f"A1Z teach failed: {exc}", err=True)
        raise typer.Exit(1)
    finally:
        if coordinator is not None:
            typer.echo("\nSupport the arm before the recording is flushed and motors disable.")
            try:
                _press_enter("Press ENTER when the arm is supported")
            except (KeyboardInterrupt, EOFError):
                pass
            coordinator.stop()

    typer.echo(f"Saved Memory2 recording: {db_path}")


@app.command()
def replay(
    source: Path = typer.Argument(..., help="Memory2 recording .db"),
    episode: int = typer.Option(-1, "--episode", "-e", help="Saved episode index; -1 is latest"),
    speed: float = typer.Option(1.0, "--speed", min=0.01, help="Requested playback speed"),
) -> None:
    """Validate and replay one saved A1Z episode through ControlCoordinator."""
    from dimos.control.coordinator import ControlCoordinator
    from dimos.core.coordination.module_coordinator import ModuleCoordinator
    from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState
    from dimos.robot.manipulators.galaxea_a1z.blueprints.basic import (
        A1Z_REPLAY_TASK_NAME,
        make_a1z_replay_blueprint,
    )
    from dimos.robot.manipulators.galaxea_a1z.teach_replay import (
        build_execution_trajectory,
        load_recorded_episode,
        prepare_episode,
    )

    source = source.expanduser().resolve()
    try:
        recorded = load_recorded_episode(source, episode)
        prepared = prepare_episode(recorded, speed=speed)
    except Exception as exc:
        typer.echo(f"A1Z replay preflight failed: {exc}", err=True)
        raise typer.Exit(1)

    typer.echo(f"Recording: {source}")
    typer.echo(
        f"Episode: {recorded.episode_index} ({len(recorded.timestamps)} measured samples, "
        f"{recorded.timestamps[-1]:.2f}s)"
    )
    if prepared.effective_speed < prepared.requested_speed * 0.999:
        typer.echo(
            f"Safety time-scaling: requested {prepared.requested_speed:.2f}x, "
            f"using {prepared.effective_speed:.2f}x"
        )
    else:
        typer.echo(f"Playback speed: {prepared.effective_speed:.2f}x")
    typer.echo("Raw recorded values passed command-limit validation; nothing was clipped.")
    typer.echo("Support the arm during startup. It has no brakes.\n")

    coordinator: ModuleCoordinator | None = None
    started = False
    try:
        coordinator = ModuleCoordinator.build(make_a1z_replay_blueprint(), {})
        control: Any = coordinator.get_instance(ControlCoordinator)
        current_positions = control.get_joint_positions()
        trajectory = build_execution_trajectory(current_positions, prepared)

        typer.echo(
            f"The robot will approach the recorded start pose, then replay for "
            f"{prepared.duration:.2f}s. Total controlled motion: {trajectory.duration:.2f}s."
        )
        if not typer.confirm("Execute this motion now?", default=False):
            typer.echo("Replay cancelled before motion.")
            return

        accepted = control.task_invoke(
            A1Z_REPLAY_TASK_NAME,
            "execute",
            {"trajectory": trajectory},
        )
        if not accepted:
            raise RuntimeError("ControlCoordinator rejected the replay trajectory")
        started = True

        deadline = time.monotonic() + trajectory.duration + 5.0
        while time.monotonic() < deadline:
            state = TrajectoryState(control.task_invoke(A1Z_REPLAY_TASK_NAME, "get_state", {}))
            if state == TrajectoryState.COMPLETED:
                typer.echo("Replay complete. The arm is holding the final pose.")
                break
            if state in (TrajectoryState.ABORTED, TrajectoryState.FAULT):
                raise RuntimeError(f"Replay ended in state {state.name}")
            time.sleep(0.05)
        else:
            control.task_invoke(A1Z_REPLAY_TASK_NAME, "cancel", {})
            raise TimeoutError("Replay did not complete before its safety timeout")
    except KeyboardInterrupt:
        typer.echo("\nReplay interrupted.", err=True)
        if coordinator is not None and started:
            control = coordinator.get_instance(ControlCoordinator)
            control.task_invoke(A1Z_REPLAY_TASK_NAME, "cancel", {})
    except Exception as exc:
        typer.echo(f"A1Z replay failed: {exc}", err=True)
        raise typer.Exit(1)
    finally:
        if coordinator is not None:
            typer.echo("Support the arm before disabling its motors.")
            try:
                _press_enter("Press ENTER when the arm is supported")
            except (KeyboardInterrupt, EOFError):
                pass
            coordinator.stop()
