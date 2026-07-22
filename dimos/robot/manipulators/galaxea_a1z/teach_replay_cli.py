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
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState

app = typer.Typer(help="Record and replay Galaxea A1Z hand-taught episodes")

_TEACH_HARDWARE_ID = "arm"
# Matches the adapter's default G1Z max opening; the adapter clamps to the
# configured range, so a full-open command stays correct if that changes.
_GRIPPER_OPEN_M = 0.1
_GRIPPER_CLOSED_M = 0.0


def _default_recording_path() -> Path:
    return STATE_DIR / "recordings" / f"a1z_teach_{datetime.now():%Y%m%d_%H%M%S}.db"


def _press_enter(message: str) -> None:
    typer.prompt(message, default="", show_default=False)


def _read_key(message: str) -> str:
    """Read one keypress without waiting for ENTER.

    Returns the lowercased character; ENTER is normalized to "". Falls back
    to line input when stdin is not an interactive terminal.
    """
    import sys

    typer.echo(message)
    if not sys.stdin.isatty():
        line = sys.stdin.readline()
        if not line:
            raise EOFError
        return line.strip().lower()[:1]

    import termios
    import tty

    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
    if key == "\x03":  # Ctrl-C arrives as a literal byte in cbreak mode
        raise KeyboardInterrupt
    if key in ("\r", "\n"):
        return ""
    return key.lower()


def _execute_demo_trajectory(control: Any, trajectory: JointTrajectory, label: str) -> None:
    accepted = control.task_invoke(
        "teach_replay_arm",
        "execute",
        {"trajectory": trajectory},
    )
    if not accepted:
        raise RuntimeError(f"ControlCoordinator rejected the {label} trajectory")

    deadline = time.monotonic() + trajectory.duration + 5.0
    while time.monotonic() < deadline:
        state = TrajectoryState(control.task_invoke("teach_replay_arm", "get_state", {}))
        if state == TrajectoryState.COMPLETED:
            typer.echo(f">> {label} complete")
            return
        if state in (TrajectoryState.ABORTED, TrajectoryState.FAULT):
            raise RuntimeError(f"{label} ended in state {state.name}")
        time.sleep(0.05)
    control.task_invoke("teach_replay_arm", "cancel", {})
    raise TimeoutError(f"{label} did not complete before its safety timeout")


@app.command()
def teach(
    output: Path | None = typer.Argument(
        None,
        help="Memory2 .db output (default: timestamped file under the DimOS state directory)",
    ),
    task: str | None = typer.Option(None, "--task", help="Task label stored with each episode"),
    camera_index: int = typer.Option(
        0,
        "--camera-index",
        min=0,
        help="Linux camera index N for /dev/videoN",
    ),
    gripper_free_drive: bool = typer.Option(
        False,
        "--gripper-free-drive",
        help="Zero-torque gripper you pinch by hand (legacy); default keeps the "
        "gripper powered and toggled with g so your hand stays out of the camera",
    ),
) -> None:
    """Hand-teach episodes into one Memory2 recording."""
    from dimos.control.coordinator import ControlCoordinator
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
    typer.echo(f"Camera: /dev/video{camera_index} (640x480 at 15 FPS)")
    typer.echo("The arm will become hand-drivable after startup.")
    if gripper_free_drive:
        typer.echo("Gripper: free drive (open and close it by hand).")
    else:
        typer.echo("Gripper: powered; press g to toggle open/closed.")
    typer.echo("Keep the arm supported: it has no brakes and can fall when motors disable.\n")

    coordinator: ModuleCoordinator | None = None
    recording = False
    gripper_open: bool | None = None
    saved_count = 0
    episode_started_at = 0.0

    def _status() -> str:
        if gripper_free_drive:
            gripper = "free-drive"
        elif gripper_open is None:
            gripper = "?"
        else:
            gripper = "open" if gripper_open else "closed"
        if recording:
            elapsed = time.monotonic() - episode_started_at
            state = f"RECORDING {int(elapsed // 60)}:{int(elapsed % 60):02d}"
            keys = "SPACE save · g gripper · d discard · q quit"
        else:
            state = "IDLE"
            keys = "SPACE record · d undo last · g gripper · q quit"
        return f"[{state} | saved: {saved_count} | gripper: {gripper}]  {keys}"

    try:
        coordinator = ModuleCoordinator.build(
            make_a1z_teach_blueprint(
                db_path,
                task_label=task,
                camera_index=camera_index,
                gripper_free_drive=gripper_free_drive,
            ),
            {},
        )
        monitor: Any = coordinator.get_instance(EpisodeMonitorModule)
        control: Any = coordinator.get_instance(ControlCoordinator)
        if not gripper_free_drive:
            measured = control.get_gripper_position(_TEACH_HARDWARE_ID)
            gripper_open = measured is not None and measured > _GRIPPER_OPEN_M / 2
        typer.echo("Ready. Move only after starting an episode.")

        def _toggle_gripper() -> None:
            nonlocal gripper_open
            if gripper_free_drive:
                typer.echo("Gripper is in free drive; open and close it by hand.")
                return
            target_open = not gripper_open
            target = _GRIPPER_OPEN_M if target_open else _GRIPPER_CLOSED_M
            if control.set_gripper_position(_TEACH_HARDWARE_ID, target):
                gripper_open = target_open
                typer.echo(f">> gripper {'opening' if target_open else 'closing'}")
            else:
                typer.echo(">> gripper command rejected; check hardware state", err=True)

        while True:
            command = _read_key(_status())

            if command == "g":
                _toggle_gripper()
                continue

            if command == " " or command == "":
                # SPACE is the documented key; bare ENTER does the same thing
                # so either habit works.
                if not recording:
                    monitor.start_episode()
                    recording = True
                    episode_started_at = time.monotonic()
                    typer.echo(">> episode started - move the arm by hand")
                else:
                    status = monitor.save_episode()
                    recording = False
                    saved_count = status.episodes_saved
                    typer.echo(f">> episode saved ({saved_count} total)")
                continue

            if command == "d":
                status = monitor.discard_episode()
                if recording:
                    recording = False
                    typer.echo(">> episode discarded")
                elif status.last_event == "undo":
                    saved_count = status.episodes_saved
                    typer.echo(f">> previous saved episode discarded ({saved_count} remain)")
                else:
                    typer.echo(">> nothing saved to discard")
                continue

            if command == "q":
                if not recording:
                    break
                choice = _read_key(
                    "Episode in progress - s to save it, d to discard it, or any other key to keep recording"
                )
                if choice == "s":
                    status = monitor.save_episode()
                    recording = False
                    saved_count = status.episodes_saved
                    typer.echo(f">> episode saved ({saved_count} total)")
                    break
                if choice == "d":
                    monitor.discard_episode()
                    recording = False
                    typer.echo(">> episode discarded")
                    break
                typer.echo(">> still recording")
                continue

            typer.echo(f">> unrecognized key {command!r}")
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
def demo(
    output: Path | None = typer.Argument(
        None,
        help="Memory2 .db output (default: timestamped file under the DimOS state directory)",
    ),
    task: str | None = typer.Option(None, "--task", help="Task label stored with each episode"),
    camera_index: int = typer.Option(0, "--camera-index", min=0),
    speed: float = typer.Option(1.0, "--speed", min=0.01, help="Replay speed"),
) -> None:
    """Teach and replay repeatedly in one live stack with a Rerun camera view."""
    from dimos.control.coordinator import ControlCoordinator
    from dimos.core.coordination.module_coordinator import ModuleCoordinator
    from dimos.learning.collection.episode_monitor import EpisodeMonitorModule
    from dimos.robot.manipulators.galaxea_a1z.blueprints.basic import (
        A1Z_REPLAY_TASK_NAME,
        make_a1z_teach_replay_demo_blueprint,
    )
    from dimos.robot.manipulators.galaxea_a1z.teach_replay import (
        PreparedEpisode,
        build_approach_trajectory,
        build_replay_trajectory,
        load_recorded_episode,
        prepare_episode,
    )

    db_path = (output or _default_recording_path()).expanduser().resolve()
    if db_path.exists():
        typer.echo(f"error: refusing to overwrite existing recording: {db_path}", err=True)
        raise typer.Exit(2)

    typer.echo("A1Z teach/replay demo")
    typer.echo(f"Recording: {db_path}")
    typer.echo(f"Camera: /dev/video{camera_index} (live in Rerun)")
    typer.echo("The arm starts hand-drivable. Keep the workspace clear.\n")

    coordinator: ModuleCoordinator | None = None
    control: Any = None
    monitor: Any = None
    recording = False
    teach_mode = True
    at_start_pose = False
    saved_count = 0
    gripper_open: bool | None = None
    prepared: PreparedEpisode | None = None
    episode_started_at = 0.0

    def _status() -> str:
        mode = "ZERO-GRAVITY" if teach_mode else "POSITION"
        state = "RECORDING" if recording else "IDLE"
        if recording:
            elapsed = time.monotonic() - episode_started_at
            state += f" {int(elapsed // 60)}:{int(elapsed % 60):02d}"
        return (
            f"[{state} | {mode} | saved: {saved_count}]  "
            "SPACE record/save · g gripper · h start pose · r replay · "
            "z zero-gravity · d discard · q quit"
        )

    def _toggle_gripper() -> None:
        nonlocal at_start_pose, gripper_open
        target_open = not gripper_open
        target = _GRIPPER_OPEN_M if target_open else _GRIPPER_CLOSED_M
        if not control.set_gripper_position(_TEACH_HARDWARE_ID, target):
            typer.echo(">> gripper command rejected", err=True)
            return
        gripper_open = target_open
        if not teach_mode:
            at_start_pose = False
        typer.echo(f">> gripper {'opening' if target_open else 'closing'}")

    def _refresh_prepared() -> PreparedEpisode:
        deadline = time.monotonic() + 2.0
        last_error: Exception | None = None
        while time.monotonic() < deadline:
            try:
                return prepare_episode(load_recorded_episode(db_path), speed=speed)
            except (OSError, ValueError) as exc:
                last_error = exc
                time.sleep(0.05)
        raise RuntimeError(f"saved episode was not readable: {last_error}")

    try:
        coordinator = ModuleCoordinator.build(
            make_a1z_teach_replay_demo_blueprint(
                db_path,
                task_label=task,
                camera_index=camera_index,
            ),
            {},
        )
        monitor = coordinator.get_instance(EpisodeMonitorModule)
        control = coordinator.get_instance(ControlCoordinator)
        measured = control.get_gripper_position(_TEACH_HARDWARE_ID)
        gripper_open = measured is not None and measured > _GRIPPER_OPEN_M / 2
        typer.echo("Ready. Rerun is showing the live camera.")

        while True:
            command = _read_key(_status())

            if command == "g":
                _toggle_gripper()
                continue

            if command in (" ", ""):
                if not teach_mode:
                    typer.echo(">> press z for zero-gravity before recording")
                elif not recording:
                    monitor.start_episode()
                    recording = True
                    episode_started_at = time.monotonic()
                    typer.echo(">> episode started")
                else:
                    status = monitor.save_episode()
                    recording = False
                    saved_count = status.episodes_saved
                    prepared = _refresh_prepared()
                    at_start_pose = False
                    typer.echo(f">> episode saved ({saved_count} total); h=start pose, r=replay")
                continue

            if command == "h":
                if recording:
                    typer.echo(">> save or discard the active episode first")
                    continue
                if prepared is None:
                    typer.echo(">> record an episode first")
                    continue
                if teach_mode:
                    if not control.set_hardware_teach_mode(_TEACH_HARDWARE_ID, False):
                        typer.echo(">> hold the arm still and press h again", err=True)
                        continue
                    teach_mode = False
                approach = build_approach_trajectory(control.get_joint_positions(), prepared)
                typer.echo(f">> moving to recorded start pose ({approach.duration:.2f}s)")
                _execute_demo_trajectory(control, approach, "start-pose move")
                at_start_pose = True
                continue

            if command == "r":
                if recording:
                    typer.echo(">> save or discard the active episode first")
                elif prepared is None:
                    typer.echo(">> record an episode first")
                elif teach_mode:
                    typer.echo(">> press h to enter position mode and move to the start pose")
                elif not at_start_pose:
                    typer.echo(">> press h to return to the recorded start pose")
                else:
                    trajectory = build_replay_trajectory(prepared)
                    typer.echo(f">> replaying ({trajectory.duration:.2f}s)")
                    _execute_demo_trajectory(control, trajectory, "replay")
                    at_start_pose = False
                continue

            if command == "z":
                if recording:
                    typer.echo(">> already recording in zero-gravity")
                elif teach_mode:
                    typer.echo(">> already in zero-gravity")
                elif control.set_hardware_teach_mode(_TEACH_HARDWARE_ID, True):
                    teach_mode = True
                    at_start_pose = False
                    typer.echo(">> zero-gravity enabled; move the arm by hand")
                else:
                    typer.echo(">> zero-gravity transition failed; press z again", err=True)
                continue

            if command == "d":
                status = monitor.discard_episode()
                if recording:
                    recording = False
                    typer.echo(">> episode discarded")
                elif status.last_event == "undo":
                    saved_count = status.episodes_saved
                    prepared = _refresh_prepared() if saved_count else None
                    typer.echo(f">> previous episode discarded ({saved_count} remain)")
                else:
                    typer.echo(">> nothing saved to discard")
                continue

            if command == "q":
                if recording:
                    typer.echo(">> save or discard the active episode before quitting")
                    continue
                break

            typer.echo(f">> unrecognized key {command!r}")
    except KeyboardInterrupt:
        typer.echo("\nDemo interrupted.", err=True)
        if recording and monitor is not None:
            monitor.discard_episode()
        if control is not None:
            control.task_invoke(A1Z_REPLAY_TASK_NAME, "cancel", {})
    except Exception as exc:
        typer.echo(f"A1Z demo failed: {exc}", err=True)
        raise typer.Exit(1)
    finally:
        if coordinator is not None:
            typer.echo("Support the arm before motors disable.")
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


@app.command("run-policy")
def run_policy(
    checkpoint: str = typer.Argument(
        ...,
        help="Local LeRobot pretrained_model directory or Hugging Face model ID",
    ),
    task: str = typer.Option("", "--task", help="Task prompt supplied to the policy"),
    duration: float = typer.Option(
        10.0,
        "--duration",
        min=0.1,
        help="Maximum policy execution time in seconds",
    ),
    camera_index: int = typer.Option(
        0,
        "--camera-index",
        min=0,
        help="Linux camera index N for /dev/videoN",
    ),
    device: str | None = typer.Option(
        None,
        "--device",
        help="Torch device override, for example cuda or cpu",
    ),
) -> None:
    """Execute a trained LeRobot policy on the live A1Z."""
    from dimos.core.coordination.module_coordinator import ModuleCoordinator
    from dimos.learning.lerobot_policy import LeRobotPolicyModule
    from dimos.robot.manipulators.galaxea_a1z.blueprints.basic import (
        make_a1z_policy_blueprint,
    )

    local_checkpoint = Path(checkpoint).expanduser()
    policy_path = str(local_checkpoint.resolve()) if local_checkpoint.exists() else checkpoint

    typer.echo("A1Z learned-policy execution")
    typer.echo(f"Checkpoint: {policy_path}")
    typer.echo(f"Camera: /dev/video{camera_index} (640x480 at 15 FPS)")
    typer.echo(f"Maximum execution: {duration:.1f}s")
    typer.echo("The arm has no brakes. Support it during startup and keep the workspace clear.\n")
    if not typer.confirm("Load the policy and initialize the robot?", default=False):
        typer.echo("Policy execution cancelled.")
        return

    coordinator: ModuleCoordinator | None = None
    policy: Any = None
    try:
        coordinator = ModuleCoordinator.build(
            make_a1z_policy_blueprint(
                policy_path,
                task=task,
                camera_index=camera_index,
                device=device,
            ),
            {},
        )
        policy = coordinator.get_instance(LeRobotPolicyModule)
        observation_deadline = time.monotonic() + 5.0
        while time.monotonic() < observation_deadline:
            status = policy.policy_status()
            if status["observations_ready"]:
                break
            time.sleep(0.1)
        else:
            raise RuntimeError(
                f"live policy observations did not become ready: {status['observation_error']}"
            )
        result = policy.execute_learned_policy("default", duration)
        typer.echo(result)
        if "started" not in result.lower():
            raise RuntimeError(result)

        deadline = time.monotonic() + duration + 5.0
        while time.monotonic() < deadline:
            status = policy.policy_status()
            if not status["running"]:
                if status["last_error"]:
                    raise RuntimeError(status["last_error"])
                typer.echo(f"Policy execution complete ({status['commands_sent']} commands sent).")
                break
            time.sleep(0.1)
        else:
            policy.stop_learned_policy()
            raise TimeoutError("Policy did not stop before its execution timeout")
    except KeyboardInterrupt:
        typer.echo("\nPolicy execution interrupted.", err=True)
        if policy is not None:
            policy.stop_learned_policy()
    except Exception as exc:
        typer.echo(f"A1Z policy execution failed: {exc}", err=True)
        raise typer.Exit(1)
    finally:
        if coordinator is not None:
            typer.echo("Support the arm before disabling its motors.")
            try:
                _press_enter("Press ENTER when the arm is supported")
            except (KeyboardInterrupt, EOFError):
                pass
            coordinator.stop()
