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

"""Safe operator commands for a running Unitree G1 teleop stack."""

from __future__ import annotations

import time
from typing import Any, NoReturn, cast

import typer

from dimos.control.tasks.trajectory_task.trajectory_task import JOINT_TRAJECTORY_TASK_NAME
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.porcelain.dimos import Dimos
from dimos.robot.unitree.g1.manip_config import (
    G1_READY_JOINTS,
    G1_READY_SPEED_SCALE,
    G1_UPPER_BODY_NAME,
)

app = typer.Typer(help="Operate a running Unitree G1 stack safely")

_COORDINATOR = "ControlCoordinator"
_MANIPULATION = "G1Manipulation"
_GROOT_TASK = "groot_wbc"
_ARM_POLL_SECONDS = 0.1


def _abort(message: str) -> NoReturn:
    typer.echo(f"ERROR: {message}", err=True)
    raise typer.Exit(1)


def _connect() -> Dimos:
    try:
        return Dimos.connect()
    except Exception as exc:
        _abort(f"cannot connect to a running DimOS stack: {exc}")


def _groot_state(coordinator: Any) -> dict[str, Any]:
    state = coordinator.task_invoke(_GROOT_TASK, "state_snapshot", {})
    if not isinstance(state, dict):
        _abort("the running stack does not expose G1 GR00T safety state")
    return cast("dict[str, Any]", state)


def _require_armed_and_enabled(coordinator: Any) -> dict[str, Any]:
    state = _groot_state(coordinator)
    if not state.get("armed") or state.get("arming") or state.get("arm_pending"):
        _abort("G1 is not fully armed; run `dimos hardware g1 arm` first")
    if state.get("dry_run"):
        _abort("motor output is still disabled; run `dimos hardware g1 enable` first")
    return state


@app.command()
def status() -> None:
    """Show the G1 safety state, trajectory state, and planning groups."""
    client = _connect()
    try:
        coordinator = cast("Any", client.get_module(_COORDINATOR))
        state = _groot_state(coordinator)
        trajectory = coordinator.task_invoke(
            JOINT_TRAJECTORY_TASK_NAME, "get_status", {"t_now": None}
        )
        try:
            groups = cast("Any", client.get_module(_MANIPULATION)).list_planning_groups()
            group_ids = [str(group.id) for group in groups]
        except (AttributeError, KeyError):
            group_ids = []

        typer.echo(f"active:      {bool(state.get('active'))}")
        typer.echo(f"armed:       {bool(state.get('armed'))}")
        typer.echo(f"arming:      {bool(state.get('arming') or state.get('arm_pending'))}")
        typer.echo(f"dry_run:     {bool(state.get('dry_run'))}")
        typer.echo(f"trajectory:  {trajectory}")
        typer.echo(f"manipulation: {', '.join(group_ids) if group_ids else 'unavailable'}")
    except (AttributeError, KeyError, RuntimeError) as exc:
        _abort(f"running stack is not a compatible G1 teleop stack: {exc}")
    finally:
        client.stop()


@app.command()
def arm(timeout: float = typer.Option(15.0, min=0.1, help="Arming timeout in seconds.")) -> None:
    """Arm in dry-run and wait for the pose ramp to finish."""
    client = _connect()
    try:
        coordinator = cast("Any", client.get_module(_COORDINATOR))
        coordinator.set_dry_run(True)
        coordinator.set_activated(True)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            state = _groot_state(coordinator)
            if state.get("armed") and not state.get("arming") and not state.get("arm_pending"):
                typer.echo(
                    "G1 armed in dry-run; inspect the robot, then run `dimos hardware g1 enable`."
                )
                return
            time.sleep(_ARM_POLL_SECONDS)
        _abort(f"G1 did not finish arming within {timeout:g}s; motor output remains in dry-run")
    except (AttributeError, KeyError, RuntimeError) as exc:
        _abort(f"failed to arm G1: {exc}")
    finally:
        client.stop()


@app.command()
def enable() -> None:
    """Enable motor output after a completed dry-run arming ramp."""
    client = _connect()
    try:
        coordinator = cast("Any", client.get_module(_COORDINATOR))
        state = _groot_state(coordinator)
        if not state.get("armed") or state.get("arming") or state.get("arm_pending"):
            _abort("G1 is not fully armed; run `dimos hardware g1 arm` first")
        coordinator.set_dry_run(False)
        if _groot_state(coordinator).get("dry_run"):
            _abort("G1 remained in dry-run after the enable request")
        typer.echo("G1 motor output enabled.")
    except (AttributeError, KeyError, RuntimeError) as exc:
        _abort(f"failed to enable G1: {exc}")
    finally:
        client.stop()


@app.command()
def ready() -> None:
    """Plan and execute the conservative bimanual ready pose."""
    client = _connect()
    try:
        coordinator = cast("Any", client.get_module(_COORDINATOR))
        _require_armed_and_enabled(coordinator)
        manipulation = cast("Any", client.get_module(_MANIPULATION))
        targets = {
            f"{G1_UPPER_BODY_NAME}/{group}": JointState(position=list(positions))
            for group, positions in G1_READY_JOINTS.items()
        }
        planned = manipulation.plan_to_joints(targets, speed_scale=G1_READY_SPEED_SCALE)
        if not planned.succeeded:
            _abort(f"ready-pose planning failed: {planned}")
        executed = manipulation.execute(blocking=True)
        if not executed.succeeded:
            _abort(f"ready-pose execution failed: {executed}")
        typer.echo("G1 reached the ready pose.")
    except (AttributeError, KeyError, RuntimeError) as exc:
        _abort(f"failed to move G1 to the ready pose: {exc}")
    finally:
        client.stop()


@app.command()
def disable() -> None:
    """Cancel arm motion, enter dry-run, and disarm the G1."""
    client = _connect()
    failures: list[str] = []
    try:
        coordinator = cast("Any", client.get_module(_COORDINATOR))
        for description, operation in (
            ("cancel trajectory", coordinator.cancel_trajectory),
            ("enter dry-run", lambda: coordinator.set_dry_run(True)),
            ("disarm", lambda: coordinator.set_activated(False)),
        ):
            try:
                operation()
            except Exception as exc:
                failures.append(f"{description}: {exc}")
        if failures:
            _abort("; ".join(failures))
        typer.echo("G1 trajectory cancelled, motor output disabled, and controller disarmed.")
    except (AttributeError, KeyError, RuntimeError) as exc:
        _abort(f"failed to disable G1: {exc}")
    finally:
        client.stop()
