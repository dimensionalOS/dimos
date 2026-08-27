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
from typing import Any, NoReturn, Protocol, TypeGuard

import typer

from dimos.control.tasks.trajectory_task.trajectory_task import JOINT_TRAJECTORY_TASK_NAME
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.porcelain.dimos import Dimos
from dimos.porcelain.module_handle import ModuleHandle
from dimos.robot.unitree.g1.manip_config import (
    G1_READY_JOINTS,
    G1_READY_SPEED_SCALE,
    G1_UPPER_BODY_NAME,
)

app = typer.Typer(help="Operate a running Unitree G1 stack safely")

_COORDINATOR = "ControlCoordinator"
_MANIPULATION = "G1Manipulation"
_TELEOP_TASK = "teleop_g1"
_ARM_POLL_SECONDS = 0.1
_LIFECYCLE_COMMANDS = frozenset({"arm", "disarm", "set_dry_run", "state_snapshot"})
_LIFECYCLE_STATE_FIELDS = frozenset(
    {"active", "armed", "arming", "arm_pending", "dry_run", "arming_duration"}
)


class _G1CoordinatorHandle(Protocol):
    def list_tasks(self) -> list[str]: ...
    def describe_task(self, task_name: str) -> dict[str, Any] | None: ...
    def task_invoke(self, task_name: str, method: str, kwargs: dict[str, Any]) -> Any: ...
    def set_dry_run(self, dry_run: bool) -> Any: ...
    def set_activated(self, activated: bool) -> Any: ...
    def get_active_tasks(self) -> list[str]: ...
    def cancel_trajectory(self) -> Any: ...


class _G1ManipulationHandle(Protocol):
    def list_planning_groups(self) -> list[Any]: ...
    def plan_to_joints(self, targets: dict[str, JointState], *, speed_scale: float) -> Any: ...
    def execute(self, *, blocking: bool) -> Any: ...


def _abort(message: str) -> NoReturn:
    typer.echo(f"ERROR: {message}", err=True)
    raise typer.Exit(1)


def _connect() -> Dimos:
    try:
        return Dimos.connect()
    except Exception as exc:
        _abort(f"cannot connect to a running DimOS stack: {exc}")


def _has_methods(handle: ModuleHandle, names: tuple[str, ...]) -> bool:
    return all(callable(getattr(handle, name, None)) for name in names)


def _is_coordinator(handle: ModuleHandle) -> TypeGuard[_G1CoordinatorHandle]:
    return _has_methods(
        handle,
        (
            "list_tasks",
            "describe_task",
            "task_invoke",
            "set_dry_run",
            "set_activated",
            "get_active_tasks",
            "cancel_trajectory",
        ),
    )


def _is_manipulation(handle: ModuleHandle) -> TypeGuard[_G1ManipulationHandle]:
    return _has_methods(handle, ("list_planning_groups", "plan_to_joints", "execute"))


def _coordinator(client: Dimos) -> _G1CoordinatorHandle:
    handle = client.get_module(_COORDINATOR)
    if not _is_coordinator(handle):
        _abort("the running stack does not expose the required G1 coordinator RPCs")
    return handle


def _manipulation(client: Dimos) -> _G1ManipulationHandle:
    try:
        handle = client.get_module(_MANIPULATION)
    except (AttributeError, KeyError):
        _abort("the running stack does not expose the required G1 manipulation RPCs")
    if not _is_manipulation(handle):
        _abort("the running stack does not expose the required G1 manipulation RPCs")
    return handle


def _lifecycle_task(coordinator: _G1CoordinatorHandle) -> str:
    matches: list[str] = []
    for task_name in coordinator.list_tasks():
        description = coordinator.describe_task(task_name)
        if not isinstance(description, dict):
            continue
        commands = description.get("commands")
        if isinstance(commands, dict) and _LIFECYCLE_COMMANDS <= commands.keys():
            matches.append(task_name)
    if not matches:
        _abort("the running stack has no G1 policy task with lifecycle controls")
    if len(matches) > 1:
        _abort(f"the running stack has multiple G1 policy lifecycle tasks: {', '.join(matches)}")
    return matches[0]


def _is_lifecycle_state(value: Any) -> TypeGuard[dict[str, Any]]:
    return isinstance(value, dict) and _LIFECYCLE_STATE_FIELDS <= value.keys()


def _policy_state(coordinator: _G1CoordinatorHandle, task_name: str) -> dict[str, Any]:
    state = coordinator.task_invoke(task_name, "state_snapshot", {})
    if not _is_lifecycle_state(state):
        _abort(f"G1 policy task {task_name!r} returned an invalid lifecycle state")
    return state


def _require_armed_and_enabled(coordinator: _G1CoordinatorHandle, task_name: str) -> dict[str, Any]:
    state = _policy_state(coordinator, task_name)
    if not state.get("armed") or state.get("arming") or state.get("arm_pending"):
        _abort("G1 is not fully armed; run `dimos hardware g1 arm` first")
    if state.get("dry_run"):
        _abort("learned-policy output is still in dry-run; run `dimos hardware g1 enable` first")
    return state


def _fully_armed(state: dict[str, Any]) -> bool:
    return bool(state.get("armed") and not state.get("arming") and not state.get("arm_pending"))


def _arm_and_wait(
    coordinator: _G1CoordinatorHandle, task_name: str, timeout: float
) -> dict[str, Any]:
    coordinator.set_dry_run(True)
    coordinator.set_activated(True)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        state = _policy_state(coordinator, task_name)
        if _fully_armed(state):
            return state
        time.sleep(_ARM_POLL_SECONDS)
    _abort(f"G1 did not finish arming within {timeout:g}s; motor output remains in dry-run")


def _enable_motor_output(
    coordinator: _G1CoordinatorHandle,
    task_name: str,
    state: dict[str, Any] | None = None,
) -> dict[str, Any]:
    current = state if state is not None else _policy_state(coordinator, task_name)
    if not _fully_armed(current):
        _abort("G1 is not fully armed; run `dimos hardware g1 arm` first")
    coordinator.set_dry_run(False)
    enabled = _policy_state(coordinator, task_name)
    if enabled.get("dry_run"):
        _abort("G1 remained in dry-run after the enable request")
    return enabled


def _require_teleop_disengaged(coordinator: _G1CoordinatorHandle) -> None:
    if _TELEOP_TASK in coordinator.get_active_tasks():
        _abort("G1 teleoperation is active; disengage both hands before moving to ready pose")


def _execute_ready_pose(
    coordinator: _G1CoordinatorHandle,
    task_name: str,
    manipulation: _G1ManipulationHandle,
) -> None:
    _require_armed_and_enabled(coordinator, task_name)
    _require_teleop_disengaged(coordinator)
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


@app.command()
def status() -> None:
    """Show the G1 safety state, trajectory state, and planning groups."""
    client = _connect()
    try:
        coordinator = _coordinator(client)
        task_name = _lifecycle_task(coordinator)
        state = _policy_state(coordinator, task_name)
        if JOINT_TRAJECTORY_TASK_NAME in coordinator.list_tasks():
            trajectory = coordinator.task_invoke(
                JOINT_TRAJECTORY_TASK_NAME, "get_status", {"t_now": None}
            )
        else:
            trajectory = "unavailable"
        try:
            manipulation = client.get_module(_MANIPULATION)
            if not _is_manipulation(manipulation):
                raise KeyError(_MANIPULATION)
            groups = manipulation.list_planning_groups()
            group_ids = [str(group.id) for group in groups]
        except (AttributeError, KeyError):
            group_ids = []

        typer.echo(f"controller:  {task_name}")
        typer.echo(f"active:      {bool(state.get('active'))}")
        typer.echo(f"armed:       {bool(state.get('armed'))}")
        typer.echo(f"arming:      {bool(state.get('arming') or state.get('arm_pending'))}")
        typer.echo(f"dry_run:     {bool(state.get('dry_run'))}")
        if "control_state" in state:
            typer.echo(f"control:     {state['control_state']}")
        if "reference_source" in state:
            typer.echo(f"reference:   {state['reference_source']}")
        webxr = state.get("webxr_teleop")
        if isinstance(webxr, dict):
            typer.echo(f"webxr:       {'engaged' if webxr.get('engaged') else 'disengaged'}")
        typer.echo(f"trajectory:  {trajectory}")
        typer.echo(f"manipulation: {', '.join(group_ids) if group_ids else 'unavailable'}")
    except (AttributeError, KeyError, RuntimeError) as exc:
        _abort(f"running stack is not a compatible G1 teleop stack: {exc}")
    finally:
        client.stop()


@app.command()
def arm(timeout: float = typer.Option(15.0, min=0.1, help="Arming timeout in seconds.")) -> None:
    """Run the policy pose ramp, then keep learned-policy output in dry-run."""
    client = _connect()
    try:
        coordinator = _coordinator(client)
        task_name = _lifecycle_task(coordinator)
        _arm_and_wait(coordinator, task_name, timeout)
        typer.echo("G1 armed in dry-run; inspect the robot, then run `dimos hardware g1 enable`.")
    except (AttributeError, KeyError, RuntimeError) as exc:
        _abort(f"failed to arm G1: {exc}")
    finally:
        client.stop()


@app.command()
def enable() -> None:
    """Enable learned-policy output after a completed dry-run arming ramp."""
    client = _connect()
    try:
        coordinator = _coordinator(client)
        task_name = _lifecycle_task(coordinator)
        _enable_motor_output(coordinator, task_name)
        typer.echo("G1 live policy output enabled.")
    except (AttributeError, KeyError, RuntimeError) as exc:
        _abort(f"failed to enable G1: {exc}")
    finally:
        client.stop()


@app.command()
def activate(
    timeout: float = typer.Option(15.0, min=0.1, help="Arming timeout in seconds."),
    ready: bool = typer.Option(
        False,
        "--ready",
        help="Move both arms to the conservative ready pose after enabling motor output.",
    ),
) -> None:
    """Arm, confirm physical safety, and enable live policy output."""
    client = _connect()
    motor_output_enabled = False
    try:
        coordinator = _coordinator(client)
        manipulation = _manipulation(client) if ready else None
        task_name = _lifecycle_task(coordinator)
        state = _policy_state(coordinator, task_name)
        if not _fully_armed(state):
            state = _arm_and_wait(coordinator, task_name, timeout)

        if state.get("dry_run"):
            typer.echo(
                "Arming ramp complete. Inspect the robot and confirm the remote and E-stop "
                "are ready."
            )
            if not typer.confirm("Enable live G1 policy motor output?", default=False):
                typer.echo("Activation cancelled; G1 remains armed in dry-run.")
                raise typer.Exit(1)
            _enable_motor_output(coordinator, task_name, state)
            motor_output_enabled = True
            typer.echo("G1 live policy output enabled.")
        else:
            motor_output_enabled = True
            typer.echo("G1 is already activated.")

        if ready:
            assert manipulation is not None
            try:
                _execute_ready_pose(coordinator, task_name, manipulation)
            except typer.Exit:
                typer.echo("G1 policy motor output remains enabled.", err=True)
                raise
            typer.echo("G1 reached the ready pose.")
        elif state.get("dry_run"):
            typer.echo("G1 activated.")
    except (AttributeError, KeyError, RuntimeError) as exc:
        suffix = "; G1 policy motor output remains enabled" if motor_output_enabled else ""
        _abort(f"failed to activate G1: {exc}{suffix}")
    finally:
        client.stop()


@app.command()
def ready() -> None:
    """Plan and execute the conservative bimanual ready pose."""
    client = _connect()
    try:
        coordinator = _coordinator(client)
        task_name = _lifecycle_task(coordinator)
        manipulation = _manipulation(client)
        _execute_ready_pose(coordinator, task_name, manipulation)
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
        coordinator = _coordinator(client)
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
        typer.echo(
            "G1 trajectory cancelled and policy disarmed into current-pose hold. "
            "Run `dimos stop` to stop low-level motor commands."
        )
    except (AttributeError, KeyError, RuntimeError) as exc:
        _abort(f"failed to disable G1: {exc}")
    finally:
        client.stop()
