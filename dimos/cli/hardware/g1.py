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

"""Safe operator commands for a running Unitree G1 GR00T stack."""

from __future__ import annotations

import time
from typing import Any, NoReturn, Protocol, TypeGuard

import typer

from dimos.control.tasks.trajectory_task.trajectory_task import JOINT_TRAJECTORY_TASK_NAME
from dimos.porcelain.dimos import Dimos
from dimos.porcelain.module_handle import ModuleHandle

app = typer.Typer(help="Operate a running Unitree G1 stack safely")

_COORDINATOR = "ControlCoordinator"
_GROOT_TASK = "groot_wbc"
_ARM_POLL_SECONDS = 0.1


class _G1CoordinatorHandle(Protocol):
    def task_invoke(self, task_name: str, method: str, kwargs: dict[str, Any]) -> Any: ...
    def set_dry_run(self, dry_run: bool) -> Any: ...
    def set_activated(self, activated: bool) -> Any: ...
    def cancel_trajectory(self) -> Any: ...


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
        ("task_invoke", "set_dry_run", "set_activated", "cancel_trajectory"),
    )


def _coordinator(client: Dimos) -> _G1CoordinatorHandle:
    handle = client.get_module(_COORDINATOR)
    if not _is_coordinator(handle):
        _abort("the running stack does not expose the required G1 coordinator RPCs")
    return handle


def _is_groot_state(value: Any) -> TypeGuard[dict[str, Any]]:
    return isinstance(value, dict)


def _groot_state(coordinator: _G1CoordinatorHandle) -> dict[str, Any]:
    state = coordinator.task_invoke(_GROOT_TASK, "state_snapshot", {})
    if not _is_groot_state(state):
        _abort("the running stack does not expose G1 GR00T safety state")
    return state


def _fully_armed(state: dict[str, Any]) -> bool:
    return bool(state.get("armed") and not state.get("arming") and not state.get("arm_pending"))


def _arm_and_wait(coordinator: _G1CoordinatorHandle, timeout: float) -> dict[str, Any]:
    coordinator.set_dry_run(True)
    coordinator.set_activated(True)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        state = _groot_state(coordinator)
        if _fully_armed(state):
            return state
        time.sleep(_ARM_POLL_SECONDS)
    _abort(f"G1 did not finish arming within {timeout:g}s; motor output remains in dry-run")


def _enable_motor_output(
    coordinator: _G1CoordinatorHandle,
    state: dict[str, Any] | None = None,
) -> dict[str, Any]:
    current = state if state is not None else _groot_state(coordinator)
    if not _fully_armed(current):
        _abort("G1 is not fully armed; run `dimos hardware g1 arm` first")
    coordinator.set_dry_run(False)
    try:
        enabled = _groot_state(coordinator)
        if enabled.get("dry_run"):
            _abort("G1 remained in dry-run after the enable request")
    except BaseException as enable_error:
        try:
            coordinator.set_dry_run(True)
        except BaseException as restore_error:
            raise RuntimeError(
                f"enable verification failed ({enable_error}); "
                f"failed to restore dry-run ({restore_error})"
            ) from restore_error
        raise
    return enabled


@app.command()
def status() -> None:
    """Show the G1 safety and arm trajectory state."""
    client = _connect()
    try:
        coordinator = _coordinator(client)
        state = _groot_state(coordinator)
        trajectory = coordinator.task_invoke(
            JOINT_TRAJECTORY_TASK_NAME, "get_status", {"t_now": None}
        )
        typer.echo(f"active:      {bool(state.get('active'))}")
        typer.echo(f"armed:       {bool(state.get('armed'))}")
        typer.echo(f"arming:      {bool(state.get('arming') or state.get('arm_pending'))}")
        typer.echo(f"dry_run:     {bool(state.get('dry_run'))}")
        typer.echo(f"trajectory:  {trajectory}")
    except (AttributeError, KeyError, RuntimeError) as exc:
        _abort(f"running stack is not a compatible G1 GR00T stack: {exc}")
    finally:
        client.stop()


@app.command()
def arm(timeout: float = typer.Option(15.0, min=0.1, help="Arming timeout in seconds.")) -> None:
    """Run the GR00T pose ramp, then keep policy output in dry-run."""
    client = _connect()
    try:
        coordinator = _coordinator(client)
        _arm_and_wait(coordinator, timeout)
        typer.echo("G1 armed in dry-run; inspect the robot, then run `dimos hardware g1 enable`.")
    except (AttributeError, KeyError, RuntimeError) as exc:
        _abort(f"failed to arm G1: {exc}")
    finally:
        client.stop()


@app.command()
def enable() -> None:
    """Enable motor output after a completed dry-run arming ramp."""
    client = _connect()
    try:
        coordinator = _coordinator(client)
        _enable_motor_output(coordinator)
        typer.echo("G1 motor output enabled.")
    except (AttributeError, KeyError, RuntimeError) as exc:
        _abort(f"failed to enable G1: {exc}")
    finally:
        client.stop()


@app.command()
def activate(
    timeout: float = typer.Option(15.0, min=0.1, help="Arming timeout in seconds."),
) -> None:
    """Arm, confirm physical safety, and enable live GR00T output."""
    client = _connect()
    motor_output_enabled = False
    try:
        coordinator = _coordinator(client)
        state = _groot_state(coordinator)
        if not _fully_armed(state):
            state = _arm_and_wait(coordinator, timeout)

        if state.get("dry_run"):
            typer.echo(
                "Arming ramp complete. Inspect the robot and confirm the remote and E-stop "
                "are ready."
            )
            if not typer.confirm("Enable live GR00T motor output?", default=False):
                typer.echo("Activation cancelled; G1 remains armed in dry-run.")
                raise typer.Exit(1)
            _enable_motor_output(coordinator, state)
            motor_output_enabled = True
            typer.echo("G1 motor output enabled.")
        else:
            motor_output_enabled = True
            typer.echo("G1 is already activated.")

        if state.get("dry_run"):
            typer.echo("G1 activated.")
    except (AttributeError, KeyError, RuntimeError) as exc:
        suffix = "; GR00T motor output remains enabled" if motor_output_enabled else ""
        _abort(f"failed to activate G1: {exc}{suffix}")
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
        typer.echo("G1 trajectory cancelled, motor output disabled, and controller disarmed.")
    except (AttributeError, KeyError, RuntimeError) as exc:
        _abort(f"failed to disable G1: {exc}")
    finally:
        client.stop()
