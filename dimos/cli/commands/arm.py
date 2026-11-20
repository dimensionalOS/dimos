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

"""The ``dimos arm`` commands: calibrate an arm, show its calibration, bake it into a URDF.

``dimos arm calibrate`` serves the calibration module's operator RPCs
(``ArmCalibrateOperator/show``, ``/ask`` and ``/done``), then runs
``dimos run <robot>-calibrate`` as its child and answers the module from the keyboard.
``q`` or Ctrl-C calls ``CalibrationModule/abort``: the arm holds where it is and going home
is offered. The child is stopped once the module is done, and on any way out.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable, Mapping
import itertools
import json
import os
from pathlib import Path
import queue
import shlex
import signal
import subprocess
import sys
import threading
from typing import Any

import typer

arm_app = typer.Typer(
    help="Calibrate an arm (gravity, gains, checks), show its calibration, bake it into a URDF.",
    no_args_is_help=True,
)


class Operator:
    """The terminal side of the operator RPCs; ``line`` takes each line the operator types."""

    def __init__(self, abort: Callable[[], None]) -> None:
        self._abort = abort
        self._lines: queue.Queue[str] = queue.Queue()
        self._asking = threading.Event()
        self.finished = threading.Event()
        self.ok = False
        self.home = True

    def show(self, text: str) -> None:
        typer.echo(text)

    def ask(self, prompt: str, choices: list[str]) -> str:
        """Wait for a line: Enter is the first choice, a choice's first letter that one, q aborts."""
        keys = "  ".join(
            [f"[Enter] {choices[0]}", *(f"[{c[0]}] {c}" for c in choices[1:]), "[q] abort"]
        )
        self._lines = queue.Queue()
        self._asking.set()
        typer.echo(f"{prompt}\n  {keys}  > ", nl=False)
        try:
            while True:
                key = self._lines.get()
                if key == "q":
                    return "abort"
                choice = choices[0] if not key else next((c for c in choices if c[0] == key), None)
                if choice is not None:
                    return choice
                typer.echo(f"  {keys}  > ", nl=False)
        finally:
            self._asking.clear()

    def done(self, ok: bool, text: str, home: bool) -> None:
        typer.echo(f"\nCalibration {'done' if ok else 'not done'}:\n{text}")
        self.ok, self.home = ok, home
        self.finished.set()

    def line(self, text: str) -> None:
        key = text.strip().lower()
        if key == "q":
            self._abort()
        if self._asking.is_set():
            self._lines.put(key)


def walk(argv: list[str], lines: Iterable[str]) -> bool:
    """Serve the operator RPCs, run ``argv`` as a child until the module is done; whether it succeeded."""
    from dimos.control.calibration.module import OPERATOR
    from dimos.core.transport_factory import rpc_backend

    rpc = rpc_backend()()
    rpc.start()

    def aborted(reply: Any) -> None:
        if isinstance(reply, BaseException):
            typer.echo(f"Abort failed: {reply}", err=True)

    def abort() -> None:
        rpc.call("CalibrationModule/abort", ([], {}), aborted)

    operator = Operator(abort)

    def interrupted(signum: int, frame: object) -> None:
        typer.echo("\nAborting: the arm holds where it is.")
        operator.line("q")

    previous = signal.signal(signal.SIGINT, interrupted)
    try:
        for name in ("show", "ask", "done"):
            rpc.serve_rpc(getattr(operator, name), f"{OPERATOR}/{name}")
        typer.echo("Starting " + shlex.join(argv))
        child = subprocess.Popen(argv, stdin=subprocess.DEVNULL, start_new_session=True)

        def watch() -> None:
            code = child.wait()
            if not operator.finished.is_set():
                typer.echo(f"\ndimos run exited ({code}) before the calibration was done", err=True)
                operator.finished.set()

        def read() -> None:
            for text in lines:
                operator.line(text)
            if not operator.finished.is_set():
                operator.line("q")

        threading.Thread(target=watch, daemon=True).start()
        threading.Thread(target=read, daemon=True).start()
        try:
            operator.finished.wait()
            if child.poll() is None and not operator.home:
                operator.ask(
                    "The arm is not home, and stopping the run turns its motors off. Support it.",
                    ["stop"],
                )
        finally:
            signal.signal(signal.SIGINT, previous)
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:
                    child.wait()
                except KeyboardInterrupt:
                    child.kill()
                    child.wait()
                    raise
    finally:
        signal.signal(signal.SIGINT, previous)
        rpc.stop()
    return operator.ok


def dimos_command() -> list[str]:
    """The ``dimos`` executable of the environment this process runs in."""
    script = Path(sys.executable).parent / "dimos"
    if script.exists():
        return [str(script)]
    return [sys.executable, "-c", "from dimos.cli.entry import main; main()"]


def use_run_globals(tokens: list[str], overrides: Mapping[str, Any]) -> None:
    """Resolve the global options the run will use, as ``dimos run`` does, so both share a bus."""
    from dimos.cli.commands.lifecycle import DEFAULT_CONFIG_PATH
    from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
    from dimos.core.global_config import global_config

    config = next(
        (Path(b) for a, b in itertools.pairwise(tokens) if a in ("--config", "-c")),
        DEFAULT_CONFIG_PATH,
    )
    global_config.update(
        **BlueprintConfigParser.preparse_global_config(
            tokens, config_path=config, environ=os.environ, global_overrides=overrides
        )
    )


def _split(text: str | None, kind: type) -> list[Any] | None:
    return (
        None if text is None else [kind(item.strip()) for item in text.split(",") if item.strip()]
    )


@arm_app.command(
    "calibrate", context_settings={"allow_extra_args": True, "ignore_unknown_options": True}
)
def calibrate_cmd(
    ctx: typer.Context,
    robot: str = typer.Argument(..., help="Robot: openyam, dual-openyam, metal, dual-metal, ..."),
    arm: str | None = typer.Option(None, help="Which arm, e.g. left, on a robot with several"),
    floor_z_m: float | None = typer.Option(None, help="Table height from the arm's base, m"),
    min_x_m: float | None = typer.Option(None, help="The fingertip stays in front of this x, m"),
    poses: int | None = typer.Option(None, help="Gravity poses to visit"),
    speed_deg_s: float | None = typer.Option(None, help="Peak joint speed, deg/s"),
    rest_window_s: float | None = typer.Option(None, help="How long a rest measures noise, s"),
    dither_deg: float | None = typer.Option(None, help="Dither at each gravity pose, deg"),
    kp_scales: str | None = typer.Option(None, help="kp multiples the sweep tries, e.g. 1,1.5,2"),
    step_deltas_deg: str | None = typer.Option(None, help="Sweep steps in turn, e.g. 1,-1,3,-3"),
    max_overshoot_pct: float | None = typer.Option(None, help="A step may overshoot this, %"),
    float_drift_limit_mm: float | None = typer.Option(None, help="Fingertip drift allowed, mm"),
    confirm_poses: int | None = typer.Option(None, help="Poses confirm_tracking revisits"),
    grid_points: int | None = typer.Option(None, help="Candidate poses the search considers"),
    steps: str | None = typer.Option(None, help="Comma-separated steps to run after check"),
    confirm_each: bool | None = typer.Option(
        None, "--confirm-each/--no-confirm-each", help="Ask before every move (default: ask)"
    ),
    dry_run: bool = typer.Option(False, "--dry-run", help="Print the command and plan only"),
) -> None:
    """Calibrate one arm from this terminal; extra flags go to `dimos run <robot>-calibrate`."""
    from dimos.control.calibration.module import CalibrationModuleConfig
    from dimos.control.calibration.robots import KNOWN_ROBOTS
    from dimos.control.calibration.steps import TITLES, plan

    given = {
        "arm": arm,
        "floor_z_m": floor_z_m,
        "min_x_m": min_x_m,
        "poses": poses,
        "speed_deg_s": speed_deg_s,
        "rest_window_s": rest_window_s,
        "dither_deg": dither_deg,
        "kp_scales": _split(kp_scales, float),
        "step_deltas_deg": _split(step_deltas_deg, float),
        "max_overshoot_pct": max_overshoot_pct,
        "float_drift_limit_mm": float_drift_limit_mm,
        "confirm_poses": confirm_poses,
        "grid_points": grid_points,
        "steps": _split(steps, str),
        "confirm_each": confirm_each,
    }
    settings: dict[str, Any] = {name: value for name, value in given.items() if value is not None}
    try:
        config = CalibrationModuleConfig(robot=robot, **settings)
        spec = config.arm_spec()
    except ValueError as error:
        typer.echo(f"Error: {error}", err=True)
        raise typer.Exit(2) from error
    flags: list[str] = []
    for name, value in settings.items():
        option = f"--calibrationmodule.{name.replace('_', '-')}"
        if isinstance(value, bool):
            flags.append(f"{option}={str(value).lower()}")
        else:
            flags += [
                option,
                json.dumps(value, separators=(",", ":")) if isinstance(value, list) else str(value),
            ]
    overrides: dict[str, Any] = dict(ctx.obj or {})
    root: list[str] = []
    for name, value in overrides.items():
        option = name.replace("_", "-")
        root.append(
            f"--{'' if value else 'no-'}{option}"
            if isinstance(value, bool)
            else f"--{option}={value}"
        )
    argv = [*dimos_command(), *root, "run", f"{robot}-calibrate", *flags, *ctx.args]
    typer.echo(f"Plan for {spec.arm_id}:")
    for step, reason in plan(config.steps, KNOWN_ROBOTS[robot].gravity):
        typer.echo(f"  {step:<17} {TITLES[step] if reason is None else 'skipped: ' + reason}")
    if dry_run:
        typer.echo("Would run:\n  " + shlex.join(argv) + "\nNothing was started.")
        return
    use_run_globals(ctx.args, overrides)
    raise typer.Exit(0 if walk(argv, sys.stdin) else 1)


def _arm_ids(robot: str, arm: str | None) -> list[str]:
    from dimos.control.calibration.robots import KNOWN_ROBOTS

    ids = KNOWN_ROBOTS[robot].arm_ids() if robot in KNOWN_ROBOTS else []
    chosen = [i for i in ids if arm in (None, i, i.rsplit("/", 1)[-1])]
    if not chosen:
        typer.echo(
            f"Error: no arm {arm or ''} of {robot}; known: {', '.join(KNOWN_ROBOTS)}", err=True
        )
        raise typer.Exit(2)
    return chosen


@arm_app.command("show")
def show_cmd(
    robot: str = typer.Argument(..., help="Robot: openyam, dual-openyam, ..."),
    arm: str | None = typer.Option(None, help="Only this arm, e.g. left"),
) -> None:
    """Print the saved calibration of a robot's arms and the files it replaced."""
    from dimos.control.calibration.schema import (
        ARMS_STATE_DIR,
        HISTORY_DIR_NAME,
        ArmCalibrationError,
        calibration_path,
        load_calibration,
    )

    unreadable = False
    for arm_id in _arm_ids(robot, arm):
        path = calibration_path(arm_id, ARMS_STATE_DIR)
        typer.echo(f"{arm_id}: {path}")
        if not path.exists():
            typer.echo("  none: the arm runs its robot's gains and URDF masses")
            continue
        try:
            calibration = load_calibration(path)
        except ArmCalibrationError as error:
            typer.echo(f"  unreadable: {error}")
            unreadable = True
            continue
        typer.echo(
            f"  created {calibration.created:%Y-%m-%d %H:%M} UTC for {calibration.robot_model.name}"
        )
        for joint, gains in sorted((calibration.gains or {}).items()):
            typer.echo(f"  {joint}: kp {gains.kp:g} kd {gains.kd:g}")
        for name, link in sorted(calibration.gravity.links.items() if calibration.gravity else []):
            com = " ".join(f"{v:+.3f}" for v in link.com_m)
            typer.echo(f"  {name}: {link.mass_kg:.3f} kg at {com} m")
        for item in sorted((path.parent / HISTORY_DIR_NAME).glob("*.json")):
            typer.echo(f"  replaced: {item}")
    if unreadable:
        raise typer.Exit(1)


@arm_app.command("bake")
def bake_cmd(
    robot: str = typer.Argument(..., help="Robot: openyam, dual-openyam, ..."),
    urdf: Path = typer.Option(..., exists=True, dir_okay=False, help="URDF to read"),
    out: Path = typer.Option(..., help="URDF to write"),
    arm: str | None = typer.Option(None, help="Only this arm, e.g. left"),
) -> None:
    """Write a URDF with the fitted link masses and centres of mass of the robot's arms."""
    from dimos.control.calibration.schema import (
        ARMS_STATE_DIR,
        ArmCalibrationError,
        bake_into_urdf,
        calibration_path,
        load_calibration,
    )

    ids = _arm_ids(robot, arm)
    try:
        baked = bake_into_urdf(
            [load_calibration(calibration_path(i, ARMS_STATE_DIR)) for i in ids], urdf.read_text()
        )
    except ArmCalibrationError as error:
        typer.echo(f"Error: {error}", err=True)
        raise typer.Exit(1) from error
    out.write_text(baked)
    typer.echo(f"Wrote {out} with the fitted masses of {', '.join(ids)}.")
