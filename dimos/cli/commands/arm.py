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

"""The `dimos arm` commands: calibrate an arm, show its calibration, bake it into a URDF.

``dimos arm calibrate`` is a terminal walk-through of the calibration module
(``dimos.robot.manipulators.calibration``). It starts ``dimos run
<robot>-calibrate --daemon`` as a subprocess (recording only happens on the
``dimos run`` path), reaches the running ``CalibrationModule`` through
``Dimos.connect()``, and answers its confirmations from the keyboard. It
refuses to start while any other dimos run is active, and stops only the run
it started.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import pwd
import queue
import re
import subprocess
import sys
import threading
import time
from typing import Any, Protocol

import typer

arm_app = typer.Typer(
    help="Calibrate an arm (gravity, gains, checks), inspect its calibration, bake it into a URDF.",
    no_args_is_help=True,
)

CONNECT_TIMEOUT_S = 180.0
LAUNCH_TIMEOUT_S = 600.0
ABORT_WAIT_S = 30.0
POLL_S = 0.2
FINISHED = ("done", "failed", "aborted")
# The blueprint's own default arm, when the robot has several and none is given.
BLUEPRINT_DEFAULT_ARM = {"dual-openyam": "left"}
RUN_ID_PATTERN = re.compile(r"Run ID:\s+(\S+)")


def echo(text: str = "", *, nl: bool = True, err: bool = False) -> None:
    typer.echo(text, nl=nl, err=err)


# The seams: everything that touches processes, the run registry or the bus.


def arms_root() -> Path:
    from dimos.robot.manipulators.calibration.schema import ARMS_STATE_DIR

    return ARMS_STATE_DIR


@dataclass(frozen=True)
class ActiveRun:
    """A live dimos run on this machine."""

    run_id: str
    pid: int
    blueprint: str
    owner: str | None
    started_at: str
    log_dir: str


def _owner(pid: int) -> str | None:
    try:
        return pwd.getpwuid(Path(f"/proc/{pid}").stat().st_uid).pw_name
    except (OSError, KeyError):
        return None


def active_runs() -> list[ActiveRun]:
    """Every live run in the run registry (``dimos status`` shows the latest of these)."""
    from dimos.core.run_registry import list_runs

    return [
        ActiveRun(e.run_id, e.pid, e.blueprint, _owner(e.pid), e.started_at, e.log_dir)
        for e in list_runs(alive_only=True)
    ]


def dimos_command() -> list[str]:
    """The ``dimos`` executable of the environment this process runs in."""
    script = Path(sys.executable).parent / "dimos"
    if script.exists():
        return [str(script)]
    return [sys.executable, "-c", "from dimos.cli.entry import main; main()"]


def launch_run(argv: list[str]) -> str:
    """Run ``argv`` (a ``dimos run ... --daemon``) to its end and return the run id it reports."""
    before = {run.run_id for run in active_runs()}
    done = subprocess.run(
        argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=LAUNCH_TIMEOUT_S
    )
    if done.returncode != 0:
        raise RuntimeError(f"dimos run exited with {done.returncode}:\n{done.stdout.strip()}")
    found = RUN_ID_PATTERN.search(done.stdout)
    if found:
        return found.group(1)
    new = [run for run in active_runs() if run.run_id not in before]
    if len(new) == 1:
        return new[0].run_id
    raise RuntimeError(
        "dimos run started but did not say which run it is; check `dimos status` and stop it "
        f"with `dimos stop` yourself.\n{done.stdout.strip()}"
    )


def stop_run(run_id: str) -> str:
    """Stop the run ``run_id`` and nothing else; what happened, in words."""
    from dimos.core.run_registry import list_runs, stop_entry

    for entry in list_runs(alive_only=True):
        if entry.run_id == run_id:
            message, _ok = stop_entry(entry)
            return f"stopped {run_id} ({message})"
    return f"{run_id} is not running any more"


class ModuleConnectError(RuntimeError):
    """The calibration module did not appear on the bus in time."""


def connect_module(timeout_s: float) -> Any:
    """The ``CalibrationModule`` proxy of the running coordinator, once it answers ``plan()``."""
    from dimos.porcelain.dimos import Dimos

    deadline = time.monotonic() + timeout_s
    last: Exception | None = None
    while time.monotonic() < deadline:
        try:
            module: Any = Dimos.connect().get_module("CalibrationModule")
            module.plan()
            return module
        except Exception as error:
            last = error
            time.sleep(1.0)
    raise ModuleConnectError(
        f"the CalibrationModule did not answer within {timeout_s:g} s (last error: {last})"
    )


EOF = object()


class KeySource(Protocol):
    """Lines the operator types, lowercased: "" is Enter, ``EOF`` the end of input, ``None`` nothing yet."""

    def drain(self) -> None: ...

    def get(self) -> object: ...


class StdinKeys:
    """Reads stdin lines on a thread so the poll loop never blocks on the keyboard."""

    def __init__(self, stream: Any = None) -> None:
        self._stream = sys.stdin if stream is None else stream
        self._lines: queue.Queue[object] = queue.Queue()
        threading.Thread(target=self._read, name="arm-calibrate-keys", daemon=True).start()

    def _read(self) -> None:
        try:
            for line in self._stream:
                self._lines.put(line.strip().lower())
        except (OSError, ValueError):
            pass
        self._lines.put(EOF)

    def drain(self) -> None:
        """Forget what was typed ahead of a prompt: it was not an answer to it."""
        pending: list[object] = []
        while True:
            try:
                item = self._lines.get_nowait()
            except queue.Empty:
                break
            pending.append(item)
        if pending and pending[-1] is EOF:
            self._lines.put(EOF)

    def get(self) -> object:
        """The next typed line, ``EOF`` at the end of input, or ``None`` when nothing was typed."""
        try:
            item = self._lines.get_nowait()
        except queue.Empty:
            return None
        if item is EOF:
            self._lines.put(EOF)
        return item


def make_keys() -> KeySource:
    return StdinKeys()


# Printing


def table(rows: list[list[str]], indent: str = "    ") -> None:
    if not rows:
        return
    widths = [max(len(row[i]) for row in rows) for i in range(len(rows[0]))]
    for row in rows:
        echo(indent + "  ".join(cell.ljust(widths[i]) for i, cell in enumerate(row)).rstrip())


def print_plan(plan: list[dict[str, Any]], selected: list[str] | None) -> None:
    echo("Plan:")
    for step in plan:
        if selected is not None and step["id"] not in selected:
            echo(f"  - {step['id']}: {step['title']} (not selected)")
        elif step["supported"]:
            echo(f"  ✓ {step['id']}: {step['title']}")
        else:
            echo(f"  x {step['id']}: {step['title']}  (cannot run: {step['reason']})")


def _f(value: Any, digits: int = 3) -> str:
    return "-" if value is None else f"{float(value):.{digits}f}"


def print_results(step_id: str, data: dict[str, Any]) -> None:
    """A compact table of what a step measured."""
    if step_id == "gravity_fit":
        rms = data.get("holdout_rms_nm") or {}
        if rms:
            echo("    hold-out RMS torque error, Nm (before -> after):")
            table([[j, f"{_f(r['before'])} -> {_f(r['after'])}"] for j, r in rms.items()])
        if data.get("at_bounds"):
            echo("    at a bound (not pinned down by the poses): " + "; ".join(data["at_bounds"]))
        for reason in data.get("refusals") or []:
            echo(f"    not saved: {reason}")
        if data.get("saved"):
            applied = "applied" if data.get("applied") else "NOT applied"
            echo(f"    saved {data['saved']} ({applied})")
    elif step_id == "float_check":
        joints = data.get("joints") or {}
        if joints:
            echo(f"    fingertip drift while floating, at {data.get('pose', 'the float pose')}:")
            rows = [["joint", "drift mm", "result"]]
            for joint, row in joints.items():
                if row.get("skipped"):
                    rows.append([joint, "-", "skipped"])
                elif "fingertip_drift_mm" in row:
                    verdict = "pass" if row.get("passed") else "FAIL"
                    rows.append([joint, _f(row["fingertip_drift_mm"], 2), verdict])
                else:
                    rows.append([joint, "-", f"FAIL: {row.get('reason', '')}"])
            table(rows)
    elif step_id == "pick_gains":
        proposals = data.get("proposals") or {}
        if proposals:
            echo("    proposed gains:")
            rows = [["joint", "kp", "kd", "why"]]
            for joint, row in proposals.items():
                rows.append([joint, _f(row.get("kp"), 1), _f(row.get("kd"), 2), row["reason"]])
            table(rows)
            for joint, row in proposals.items():
                for tried, why in (row.get("rejected") or {}).items():
                    echo(f"    rejected {joint} {tried}: {why}")
        if data.get("saved"):
            echo(f"    saved {data['saved']}")
    elif step_id == "confirm_tracking":
        rest, ramp = data.get("rest_rms_deg") or {}, data.get("ramp_rms_deg") or {}
        base = (data.get("baseline") or {}).get("rest_rms_deg") or {}
        if rest:
            echo("    tracking RMS, deg (resting; baseline from the gravity poses; moving):")
            rows = [["joint", "rest", "baseline", "moving"]]
            for joint, value in rest.items():
                rows.append([joint, _f(value), _f(base.get(joint)), _f(ramp.get(joint))])
            table(rows)
    elif step_id == "summary":
        for item in data.get("next_steps") or []:
            echo(f"    next: {item}")


# The walk-through


@dataclass
class Outcome:
    state: str
    message: str
    results: dict[str, Any]


class Wizard:
    """Drives one calibration run over the module's RPCs and the keyboard."""

    def __init__(self, module: Any, keys: KeySource) -> None:
        self.module = module
        self.keys = keys
        self._prompted = -1
        self._seen_steps: dict[str, str] = {}
        self._last_line = ""

    def _answer(self, status: dict[str, Any]) -> None:
        """Show a waiting prompt once, then act on the key typed for it."""
        prompt_id = status["prompt_id"]
        if prompt_id != self._prompted:
            self._prompted = prompt_id
            self.keys.drain()
            echo(f"\n? {status['prompt']}")
            echo("  [Enter] go on   [s] skip   [q] abort (the arm holds where it is)  > ", nl=False)
            sys.stdout.flush()
            return
        key = self.keys.get()
        if key is None:
            return
        if key is EOF:
            echo("(input closed, aborting)")
            self.module.abort()
        elif key == "":
            self.module.confirm(prompt_id)
        elif key == "s":
            self.module.skip(prompt_id)
        elif key in ("q", "p"):
            echo("aborting; the arm holds where it is")
            self.module.abort()
        else:
            echo(f"  not understood: {key!r}; Enter, s or q  > ", nl=False)

    def _report(self, status: dict[str, Any]) -> None:
        for step in status["steps"]:
            before = self._seen_steps.get(step["id"])
            if step["status"] != before and step["status"] in (
                "done",
                "skipped",
                "failed",
                "unsupported",
            ):
                self._seen_steps[step["id"]] = step["status"]
                mark = {"done": "✓", "skipped": "-", "failed": "x", "unsupported": "x"}
                echo(f"\n{mark[step['status']]} {step['title']}: {step['message']}")
                if step["status"] == "done":
                    data = self.module.results().get(step["id"])
                    if data:
                        print_results(step["id"], data)
        if status["state"] == "running":
            progress = status.get("progress")
            line = status["message"]
            if progress:
                line = f"[{progress['done']}/{progress['total']}] {progress['label']}"
            if line != self._last_line and status.get("step"):
                self._last_line = line
                echo(f"  {line}")

    def drive(self, status: dict[str, Any]) -> dict[str, Any]:
        """Poll (about 5 Hz) and answer until the state is done, failed or aborted."""
        while True:
            self._report(status)
            if status["state"] == "waiting_confirm":
                self._answer(status)
            if status["state"] in FINISHED:
                return status
            time.sleep(POLL_S)
            status = self.module.status()

    def run(self, steps: list[str] | None) -> Outcome:
        status = self.drive(self.module.run(steps))
        return Outcome(status["state"], status["message"], self.module.results())

    def wait_aborted(self) -> dict[str, Any]:
        """After ``abort()``: wait for the engine to say it stopped."""
        deadline = time.monotonic() + ABORT_WAIT_S
        status: dict[str, Any] = self.module.status()
        while status["state"] not in FINISHED and time.monotonic() < deadline:
            time.sleep(POLL_S)
            status = self.module.status()
        return status

    def offer_home(self) -> None:
        """``h`` goes home (after its own confirmation), Enter or anything else finishes."""
        echo("\nThe arm holds where it is. [h] go home   [Enter] finish  > ", nl=False)
        sys.stdout.flush()
        self.keys.drain()
        while True:
            key = self.keys.get()
            if key is None:
                time.sleep(POLL_S)
                continue
            break
        if key != "h":
            echo()
            return
        self._prompted = -1
        status = self.drive(self.module.go_home())
        echo(f"\n{status['message']}")


def saved_files(results: dict[str, Any], arm_id: str) -> Path | None:
    """The calibration file this run wrote, from the steps' results, else None."""
    for step in ("summary", "pick_gains", "gravity_fit"):
        data = results.get(step) or {}
        path = data.get("file") if step == "summary" else data.get("saved")
        if path:
            return Path(path)
    return None


def print_ending(outcome: Outcome, arm_id: str, connection_hint: str) -> None:
    echo(f"\nRun {outcome.state}: {outcome.message}")
    path = saved_files(outcome.results, arm_id)
    if path is None:
        echo("No calibration file was written by this run.")
        return
    echo(f"Calibration file: {path}")
    echo(f"Earlier files:    {path.parent / 'history'}/")
    echo("It is applied every time the robot starts. To undo it, delete the file (the previous")
    echo(f"one is in history/) or start the robot with `--{connection_hint}.calibration off`.")


def _split_steps(steps: str | None, known: list[str]) -> list[str] | None:
    if steps is None:
        return None
    chosen = [s.strip() for s in steps.split(",") if s.strip()]
    unknown = [s for s in chosen if s not in known]
    if unknown:
        echo(f"Unknown steps {unknown}; the steps are {', '.join(known)}", err=True)
        raise typer.Exit(2)
    return chosen


def _check_robot(robot: str) -> None:
    from dimos.robot.manipulators.calibration.robots import KNOWN_ROBOTS

    if robot not in KNOWN_ROBOTS:
        echo(f"Unknown robot {robot!r}. Available: {', '.join(KNOWN_ROBOTS)}", err=True)
        raise typer.Exit(2)


def _precheck(
    robot: str, arm: str | None, floor_z_m: float | None, min_x_m: float | None
) -> tuple[Any, list[dict[str, Any]] | None]:
    """The module's own config check and workspace-guard rule, before anything starts.

    Returns the arm's spec and the offline plan, or ``(None, None)`` when the robot model is
    not on this machine (the engine's check step says so at start).
    """
    from pydantic import ValidationError

    from dimos.robot.manipulators.calibration.module import CalibrationModuleConfig
    from dimos.robot.manipulators.calibration.procedure import Capabilities, Procedure
    from dimos.robot.manipulators.calibration.robots import KNOWN_ROBOTS
    from dimos.robot.manipulators.calibration.spec import NoWorkspaceGuardError
    from dimos.robot.manipulators.calibration.steps import calibration_steps

    try:
        config = CalibrationModuleConfig(robot=robot, arm=arm, floor_z_m=floor_z_m, min_x_m=min_x_m)
    except ValidationError as error:
        echo(f"Error: {error.errors()[0]['msg'].removeprefix('Value error, ')}", err=True)
        raise typer.Exit(2) from error
    try:
        spec = config.arm_spec()
    except Exception as error:
        echo(f"(could not load the {robot} model here to check it first: {error})")
        return None, None
    try:
        spec.require_workspace_guard()
    except NoWorkspaceGuardError as error:
        echo(f"Error: {error}", err=True)
        echo("Give the table height and the front line: --floor-z-m F --min-x-m X", err=True)
        raise typer.Exit(2) from error
    caps = Capabilities(gravity_compensation=KNOWN_ROBOTS[robot].gravity)
    # Never reaches an arm: only the plan, which needs no connection, is asked.
    plan = Procedure(spec, None, calibration_steps(), caps=caps).plan()  # type: ignore[arg-type]
    return spec, [dict(step) for step in plan]


def _refuse_if_busy() -> None:
    runs = active_runs()
    if not runs:
        return
    echo("Another dimos run is active on this machine; not starting a calibration.", err=True)
    for run in runs:
        owner = f", started by {run.owner}" if run.owner else ""
        echo(
            f"  {run.run_id}  pid {run.pid}  {run.blueprint}{owner}  since {run.started_at}",
            err=True,
        )
        echo(f"  log: {run.log_dir}", err=True)
    echo("Stop it first (`dimos stop`) only if it is yours.", err=True)
    raise typer.Exit(1)


@arm_app.command(
    "calibrate",
    context_settings={"allow_extra_args": True, "ignore_unknown_options": True},
)
def calibrate_cmd(
    ctx: typer.Context,
    robot: str = typer.Argument(..., help="Robot: openyam, dual-openyam, metal, ..."),
    arm: str | None = typer.Option(None, "--arm", help="Which arm, e.g. left (several-arm robots)"),
    floor_z_m: float | None = typer.Option(
        None, "--floor-z-m", help="Table height in the arm frame, m (arms with no guard)"
    ),
    min_x_m: float | None = typer.Option(
        None, "--min-x-m", help="Fingertip stays in front of this x, m (arms with no guard)"
    ),
    poses: int | None = typer.Option(None, "--poses", min=1, help="Gravity poses to visit"),
    confirm_each: bool = typer.Option(
        True, "--confirm-each/--no-confirm-each", help="Ask before every pose (default: ask)"
    ),
    steps: str | None = typer.Option(None, "--steps", help="Comma-separated steps to run"),
    speed_deg_s: float | None = typer.Option(None, "--speed-deg-s", help="Peak joint speed"),
    dry_run: bool = typer.Option(
        False, "--dry-run", help="Print the command and plan, start nothing"
    ),
) -> None:
    """Walk through the calibration of one arm in this terminal.

    Extra flags go to `dimos run <robot>-calibrate`, e.g.
    --dualopenyamconnection.left-can-port can0.
    """
    from dimos.robot.manipulators.calibration.steps import calibration_steps

    _check_robot(robot)
    known = [step.id for step in calibration_steps()]
    selected = _split_steps(steps, known)
    arm = arm or BLUEPRINT_DEFAULT_ARM.get(robot)
    blueprint = f"{robot}-calibrate"
    flags: list[str] = []
    if arm is not None:
        flags += ["--calibrationmodule.arm", arm]
    if floor_z_m is not None:
        flags += ["--calibrationmodule.floor-z-m", str(floor_z_m)]
    if min_x_m is not None:
        flags += ["--calibrationmodule.min-x-m", str(min_x_m)]
    if poses is not None:
        flags += ["--calibrationmodule.poses", str(poses)]
    if speed_deg_s is not None:
        flags += ["--calibrationmodule.speed-deg-s", str(speed_deg_s)]
    if not confirm_each:
        flags += ["--calibrationmodule.confirm-each=false"]
    argv = [*dimos_command(), "run", blueprint, "--daemon", *flags, *ctx.args]

    spec, plan = _precheck(robot, arm, floor_z_m, min_x_m)
    if dry_run:
        echo("Would run:")
        echo("  " + " ".join(argv))
        if plan is not None:
            print_plan(plan, selected)
        else:
            echo("Steps: " + ", ".join(selected or known))
        echo("Nothing was started.")
        return
    _refuse_if_busy()

    arm_id = "" if spec is None else spec.arm_id
    hint = f"{blueprint.removesuffix('-calibrate').replace('-', '')}connection"
    echo(f"Starting {blueprint} (this takes a minute)...")
    try:
        run_id = launch_run(argv)
    except Exception as error:
        echo(f"Error: {error}", err=True)
        raise typer.Exit(1) from error
    echo(f"Started run {run_id}; it is stopped when this walk-through ends.")

    code = 1
    try:
        code = _walk(run_id, arm_id, selected, hint)
    finally:
        echo(stop_run(run_id))
    raise typer.Exit(code)


def _walk(run_id: str, arm_id: str, selected: list[str] | None, hint: str) -> int:
    try:
        module = connect_module(CONNECT_TIMEOUT_S)
    except (ModuleConnectError, KeyboardInterrupt) as error:
        echo(f"Error: {error or 'interrupted'}; logs: `dimos log -r {run_id}`", err=True)
        return 1
    keys = make_keys()
    wizard = Wizard(module, keys)
    print_plan(module.plan(), selected)
    echo("\nAt every prompt: Enter goes on, s skips, q (or Ctrl-C) aborts and the arm holds.")
    outcome: Outcome | None = None
    try:
        outcome = wizard.run(selected)
    except KeyboardInterrupt:
        echo("\nCtrl-C: aborting; the arm holds where it is...")
        try:
            module.abort()
            status = wizard.wait_aborted()
            outcome = Outcome("aborted", status["message"], module.results())
        except Exception as error:
            echo(f"Could not confirm the abort ({error}); the arm's deadman holds it.", err=True)
            return 130
    if outcome is None:
        return 1
    print_ending(outcome, arm_id, hint)
    try:
        wizard.offer_home()
    except KeyboardInterrupt:
        echo("\nLeaving the arm where it is.")
    return {"done": 0, "aborted": 130}.get(outcome.state, 1)


# show and bake


def _fmt_time(value: datetime) -> str:
    return value.strftime("%Y-%m-%d %H:%M:%S %Z").strip()


def print_calibration(arm_id: str, root: Path) -> None:
    from dimos.robot.manipulators.calibration.schema import (
        HISTORY_DIR_NAME,
        ArmCalibrationError,
        calibration_path,
        load_calibration,
    )

    path = calibration_path(arm_id, root)
    echo(f"{arm_id}")
    echo(f"  file:    {path}")
    try:
        calibration = load_calibration(path)
    except ArmCalibrationError as error:
        echo("  none" if not path.exists() else f"  unreadable: {error}")
        return
    model = calibration.robot_model
    echo(f"  created: {_fmt_time(calibration.created)}")
    echo(f"  model:   {model.name} ({model.sha256[:12]})")
    if calibration.gains:
        echo("  gains:")
        table(
            [["joint", "kp", "kd"]]
            + [[j, f"{g.kp:g}", f"{g.kd:g}"] for j, g in sorted(calibration.gains.items())],
            "    ",
        )
    else:
        echo("  gains:   none (robot gains)")
    if calibration.gravity:
        echo("  link masses:")
        table(
            [["link", "mass kg", "com m"]]
            + [
                [name, f"{link.mass_kg:.3f}", " ".join(f"{v:+.3f}" for v in link.com_m)]
                for name, link in sorted(calibration.gravity.links.items())
            ],
            "    ",
        )
    else:
        echo("  link masses: none (URDF masses)")
    if calibration.friction_nm:
        echo(
            "  friction Nm: "
            + ", ".join(f"{j} {v:.3f}" for j, v in sorted(calibration.friction_nm.items()))
        )
    for name, check in sorted((calibration.checks or {}).items()):
        created = check.get("created", "?") if isinstance(check, dict) else "?"
        echo(f"  check {name}: {_check_line(name, check)} (at {created})")
    history = sorted((path.parent / HISTORY_DIR_NAME).glob("*.json"))
    echo(f"  history: {len(history)} earlier file(s)" + ("" if not history else " in"))
    for item in history:
        echo(f"    {item}")


def _check_line(name: str, check: Any) -> str:
    if not isinstance(check, dict):
        return str(check)
    if name == "float":
        rows = check.get("joints", {})
        tried = {j: r for j, r in rows.items() if "passed" in r}
        passed = sum(1 for r in tried.values() if r["passed"])
        return f"{passed} of {len(tried)} joints within {check.get('limit_mm', '?')} mm"
    if name == "tracking":
        rest = check.get("rest_rms_deg") or {}
        worst = max(rest.values(), default=0.0)
        return f"{check.get('poses', '?')} poses, worst resting RMS {worst:.3f} deg"
    return ", ".join(sorted(k for k in check if k != "created"))


@arm_app.command("show")
def show_cmd(
    robot: str = typer.Argument(..., help="Robot: openyam, dual-openyam, ..."),
    arm: str | None = typer.Option(None, "--arm", help="Only this arm, e.g. left"),
) -> None:
    """Print the saved calibration of a robot's arms."""
    from dimos.robot.manipulators.calibration.robots import KNOWN_ROBOTS

    _check_robot(robot)
    ids = KNOWN_ROBOTS[robot].arm_ids()
    if arm is not None:
        ids = [i for i in ids if arm in (i, i.rsplit("/", 1)[-1])]
        if not ids:
            names = [i.rsplit("/", 1)[-1] for i in KNOWN_ROBOTS[robot].arm_ids()]
            echo(f"{robot} has no arm {arm!r}; choose one of {names}", err=True)
            raise typer.Exit(2)
    for arm_id in ids:
        print_calibration(arm_id, arms_root())


@arm_app.command("bake")
def bake_cmd(
    robot: str = typer.Argument(..., help="Robot: openyam, dual-openyam, ..."),
    arm: str = typer.Option(..., "--arm", help="Which arm, e.g. left"),
    urdf: Path = typer.Option(..., "--urdf", exists=True, dir_okay=False, help="URDF to read"),
    out: Path = typer.Option(..., "--out", help="URDF to write"),
) -> None:
    """Write a URDF with the arm's fitted link masses and centres of mass."""
    from dimos.robot.manipulators.calibration.robots import KNOWN_ROBOTS
    from dimos.robot.manipulators.calibration.schema import (
        ArmCalibrationError,
        bake_into_urdf,
        calibration_path,
        load_calibration,
    )

    _check_robot(robot)
    ids = [i for i in KNOWN_ROBOTS[robot].arm_ids() if arm in (i, i.rsplit("/", 1)[-1])]
    if len(ids) != 1:
        names = [i.rsplit("/", 1)[-1] for i in KNOWN_ROBOTS[robot].arm_ids()]
        echo(f"{robot} has no arm {arm!r}; choose one of {names}", err=True)
        raise typer.Exit(2)
    try:
        calibration = load_calibration(calibration_path(ids[0], arms_root()))
        baked = bake_into_urdf(calibration, urdf.read_text())
    except ArmCalibrationError as error:
        echo(f"Error: {error}", err=True)
        raise typer.Exit(1) from error
    out.write_text(baked)
    echo(f"Wrote {out} with the fitted masses of {ids[0]}.")
