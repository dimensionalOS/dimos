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
(``dimos.control.calibration``). It starts ``dimos run
<robot>-calibrate --daemon`` as a subprocess (recording only happens on the
``dimos run`` path), reaches the running ``CalibrationModule`` through
``Dimos.connect()``, and answers its confirmations from the keyboard.

It refuses to start next to any other run it can see (this user's run
registry, and any coordinator answering on the bus), and holds a per-robot
lock while it runs. It signals only the run it started (``arm_run``).

Before it stops its run, the walk-through brings the arm home through the
module's ``go_home`` (the engine asks first). When the arm is not at home
after that (going home failed, was refused, or the operator declined), it
stops the run only once the operator types ``yes``; otherwise it leaves the
run up, the arm held where it is, and says so. The module's park on stop is
a backstop only.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator, Mapping
from contextlib import contextmanager
from dataclasses import dataclass
from datetime import datetime
import os
from pathlib import Path
import pwd
import queue
import signal
import sys
import threading
import time
from typing import Any, Protocol

import typer

from dimos.cli.commands.arm_run import (
    CalibrateLockedError,
    LaunchError,
    LaunchInterruptedError,
    OurRun,
    calibrate_lock,
    coordinator_answers,
    launch_run,
    new_token,
    run_alive,
    stop_run,
)

arm_app = typer.Typer(
    help="Calibrate an arm (gravity, gains, checks), inspect its calibration, bake it into a URDF.",
    no_args_is_help=True,
)

CONNECT_TIMEOUT_S = 180.0
# One Dimos.connect() attempt waits this long for the coordinator, then the next one this long.
CONNECT_ATTEMPT_S = 2.0
CONNECT_RETRY_S = 1.0
ABORT_WAIT_S = 30.0
# Going home may take this long, not counting the time its confirmation waits.
HOME_TIMEOUT_S = 60.0
POLL_S = 0.2
FINISHED = ("idle", "done", "failed", "aborted")
# What a walk-through that did not get a final state from the engine ends in.
GONE = "gone"
UNCONFIRMED = "unconfirmed"
EXIT_CODES = {"done": 0, "aborted": 130}
# q quits; p is the old calibration scripts' "park and quit". Both abort, and home is offered next.
ABORT_KEYS = ("q", "p")
# The module's RPCs answer at once (status, confirm, ...): a slower answer means trouble.
CALL_TIMEOUT_S = 2.0
# abort ends the arm's config session over RPC; run and go_home join a finished thread first.
CALL_TIMEOUTS = {"abort": 5.0, "run": 10.0, "go_home": 10.0}
WIZARD_CALLS = (
    "plan",
    "status",
    "results",
    "confirm",
    "skip",
    "abort",
    "run",
    "go_home",
    "dimos_run_id",
)
# The blueprint's own default arm, when the robot has several and none is given.
BLUEPRINT_DEFAULT_ARM = {"dual-openyam": "left"}

KEYS_HELP = (
    "\nAt every prompt: Enter goes on, s skips. At any time: q (or p, or Ctrl-C) aborts: the arm "
    "stops and holds where it is, its motors on, and going home is offered next."
)
PROMPT_KEYS = "  [Enter] go on   [s] skip   [q] abort (the arm holds where it is, motors on)  > "
OFFER = (
    "\nThe arm holds where it is, its motors on.\n"
    "[Enter or h] go home, then finish   [k] finish without going home  > "
)
BACKSTOP_NOTE = (
    "Stopping the run still tries its backstop park: the arm may move home on its own for a few "
    "seconds, and goes limp wherever that leaves it."
)
STOP_ANYWAY = (
    "Stop anyway: the arm will go limp where it is. Are you supporting it or is it on its "
    "supports? [yes/No] "
)


def echo(text: str = "", *, nl: bool = True, err: bool = False) -> None:
    """Print, even once the terminal has gone (SIGHUP): the walk-through must still finish."""
    try:
        typer.echo(text, nl=nl, err=err)
    except OSError:
        pass


# The seams: everything that touches processes, the run registry or the bus.


def arms_root() -> Path:
    from dimos.control.calibration.schema import ARMS_STATE_DIR

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
    """Every live run in this user's run registry (``dimos status`` shows the latest of these)."""
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


def use_run_globals(tokens: list[str], overrides: Mapping[str, Any]) -> None:
    """Resolve the global options the run will use, as ``dimos run`` does, and use them here too.

    The walk-through then connects on the bus its run serves, whether the
    transport came from a flag before ``arm``, one after the robot name, the
    environment or the config file.
    """
    from pydantic import ValidationError

    from dimos.cli.commands.lifecycle import DEFAULT_CONFIG_PATH
    from dimos.core.coordination.blueprint_config.errors import BlueprintConfigError
    from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
    from dimos.core.global_config import global_config

    try:
        values = BlueprintConfigParser.preparse_global_config(
            tokens,
            config_path=_config_path(tokens) or DEFAULT_CONFIG_PATH,
            environ=os.environ,
            global_overrides=overrides,
        )
        global_config.update(**values)
    except (BlueprintConfigError, ValidationError) as error:
        echo(f"Error: {error}", err=True)
        raise typer.Exit(2) from error


def _config_path(tokens: list[str]) -> Path | None:
    """The ``--config``/``-c`` given for ``dimos run``, if any."""
    for index, token in enumerate(tokens):
        if token.startswith("--config="):
            return Path(token.removeprefix("--config="))
        if token in ("--config", "-c") and index + 1 < len(tokens):
            return Path(tokens[index + 1])
    return None


class ModuleConnectError(RuntimeError):
    """The run's calibration module did not answer, or is not our run's."""


def set_call_timeouts(module: Any) -> None:
    """Give the walk-through's RPCs short timeouts instead of the two-minute default.

    The proxy's RPC client resolves a call's timeout from its ``rpc_timeouts``
    by full method name (``CalibrationModule/status``), as the engine does for
    its streamed calls; a fake module has no client and is left alone.
    """
    client = getattr(module, "rpc", None) or getattr(module, "_rpc", None)
    timeouts = getattr(client, "rpc_timeouts", None)
    if not isinstance(timeouts, dict):
        return
    remote = getattr(module, "remote_name", None) or "CalibrationModule"
    for name in WIZARD_CALLS:
        timeouts[f"{remote}/{name}"] = CALL_TIMEOUTS.get(name, CALL_TIMEOUT_S)


def connect_module(timeout_s: float, alive: Callable[[], bool]) -> Any:
    """The ``CalibrationModule`` proxy of the running coordinator, once it answers ``plan()``.

    Each failed attempt's connection is closed before the next. Gives up as
    soon as our run's process is gone.
    """
    from dimos.porcelain.dimos import Dimos

    deadline = time.monotonic() + timeout_s
    last: Exception | None = None
    while time.monotonic() < deadline:
        if not alive():
            raise ModuleConnectError(
                "the run's process exited before its calibration module answered"
            )
        app: Dimos | None = None
        connected = False
        try:
            app = Dimos.connect(timeout=CONNECT_ATTEMPT_S)
            module: Any = app.get_module("CalibrationModule")
            set_call_timeouts(module)
            module.plan()
            connected = True
            return module
        except Exception as error:
            last = error
        finally:
            if not connected and app is not None:
                app.stop()
        time.sleep(CONNECT_RETRY_S)
    raise ModuleConnectError(
        f"the CalibrationModule did not answer within {timeout_s:g} s (last error: {last})"
    )


def check_identity(module: Any, run_id: str, arm_id: str) -> None:
    """Refuse a module that is not our run's, or not on our arm, or already busy.

    Nothing that moves the arm is asked of it before this passes.
    """
    try:
        theirs = module.dimos_run_id()
        status = module.status()
    except Exception as error:
        raise ModuleConnectError(
            f"could not ask the calibration module which run it is: {error}"
        ) from error
    if theirs != run_id:
        raise ModuleConnectError(
            f"connected to a different run: the calibration module answering belongs to "
            f"{theirs or 'no dimos run'}, not {run_id}; nothing was asked of it"
        )
    if status.get("arm_id") != arm_id:
        raise ModuleConnectError(
            f"connected to a different run: its calibration module calibrates "
            f"{status.get('arm_id')!r}, not {arm_id!r}; nothing was asked of it"
        )
    if status.get("state") != "idle":
        raise ModuleConnectError(
            f"the calibration module is already {status.get('state')}, not a fresh run; "
            "nothing was asked of it"
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
        """Forget what was typed ahead of a prompt, which did not answer it; keep q, p and EOF."""
        kept: list[object] = []
        while True:
            try:
                item = self._lines.get_nowait()
            except queue.Empty:
                break
            if item is EOF or item in ABORT_KEYS:
                kept.append(item)
        for item in kept:
            self._lines.put(item)

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


def ask(keys: KeySource, text: str) -> object:
    """Show ``text``, forget what was typed before it, and wait for a line (or ``EOF``)."""
    echo(text, nl=False)
    keys.drain()
    while True:
        key = keys.get()
        if key is not None:
            return key
        time.sleep(POLL_S)


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
    """How the calibration run ended: an engine state, or ``GONE`` / ``UNCONFIRMED``."""

    state: str
    message: str
    results: dict[str, Any]


@dataclass(frozen=True)
class Home:
    """How going home ended."""

    at_home: bool
    message: str
    failed: bool = False
    gone: bool = False


@dataclass(frozen=True)
class Ending:
    """Whether the walk-through may stop its run, and whether going home failed on the way."""

    stop: bool
    home_failed: bool


class RunGoneError(RuntimeError):
    """Our run's process has exited."""


class AbortUnconfirmedError(RuntimeError):
    """The engine did not report a final state within ``ABORT_WAIT_S`` of an abort."""


def _interrupt_name(error: KeyboardInterrupt) -> str:
    """What interrupted: Ctrl-C, or the signal ``_signals_interrupt`` turned into one."""
    name = str(error.args[0]) if error.args else ""
    return {"SIGHUP": "SIGHUP (the terminal closed)", "SIGTERM": "SIGTERM"}.get(name, "Ctrl-C")


class Wizard:
    """Drives one calibration run over the module's RPCs and the keyboard."""

    def __init__(
        self, module: Any, keys: KeySource, alive: Callable[[], bool] | None = None
    ) -> None:
        self.module = module
        self.keys = keys
        self._alive = alive or (lambda: True)
        self._seen_steps: dict[str, str] = {}
        self._last_line = ""
        self._begin()

    def _begin(self) -> None:
        """A new action (the run, or going home): its own prompts, answers and one abort."""
        self._prompted: int | None = None
        self._abort_sent = False
        self._timed_out = False
        self._confirmed = 0
        self._skipped = 0

    def call(self, name: str, *args: Any) -> Any:
        """One module RPC, refused once our run's process is gone."""
        if not self._alive():
            raise RunGoneError(
                "the calibration run's process has exited, and the arm's connection with it: "
                "support the arm, its motors may be off"
            )
        return getattr(self.module, name)(*args)

    def abort(self, why: str) -> None:
        """Abort the current action, once: the arm stops and holds where it is, its motors on."""
        if self._abort_sent:
            return
        self._abort_sent = True
        echo(f"\n{why}: aborting; the arm stops and holds where it is, its motors on")
        try:
            self.call("abort")
        except RunGoneError:
            raise
        except Exception as error:
            echo(f"  the abort call failed ({error}); watching for the engine to stop", err=True)

    def _show_prompt(self, status: dict[str, Any]) -> None:
        echo(f"\n? {status['prompt']}")
        deadline = status.get("prompt_deadline")
        if deadline:
            at = time.strftime("%H:%M", time.localtime(deadline))
            echo(f"  (no answer by {at} aborts the run and parks the arm)")
        echo(PROMPT_KEYS, nl=False)

    def _keys(self, status: dict[str, Any]) -> None:
        """Act on what was typed: q, p and EOF abort in any state; Enter and s answer a prompt."""
        waiting = status["state"] == "waiting_confirm"
        if waiting and status["prompt_id"] != self._prompted:
            self._prompted = status["prompt_id"]
            self.keys.drain()
            self._show_prompt(status)
        key = self.keys.get()
        if key is None:
            return
        if key is EOF:
            self.abort("input closed")
        elif key in ABORT_KEYS:
            self.abort(str(key))
        elif not waiting:
            if key not in ("", "s"):
                echo(f"  not understood: {key!r}; q aborts")
        elif key == "":
            if self.call("confirm", status["prompt_id"]):
                self._confirmed += 1
        elif key == "s":
            if self.call("skip", status["prompt_id"]):
                self._skipped += 1
        else:
            echo(f"  not understood: {key!r}; Enter, s or q  > ", nl=False)
            self.keys.drain()

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
                    data = self.call("results").get(step["id"])
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

    def drive(self, status: dict[str, Any], *, timeout_s: float | None = None) -> dict[str, Any]:
        """Poll (about 5 Hz), report and answer until the state is final.

        Keys are read on every poll. ``timeout_s`` bounds the time spent
        outside prompts; past it the action is aborted. Once aborted, the
        engine has ``ABORT_WAIT_S`` to report a final state, else
        ``AbortUnconfirmedError``. RPC errors propagate.
        """
        moving = 0.0
        last = time.monotonic()
        aborted_at: float | None = None
        while True:
            self._report(status)
            if status["state"] in FINISHED:
                return status
            self._keys(status)
            now = time.monotonic()
            if status["state"] != "waiting_confirm":
                moving += now - last
            last = now
            if timeout_s is not None and moving > timeout_s and not self._abort_sent:
                self._timed_out = True
                self.abort(f"not done within {timeout_s:g} s")
            if self._abort_sent:
                aborted_at = aborted_at if aborted_at is not None else now
                if now - aborted_at > ABORT_WAIT_S:
                    raise AbortUnconfirmedError(
                        f"the engine did not confirm the abort within {ABORT_WAIT_S:g} s"
                    )
            time.sleep(POLL_S)
            status = self.call("status")

    def _results(self) -> dict[str, Any]:
        try:
            results: dict[str, Any] = self.call("results")
            return results
        except Exception:
            return {}

    def run(self, steps: list[str] | None) -> Outcome:
        self._begin()
        status = self.drive(self.call("run", steps))
        return Outcome(status["state"], status["message"], self._results())

    def settle(self, reason: str) -> Outcome:
        """Make sure nothing runs any more: abort (once) unless the engine already says it stopped.

        Never raises: a run that is gone or does not confirm the abort comes
        back as ``GONE`` or ``UNCONFIRMED``, never as aborted.
        """
        try:
            try:
                status = self.call("status")
                finished = status["state"] in FINISHED
            except RunGoneError:
                raise
            except Exception:
                finished = False
            if finished:
                return Outcome(status["state"], status["message"], self._results())
            self.abort(reason)
            deadline = time.monotonic() + ABORT_WAIT_S
            while time.monotonic() < deadline:
                time.sleep(POLL_S)
                try:
                    status = self.call("status")
                except RunGoneError:
                    raise
                except Exception:
                    continue
                if status["state"] in FINISHED:
                    return Outcome(status["state"], status["message"], self._results())
            return Outcome(
                UNCONFIRMED,
                f"The engine did not confirm the abort within {ABORT_WAIT_S:g} s: the arm may "
                "still be moving or held by the run",
                {},
            )
        except RunGoneError as error:
            return Outcome(GONE, str(error), {})
        except KeyboardInterrupt:
            return Outcome(
                UNCONFIRMED, "Interrupted again before the engine confirmed the abort", {}
            )

    def go_home(self) -> Home:
        """``go_home`` over RPC (the engine asks first), followed to its end."""
        status: dict[str, Any] | None = None
        for attempt in range(2):
            self._begin()
            try:
                status = self.call("go_home")
                break
            except RunGoneError as error:
                return Home(False, str(error), failed=True, gone=True)
            except KeyboardInterrupt:
                return Home(False, "Interrupted: not moved home; the arm holds where it is")
            except Exception as error:
                if attempt == 0 and "already" in str(error):
                    echo("  the calibration is still busy; stopping it first...")
                    if self.settle("going home").state in FINISHED:
                        continue
                return Home(False, f"Going home was refused: {error}", failed=True)
        assert status is not None
        try:
            status = self.drive(status, timeout_s=HOME_TIMEOUT_S)
        except RunGoneError as error:
            return Home(False, str(error), failed=True, gone=True)
        except (KeyboardInterrupt, Exception) as error:
            interrupted = isinstance(error, KeyboardInterrupt)
            why = "Interrupted" if interrupted else f"Lost touch going home ({error})"
            settled = self.settle(why)
            return Home(
                False,
                f"{why}: going home stopped. {settled.message}",
                failed=not interrupted,
                gone=settled.state == GONE,
            )
        if status["state"] == "done" and self._confirmed and not self._skipped:
            return Home(True, status["message"])
        if status["state"] == "done":
            return Home(False, "Not moved home (skipped); the arm holds where it is")
        return Home(
            False,
            f"Not at home: {status['message']}",
            failed=status["state"] == "failed" or self._timed_out,
        )

    def finish(self) -> Ending:
        """Bring the arm home before the run stops; stop away from home only on an explicit yes."""
        home_failed = False
        try:
            while True:
                key = ask(self.keys, OFFER)
                if key in ("", "h"):
                    home = self.go_home()
                    echo(f"\n{home.message}")
                    home_failed = home_failed or home.failed
                    if home.at_home or home.gone:
                        return Ending(stop=True, home_failed=home_failed)
                elif key is not EOF and key not in ("k", *ABORT_KEYS):
                    echo(f"  not understood: {key!r}")
                    continue
                echo(BACKSTOP_NOTE)
                answer = ask(self.keys, STOP_ANYWAY)
                if answer == "yes":
                    return Ending(stop=True, home_failed=home_failed)
                if answer is EOF:
                    echo("\n(input closed)")
                    return Ending(stop=False, home_failed=home_failed)
        except KeyboardInterrupt:
            echo("\nInterrupted.")
            return Ending(stop=False, home_failed=home_failed)

    def walk(self, selected: list[str] | None, connection_hint: str) -> tuple[int, bool]:
        """The session: plan, run, finish. The exit code, and whether the run may be stopped."""
        outcome: Outcome | None = None
        reason = "stopped"
        interrupted = errored = False
        try:
            print_plan(self.call("plan"), selected)
            echo(KEYS_HELP)
            outcome = self.run(selected)
        except KeyboardInterrupt as error:
            interrupted = True
            reason = _interrupt_name(error)
        except AbortUnconfirmedError as error:
            outcome = Outcome(
                UNCONFIRMED, f"{error}: the arm may still be moving or held by the run", {}
            )
        except Exception as error:
            errored = True
            reason = f"lost touch with the calibration module ({type(error).__name__}: {error})"
            echo(f"\nError: {reason}", err=True)
        if outcome is None:
            outcome = self.settle(reason)
        try:
            print_ending(outcome, connection_hint)
        except KeyboardInterrupt:
            interrupted = True
        if outcome.state == GONE:
            return 1, True
        ending = self.finish()
        code = EXIT_CODES.get(outcome.state, 1)
        if interrupted and outcome.state != "done":
            code = 130
        elif errored:
            code = 1
        if code == 0 and (ending.home_failed or not ending.stop):
            code = 1
        return code, ending.stop


def saved_files(results: dict[str, Any]) -> Path | None:
    """The calibration file this run wrote, from the steps' results, else None."""
    for step in ("summary", "pick_gains", "gravity_fit"):
        data = results.get(step) or {}
        path = data.get("file") if step == "summary" else data.get("saved")
        if path:
            return Path(path)
    return None


def print_ending(outcome: Outcome, connection_hint: str) -> None:
    if outcome.state in (GONE, UNCONFIRMED):
        echo(f"\n{outcome.message}")
    else:
        echo(f"\nRun {outcome.state}: {outcome.message}")
    path = saved_files(outcome.results)
    if path is None:
        echo("No calibration file was written by this run.")
        return
    echo(f"Calibration file: {path}")
    echo(f"Earlier files:    {path.parent / 'history'}/")
    echo("It is applied every time the robot starts. To undo it, delete the file (the previous")
    echo(f"one is in history/) or start the robot with `--{connection_hint}.calibration off`.")


def left_running(run: OurRun) -> str:
    return (
        f"Left {run.run_id} running, so the arm stays held where it is, its motors on.\n"
        "Support the arm or put it on its supports, then stop the run with `dimos stop`. "
        "Stopping parks it home only when that is quick, and `dimos stop` may cut it short."
    )


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
    from dimos.control.calibration.robots import KNOWN_ROBOTS

    if robot not in KNOWN_ROBOTS:
        echo(f"Unknown robot {robot!r}. Available: {', '.join(KNOWN_ROBOTS)}", err=True)
        raise typer.Exit(2)


def _arm_ids(robot: str, arm: str | None) -> list[str]:
    """The robot's arm ids ``arm`` names: "left", its bus "left_arm" or its id; all when None."""
    from dimos.control.calibration.robots import KNOWN_ROBOTS
    from dimos.control.calibration.spec import arm_name

    machine = KNOWN_ROBOTS[robot]
    ids = machine.arm_ids()
    if arm is None:
        return ids
    return [
        arm_id
        for arm_id, bus in zip(ids, machine.buses, strict=True)
        if arm in (arm_id, arm_id.rsplit("/", 1)[-1], bus.name, arm_name(bus.name))
    ]


def _one_arm_id(robot: str, arm: str) -> str:
    ids = _arm_ids(robot, arm)
    if len(ids) != 1:
        names = [i.rsplit("/", 1)[-1] for i in _arm_ids(robot, None)]
        echo(f"{robot} has no arm {arm!r}; choose one of {names}", err=True)
        raise typer.Exit(2)
    return ids[0]


def _precheck(
    robot: str,
    arm: str | None,
    floor_z_m: float | None,
    min_x_m: float | None,
    poses: int | None = None,
    speed_deg_s: float | None = None,
) -> tuple[Any, list[dict[str, Any]] | None]:
    """The module's own config check and workspace-guard rule, before anything starts.

    Returns the arm's spec and the offline plan, or ``(None, None)`` when the robot model is
    not on this machine (the engine's check step says so at start).
    """
    from pydantic import ValidationError

    from dimos.control.calibration.module import CalibrationModuleConfig
    from dimos.control.calibration.procedure import Capabilities, Procedure
    from dimos.control.calibration.robots import KNOWN_ROBOTS, calibration_keys
    from dimos.control.calibration.spec import NoWorkspaceGuardError
    from dimos.control.calibration.steps import calibration_steps

    given: dict[str, Any] = {"poses": poses, "speed_deg_s": speed_deg_s}
    try:
        config = CalibrationModuleConfig(
            robot=robot,
            arm=arm,
            floor_z_m=floor_z_m,
            min_x_m=min_x_m,
            **{name: value for name, value in given.items() if value is not None},
        )
    except ValidationError as error:
        first = error.errors()[0]
        where = "".join(f"--{part}".replace("_", "-") for part in first["loc"])
        message = first["msg"].removeprefix("Value error, ")
        echo(f"Error: {where + ': ' if where else ''}{message}", err=True)
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
    # What the robot's calibration connection will say it serves, asked of no connection.
    caps = Capabilities(keys=calibration_keys(KNOWN_ROBOTS[robot]))
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


def _refuse_if_bus_busy() -> None:
    """Refuse when a coordinator answers on the bus: a run still building, or another user's."""
    from dimos.core.global_config import global_config

    bus = f"the {global_config.transport} bus"
    try:
        busy = coordinator_answers()
    except Exception as error:
        echo(f"Error: could not check {bus} for another dimos run: {error}", err=True)
        raise typer.Exit(1) from error
    if busy:
        echo(
            f"A dimos coordinator already answers on {bus} (a run still starting, or another "
            "user's); not starting a calibration.",
            err=True,
        )
        raise typer.Exit(1)


@contextmanager
def _signals_interrupt() -> Iterator[None]:
    """SIGHUP (the terminal closed) and SIGTERM take Ctrl-C's path: abort, then finish."""

    def interrupt(signum: int, _frame: object) -> None:
        raise KeyboardInterrupt(signal.Signals(signum).name)

    previous = {sig: signal.signal(sig, interrupt) for sig in (signal.SIGHUP, signal.SIGTERM)}
    try:
        yield
    finally:
        for sig, handler in previous.items():
            if handler is not None:
                signal.signal(sig, handler)


def global_flags(overrides: Mapping[str, Any]) -> list[str]:
    """The root ``dimos`` options this command was given, to pass on before ``run``."""
    flags: list[str] = []
    for name, value in overrides.items():
        option = name.replace("_", "-")
        if isinstance(value, bool):
            flags.append(f"--{option}" if value else f"--no-{option}")
        else:
            flags += [f"--{option}", str(value)]
    return flags


def calibrate_flags(
    arm: str | None,
    floor_z_m: float | None,
    min_x_m: float | None,
    poses: int | None,
    speed_deg_s: float | None,
    confirm_each: bool,
) -> list[str]:
    """The ``--calibrationmodule.*`` flags of ``dimos run`` for this command's options."""
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
    return flags


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
    poses: int | None = typer.Option(None, "--poses", help="Gravity poses to visit, at least 1"),
    confirm_each: bool = typer.Option(
        True, "--confirm-each/--no-confirm-each", help="Ask before every pose (default: ask)"
    ),
    steps: str | None = typer.Option(None, "--steps", help="Comma-separated steps to run"),
    speed_deg_s: float | None = typer.Option(
        None, "--speed-deg-s", help="Peak joint speed, deg/s, at most 30"
    ),
    dry_run: bool = typer.Option(
        False, "--dry-run", help="Print the command and plan, start nothing"
    ),
) -> None:
    """Walk through the calibration of one arm in this terminal.

    Extra flags go to `dimos run <robot>-calibrate`, e.g.
    --dualopenyamconnection.left-can-port can0.
    """
    from dimos.control.calibration.steps import calibration_steps

    _check_robot(robot)
    known = [step.id for step in calibration_steps()]
    selected = _split_steps(steps, known)
    arm = arm or BLUEPRINT_DEFAULT_ARM.get(robot)
    if arm is not None:
        _one_arm_id(robot, arm)
    blueprint = f"{robot}-calibrate"
    overrides: dict[str, Any] = dict(ctx.obj or {})
    flags = calibrate_flags(arm, floor_z_m, min_x_m, poses, speed_deg_s, confirm_each)
    argv = [
        *dimos_command(),
        *global_flags(overrides),
        "run",
        blueprint,
        "--daemon",
        *flags,
        *ctx.args,
    ]

    spec, plan = _precheck(robot, arm, floor_z_m, min_x_m, poses, speed_deg_s)
    if dry_run:
        echo("Would run:")
        echo("  " + " ".join(argv))
        if plan is not None:
            print_plan(plan, selected)
        else:
            echo("Steps: " + ", ".join(selected or known))
        echo("Nothing was started.")
        return
    arm_id = spec.arm_id if spec is not None else _arm_ids(robot, arm)[0]
    use_run_globals(ctx.args, overrides)
    hint = f"{robot.replace('-', '')}connection"
    try:
        with calibrate_lock(arms_root() / f"{robot}.calibrate.lock"):
            _refuse_if_busy()
            _refuse_if_bus_busy()
            with _signals_interrupt():
                code = _launch_and_walk(argv, blueprint, arm_id, selected, hint)
    except CalibrateLockedError as error:
        echo(f"Error: {error}", err=True)
        raise typer.Exit(1) from error
    raise typer.Exit(code)


def _launch_and_walk(
    argv: list[str], blueprint: str, arm_id: str, selected: list[str] | None, hint: str
) -> int:
    echo(f"Starting {blueprint} (this takes a minute)...")
    try:
        run = launch_run(argv, token=new_token(blueprint))
    except LaunchInterruptedError as error:
        echo(f"\nInterrupted: {error}", err=True)
        return 130
    except LaunchError as error:
        echo(f"Error: {error}", err=True)
        return 1
    stop = False
    try:
        echo(f"Started run {run.run_id} (pid {run.pid}).")
        code, stop = _session(run, arm_id, selected, hint)
    finally:
        echo(stop_run(run) if stop else left_running(run))
    return code


def _session(run: OurRun, arm_id: str, selected: list[str] | None, hint: str) -> tuple[int, bool]:
    """Connect to our run's module and walk through; the exit code and whether to stop the run."""
    keys = make_keys()
    try:
        module = connect_module(CONNECT_TIMEOUT_S, lambda: run_alive(run))
        check_identity(module, run.run_id, arm_id)
    except KeyboardInterrupt as error:
        echo(f"\n{_interrupt_name(error)} while connecting; the calibration has not moved the arm.")
        return 130, _stop_unconnected(keys, run)
    except ModuleConnectError as error:
        echo(f"Error: {error}; logs: `dimos log -r {run.run_id}`", err=True)
        return 1, _stop_unconnected(keys, run)
    return Wizard(module, keys, lambda: run_alive(run)).walk(selected, hint)


def _stop_unconnected(keys: KeySource, run: OurRun) -> bool:
    """Whether to stop a run whose module the walk-through never drove: only on a yes."""
    if not run_alive(run):
        return True
    echo("The calibration has not moved the arm; the run's connection may hold it where it is.")
    try:
        echo(BACKSTOP_NOTE)
        return ask(keys, STOP_ANYWAY) == "yes"
    except KeyboardInterrupt:
        return False


# show and bake


def _fmt_time(value: datetime) -> str:
    return value.strftime("%Y-%m-%d %H:%M:%S %Z").strip()


def print_calibration(arm_id: str, root: Path) -> None:
    from dimos.control.calibration.schema import (
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
    arm: str | None = typer.Option(
        None, "--arm", help="Only this arm: left, its bus left_arm, or its id"
    ),
) -> None:
    """Print the saved calibration of a robot's arms."""
    _check_robot(robot)
    ids = _arm_ids(robot, None) if arm is None else [_one_arm_id(robot, arm)]
    for arm_id in ids:
        print_calibration(arm_id, arms_root())


@arm_app.command("bake")
def bake_cmd(
    robot: str = typer.Argument(..., help="Robot: openyam, dual-openyam, ..."),
    arm: str = typer.Option(..., "--arm", help="Which arm: left, its bus left_arm, or its id"),
    urdf: Path = typer.Option(..., "--urdf", exists=True, dir_okay=False, help="URDF to read"),
    out: Path = typer.Option(..., "--out", help="URDF to write"),
) -> None:
    """Write a URDF with the arm's fitted link masses and centres of mass."""
    from dimos.control.calibration.schema import (
        ArmCalibrationError,
        bake_into_urdf,
        calibration_path,
        load_calibration,
    )

    _check_robot(robot)
    arm_id = _one_arm_id(robot, arm)
    try:
        calibration = load_calibration(calibration_path(arm_id, arms_root()))
        baked = bake_into_urdf(calibration, urdf.read_text())
    except ArmCalibrationError as error:
        echo(f"Error: {error}", err=True)
        raise typer.Exit(1) from error
    out.write_text(baked)
    echo(f"Wrote {out} with the fitted masses of {arm_id}.")
