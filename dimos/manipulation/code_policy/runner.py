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

"""Check and run a program: ``validate_program`` checks it imports only
``math`` and defines ``run(robot, task)``; ``run_program`` runs it in a fresh
``python -I`` process with a hard timeout. From a shell, on the stack running
on this machine:
``python -m dimos.manipulation.code_policy.runner program.py --task "..."``."""

from __future__ import annotations

import argparse
import ast
import builtins
from collections.abc import Callable, Mapping, Sequence
import importlib
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import traceback
from types import ModuleType
from typing import Any, Literal

from pydantic import BaseModel

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.manipulation.code_policy.api import (
    TRACE_NAME,
    BackendFactory,
    Robot,
    RunStoppedError,
    Task,
)

ProgramStatus = Literal["completed", "raised", "timeout"]

# The backend factory that attaches to the DimOS stack running on this machine.
LIVE_BACKEND = "dimos.manipulation.code_policy.backend:backend"
# How long a program may run before it is killed, seconds.
DEFAULT_TIMEOUT_S = 120.0
# How often the runner checks whether the caller asked it to stop, seconds.
_STOP_POLL_S = 0.2


def validate_program(text: str) -> None:
    """Raise ``ValueError`` unless ``text`` parses, imports only ``math`` and
    defines a top-level ``run`` taking two arguments."""
    try:
        tree = ast.parse(text)
    except SyntaxError as error:
        raise ValueError(f"program does not parse: {error}") from error
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported = [alias.name for alias in node.names]
        elif isinstance(node, ast.ImportFrom):
            imported = [node.module or ""]
        else:
            continue
        for name in imported:
            if name != "math":
                raise ValueError(f"program imports {name!r}; only math may be imported")
    runs = [node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "run"]
    if len(runs) != 1 or len(runs[0].args.args) != 2:
        raise ValueError("program must define run(robot, task) at the top level")


def _import_math(name: str, *args: object, **kwargs: object) -> ModuleType:
    if name != "math":
        raise ImportError(f"program imports {name!r}; only math may be imported")
    return math


# What a program's code may use: the builtins minus file and code access, with
# imports resolving math only. Import statements go through __import__.
_PROGRAM_BUILTINS: dict[str, Any] = {
    name: value
    for name, value in vars(builtins).items()
    if name not in ("open", "exec", "eval", "compile")
}
_PROGRAM_BUILTINS["__import__"] = _import_math


def load_program(path: Path) -> Callable[[Robot, Task], None]:
    """Run the module body of the program at ``path`` with the program builtins
    (no ``open``, ``exec``, ``eval`` or ``compile``; imports resolve ``math``
    only) and return its ``run`` function."""
    namespace: dict[str, Any] = {
        "__name__": "candidate",
        "__file__": str(path),
        "__builtins__": dict(_PROGRAM_BUILTINS),
    }
    exec(compile(path.read_text(), str(path), "exec"), namespace)
    run: Callable[[Robot, Task], None] = namespace["run"]
    return run


class ProgramOutcome(BaseModel):
    """How one program run ended. ``error`` is the traceback when it raised
    or a note when it was killed; ``duration_s`` counts the program's own run
    time, not the process start-up."""

    status: ProgramStatus
    error: str = ""
    duration_s: float


_BOOTSTRAP = (
    f"import sys; sys.path.insert(0, {str(DIMOS_PROJECT_ROOT)!r}); "
    "from dimos.manipulation.code_policy.runner import run_child; run_child(sys.argv[1:])"
)


def run_program(
    program_path: Path,
    run_dir: Path,
    task: str,
    timeout_s: float,
    backend_factory: str,
    env: Mapping[str, str] | None = None,
    stop: threading.Event | None = None,
) -> ProgramOutcome:
    """Run a program once in a fresh ``python -I`` process, killed after
    ``timeout_s`` seconds. ``backend_factory`` is a ``"package.module:function"``
    returning the ``RobotBackend``; ``env`` replaces the child's environment;
    setting ``stop`` kills the child and raises ``RunStoppedError``. A child that
    dies before writing an outcome (no backend, or killed first) raises ``RuntimeError``."""
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / TRACE_NAME).unlink(missing_ok=True)
    (run_dir / "outcome.json").unlink(missing_ok=True)
    command = [
        sys.executable,
        "-I",
        "-c",
        _BOOTSTRAP,
        str(program_path),
        str(run_dir),
        task,
        backend_factory,
    ]
    log_path = run_dir / "program.log"
    started = time.monotonic()
    with log_path.open("wb") as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, env=env)
        try:
            _wait(process, timeout_s, stop)
        except subprocess.TimeoutExpired as error:
            if not (run_dir / TRACE_NAME).is_file():
                raise RuntimeError(
                    f"the program runner was killed after {timeout_s} s before its backend "
                    f"was up; see {log_path}"
                ) from error
            return ProgramOutcome(
                status="timeout",
                error=f"killed after {timeout_s} s",
                duration_s=time.monotonic() - started,
            )
    outcome = run_dir / "outcome.json"
    if process.returncode != 0 or not outcome.is_file():
        raise RuntimeError(
            f"the program runner exited with code {process.returncode} before writing "
            f"an outcome; see {log_path}"
        )
    return ProgramOutcome.model_validate_json(outcome.read_text())


def _wait(process: subprocess.Popen[bytes], timeout_s: float, stop: threading.Event | None) -> None:
    """Wait for the child; kill it at ``timeout_s`` (``TimeoutExpired``) or once ``stop`` is set (``RunStoppedError``)."""
    deadline = time.monotonic() + timeout_s
    while True:
        try:
            process.wait(timeout=min(_STOP_POLL_S, max(deadline - time.monotonic(), 0.0)))
            return
        except subprocess.TimeoutExpired:
            if stop is not None and stop.is_set():
                process.kill()
                process.wait()
                raise RunStoppedError("the run was stopped while the program was running") from None
            if time.monotonic() >= deadline:
                process.kill()
                process.wait()
                raise


def run_child(argv: Sequence[str]) -> None:
    """Entry point of the program subprocess. ``argv`` is the program path, the
    run directory, the task text and the backend factory, in that order."""
    program, run, task, factory = argv
    run_dir = Path(run)
    module_name, function_name = factory.split(":")
    make_backend: BackendFactory = getattr(importlib.import_module(module_name), function_name)
    robot = Robot(make_backend(run_dir), run_dir)
    started = time.monotonic()
    try:
        entry = load_program(Path(program))
        entry(robot, Task(task))
    except (Exception, SystemExit):
        outcome = ProgramOutcome(
            status="raised", error=traceback.format_exc(), duration_s=time.monotonic() - started
        )
    else:
        outcome = ProgramOutcome(status="completed", duration_s=time.monotonic() - started)
    (run_dir / "outcome.json").write_text(outcome.model_dump_json(indent=2))


def main(argv: Sequence[str] | None = None) -> int:
    """Run a program once on the DimOS stack running on this machine and print
    how it ended and where its trace and keyframes are. Returns the exit code:
    0 when the program completed, 1 otherwise."""
    parser = argparse.ArgumentParser(
        prog="python -m dimos.manipulation.code_policy.runner",
        description="Run a program, run(robot, task), once on the running DimOS stack.",
    )
    parser.add_argument("program", type=Path, help="the program file")
    parser.add_argument("--task", required=True, help="the task in words, as task.text")
    parser.add_argument("--out", type=Path, help="trace and keyframe folder (default: a temp dir)")
    parser.add_argument(
        "--timeout", type=float, default=DEFAULT_TIMEOUT_S, help="seconds before a kill"
    )
    args = parser.parse_args(argv)
    program: Path = args.program.resolve()
    validate_program(program.read_text())
    out: Path = args.out or Path(tempfile.mkdtemp(prefix=f"{program.stem}-"))
    outcome = run_program(program, out, args.task, args.timeout, LIVE_BACKEND)
    print(f"outcome: {outcome.status} after {outcome.duration_s:.1f} s")
    if outcome.error:
        print(outcome.error)
    print(f"trace: {out / TRACE_NAME}")
    print(f"keyframes: {out / 'keyframes'}")
    return 0 if outcome.status == "completed" else 1


if __name__ == "__main__":
    sys.exit(main())
