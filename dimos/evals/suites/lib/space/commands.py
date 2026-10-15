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

"""SPACE setup and saved-response scoring commands for ``dimos evals``."""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import ExitStack, contextmanager
import json
from pathlib import Path
import signal
from types import FrameType

import typer

setup_app = typer.Typer(help="Prepare benchmark inputs.", no_args_is_help=True)
score_app = typer.Typer(help="Score saved benchmark replies.", no_args_is_help=True)


def _terminate(signum: int, _frame: FrameType | None) -> None:
    # Let process cleanup finish even if the terminal sends another signal.
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, signal.SIG_IGN)
    raise SystemExit(128 + signum)


@contextmanager
def _termination_handlers() -> Iterator[None]:
    with ExitStack() as handlers:
        for sig in (signal.SIGTERM, signal.SIGHUP):
            previous = signal.signal(sig, _terminate)
            handlers.callback(signal.signal, sig, previous)
        yield


@setup_app.command("space")
def setup_space() -> None:
    """Acquire pinned SPACE source and task data."""
    # Keep the optional benchmark dependencies out of CLI startup.
    from dimos.evals.suites.lib.space.data import SpacePaths, setup

    with _termination_handlers():
        result = setup(SpacePaths())
    typer.echo(json.dumps(result, indent=2, allow_nan=False))


@score_app.command("space")
def score_space(
    run_dir: Path = typer.Argument(
        ..., exists=True, file_okay=False, help="Directory produced by dimos evals run."
    ),
) -> None:
    """Score saved SPACE replies without model calls."""
    # Load scoring dependencies only when this command runs.
    from dimos.evals.suites.lib.space.data import SpacePaths
    from dimos.evals.suites.lib.space.report import score_run

    with _termination_handlers():
        result = score_run(SpacePaths(), run_dir)
    typer.echo(json.dumps(result, indent=2, allow_nan=False))
    if not result["complete"]:
        raise typer.Exit(1)
