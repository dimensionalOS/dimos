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

"""The toy arm's calibration deployed as ``dimos run`` deploys it, driven from its RPCs as
``dimos shell`` drives them."""

from __future__ import annotations

from collections.abc import Callable
import time
from typing import TypeVar

from IPython.lib.pretty import pretty

from dimos.control.calibration._calibration_test_helpers import toy_run
from dimos.control.calibration.module import CalibrationStatus
from dimos.porcelain.dimos import Dimos

WAIT_S = 30.0
T = TypeVar("T")


def until(read: Callable[[], T], done: Callable[[T], bool]) -> T:
    """``read()`` once ``done`` takes it, or its last value after ``WAIT_S``."""
    deadline = time.monotonic() + WAIT_S
    value = read()
    while not done(value) and time.monotonic() < deadline:
        time.sleep(0.05)
        value = read()
    return value


def test_the_shell_runs_answers_and_aborts_the_deployed_calibration() -> None:
    with toy_run():
        app = Dimos.connect()
        try:
            cal = app.CalibrationModule
            idle = cal.status()
            assert isinstance(idle, CalibrationStatus)
            assert (idle.state, idle.arm, idle.question) == ("idle", "toy", None)
            assert f"  {'check':<17} {'pending':<8} measure the noise" in str(idle)
            assert pretty(idle) == str(idle)
            assert cal.answer("go") == "no question pending"
            assert cal.abort() == "not running"
            assert cal.run(["dance"]).startswith("unknown steps ['dance']")
            assert (
                until(lambda: cal.run(["summary"]), lambda reply: reply == "started") == "started"
            )
            assert cal.run() == "already running"

            asked = until(cal.status, lambda status: status.state == "waiting")
            assert (asked.step, asked.answers) == ("check", ["go"]), str(asked)
            assert asked.question is not None and "Clear the workspace" in asked.question
            assert cal.abort() == "aborting"
            offered = until(cal.status, lambda status: "Go home?" in (status.question or ""))
            assert offered.question == (
                "check: aborted by the operator\nThe arm holds where it is. Go home?"
            ), str(offered)
            assert cal.answer("stay") == "ok"
            aborted = until(cal.status, lambda status: status.state == "aborted")
            assert aborted.summary == "check: aborted by the operator", str(aborted)

            assert (
                until(lambda: cal.run(["summary"]), lambda reply: reply == "started") == "started"
            )
            until(cal.status, lambda status: status.state == "waiting")
            assert cal.answer("go") == "ok"
            until(cal.status, lambda status: "Move to rest" in (status.question or ""))
            assert cal.answer("go") == "ok"
            done = until(cal.status, lambda status: status.state == "done")
            assert done.summary.endswith("  summary: fake/calibration.json"), str(done)
            assert done.message.endswith("\nParked at rest; you can stop the run."), str(done)
            assert "> go" in done.log
            assert done.file is not None and done.file.created == "2026-11-21T00:00:00+00:00"
        finally:
            app.stop()
