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

from pathlib import Path
import sys
import threading
import time

import pytest

# TODO: remove this skip once the pin is cel-expr-python 0.1.4, which supports Python 3.10.
if sys.version_info < (3, 11):
    pytest.skip("cel-expr-python 0.1.3 needs Python 3.11", allow_module_level=True)

from dimos_lcm.actionlib_msgs import GoalID, GoalStatus

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.introspection.module.info import extract_rpc_info
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.orchestration.loader import load_and_check
from dimos.orchestration.orchestrator import Deployed, Orchestrator, bind

JOB = """\
root = "Job"

[constants]
SUCCEEDED = 3

[[signal]]
name = "status"
stream = "FakeBase.status"

[[event]]
name = "arrived"
when = "status.status == SUCCEEDED && status.goal_id.id == job.goal"

[[activity]]
name = "go"
start = "FakeBase.go_to"
cancel = "FakeBase.halt"

[[activity]]
name = "mark"
start = "FakeBase.mark"

[[node]]
name = "Job"
children = ["Going", "Arrived"]
initial = "Going"

[[node]]
name = "Going"
activity = "go"
args = { name = "'lobby'" }
result = "job.goal"

[[node.transition]]
trigger = "arrived"
goto = "Arrived"

[[node]]
name = "Arrived"
activity = "mark"

[[node.transition]]
after = 60.0
goto = "Going"
"""

GO_TO_ARGS = "args = { name = \"'lobby'\" }"

# Each row replaces one piece of the job file and gives the problem the blueprint must raise.
UNBOUND: list[tuple[str, str, str]] = [
    (
        'start = "FakeBase.go_to"',
        'start = "FakeBase.goto"',
        "activity `go`: `start` names `FakeBase.goto`, and `FakeBase` has no RPC `goto`",
    ),
    (
        'start = "FakeBase.mark"',
        'start = "Base.mark"',
        "activity `mark`: `start` names `Base.mark`, and `Base` is not a deployed module",
    ),
    (
        'cancel = "FakeBase.halt"',
        'cancel = "FakeBase.go_to"',
        "activity `go`: `cancel` RPC `FakeBase.go_to` needs `name`",
    ),
    (
        GO_TO_ARGS,
        "args = { nmae = \"'lobby'\" }",
        "node `Going`: `args` names `nmae`, which is not a parameter of `FakeBase.go_to`",
    ),
    (
        GO_TO_ARGS,
        "args = {}",
        "node `Going`: `args` has no entry for `name`, which `FakeBase.go_to` needs",
    ),
    (
        GO_TO_ARGS,
        "args = { name = 5 }",
        "node `Going`: `args` entry `name`: `5` gives INT, and STRING is needed",
    ),
    (
        'stream = "FakeBase.status"',
        'stream = "FakeBase.state"',
        "signal `status`: `stream` names `FakeBase.state`, and `FakeBase` has no stream `state`",
    ),
    (
        'stream = "FakeBase.status"',
        'stream = "FakeBase.status"\nfield = "nope"',
        "signal `status`: `field` names `nope`, which is not a field of `GoalStatus`",
    ),
    ("status.goal_id.id ==", "status.goal_id.idd ==", "event `arrived`: `when`"),
    (
        'activity = "mark"',
        'activity = "mark"\nresult = "job.goal"',
        "key `job.goal` is written by `Going` and `Arrived`, and their `start` RPCs return "
        "different shapes",
    ),
]


class FakeBase(Module):
    """Reports each goal as reached, by its id, until it is halted."""

    status: Out[GoalStatus]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._calls: list[str] = []
        self._halted = threading.Event()

    @rpc
    def go_to(self, name: str) -> str:
        self._calls.append(f"go_to {name}")
        reached = GoalStatus(goal_id=GoalID(id=name), status=GoalStatus.SUCCEEDED)
        threading.Thread(target=self._report, args=(reached,), daemon=True).start()
        return name

    def _report(self, reached: GoalStatus) -> None:
        for _ in range(200):
            if self._halted.wait(0.05):
                return
            self.status.publish(reached)

    @rpc
    def halt(self) -> None:
        self._calls.append("halt")
        self._halted.set()

    @rpc
    def mark(self) -> None:
        self._calls.append("mark")

    @rpc
    def boom(self, name: str) -> str:
        self._calls.append(f"boom {name}")
        raise RuntimeError(f"cannot go to {name}")

    @rpc
    def calls(self) -> list[str]:
        return list(self._calls)


def _deployed() -> Deployed:
    return Deployed(
        modules=frozenset({"FakeBase"}),
        rpcs={f"FakeBase.{name}": extract_rpc_info(fn) for name, fn in FakeBase.rpcs.items()},
        streams={"FakeBase.status": GoalStatus},
    )


def _written(tmp_path: Path, text: str) -> Path:
    path = tmp_path / "job.toml"
    path.write_text(text)
    return path


def _build(job: Path) -> ModuleCoordinator:
    blueprint: Blueprint = autoconnect(FakeBase.blueprint(), Orchestrator.blueprint(file=job))
    parsed = BlueprintConfigParser(blueprint).parse(environ={}, overrides={"g": {"viewer": "none"}})
    return ModuleCoordinator.build(blueprint, parsed)


def test_the_job_file_fits_the_blueprint(tmp_path: Path) -> None:
    checked, problems = bind(load_and_check(_written(tmp_path, JOB)), _deployed())
    assert problems == []
    assert checked.problems == ()


@pytest.mark.parametrize(("old", "new", "problem"), UNBOUND)
def test_a_file_that_does_not_fit_the_blueprint(
    tmp_path: Path, old: str, new: str, problem: str
) -> None:
    assert JOB.count(old) == 1
    _, problems = bind(load_and_check(_written(tmp_path, JOB.replace(old, new))), _deployed())
    assert any(problem in found for found in problems), problems


def _calls_with(coordinator: ModuleCoordinator, wanted: set[str]) -> list[str]:
    """The fake's calls, read a moment after it has had every wanted one."""
    base = coordinator.get_instance(FakeBase)
    deadline = time.monotonic() + 10.0
    while not wanted <= set(base.calls()) and time.monotonic() < deadline:
        time.sleep(0.05)
    time.sleep(0.2)
    calls: list[str] = base.calls()
    return calls


def test_the_orchestrator_reaches_a_node_that_needs_a_message_and_a_return(tmp_path: Path) -> None:
    coordinator = _build(_written(tmp_path, JOB))
    try:
        calls = _calls_with(coordinator, {"halt", "mark"})
    finally:
        coordinator.stop()
    # Arrived starts mark, and it is entered only once go_to returned and a status matched it.
    assert calls[0] == "go_to lobby"
    assert set(calls[1:]) == {"halt", "mark"}


def test_a_fault_cancels_every_activity_once(tmp_path: Path) -> None:
    job = _written(tmp_path, JOB.replace('start = "FakeBase.go_to"', 'start = "FakeBase.boom"'))
    coordinator = _build(job)
    try:
        calls = _calls_with(coordinator, {"halt"})
    finally:
        coordinator.stop()
    # Nothing handles the failed start, so the fault sends the one cancel in the file.
    assert calls == ["boom lobby", "halt"]


def test_a_file_that_does_not_fit_the_blueprint_fails_the_build(tmp_path: Path) -> None:
    job = _written(tmp_path, JOB.replace('start = "FakeBase.go_to"', 'start = "FakeBase.goto"'))
    with pytest.raises(Exception, match="`FakeBase` has no RPC `goto`"):
        _build(job)
