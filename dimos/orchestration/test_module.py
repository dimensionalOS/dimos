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

from collections.abc import Callable, Iterator
import json
from pathlib import Path
import threading
import time
from typing import Any

import pytest

pytest.importorskip("cel_expr_python")

from dimos_lcm.actionlib_msgs import GoalID, GoalStatus

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.msgs.std_msgs.Bool import Bool
from dimos.msgs.std_msgs.String import String
from dimos.orchestration.definition import DefinitionError
from dimos.orchestration.module import (
    StateMachineModule,
    check_state_machines,
    state_machine,
)
from dimos.spec.utils import get_protocol_method_signatures

_MACHINE = """
[machine]
name = "{name}"
initial = "idle"

[inputs]
trigger = "{input_type}"

[outputs]
notice = "std_msgs.String"

[[state]]
name = "idle"
entry = [{{ call = "worker.reset", args = {{ hard = true }} }}]

[[state.transition]]
event = "trigger"
to = "busy"

[[state]]
name = "busy"
do = {{ action = "worker.work", args = {{ amount = 3 }} }}

[[state.transition]]
event = "do.failed"
to = "idle"
"""


def _write(tmp_path: Path, name: str, input_type: str = "std_msgs.Bool") -> Path:
    path = tmp_path / f"{name}.toml"
    path.write_text(_MACHINE.format(name=name, input_type=input_type))
    return path


def test_blueprint_has_file_ports_and_role_refs(tmp_path: Path) -> None:
    (atom,) = state_machine(_write(tmp_path, "toy")).blueprints

    assert issubclass(atom.module, StateMachineModule)
    assert atom.name == "toy"
    streams = {(s.name, s.type, s.direction) for s in atom.streams}
    assert streams == {
        ("trigger", Bool, "in"),
        ("notice", String, "out"),
        ("action_result", GoalStatus, "in"),
        ("machine_state", String, "out"),
    }
    (ref,) = atom.module_refs
    assert ref.name == "worker"
    methods = {
        name: list(signature.parameters)
        for name, signature in get_protocol_method_signatures(ref.spec).items()
    }
    assert methods == {
        "cancel_action": ["self", "goal_id"],
        "reset": ["self", "hard"],
        "work": ["self", "amount", "goal_id"],
    }


def test_unknown_message_type_fails_at_load(tmp_path: Path) -> None:
    with pytest.raises(DefinitionError, match="unknown message type 'std_msgs.Nope'"):
        state_machine(_write(tmp_path, "toy", input_type="std_msgs.Nope"))


def test_same_ports_and_roles_share_a_class(tmp_path: Path) -> None:
    (first,) = state_machine(_write(tmp_path, "first")).blueprints
    (second,) = state_machine(_write(tmp_path, "second")).blueprints
    assert first.module is second.module
    assert first.name != second.name


class _Worker(Module):
    action_result: Out[GoalStatus]

    @rpc
    def reset(self, hard: bool) -> None: ...

    @rpc
    def work(self, goal_id: str, amount: int) -> None: ...

    @rpc
    def cancel_action(self, goal_id: str) -> None: ...


class _WrongArgWorker(_Worker):
    @rpc
    def work(self, goal_id: str, count: int) -> None: ...


class _ResultlessWorker(Module):
    @rpc
    def reset(self, hard: bool) -> None: ...

    @rpc
    def work(self, goal_id: str, amount: int) -> None: ...

    @rpc
    def cancel_action(self, goal_id: str) -> None: ...


def test_check_accepts_a_fitting_provider(tmp_path: Path) -> None:
    blueprint = autoconnect(state_machine(_write(tmp_path, "toy")), _Worker.blueprint())
    assert check_state_machines(blueprint) is blueprint


def test_check_rejects_a_wrong_arg_name(tmp_path: Path) -> None:
    blueprint = autoconnect(state_machine(_write(tmp_path, "toy")), _WrongArgWorker.blueprint())
    with pytest.raises(DefinitionError, match="toy: state 'busy': do: worker.work: .*'count'"):
        check_state_machines(blueprint)


def test_check_rejects_a_provider_without_action_result(tmp_path: Path) -> None:
    blueprint = autoconnect(state_machine(_write(tmp_path, "toy")), _ResultlessWorker.blueprint())
    with pytest.raises(DefinitionError, match="_resultlessworker has no action_result"):
        check_state_machines(blueprint)


_RUNTIME_MACHINE = """
[machine]
name = "runtime_toy"
initial = "idle"

[vars]
count = 0

[inputs]
trigger = "std_msgs.Bool"

[outputs]
notice = "std_msgs.String"

[[state]]
name = "idle"

[[state.transition]]
event = "trigger"
if = "event.msg.data"
to = "busy"
set = { count = "vars.count + 1" }
run = [{ call = "worker.ping", args = { n = "vars.count" } }]

[[state]]
name = "busy"
do = { action = "worker.work", args = { amount = 3 } }

[[state.transition]]
event = "do.succeeded"
to = "done"

[[state.transition]]
event = "do.failed"
to = "idle"

[[state]]
name = "done"
entry = [{ publish = "notice", value = "'done'" }]

[[state.transition]]
after = "0.05s"
to = "idle"
"""


class _FakeTransport:
    def __init__(self) -> None:
        self.subscribers: list[Callable[[Any], None]] = []
        self.sent: list[Any] = []

    def subscribe(self, cb: Callable[[Any], None], stream: Any = None) -> Callable[[], None]:
        self.subscribers.append(cb)
        return lambda: self.subscribers.remove(cb)

    def publish(self, msg: Any) -> None:
        for cb in list(self.subscribers):
            cb(msg)

    def broadcast(self, stream: Any, msg: Any) -> None:
        self.sent.append(msg)

    def stop(self) -> None:
        self.subscribers.clear()


class _FakeWorker:
    def __init__(self) -> None:
        self.calls: list[tuple[str, dict[str, Any]]] = []
        self._lock = threading.Lock()

    def _record(self, name: str, **args: Any) -> None:
        with self._lock:
            self.calls.append((name, args))

    def ping(self, n: int) -> None:
        self._record("ping", n=n)

    def work(self, goal_id: str, amount: int) -> None:
        self._record("work", goal_id=goal_id, amount=amount)

    def cancel_action(self, goal_id: str) -> None:
        self._record("cancel_action", goal_id=goal_id)


def _wait_for(predicate: Callable[[], bool], timeout: float = 2.0) -> None:
    deadline = time.monotonic() + timeout
    while not predicate():
        assert time.monotonic() < deadline, "timed out waiting"
        time.sleep(0.01)


def _active(machine: StateMachineModule) -> list[str]:
    status = machine.status()
    return json.loads(status)["active"] if status else []


def _transport(machine: StateMachineModule, name: str) -> _FakeTransport:
    transport = getattr(machine, name).transport
    assert isinstance(transport, _FakeTransport)
    return transport


@pytest.fixture
def build(tmp_path: Path) -> Iterator[Callable[[_FakeWorker], StateMachineModule]]:
    path = tmp_path / "runtime_toy.toml"
    path.write_text(_RUNTIME_MACHINE)
    (atom,) = state_machine(path).blueprints
    built: list[StateMachineModule] = []

    def start(worker: _FakeWorker) -> StateMachineModule:
        machine = atom.module(**atom.kwargs, state_file=tmp_path / "state.json")
        for stream in [*machine.inputs.values(), *machine.outputs.values()]:
            stream.transport = _FakeTransport()
        machine.worker = worker  # type: ignore[attr-defined]
        machine.start()
        built.append(machine)
        return machine

    yield start
    for machine in built:
        machine.stop()


def _work_goal(worker: _FakeWorker) -> str:
    _wait_for(lambda: any(name == "work" for name, _ in worker.calls))
    return next(args["goal_id"] for name, args in worker.calls if name == "work")


def _finish(machine: StateMachineModule, goal_id: str) -> None:
    status = GoalStatus(goal_id=GoalID(id=goal_id), status=GoalStatus.SUCCEEDED)
    _transport(machine, "action_result").publish(status)


def test_input_causes_transition_and_rpc(
    build: Callable[[_FakeWorker], StateMachineModule],
) -> None:
    worker = _FakeWorker()
    machine = build(worker)
    assert _active(machine) == ["idle"]

    _transport(machine, "trigger").publish(Bool(True))

    _wait_for(lambda: _active(machine) == ["busy"])
    _work_goal(worker)
    _wait_for(lambda: ("ping", {"n": 1}) in worker.calls)
    assert ("work", {"goal_id": _work_goal(worker), "amount": 3}) in worker.calls


def test_action_result_ends_do(build: Callable[[_FakeWorker], StateMachineModule]) -> None:
    worker = _FakeWorker()
    machine = build(worker)
    _transport(machine, "trigger").publish(Bool(True))

    _finish(machine, _work_goal(worker))

    _wait_for(lambda: _active(machine) in (["done"], ["idle"]))
    assert [msg.data for msg in _transport(machine, "notice").sent] == ["done"]


def test_timer_fires(build: Callable[[_FakeWorker], StateMachineModule]) -> None:
    worker = _FakeWorker()
    machine = build(worker)
    _transport(machine, "trigger").publish(Bool(True))
    _finish(machine, _work_goal(worker))

    _wait_for(lambda: json.loads(machine.status())["last_event"] == "timer")
    assert _active(machine) == ["idle"]


def test_restart_restores_vars(build: Callable[[_FakeWorker], StateMachineModule]) -> None:
    first = build(_FakeWorker())
    _transport(first, "trigger").publish(Bool(True))
    _wait_for(lambda: _active(first) == ["busy"])
    first.stop()

    second = build(_FakeWorker())
    status = json.loads(second.status())
    assert status["active"] == ["idle"]
    assert status["vars"] == {"count": 1}
