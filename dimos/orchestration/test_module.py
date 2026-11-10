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

from pathlib import Path

import pytest

pytest.importorskip("cel_expr_python")

from dimos_lcm.actionlib_msgs import GoalStatus

from dimos.msgs.std_msgs.Bool import Bool
from dimos.msgs.std_msgs.String import String
from dimos.orchestration.definition import DefinitionError
from dimos.orchestration.module import StateMachineModule, state_machine
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
