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

from pydantic import TypeAdapter

from dimos.evals.sandbox.case import Conversation, FunctionRef, SandboxCase, load_suite
from dimos.evals.sandbox.example import SUITE


def test_cases_round_trip_through_json(tmp_path: Path) -> None:
    suite = tmp_path / "suite.json"
    suite.write_bytes(TypeAdapter(list[SandboxCase]).dump_json(SUITE))
    assert load_suite(str(suite)) == SUITE


def test_until_done_nags_until_done_or_out_of_times() -> None:
    conversation = Conversation(exec=lambda command: None, case_dir=Path("."))
    human = FunctionRef(
        function="dimos.evals.sandbox.human:until_done", params={"message": "go on", "times": 2}
    )(conversation)
    assert next(human) == "go on"
    assert next(human) == "go on"
    assert next(human, None) is None  # out of times

    conversation = Conversation(exec=lambda command: None, case_dir=Path("."))
    human = FunctionRef(function="dimos.evals.sandbox.human:until_done")(conversation)
    conversation.done = {"answer": 1}
    assert next(human, None) is None  # the agent called done
