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

from collections.abc import Iterator
import json
from pathlib import Path

from langchain_core.messages import AIMessage, HumanMessage
import pytest

from dimos.agents.harness_agent import HarnessAgent, _answer


def _lines(*records: dict[str, object]) -> str:
    return "\n".join(json.dumps(record) for record in records) + "\n"


def _assistant(entry: int, text: str) -> dict[str, object]:
    content = [{"type": "thinking", "thinking": "hm"}, {"type": "text", "text": text}]
    return {"item": {"kind": "assistant", "entry": entry, "message": {"content": content}}}


_DONE = {"outcome": {"submission": 1, "status": "done", "answer": 7}}


def test_the_answer_is_the_entry_the_outcome_names() -> None:
    out = "http3 loader: Error: not json\n" + _lines(
        _assistant(5, "let me look"), _assistant(7, "It is a door."), _DONE
    )
    assert _answer(out, "").content == "It is a door."


@pytest.mark.parametrize(
    "out, error",
    [
        ("", "printed no outcome"),
        (_lines({"outcome": {"status": "unanswered", "reason": "aborted"}}), "unanswered: aborted"),
        (_lines(_assistant(5, "other"), _DONE), "did not print it"),
    ],
)
def test_a_run_without_an_answer_raises(out: str, error: str) -> None:
    with pytest.raises(RuntimeError, match=error):
        _answer(out, "the gateway did not start")


@pytest.fixture
def agent(tmp_path: Path) -> Iterator[HarnessAgent]:
    """A HarnessAgent whose cli is a script that records its arguments and answers."""
    cli = tmp_path / "dimos"
    cli.write_text(
        "#!/bin/sh\n"
        f"printf '%s\\n' \"$*\" >> {tmp_path}/calls\n"
        "echo 'Conversation 3' >&2\n"
        f"cat {tmp_path}/out\n"
    )
    cli.chmod(0o755)
    (tmp_path / "out").write_text(_lines(_assistant(7, "done"), _DONE))
    harness = HarnessAgent(cli=str(cli), cwd=tmp_path)
    try:
        yield harness
    finally:
        harness.stop()


def test_infer_runs_one_turn_and_keeps_the_conversation(
    agent: HarnessAgent, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    published: list[AIMessage] = []
    monkeypatch.setattr(agent.agent_output, "publish", published.append)

    assert agent.infer(HumanMessage("first")).content == "done"
    assert agent.conversation == 3
    assert agent.infer(HumanMessage("second")).content == "done"

    assert (tmp_path / "calls").read_text().splitlines() == [
        f"agent run --json --cwd {tmp_path} first",
        f"agent run --json --conversation 3 --cwd {tmp_path} second",
    ]
    assert [message.content for message in published] == ["done", "done"]
    assert (HarnessAgent.input_type, HarnessAgent.output_type) == (HumanMessage, AIMessage)
