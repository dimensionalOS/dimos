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
import time
from types import SimpleNamespace
from typing import Any

from langchain_core.messages import AIMessage, HumanMessage
import pytest

from dimos.agents.base_agent import BaseAgent
from dimos.core.core import rpc
from dimos.evals.agents.base_agent_adapter import BaseAgentAdapter, as_input
from dimos.evals.types import RunningEnvironment
from dimos.msgs.geometry_msgs.Twist import Twist

_HERE = "dimos.evals.agents.test_base_agent_adapter"
_ENV = RunningEnvironment(mcp_url="", streams=(), artifacts={})


class EchoAgent(BaseAgent[str, str]):
    @rpc
    def infer(self, x: str) -> str:
        return x.upper()


class ChatAgent(BaseAgent[HumanMessage, AIMessage]):
    @rpc
    def infer(self, x: HumanMessage) -> AIMessage:
        return AIMessage(f"you said {x.text}")


class TwistAgent(BaseAgent[Twist, Twist]):
    pass


class SlowAgent(BaseAgent[str, str]):
    @rpc
    def infer(self, x: str) -> str:
        time.sleep(5)
        return x


def _adapter(agent: type[BaseAgent[Any, Any]]) -> BaseAgentAdapter:
    return BaseAgentAdapter(via="class", agent_class=f"{_HERE}:{agent.__name__}")


def test_case_text_becomes_the_input_the_agent_takes() -> None:
    assert as_input(str, "EchoAgent", "hi") == "hi"
    assert as_input(HumanMessage, "ChatAgent", "hi") == HumanMessage("hi")
    with pytest.raises(TypeError, match="TwistAgent takes .*Twist.*; an eval case gives text"):
        as_input(Twist, "TwistAgent", "hi")


def test_preflight_refuses_an_agent_that_cannot_take_the_case() -> None:
    environment: Any = SimpleNamespace(has_robot=False)
    _adapter(EchoAgent).preflight(environment)
    with pytest.raises(TypeError, match="TwistAgent takes"):
        _adapter(TwistAgent).preflight(environment)
    with pytest.raises(TypeError, match="is not a BaseAgent"):
        BaseAgentAdapter(via="class", agent_class="pathlib:Path").preflight(environment)
    with pytest.raises(ValueError, match="needs agent_class"):
        BaseAgentAdapter(via="class").preflight(environment)
    with pytest.raises(RuntimeError, match="needs a running agent"):
        BaseAgentAdapter(via="rpc").preflight(environment)


@pytest.mark.parametrize(
    "agent, answer", [(EchoAgent, "GO TO THE BED"), (ChatAgent, "you said go to the bed")]
)
def test_an_agent_runs_as_a_plain_class(
    tmp_path: Path, agent: type[BaseAgent[Any, Any]], answer: str
) -> None:
    trajectory = _adapter(agent).run("go to the bed", _ENV, tmp_path, timeout_s=10)

    assert trajectory.extra.ended_by == "answer"
    assert [step.message for step in trajectory.steps] == ["go to the bed", answer]
    assert (tmp_path / "raw" / "response.json").exists()


def test_a_run_past_its_time_limit_ends_as_timeout(tmp_path: Path) -> None:
    trajectory = _adapter(SlowAgent).run("wait", _ENV, tmp_path, timeout_s=0.2)
    assert trajectory.extra.ended_by == "timeout"
