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

"""Run a case on any BaseAgent: over RPC, over its streams, or as a plain class."""

from concurrent.futures import ThreadPoolExecutor, TimeoutError as FutureTimeoutError
import importlib
import json
from pathlib import Path
from threading import Event
import time
from typing import Any, Literal

from langchain_core.messages import BaseMessage, HumanMessage

from dimos.agents.base_agent import BaseAgent, _matches
from dimos.agents.client import agent_transports, running_agents
from dimos.evals.agents.base import EvalSubject, SubjectConfig
from dimos.evals.agents.lib.trajectory_builder import TrajectoryBuilder
from dimos.evals.environments.base import Environment
from dimos.evals.types import RunningEnvironment, Trajectory
from dimos.porcelain.dimos import Dimos


class BaseAgentAdapterConfig(SubjectConfig):
    # rpc: infer() on a running agent. stream: publish on its agent_input and read
    # agent_output. class: construct agent_class in this process and call infer().
    via: Literal["rpc", "stream", "class"] = "rpc"
    # Instance name of the running agent; may be left out when one agent is running.
    agent: str | None = None
    # "package.module:Class". Required for via="class"; with the other routes it lets
    # preflight check the types before anything starts.
    agent_class: str = ""
    agent_kwargs: dict[str, Any] = {}


def _load(path: str) -> type[BaseAgent[Any, Any]]:
    module, _, name = path.partition(":")
    cls = getattr(importlib.import_module(module), name)
    if not (isinstance(cls, type) and issubclass(cls, BaseAgent)):
        raise TypeError(f"{path} is not a BaseAgent")
    return cls


def as_input(input_type: Any, agent: str, inputs: str) -> Any:
    """The case's text as the agent's input type, or TypeError when the agent takes neither
    text nor a HumanMessage."""
    for candidate in (inputs, HumanMessage(inputs)):
        if _matches(input_type, candidate):
            return candidate
    raise TypeError(f"{agent} takes {input_type!r}; an eval case gives text")


def _text(output: Any) -> str:
    if isinstance(output, BaseMessage):
        return str(output.text)
    return output if isinstance(output, str) else json.dumps(output, default=str)


class BaseAgentAdapter(EvalSubject):
    """An eval subject for any ``BaseAgent``, with no code per agent.

    The agent's input type is read before the case runs; a case the agent cannot take
    raises instead of running. The trajectory is the input and the agent's answer.
    """

    config: BaseAgentAdapterConfig

    def preflight(self, environment: Environment) -> None:
        if self.config.via == "class" and not self.config.agent_class:
            raise ValueError('via="class" needs agent_class="package.module:Class"')
        if self.config.agent_class:
            cls = _load(self.config.agent_class)
            as_input(cls.input_type, cls.__name__, "")
        elif not environment.has_robot and not self.config.modules:
            raise RuntimeError(
                "BaseAgentAdapter needs a running agent: the environment has no robot and "
                "this subject adds no modules"
            )

    def run(
        self, inputs: str, env: RunningEnvironment, run_dir: Path, *, timeout_s: float
    ) -> Trajectory:
        name = self.config.agent_class.rpartition(":")[2] or self.config.agent or "agent"
        trajectory = TrajectoryBuilder(inputs, name=name)
        raw = run_dir / "raw"
        raw.mkdir(parents=True, exist_ok=True)
        started = time.time()
        try:
            output = getattr(self, f"_via_{self.config.via}")(inputs, timeout_s)
        except (FutureTimeoutError, TimeoutError):
            return trajectory.build("timeout")
        except Exception as error:
            return trajectory.build("error", error=f"{type(error).__name__}: {error}")
        request, response = raw / "request.json", raw / "response.json"
        request.write_text(json.dumps({"via": self.config.via, "input": inputs}))
        response.write_text(json.dumps({"output": _text(output)}))
        trajectory.step(
            message=_text(output),
            request=request,
            response=response,
            at=started,
            latency_s=time.time() - started,
        )
        return trajectory.build("answer")

    def _via_class(self, inputs: str, timeout_s: float) -> Any:
        cls = _load(self.config.agent_class)
        agent = cls(**self.config.agent_kwargs)
        try:
            # The two lifecycle calls a coordinator makes; here nothing else is deployed.
            agent.start()
            if callable(on_system_modules := getattr(agent, "on_system_modules", None)):
                on_system_modules([])
            return _within(timeout_s, agent.infer, as_input(cls.input_type, cls.__name__, inputs))
        finally:
            agent.stop()

    def _via_rpc(self, inputs: str, timeout_s: float) -> Any:
        app = Dimos.connect()
        try:
            name = self._running(app)
            agent: Any = app.get_module(name)
            return _within(timeout_s, agent.infer, as_input(agent.agent_types()[0], name, inputs))
        finally:
            app.stop()

    def _via_stream(self, inputs: str, timeout_s: float) -> Any:
        app = Dimos.connect()
        try:
            name = self._running(app)
            agent: Any = app.get_module(name)
            value = as_input(agent.agent_types()[0], name, inputs)
        finally:
            app.stop()
        transports = agent_transports(name)
        answers: list[Any] = []
        done = Event()

        def on_idle(idle: bool) -> None:
            # Done as infer() is: the agent is idle again, background skills included.
            if idle and answers:
                done.set()

        try:
            transports["agent_output"].subscribe(answers.append)
            transports["agent_idle"].subscribe(on_idle)
            transports["agent_input"].publish(value)
            if not done.wait(timeout_s):
                raise TimeoutError
            return answers[-1]
        finally:
            for transport in transports.values():
                transport.stop()

    def _running(self, app: Dimos) -> str:
        if self.config.agent is not None:
            return self.config.agent
        agents = running_agents(app)
        if len(agents) != 1:
            raise LookupError(f"Name the agent to evaluate; running agents: {agents}")
        return agents[0]


def _within(timeout_s: float, call: Any, x: Any) -> Any:
    pool = ThreadPoolExecutor(max_workers=1)
    try:
        return pool.submit(call, x).result(timeout=timeout_s)
    finally:
        pool.shutdown(wait=False)
