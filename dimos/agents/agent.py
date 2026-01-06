# Copyright 2025 Dimensional Inc.
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
import asyncio
import datetime
import json
from operator import itemgetter
import os
from typing import Any, TypedDict
import uuid

from langchain.chat_models import init_chat_model
from langchain_core.messages import (
    AIMessage,
    HumanMessage,
    SystemMessage,
    ToolCall,
    ToolMessage,
)
from langchain_huggingface import ChatHuggingFace, HuggingFacePipeline

from dimos.agents.ollama_agent import ensure_ollama_model
from dimos.agents.spec import AgentSpec, Model, Provider
from dimos.agents.system_prompt import get_system_prompt
from dimos.core import DimosCluster, rpc
from dimos.protocol.skill.coordinator import (
    SkillCoordinator,
    SkillState,
    SkillStateDict,
)
from dimos.protocol.skill.skill import SkillContainer
from dimos.protocol.skill.type import Output
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


SYSTEM_MSG_APPEND = "\nYour message history will always be appended with a System Overview message that provides situational awareness."


def toolmsg_from_state(state: SkillState) -> ToolMessage:
    if state.skill_config.output != Output.standard:
        content = "output attached in separate messages"
    else:
        content = state.content()  # type: ignore[assignment]

    return ToolMessage(
        # if agent call has been triggered by another skill,
        # and this specific skill didn't finish yet but we need a tool call response
        # we return a message explaining that execution is still ongoing
        content=content
        or "Running, you will be called with an update, no need for subsequent tool calls",
        name=state.name,
        tool_call_id=state.call_id,
    )


class SkillStateSummary(TypedDict):
    name: str
    call_id: str
    state: str
    data: Any


def summary_from_state(state: SkillState, special_data: bool = False) -> SkillStateSummary:
    content = state.content()
    if isinstance(content, dict):
        content = json.dumps(content)

    if not isinstance(content, str):
        content = str(content)

    return {
        "name": state.name,
        "call_id": state.call_id,
        "state": state.state.name,
        "data": state.content() if not special_data else "data will be in a separate message",
    }


def _custom_json_serializers(obj):  # type: ignore[no-untyped-def]
    if isinstance(obj, datetime.date | datetime.datetime):
        return obj.isoformat()
    raise TypeError(f"Type {type(obj)} not serializable")


# takes an overview of running skills from the coorindator
# and builds messages to be sent to an agent
def snapshot_to_messages(
    state: SkillStateDict,
    tool_calls: list[ToolCall],
) -> tuple[list[ToolMessage], AIMessage | None]:
    # builds a set of tool call ids from a previous agent request
    tool_call_ids = set(
        map(itemgetter("id"), tool_calls),
    )

    # build a tool msg responses
    tool_msgs: list[ToolMessage] = []

    # build a general skill state overview (for longer running skills)
    state_overview: list[dict[str, SkillStateSummary]] = []

    # for special skills that want to return a separate message
    # (images for example, requires to be a HumanMessage)
    special_msgs: list[HumanMessage] = []

    # for special skills that want to return a separate message that should
    # stay in history, like actual human messages, critical events
    history_msgs: list[HumanMessage] = []

    # Initialize state_msg
    state_msg = None

    for skill_state in sorted(
        state.values(),
        key=lambda skill_state: skill_state.duration(),
    ):
        if skill_state.call_id in tool_call_ids:
            tool_msgs.append(toolmsg_from_state(skill_state))

        if skill_state.skill_config.output == Output.human:
            content = skill_state.content()
            if not content:
                continue
            history_msgs.append(HumanMessage(content=content))  # type: ignore[arg-type]
            continue

        special_data = skill_state.skill_config.output == Output.image
        if special_data:
            content = skill_state.content()
            if not content:
                continue
            special_msgs.append(HumanMessage(content=content))  # type: ignore[arg-type]

        if skill_state.call_id in tool_call_ids:
            continue

        state_overview.append(summary_from_state(skill_state, special_data))  # type: ignore[arg-type]

    if state_overview:
        state_overview_str = "\n".join(
            json.dumps(s, default=_custom_json_serializers) for s in state_overview
        )
        state_msg = AIMessage("State Overview:\n" + state_overview_str)

    return {  # type: ignore[return-value]
        "tool_msgs": tool_msgs,
        "history_msgs": history_msgs,
        "state_msgs": ([state_msg] if state_msg else []) + special_msgs,
    }


# Agent class job is to glue skill coordinator state to an agent, builds langchain messages
class Agent(AgentSpec):
    system_message: SystemMessage
    state_messages: list[AIMessage | HumanMessage]

    def __init__(  # type: ignore[no-untyped-def]
        self,
        *args,
        **kwargs,
    ) -> None:
        AgentSpec.__init__(self, *args, **kwargs)

        self.state_messages = []
        self.coordinator = SkillCoordinator()
        self._history = []  # type: ignore[var-annotated]
        self._agent_id = str(uuid.uuid4())
        self._agent_stopped = False

        if self.config.system_prompt:
            if isinstance(self.config.system_prompt, str):
                self.system_message = SystemMessage(self.config.system_prompt + SYSTEM_MSG_APPEND)
            else:
                self.config.system_prompt.content += SYSTEM_MSG_APPEND  # type: ignore[operator]
                self.system_message = self.config.system_prompt
        else:
            self.system_message = SystemMessage(get_system_prompt() + SYSTEM_MSG_APPEND)

        self.publish(self.system_message)

        # Use provided model instance if available, otherwise initialize from config
        if self.config.model_instance:
            self._llm = self.config.model_instance
        else:
            # For Ollama provider, ensure the model is available before initializing
            if self.config.provider.value.lower() == "ollama":
                ensure_ollama_model(self.config.model)

            # For HuggingFace, we need to create a pipeline and wrap it in ChatHuggingFace
            if self.config.provider.value.lower() == "huggingface":
                llm = HuggingFacePipeline.from_model_id(
                    model_id=self.config.model,
                    task="text-generation",
                    pipeline_kwargs={
                        "max_new_tokens": 512,
                        "temperature": 0.7,
                    },
                )
                self._llm = ChatHuggingFace(llm=llm, model_id=self.config.model)
            else:
                self._llm = init_chat_model(  # type: ignore[call-overload]
                    model_provider=self.config.provider, model=self.config.model
                )

    @rpc
    def get_agent_id(self) -> str:
        return self._agent_id

    @rpc
    def start(self) -> None:
        super().start()
        self.coordinator.start()

    @rpc
    def stop(self) -> None:
        self.coordinator.stop()
        self._agent_stopped = True
        super().stop()

    def clear_history(self) -> None:
        self._history.clear()

    def append_history(self, *msgs: list[AIMessage | HumanMessage]) -> None:
        for msg in msgs:
            self.publish(msg)  # type: ignore[arg-type]

        self._history.extend(msgs)

    def history(self):  # type: ignore[no-untyped-def]
        return [self.system_message, *self._history, *self.state_messages]

    # Used by agent to execute tool calls
    def execute_tool_calls(self, tool_calls: list[ToolCall]) -> None:
        """Execute a list of tool calls from the agent."""
        if self._agent_stopped:
            logger.warning("Agent is stopped, cannot execute tool calls.")
            return
        for tool_call in tool_calls:
            logger.info(f"executing skill call {tool_call}")
            self.coordinator.call_skill(
                tool_call.get("id"),  # type: ignore[arg-type]
                tool_call.get("name"),  # type: ignore[arg-type]
                tool_call.get("args"),  # type: ignore[arg-type]
            )

    # used to inject skill calls into the agent loop without agent asking for it
    def run_implicit_skill(self, skill_name: str, **kwargs) -> None:  # type: ignore[no-untyped-def]
        if self._agent_stopped:
            logger.warning("Agent is stopped, cannot execute implicit skill calls.")
            return
        self.coordinator.call_skill(False, skill_name, {"args": kwargs})

    async def agent_loop(self, first_query: str = ""):  # type: ignore[no-untyped-def]
        # TODO: Should I add a lock here to prevent concurrent calls to agent_loop?

        if self._agent_stopped:
            logger.warning("Agent is stopped, cannot run agent loop.")
            # return "Agent is stopped."
            import traceback

            traceback.print_stack()
            return "Agent is stopped."

        self.state_messages = []
        if first_query:
            self.append_history(HumanMessage(first_query))  # type: ignore[arg-type]

        def _get_state() -> str:
            # TODO: FIX THIS EXTREME HACK
            update = self.coordinator.generate_snapshot(clear=False)
            snapshot_msgs = snapshot_to_messages(update, msg.tool_calls)  # type: ignore[attr-defined]
            return json.dumps(snapshot_msgs, sort_keys=True, default=lambda o: repr(o))

        try:
            while True:
                # we are getting tools from the coordinator on each turn
                # since this allows for skillcontainers to dynamically provide new skills
                tools = self.get_tools()  # type: ignore[no-untyped-call]
                self._llm = self._llm.bind_tools(tools)  # type: ignore[assignment]

                # publish to /agent topic for observability
                for state_msg in self.state_messages:
                    self.publish(state_msg)

                # history() builds our message history dynamically
                # ensures we include latest system state, but not old ones.
                messages = self.history()  # type: ignore[no-untyped-call]

                # Some LLMs don't work without any human messages. Add an initial one.
                if len(messages) == 1 and isinstance(messages[0], SystemMessage):
                    messages.append(
                        HumanMessage(
                            "Everything is initialized. I'll let you know when you should act."
                        )
                    )
                    self.append_history(messages[-1])

                msg = self._llm.invoke(messages)

                self.append_history(msg)  # type: ignore[arg-type]

                logger.info(f"Agent response: {msg.content}")

                state = _get_state()

                if msg.tool_calls:  # type: ignore[attr-defined]
                    self.execute_tool_calls(msg.tool_calls)  # type: ignore[attr-defined]

                # print(self)
                # print(self.coordinator)

                self._write_debug_history_file()

                if not self.coordinator.has_active_skills():
                    logger.info("No active tasks, exiting agent loop.")
                    return msg.content

                # coordinator will continue once a skill state has changed in
                # such a way that agent call needs to be executed

                if state == _get_state():
                    await self.coordinator.wait_for_updates()

                # we request a full snapshot of currently running, finished or errored out skills
                # we ask for removal of finished skills from subsequent snapshots (clear=True)
                update = self.coordinator.generate_snapshot(clear=True)

                # generate tool_msgs and general state update message,
                # depending on a skill having associated tool call from previous interaction
                # we will return a tool message, and not a general state message
                snapshot_msgs = snapshot_to_messages(update, msg.tool_calls)  # type: ignore[attr-defined]

                self.state_messages = snapshot_msgs.get("state_msgs", [])  # type: ignore[attr-defined]
                self.append_history(
                    *snapshot_msgs.get("tool_msgs", []),  # type: ignore[attr-defined]
                    *snapshot_msgs.get("history_msgs", []),  # type: ignore[attr-defined]
                )

        except Exception as e:
            logger.error(f"Error in agent loop: {e}")
            import traceback

            traceback.print_exc()

    @rpc
    def loop_thread(self) -> bool:
        asyncio.run_coroutine_threadsafe(self.agent_loop(), self._loop)  # type: ignore[arg-type]
        return True

    @rpc
    def query(self, query: str):  # type: ignore[no-untyped-def]
        # TODO: could this be
        # from distributed.utils import sync
        # return sync(self._loop, self.agent_loop, query)
        return asyncio.run_coroutine_threadsafe(self.agent_loop(query), self._loop).result()  # type: ignore[arg-type]

    async def query_async(self, query: str):  # type: ignore[no-untyped-def]
        return await self.agent_loop(query)

    @rpc
    def register_skills(self, container, run_implicit_name: str | None = None):  # type: ignore[no-untyped-def]
        ret = self.coordinator.register_skills(container)  # type: ignore[func-returns-value]

        if run_implicit_name:
            self.run_implicit_skill(run_implicit_name)

        return ret

    def get_tools(self):  # type: ignore[no-untyped-def]
        return self.coordinator.get_tools()

    def _write_debug_history_file(self) -> None:
        file_path = os.getenv("DEBUG_AGENT_HISTORY_FILE")
        if not file_path:
            return

        history = [x.__dict__ for x in self.history()]  # type: ignore[no-untyped-call]

        with open(file_path, "w") as f:
            json.dump(history, f, default=lambda x: repr(x), indent=2)


class LlmAgent(Agent):
    @rpc
    def start(self) -> None:
        super().start()
        self.loop_thread()

    @rpc
    def stop(self) -> None:
        super().stop()


llm_agent = LlmAgent.blueprint


def deploy(
    dimos: DimosCluster,
    system_prompt: str = "You are a helpful assistant for controlling a Unitree Go2 robot.",
    model: Model = Model.GPT_4O,
    provider: Provider = Provider.OPENAI,  # type: ignore[attr-defined]
    skill_containers: list[SkillContainer] | None = None,
) -> Agent:
    from dimos.agents.cli.human import HumanInput

    if skill_containers is None:
        skill_containers = []
    agent = dimos.deploy(  # type: ignore[attr-defined]
        Agent,
        system_prompt=system_prompt,
        model=model,
        provider=provider,
    )

    human_input = dimos.deploy(HumanInput)  # type: ignore[attr-defined]
    human_input.start()

    agent.register_skills(human_input)

    for skill_container in skill_containers:
        print("Registering skill container:", skill_container)
        agent.register_skills(skill_container)

    agent.run_implicit_skill("human")
    agent.start()
    agent.loop_thread()

    return agent  # type: ignore[no-any-return]


__all__ = ["Agent", "deploy", "llm_agent"]
