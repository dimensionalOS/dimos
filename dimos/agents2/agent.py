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
import json
from operator import itemgetter
from typing import Any, Dict, List, Optional, Tuple, TypedDict, Union

from langchain.chat_models import init_chat_model
from langchain_core.messages import (
    AIMessage,
    HumanMessage,
    SystemMessage,
    ToolCall,
    ToolMessage,
)

from dimos.agents2.spec import AgentSpec
from dimos.core import rpc
from dimos.msgs.sensor_msgs import Image
from dimos.protocol.skill.coordinator import SkillCoordinator, SkillState, SkillStateDict
from dimos.protocol.skill.type import Output
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.protocol.agents2")


SYSTEM_MSG_APPEND = "\nYour message history will always be appended with a System Overview message that provides situational awareness."


def toolmsg_from_state(state: SkillState) -> ToolMessage:
    if state.skill_config.output != Output.standard:
        content = "Special output, see separate message"
    else:
        content = state.content()

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


# takes an overview of running skills from the coorindator
# and builds messages to be sent to an agent
def snapshot_to_messages(
    state: SkillStateDict,
    tool_calls: List[ToolCall],
) -> Tuple[List[ToolMessage], Optional[AIMessage]]:
    # builds a set of tool call ids from a previous agent request
    tool_call_ids = set(
        map(itemgetter("id"), tool_calls),
    )

    # build a tool msg responses
    tool_msgs: list[ToolMessage] = []

    # build a general skill state overview (for longer running skills)
    state_overview: list[Dict[str, SkillStateSummary]] = []

    # for special skills that want to return a separate message
    # (images for example, requires to be a HumanMessage)
    special_msgs: List[HumanMessage] = []

    # Initialize state_msg
    state_msg = None

    for skill_state in sorted(
        state.values(),
        key=lambda skill_state: skill_state.duration(),
    ):
        if skill_state.call_id in tool_call_ids:
            tool_msgs.append(toolmsg_from_state(skill_state))

        special_data = skill_state.skill_config.output != Output.standard
        if special_data:
            content = skill_state.content()
            if not content:
                continue
            special_msgs.append(HumanMessage(content=[content]))

        if skill_state.call_id in tool_call_ids:
            continue

        state_overview.append(summary_from_state(skill_state, special_data))

    if state_overview:
        state_msg = AIMessage(
            "State Overview:\n" + "\n".join(map(json.dumps, state_overview)),
        )

    return {
        "tool_msgs": tool_msgs if tool_msgs else [],
        "state_msgs": ([state_msg] if state_msg else []) + special_msgs,
    }


# Agent class job is to glue skill coordinator state to an agent, builds langchain messages
class Agent(AgentSpec):
    system_message: SystemMessage
    state_messages: List[Union[AIMessage, HumanMessage]]

    def __init__(
        self,
        *args,
        **kwargs,
    ):
        AgentSpec.__init__(self, *args, **kwargs)

        self.state_messages = []
        self.coordinator = SkillCoordinator()
        self._history = []

        if self.config.system_prompt:
            if isinstance(self.config.system_prompt, str):
                self.system_message = SystemMessage(self.config.system_prompt + SYSTEM_MSG_APPEND)
            else:
                self.config.system_prompt.content += SYSTEM_MSG_APPEND
                self.system_message = self.config.system_prompt

        self.publish(self.system_message)

        # Use provided model instance if available, otherwise initialize from config
        if self.config.model_instance:
            self._llm = self.config.model_instance
        else:
            self._llm = init_chat_model(
                model_provider=self.config.provider, model=self.config.model
            )

    @rpc
    def start(self):
        self.coordinator.start()

    @rpc
    def stop(self):
        self.coordinator.stop()

    def clear_history(self):
        self._history.clear()

    def append_history(self, *msgs: List[Union[AIMessage, HumanMessage]]):
        for msg in msgs:
            self.publish(msg)

        self._history.extend(msgs)

    def history(self):
        return [self.system_message] + self._history + self.state_messages

    # Used by agent to execute tool calls
    def execute_tool_calls(self, tool_calls: List[ToolCall]) -> None:
        """Execute a list of tool calls from the agent."""
        for tool_call in tool_calls:
            logger.info(f"executing skill call {tool_call}")
            self.coordinator.call_skill(
                tool_call.get("id"),
                tool_call.get("name"),
                tool_call.get("args"),
            )

    # used to inject skill calls into the agent loop without agent asking for it
    def run_implicit_skill(self, skill_name: str, *args, **kwargs) -> None:
        self.coordinator.call_skill(False, skill_name, {"args": args, "kwargs": kwargs})

    async def agent_loop(self, seed_query: str = ""):
        self.state_messages = []
        self.append_history(HumanMessage(seed_query))

        try:
            while True:
                # we are getting tools from the coordinator on each turn
                # since this allows for skillcontainers to dynamically provide new skills
                tools = self.get_tools()
                self._llm = self._llm.bind_tools(tools)

                # publish to /agent topic for observability
                for state_msg in self.state_messages:
                    self.publish(state_msg)

                # history() builds our message history dynamically
                # ensures we include latest system state, but not old ones.
                msg = self._llm.invoke(self.history())
                self.append_history(msg)

                logger.info(f"Agent response: {msg.content}")

                if msg.tool_calls:
                    self.execute_tool_calls(msg.tool_calls)

                print(self)
                print(self.coordinator)

                if not self.coordinator.has_active_skills():
                    logger.info("No active tasks, exiting agent loop.")
                    return msg.content

                # coordinator will continue once a skill state has changed in
                # such a way that agent call needs to be executed
                await self.coordinator.wait_for_updates()

                # we request a full snapshot of currently running, finished or errored out skills
                # we ask for removal of finished skills from subsequent snapshots (clear=True)
                update = self.coordinator.generate_snapshot(clear=True)

                # generate tool_msgs and general state update message,
                # depending on a skill having associated tool call from previous interaction
                # we will return a tool message, and not a general state message
                snapshot_msgs = snapshot_to_messages(update, msg.tool_calls)

                self.state_messages = snapshot_msgs.get("state_msgs", [])
                self.append_history(*snapshot_msgs.get("tool_msgs", []))

        except Exception as e:
            logger.error(f"Error in agent loop: {e}")
            import traceback

            traceback.print_exc()

    def query(self, query: str):
        return asyncio.ensure_future(self.agent_loop(query), loop=self._loop)

    def query_async(self, query: str):
        return self.agent_loop(query)

    def register_skills(self, container):
        return self.coordinator.register_skills(container)

    def get_tools(self):
        return self.coordinator.get_tools()
