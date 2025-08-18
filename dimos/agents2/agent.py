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
from functools import reduce
from pprint import pprint
from typing import Any, Dict, List, Optional, Tuple, Union

from langchain.chat_models import init_chat_model
from langchain_core.language_models.chat_models import BaseChatModel
from langchain_core.messages import (
    AIMessage,
    HumanMessage,
    MessageLikeRepresentation,
    SystemMessage,
    ToolCall,
    ToolMessage,
)

from dimos.agents2.spec import AgentSpec
from dimos.core import Module, rpc
from dimos.protocol.skill import skill
from dimos.protocol.skill.coordinator import SkillCoordinator, SkillState, SkillStateDict
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.protocol.agents2")


SYSTEM_MSG_APPEND = """
Your message history will always be appended with a System Overview message that provides situational awareness.
"""


def toolmsg_from_state(state: SkillState) -> ToolMessage:
    return ToolMessage(
        # if agent call has been triggered by another skill,
        # but this specific skill didn't finish yet so we don't have data for a tool call response
        state.content()
        or "Loading, you will be called with an update, no need for subsequent tool calls",
        name=state.name,
        tool_call_id=state.call_id,
    )


def summary_from_state(state: SkillState) -> Dict[str, Any]:
    return {
        "name": state.name,
        "call_id": state.call_id,
        "state": state.state.name,
        "data": state.content(),
    }


def snapshot_to_messages(
    state: SkillStateDict,
    tool_calls: List[ToolCall],
) -> Tuple[List[ToolMessage], Optional[AIMessage]]:
    # tool call ids from a previous agent call
    tool_call_ids = set(
        map(
            lambda tool_call: tool_call.get("id"),
            tool_calls,
        )
    )

    # we build a tool msg responses
    tool_msgs: list[ToolMessage] = []

    # we build a general skill state overview (for longer running skills)
    state_overview: list[Dict[str, Any]] = []

    for skill_state in sorted(
        state.values(),
        key=lambda skill_state: skill_state.duration(),
    ):
        if skill_state.call_id in tool_call_ids:
            tool_msgs.append(toolmsg_from_state(skill_state))
            continue

        state_overview.append(summary_from_state(skill_state))

    if state_overview:
        state_msg = AIMessage(
            "State Overview:\n" + "\n".join(map(json.dumps, state_overview)),
            metadata={"state": True},
        )

        return tool_msgs, state_msg

    return tool_msgs, None


# Agent class job is to glue skill coordinator state to agent messages
class Agent(AgentSpec):
    system_message: SystemMessage
    state_message: Optional[AIMessage] = None

    def __init__(
        self,
        *args,
        **kwargs,
    ):
        AgentSpec.__init__(self, *args, **kwargs)

        self.coordinator = SkillCoordinator()
        self._history = []

        if self.config.system_prompt:
            if isinstance(self.config.system_prompt, str):
                self.system_message = SystemMessage(self.config.system_prompt + SYSTEM_MSG_APPEND)
            else:
                self.config.system_prompt.content += SYSTEM_MSG_APPEND
                self.system_message = self.config.system_prompt

        self._llm = init_chat_model(model_provider=self.config.provider, model=self.config.model)

    @rpc
    def start(self):
        self.coordinator.start()

    @rpc
    def stop(self):
        self.coordinator.stop()

    @rpc
    def clear_history(self):
        self._history.clear()

    def append_history(self, *msgs: List[Union[AIMessage, HumanMessage]]):
        self._history.extend(msgs)

    def history(self):
        return (
            [self.system_message]
            + self._history
            + ([self.state_message] if self.state_message else [])
        )

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
        self.append_history(HumanMessage(seed_query))

        try:
            while True:
                tools = self.get_tools()
                self._llm = self._llm.bind_tools(tools)

                # history() call ensures we include latest system state
                # and system message in our invocation
                msg = self._llm.invoke(self.history())
                self.append_history(msg)

                logger.info(f"Agent response: {msg.content}")

                if msg.tool_calls:
                    self.execute_tool_calls(msg.tool_calls)

                if not self.coordinator.has_active_skills():
                    logger.info("No active tasks, exiting agent loop.")
                    return msg.content

                # coordinator will continue once a skill state has changed in
                # such a way that agent call needs to be executed
                await self.coordinator.wait_for_updates()

                # we build a full snapshot of currently running skills
                # we also remove finished/errored out skills from subsequent snapshots (clear=True)
                update = self.coordinator.generate_snapshot(clear=True)

                # generate tool_msgs and general state update message,
                # depending on a skill is a tool call from previous interaction or not
                tool_msgs, state_msg = snapshot_to_messages(update, msg.tool_calls)

                self.state_message = state_msg
                self.append_history(*tool_msgs)

                print(self)
                print(self.coordinator)

        except Exception as e:
            logger.error(f"Error in agent loop: {e}")
            import traceback

            traceback.print_exc()

    @rpc
    def query_async(self, query: str):
        return asyncio.ensure_future(self.agent_loop(query), loop=self._loop)

    def query(self, query: str):
        return asyncio.run_coroutine_threadsafe(self.agent_loop(query), self._loop).result()

    @rpc
    def register_skills(self, container):
        return self.coordinator.register_skills(container)

    def get_tools(self):
        return self.coordinator.get_tools()
