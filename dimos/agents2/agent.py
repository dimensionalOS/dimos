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
from pprint import pprint
from typing import List, Optional

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
from rich.console import Console
from rich.table import Table
from rich.text import Text

from dimos.agents2.spec import AgentSpec
from dimos.core import Module, rpc
from dimos.protocol.skill import skill
from dimos.protocol.skill.coordinator import SkillCoordinator, SkillState
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.protocol.agents2")


SYSTEM_MSG_APPEND = """
Your message history will always be appended with a System Overview message that provides situational awareness.
"""


class Agent(AgentSpec):
    def __init__(
        self,
        *args,
        **kwargs,
    ):
        AgentSpec.__init__(self, *args, **kwargs)

        self.coordinator = SkillCoordinator()
        self.messages = []

        if self.config.system_prompt:
            if isinstance(self.config.system_prompt, str):
                self.messages.append(SystemMessage(self.config.system_prompt + SYSTEM_MSG_APPEND))
            else:
                self.config.system_prompt.content += SYSTEM_MSG_APPEND
                self.messages.append(self.config.system_prompt)

        self._llm = init_chat_model(model_provider=self.config.provider, model=self.config.model)

    @rpc
    def start(self):
        self.coordinator.start()

    @rpc
    def stop(self):
        self.coordinator.stop()

    @rpc
    def clear_history(self):
        self.messages.clear()

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

    def __str__(self) -> str:
        console = Console(force_terminal=True, legacy_windows=False)

        table = Table(title="Agent History", show_header=True)

        table.add_column("Message Type", style="cyan", no_wrap=True)
        table.add_column("Content")

        for message in self.messages:
            if isinstance(message, HumanMessage):
                table.add_row(Text("Human", style="green"), Text(message.content, style="green"))
            elif isinstance(message, AIMessage):
                if hasattr(message, "metadata") and message.metadata.get("state"):
                    table.add_row(
                        Text("State Summary", style="blue"),
                        Text(message.content, style="blue"),
                    )
                else:
                    table.add_row(
                        Text("Agent", style="magenta"), Text(message.content, style="magenta")
                    )

                for tool_call in message.tool_calls:
                    table.add_row(
                        "Tool Call",
                        Text(
                            f"{tool_call.get('name')}({tool_call.get('args').get('args')})",
                            style="bold magenta",
                        ),
                    )
            elif isinstance(message, ToolMessage):
                table.add_row(
                    "Tool Response", Text(f"{message.name}() -> {message.content}"), style="red"
                )
            elif isinstance(message, SystemMessage):
                table.add_row("System", Text(message.content, style="yellow"))
            else:
                table.add_row("Unknown", str(message))

        # Render to string with title above
        with console.capture() as capture:
            console.print(Text("  Agent", style="bold blue"))
            console.print(table)
        return capture.get().strip()

    # used to inject skill calls into the agent loop without agent asking for it
    def run_implicit_skill(self, skill_name: str, *args, **kwargs) -> None:
        self.coordinator.call_skill(
            False,
            skill_name,
            {"args": args, "kwargs": kwargs},
        )

    async def agent_loop(self, seed_query: str = ""):
        self.messages.append(HumanMessage(seed_query))
        try:
            while True:
                tools = self.get_tools()
                self._llm = self._llm.bind_tools(tools)

                msg = self._llm.invoke(self.messages)
                self.messages.append(msg)

                logger.info(f"Agent response: {msg.content}")
                if msg.tool_calls:
                    self.execute_tool_calls(msg.tool_calls)

                if not self.coordinator.has_active_skills():
                    logger.info("No active tasks, exiting agent loop.")
                    return msg.content

                await self.coordinator.wait_for_updates()

                update = self.coordinator.generate_snapshot(clear=True)
                self.messages = self.messages + update.agent_encode()
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
