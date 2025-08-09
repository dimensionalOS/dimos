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

from dimos.core import Module, rpc
from dimos.protocol.skill import SkillCoordinator, SkillState, skill
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.protocol.agents2")


class Agent(Module):
    _coordinator: SkillCoordinator

    def __init__(self, model: str = "gpt-4o", model_provider: str = "openai", *args, **kwargs):
        super().__init__(*args, **kwargs)

        # Ensure asyncio loop exists
        try:
            self._loop = asyncio.get_running_loop()
        except RuntimeError:
            self._loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self._loop)

        self._coordinator = SkillCoordinator()
        self._coordinator.start()

        self.messages = []
        self._llm = init_chat_model(
            model=model,
            model_provider=model_provider,
        )

    def register_skills(self, container: SkillCoordinator):
        return self._coordinator.register_skills(container)

    async def agent_loop(self, seed_query: str = ""):
        try:
            self.messages.append(HumanMessage(seed_query))

            while True:
                tools = self._coordinator.get_tools()
                self._llm = self._llm.bind_tools(tools)

                msg = self._llm.invoke(self.messages)
                self.messages.append(msg)

                logger.info(f"Agent response: {msg.content}")
                if msg.tool_calls:
                    self._coordinator.execute_tool_calls(msg.tool_calls)

                if not self._coordinator.has_active_skills():
                    logger.info("No active tasks, exiting agent loop.")
                    return

                await self._coordinator.wait_for_updates()

                for call_id, update in self._coordinator.generate_snapshot(clear=True).items():
                    self.messages.append(update.agent_encode())

        except Exception as e:
            print("Agent loop exception:", e)
            import traceback

            traceback.print_exc()

    @rpc
    def query(self, query: str):
        asyncio.ensure_future(self.agent_loop(query), loop=self._loop)
