# Copyright 2025-2026 Dimensional Inc.
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

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from langchain.chat_models import init_chat_model
from langchain_core.messages import AIMessage, HumanMessage, SystemMessage

from dimos.agents.system_prompt import SYSTEM_PROMPT
from dimos.core import Module, rpc
from dimos.core.module import ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs import Image
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from langchain_core.language_models.chat_models import BaseChatModel

logger = setup_logger()


@dataclass
class VLMAgentConfig(ModuleConfig):
    model: str = "gpt-4o"
    system_prompt: str | None = SYSTEM_PROMPT


class VLMAgent(Module):
    """Stream-first agent for vision queries with optional RPC access."""

    default_config: type[VLMAgentConfig] = VLMAgentConfig
    config: VLMAgentConfig

    color_image: In[Image]
    query_stream: In[HumanMessage]
    answer_stream: Out[AIMessage]

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)

        if self.config.model.startswith("ollama:"):
            from dimos.agents.ollama_agent import ensure_ollama_model

            ensure_ollama_model(self.config.model.removeprefix("ollama:"))

        self._llm: BaseChatModel = init_chat_model(self.config.model)  # type: ignore[assignment]
        self._latest_image: Image | None = None
        self._history: list[AIMessage | HumanMessage] = []
        self._system_message = SystemMessage(self.config.system_prompt or SYSTEM_PROMPT)

    @rpc
    def start(self) -> None:
        super().start()
        self._disposables.add(self.color_image.subscribe(self._on_image))  # type: ignore[arg-type]
        self._disposables.add(self.query_stream.subscribe(self._on_query))  # type: ignore[arg-type]

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_image(self, image: Image) -> None:
        self._latest_image = image

    def _on_query(self, msg: HumanMessage) -> None:
        if not self._latest_image:
            self.answer_stream.publish(AIMessage(content="No image available yet."))
            return

        query_text = self._extract_text(msg)
        response = self._invoke_image(self._latest_image, query_text)
        self.answer_stream.publish(response)

    def _extract_text(self, msg: HumanMessage) -> str:
        content = msg.content
        if isinstance(content, str):
            return content
        if isinstance(content, list):
            for part in content:
                if isinstance(part, dict) and part.get("type") == "text":
                    return str(part.get("text", ""))
        return str(content)

    def _invoke(self, msg: HumanMessage, **kwargs: Any) -> AIMessage:
        messages = [self._system_message, msg]
        response = self._llm.invoke(messages, **kwargs)
        self._history.extend([msg, response])  # type: ignore[arg-type]
        return response  # type: ignore[return-value]

    def _invoke_image(
        self, image: Image, query: str, response_format: dict[str, Any] | None = None
    ) -> AIMessage:
        content = [{"type": "text", "text": query}, *image.agent_encode()]
        kwargs: dict[str, Any] = {}
        if response_format:
            kwargs["response_format"] = response_format
        return self._invoke(HumanMessage(content=content), **kwargs)

    @rpc
    def clear_history(self) -> None:
        self._history.clear()

    @rpc
    def query(self, query: str) -> str:
        response = self._invoke(HumanMessage(query))
        content = response.content
        return content if isinstance(content, str) else str(content)

    @rpc
    def query_image(
        self, image: Image, query: str, response_format: dict[str, Any] | None = None
    ) -> str:
        response = self._invoke_image(image, query, response_format=response_format)
        content = response.content
        return content if isinstance(content, str) else str(content)


vlm_agent = VLMAgent.blueprint

__all__ = ["VLMAgent", "vlm_agent"]
