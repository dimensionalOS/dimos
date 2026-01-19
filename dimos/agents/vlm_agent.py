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

from typing import Any

from langchain_core.messages import AIMessage, HumanMessage, SystemMessage

from dimos.agents.llm_init import build_llm, build_system_message
from dimos.agents.spec import AgentSpec, AnyMessage
from dimos.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs import Image
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class VLMAgent(AgentSpec):
    """Stream-first agent for vision queries with optional RPC access."""

    color_image: In[Image]
    query_stream: In[HumanMessage]
    answer_stream: Out[AIMessage]

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._llm = build_llm(self.config)
        self._latest_image: Image | None = None
        self._history: list[AIMessage | HumanMessage] = []
        self._system_message = build_system_message(self.config)
        self.publish(self._system_message)

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
        self.append_history([msg, response])  # type: ignore[arg-type]
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

    def append_history(self, *msgs: list[AIMessage | HumanMessage]) -> None:
        for msg_list in msgs:
            for msg in msg_list:
                self.publish(msg)  # type: ignore[arg-type]
            self._history.extend(msg_list)

    def history(self) -> list[AnyMessage]:
        return [self._system_message, *self._history]

    @rpc
    def register_skills(self, container: Any, run_implicit_name: str | None = None) -> None:
        logger.warning(
            "VLMAgent does not manage skills; register_skills is a no-op",
            container=str(container),
            run_implicit_name=run_implicit_name,
        )

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
