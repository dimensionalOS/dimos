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
from __future__ import annotations

from typing import TYPE_CHECKING

from langchain.chat_models import init_chat_model
from langchain_core.messages import AIMessage, HumanMessage, SystemMessage
from langchain_huggingface import ChatHuggingFace, HuggingFacePipeline

from dimos.agents.ollama_agent import ensure_ollama_model
from dimos.agents.spec import AgentSpec
from dimos.agents.system_prompt import SYSTEM_PROMPT
from dimos.core import rpc
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core.stream import In, Out
    from dimos.msgs.sensor_msgs import Image

logger = setup_logger()


class VLMAgent(AgentSpec):
    """Stream-first agent for vision queries with optional RPC access."""

    color_image: In[Image]
    query: In[HumanMessage]
    answer: Out[AIMessage]

    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(*args, **kwargs)
        if self.config.model_instance:
            self._llm = self.config.model_instance
        else:
            if self.config.provider.value.lower() == "ollama":
                ensure_ollama_model(self.config.model)

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
        self._latest_image: Image | None = None
        self._history: list[AIMessage | HumanMessage] = []

        if self.config.system_prompt:
            if isinstance(self.config.system_prompt, str):
                self._system_message = SystemMessage(self.config.system_prompt)
            else:
                self._system_message = self.config.system_prompt
        else:
            self._system_message = SystemMessage(SYSTEM_PROMPT)

        self.publish(self._system_message)

    @rpc
    def start(self) -> None:
        super().start()
        self._disposables.add(self.color_image.subscribe(self._on_image))  # type: ignore[arg-type]
        self._disposables.add(self.query.subscribe(self._on_query))  # type: ignore[arg-type]

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_image(self, image: Image) -> None:
        self._latest_image = image

    def _on_query(self, msg: HumanMessage) -> None:
        if not self._latest_image:
            self.answer.publish(AIMessage(content="No image available yet."))
            return

        query_text = self._extract_text(msg)
        response = self._invoke_image(self._latest_image, query_text)
        self.answer.publish(response)

    def _extract_text(self, msg: HumanMessage) -> str:
        content = msg.content
        if isinstance(content, str):
            return content
        if isinstance(content, list):
            for part in content:
                if isinstance(part, dict) and part.get("type") == "text":
                    return str(part.get("text", ""))
        return str(content)

    def _invoke(self, msg: HumanMessage) -> AIMessage:
        messages = [self._system_message, msg]
        response = self._llm.invoke(messages)
        self.append_history(msg, response)  # type: ignore[arg-type]
        return response  # type: ignore[return-value]

    def _invoke_image(self, image: Image, query: str) -> AIMessage:
        content = [{"type": "text", "text": query}, *image.agent_encode()]
        return self._invoke(HumanMessage(content=content))

    @rpc
    def clear_history(self):  # type: ignore[no-untyped-def]
        self._history.clear()

    def append_history(self, *msgs: list[AIMessage | HumanMessage]) -> None:
        for msg in msgs:
            self.publish(msg)  # type: ignore[arg-type]
        self._history.extend(msgs)

    def history(self) -> list[SystemMessage | AIMessage | HumanMessage]:
        return [self._system_message, *self._history]

    @rpc
    def register_skills(  # type: ignore[no-untyped-def]
        self, container, run_implicit_name: str | None = None
    ) -> None:
        logger.warning(
            "VLMAgent does not manage skills; register_skills is a no-op",
            container=str(container),
            run_implicit_name=run_implicit_name,
        )

    @rpc
    def query(self, query: str):  # type: ignore[no-untyped-def]
        response = self._invoke(HumanMessage(query))
        return response.content

    @rpc
    def query_image(self, image: Image, query: str):  # type: ignore[no-untyped-def]
        response = self._invoke_image(image, query)
        return response.content


vlm_agent = VLMAgent.blueprint

__all__ = ["VLMAgent", "vlm_agent"]
