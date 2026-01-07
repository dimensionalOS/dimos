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

from typing import cast

from langchain.chat_models import init_chat_model
from langchain_core.language_models.chat_models import BaseChatModel
from langchain_core.messages import SystemMessage
from langchain_huggingface import ChatHuggingFace, HuggingFacePipeline

from dimos.agents.ollama_agent import ensure_ollama_model
from dimos.agents.spec import AgentConfig
from dimos.agents.system_prompt import SYSTEM_PROMPT


def build_llm(config: AgentConfig) -> BaseChatModel:
    if config.model_instance:
        return config.model_instance

    if config.provider.value.lower() == "ollama":
        ensure_ollama_model(config.model)

    if config.provider.value.lower() == "huggingface":
        llm = HuggingFacePipeline.from_model_id(
            model_id=config.model,
            task="text-generation",
            pipeline_kwargs={
                "max_new_tokens": 512,
                "temperature": 0.7,
            },
        )
        return ChatHuggingFace(llm=llm, model_id=config.model)

    return cast(
        "BaseChatModel",
        init_chat_model(  # type: ignore[call-overload]
            model_provider=config.provider,
            model=config.model,
        ),
    )


def build_system_message(config: AgentConfig, *, append: str = "") -> SystemMessage:
    if config.system_prompt:
        if isinstance(config.system_prompt, str):
            return SystemMessage(config.system_prompt + append)
        if append:
            config.system_prompt.content += append  # type: ignore[operator]
        return config.system_prompt

    return SystemMessage(SYSTEM_PROMPT + append)
