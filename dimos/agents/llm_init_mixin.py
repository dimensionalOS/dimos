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
from langchain.chat_models import init_chat_model
from langchain_core.language_models.chat_models import BaseChatModel
from langchain_core.messages import SystemMessage
from langchain_huggingface import ChatHuggingFace, HuggingFacePipeline

from dimos.agents.ollama_agent import ensure_ollama_model
from dimos.agents.system_prompt import SYSTEM_PROMPT


class LlmInitMixin:
    def _init_llm(self) -> BaseChatModel:
        if self.config.model_instance:
            return self.config.model_instance

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
            return ChatHuggingFace(llm=llm, model_id=self.config.model)

        return init_chat_model(  # type: ignore[call-overload]
            model_provider=self.config.provider,
            model=self.config.model,
        )

    def _init_system_message(self, *, append: str = "") -> SystemMessage:
        if self.config.system_prompt:
            if isinstance(self.config.system_prompt, str):
                system_message = SystemMessage(self.config.system_prompt + append)
            else:
                if append:
                    self.config.system_prompt.content += append  # type: ignore[operator]
                system_message = self.config.system_prompt
        else:
            system_message = SystemMessage(SYSTEM_PROMPT + append)

        self.publish(system_message)
        return system_message
