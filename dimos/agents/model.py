# Copyright 2026 Dimensional Inc.
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

"""Shared chat-model construction for agents and evals."""

from __future__ import annotations

from typing import TYPE_CHECKING

from langchain.chat_models import init_chat_model
from langchain_openai import ChatOpenAI

if TYPE_CHECKING:
    from langchain_core.language_models.chat_models import BaseChatModel

_RESPONSES_REASONING_MODEL_PREFIXES = ("gpt-5", "o1", "o3", "o4")


def init_model(model_name: str) -> BaseChatModel:
    """Initialize a model while preserving LangChain provider resolution.

    OpenAI reasoning models (gpt-5*/o*) without an explicit ``provider:`` prefix
    go through the Responses API with reasoning enabled — the same configuration
    the production ``McpClient`` runs, so evals measure what deploys.
    """
    if ":" in model_name or not model_name.startswith(_RESPONSES_REASONING_MODEL_PREFIXES):
        return init_chat_model(model=model_name)

    return ChatOpenAI(
        model=model_name,
        use_responses_api=True,
        reasoning={"effort": "medium", "summary": "auto"},
    )
