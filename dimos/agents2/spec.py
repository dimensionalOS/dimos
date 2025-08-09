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

"""Base agent module that wraps BaseAgent for DimOS module usage."""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple, Union

from langchain.chat_models.base import _SUPPORTED_PROVIDERS
from langchain_core.messages import (
    SystemMessage,
)

from dimos.core import rpc
from dimos.protocol.service import Service
from dimos.protocol.skill.skill import SkillContainer
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.agents.modules.base_agent")


# Dynamically create ModelProvider enum from LangChain's supported providers
Provider = Enum(
    "Provider", {provider.upper(): provider for provider in _SUPPORTED_PROVIDERS}, type=str
)


class Model(str, Enum):
    """Common model names across providers.

    Note: This is not exhaustive as model names change frequently.
    Based on langchain's _attempt_infer_model_provider patterns.
    """

    # OpenAI models (prefix: gpt-3, gpt-4, o1, o3)
    GPT_4O = "gpt-4o"
    GPT_4O_MINI = "gpt-4o-mini"
    GPT_4_TURBO = "gpt-4-turbo"
    GPT_4_TURBO_PREVIEW = "gpt-4-turbo-preview"
    GPT_4 = "gpt-4"
    GPT_35_TURBO = "gpt-3.5-turbo"
    GPT_35_TURBO_16K = "gpt-3.5-turbo-16k"
    O1_PREVIEW = "o1-preview"
    O1_MINI = "o1-mini"
    O3_MINI = "o3-mini"

    # Anthropic models (prefix: claude)
    CLAUDE_3_OPUS = "claude-3-opus-20240229"
    CLAUDE_3_SONNET = "claude-3-sonnet-20240229"
    CLAUDE_3_HAIKU = "claude-3-haiku-20240307"
    CLAUDE_35_SONNET = "claude-3-5-sonnet-20241022"
    CLAUDE_35_SONNET_LATEST = "claude-3-5-sonnet-latest"
    CLAUDE_3_7_SONNET = "claude-3-7-sonnet-20250219"

    # Google models (prefix: gemini)
    GEMINI_20_FLASH = "gemini-2.0-flash"
    GEMINI_15_PRO = "gemini-1.5-pro"
    GEMINI_15_FLASH = "gemini-1.5-flash"
    GEMINI_10_PRO = "gemini-1.0-pro"

    # Amazon Bedrock models (prefix: amazon)
    AMAZON_TITAN_EXPRESS = "amazon.titan-text-express-v1"
    AMAZON_TITAN_LITE = "amazon.titan-text-lite-v1"

    # Cohere models (prefix: command)
    COMMAND_R_PLUS = "command-r-plus"
    COMMAND_R = "command-r"
    COMMAND = "command"
    COMMAND_LIGHT = "command-light"

    # Fireworks models (prefix: accounts/fireworks)
    FIREWORKS_LLAMA_V3_70B = "accounts/fireworks/models/llama-v3-70b-instruct"
    FIREWORKS_MIXTRAL_8X7B = "accounts/fireworks/models/mixtral-8x7b-instruct"

    # Mistral models (prefix: mistral)
    MISTRAL_LARGE = "mistral-large"
    MISTRAL_MEDIUM = "mistral-medium"
    MISTRAL_SMALL = "mistral-small"
    MIXTRAL_8X7B = "mixtral-8x7b"
    MIXTRAL_8X22B = "mixtral-8x22b"
    MISTRAL_7B = "mistral-7b"

    # DeepSeek models (prefix: deepseek)
    DEEPSEEK_CHAT = "deepseek-chat"
    DEEPSEEK_CODER = "deepseek-coder"
    DEEPSEEK_R1_DISTILL_LLAMA_70B = "deepseek-r1-distill-llama-70b"

    # xAI models (prefix: grok)
    GROK_1 = "grok-1"
    GROK_2 = "grok-2"

    # Perplexity models (prefix: sonar)
    SONAR_SMALL_CHAT = "sonar-small-chat"
    SONAR_MEDIUM_CHAT = "sonar-medium-chat"
    SONAR_LARGE_CHAT = "sonar-large-chat"

    # Meta Llama models (various providers)
    LLAMA_3_70B = "llama-3-70b"
    LLAMA_3_8B = "llama-3-8b"
    LLAMA_31_70B = "llama-3.1-70b"
    LLAMA_31_8B = "llama-3.1-8b"
    LLAMA_33_70B = "llama-3.3-70b"
    LLAMA_2_70B = "llama-2-70b"
    LLAMA_2_13B = "llama-2-13b"
    LLAMA_2_7B = "llama-2-7b"


@dataclass
class AgentConfig:
    system_prompt: Optional[str | SystemMessage] = None
    skills: Optional[SkillContainer | list[SkillContainer]] = None
    model: Model = Model.GPT_4O
    provider: Provider = Provider.OPENAI


class AgentSpec(Service[AgentConfig], ABC):
    default_config: type[AgentConfig] = AgentConfig

    @rpc
    @abstractmethod
    def start(self): ...

    @rpc
    @abstractmethod
    def stop(self): ...

    @rpc
    @abstractmethod
    def clear_history(self): ...

    @rpc
    @abstractmethod
    def query(self, query: str): ...
