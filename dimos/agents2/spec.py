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

"""Agent specification layer for the agents2 system.

This module provides the abstract base class that all LLM-based agents in DimOS
must extend. The specification pattern separates "what agents must do" (history
management, query processing) from "how agents do it" (LLM provider, tool
calling implementation), enabling multiple agent implementations to share common
infrastructure.

When to use this module
-----------------------
**For most users**: Use `llm_agent()` from `dimos.agents2.agent` directly.
This module is primarily relevant when:

- Creating a custom agent implementation with different LLM backends
- Extending the agent system with new capabilities
- Understanding the agent architecture for debugging

The `AgentSpec` abstract base class defines required methods (`clear_history`,
`append_history`, `history`, `query`) while providing shared infrastructure
(message transport, lifecycle management, conversation display). Concrete
implementations like `Agent` in `dimos.agents2.agent` handle LLM interaction,
tool calling, and neurosymbolic orchestration.

Core classes
------------
AgentSpec
    Abstract base class defining required methods (history management, query
    interface) and providing shared infrastructure (transport, lifecycle,
    display).

AgentConfig
    Configuration dataclass specifying system prompt, model selection, skills,
    and message transport settings.

Enums
-----
Provider
    Dynamically generated enum of LLM providers (OPENAI, ANTHROPIC, etc.) based
    on LangChain's supported providers.

Model
    Common model identifiers across providers (GPT_4O, CLAUDE_35_SONNET, etc.).

Type aliases
------------
AnyMessage
    Union of LangChain message types: SystemMessage, ToolMessage, AIMessage,
    HumanMessage.

See also
--------
dimos.agents2.agent : Concrete Agent implementation and llm_agent() blueprint
dimos.protocol.service : Service base class for lifecycle management
dimos.core.module : Module base class for distributed execution
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum
from typing import Annotated, Any, Union

from annotated_doc import Doc
from langchain.chat_models.base import _SUPPORTED_PROVIDERS
from langchain_core.language_models.chat_models import BaseChatModel
from langchain_core.messages import (
    AIMessage,
    HumanMessage,
    SystemMessage,
    ToolMessage,
)
from rich.console import Console
from rich.table import Table
from rich.text import Text

from dimos.core import Module, rpc
from dimos.core.module import ModuleConfig
from dimos.protocol.pubsub import PubSub, lcm  # type: ignore[attr-defined]
from dimos.protocol.service import Service  # type: ignore[attr-defined]
from dimos.protocol.skill.skill import SkillContainer
from dimos.utils.generic import truncate_display_string
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


# Dynamically create ModelProvider enum from LangChain's supported providers
_providers = {provider.upper(): provider for provider in _SUPPORTED_PROVIDERS}
Provider = Enum("Provider", _providers, type=str)  # type: ignore[misc]


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
class AgentConfig(ModuleConfig):
    """Configuration for agent instances specifying model, prompt, and transport.

    Notes:
        Either use (`model`, `provider`) or `model_instance`, not both. When
        `model_instance` is provided, the other two are ignored.

        Set `agent_transport` to None to disable message publishing.

    Examples:
        Basic configuration with string prompt:

        >>> config = AgentConfig(
        ...     system_prompt="You are a helpful robot assistant.",
        ...     model=Model.GPT_4O_MINI,
        ...     provider=Provider.OPENAI
        ... )

        Using a mock model instance for testing:

        >>> from dimos.agents2.testing import MockModel
        >>> from langchain_core.messages import AIMessage
        >>> mock = MockModel(responses=[AIMessage(content="Test response")])
        >>> config = AgentConfig(
        ...     system_prompt="You are a helpful robot assistant.",
        ...     model_instance=mock
        ... )

        Disabling message transport:

        >>> config = AgentConfig(
        ...     system_prompt="Test agent",
        ...     agent_transport=None
        ... )
    """

    system_prompt: Annotated[
        str | SystemMessage | None,
        Doc(
            """Initial system instructions for the LLM. Can be provided as a string or
            pre-constructed SystemMessage. If None, a default prompt is used from
            `dimos.agents2.system_prompt.get_system_prompt()`."""
        ),
    ] = None

    skills: Annotated[
        SkillContainer | list[SkillContainer] | None,
        Doc(
            """Pre-registered skill containers. Currently unused by the agent system.
            Skills are typically registered via the skill coordinator after initialization."""
        ),
    ] = None

    model: Annotated[Model, Doc("Which LLM model to use (e.g., GPT_4O, CLAUDE_35_SONNET).")] = (
        Model.GPT_4O
    )

    provider: Annotated[
        Provider, Doc("Which LLM provider hosts the model (e.g., OPENAI, ANTHROPIC).")
    ] = Provider.OPENAI  # type: ignore[attr-defined]

    model_instance: Annotated[
        BaseChatModel | None,
        Doc(
            """Direct LangChain chat model instance. When provided, overrides `model` and
            `provider`. Useful for testing with mock models."""
        ),
    ] = None

    agent_transport: Annotated[
        type[PubSub],
        Doc(
            """Transport class for publishing agent messages. Must be a PubSub subclass that
            can be instantiated with no arguments. Used for observability (e.g., by the
            `agentspy` CLI tool)."""
        ),
    ] = lcm.PickleLCM  # type: ignore[type-arg]

    agent_topic: Annotated[Any, Doc("Topic identifier for agent message publishing.")] = field(
        default_factory=lambda: lcm.Topic("/agent")
    )


AnyMessage = Union[SystemMessage, ToolMessage, AIMessage, HumanMessage]
"""Union of LangChain message types returned by `AgentSpec.history()`.

Represents the four message types that can appear in an agent's conversation
history. Users typically encounter this type when inspecting message history
or implementing custom conversation processing logic.

Message types
-------------
SystemMessage
    Initial agent instructions loaded from `AgentConfig.system_prompt`. Always
    appears as the first message (`history()[0]`).

HumanMessage
    User queries submitted via `query()`, or outputs from skills configured with
    `Output.human`. May contain text or multimodal content (images, documents).

AIMessage
    LLM-generated responses. May include `tool_calls` requesting skill execution,
    or represent transient state awareness messages tracking long-running skills.

ToolMessage
    Results from executed skills, linked to AIMessage tool calls via `call_id`.
    The `content` field contains the skill's return value.

Working with message history
-----------------------------
Use `isinstance()` to distinguish message types when processing conversation history:

```pycon
>>> agent = llm_agent(...)
>>> for msg in agent.history():
...     if isinstance(msg, HumanMessage):
...         print(f"User: {msg.content}")
...     elif isinstance(msg, AIMessage):
...         print(f"Agent: {msg.content}")
```

All message types are from `langchain_core.messages`.
"""


class AgentSpec(Service[AgentConfig], Module, ABC):
    """Abstract specification for LLM-based agents in DimOS.

    Defines the interface contract that all agents must implement while providing
    common infrastructure for message transport, lifecycle management, and conversation
    display. Agents bridge high-level reasoning (LLMs) with low-level robot actions
    (skills) through a neurosymbolic orchestration pattern.

    This class is abstract. Concrete implementations must provide:

    - History management (`clear_history`, `append_history`, `history`)
    - Query processing (`query`)

    Concrete implementations receive:

    - Message publishing infrastructure (`publish`)
    - Lifecycle coordination (`start`, `stop`)
    - Rich conversation display (`__str__`)
    - Transport initialization

    Inheritance:
        Inherits from `Service[AgentConfig]` (configuration and lifecycle),
        `Module` (distributed execution and RPC), and `ABC` (abstract base class marker).

    Attributes:
        config (AgentConfig): Configuration instance created from `default_config`
            and constructor kwargs.
        transport (PubSub | None): Message transport for observability, initialized
            from `config.agent_transport` if provided.

    Notes:
        Concrete implementations must set `self._agent_id` (string identifier) for
        the `__str__` method to function correctly. See `dimos.agents2.agent.Agent`
        for the reference implementation.

        The `__init__` explicitly calls both `Service.__init__` and `Module.__init__`
        because multiple inheritance would otherwise skip one initialization path.

    See also:
        Agent: Concrete implementation in `dimos.agents2.agent`
        AgentConfig: Configuration dataclass
        Service: Base class for lifecycle management (`dimos.protocol.service`)
        Module: Base class for distributed execution (`dimos.core.module`)
    """

    default_config: type[AgentConfig] = AgentConfig

    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        """Initialize agent with configuration and transport."""
        Service.__init__(self, *args, **kwargs)
        Module.__init__(self, *args, **kwargs)

        if self.config.agent_transport:
            self.transport = self.config.agent_transport()

    def publish(
        self,
        msg: Annotated[
            AnyMessage,
            Doc("Message to publish (SystemMessage, HumanMessage, AIMessage, or ToolMessage)."),
        ],
    ) -> None:
        """Publish message to transport for observability.

        Used by concrete implementations to broadcast conversation messages to
        external monitoring tools like `agentspy`. Fire-and-forget semantics: if
        transport is None, the message is silently dropped.
        """
        if self.transport:
            self.transport.publish(self.config.agent_topic, msg)

    def start(self) -> None:
        """Start agent lifecycle, delegating to Service and Module initialization."""
        super().start()

    def stop(self) -> None:
        """Stop agent lifecycle, delegating to Service and Module cleanup."""
        super().stop()

    @rpc
    @abstractmethod
    def clear_history(self):  # type: ignore[no-untyped-def]
        """Clear persistent conversation history.

        Removes all accumulated conversation messages while preserving the system
        message. Transient state messages (managed by `agent_loop()`) are unaffected
        and will still appear in `history()`. Allows resetting conversation context
        while maintaining agent identity.
        """
        ...

    @abstractmethod
    def append_history(
        self,
        *msgs: Annotated[
            AIMessage | HumanMessage,
            Doc(
                "Variable number of AIMessage or HumanMessage instances to append to the conversation history."
            ),
        ],
    ) -> None:
        """Add messages to conversation history.

        Implementations must extend the history with provided messages in order
        and should publish each message to the transport for observability.
        """
        ...

    @abstractmethod
    def history(
        self,
    ) -> Annotated[
        list[AnyMessage],
        Doc(
            """List of all messages in the conversation, starting with the system
            message. May include HumanMessage, AIMessage, ToolMessage, and
            transient state awareness messages."""
        ),
    ]:
        """Return complete message history including transient state messages.

        Implementations must return a list where `history()[0]` is always the
        SystemMessage (initial prompt), followed by messages in chronological
        order. May include transient state messages representing skill execution.
        """
        ...

    @rpc
    @abstractmethod
    def query(
        self,
        query: Annotated[str, Doc("User query string to process.")],
    ) -> Annotated[
        str | None,
        Doc("Final agent response as string, or None if no response is generated."),
    ]:
        """Process user query through agent reasoning loop.

        Implementations must append query as HumanMessage to history, execute
        the agent loop until completion, process any tool calls via skill
        coordinator, and return the final response.

        Notes:
            This method typically blocks until the agent completes reasoning and
            any invoked skills finish execution.
        """
        ...

    def __str__(
        self,
    ) -> Annotated[
        str,
        Doc(
            """Formatted string containing agent ID header and colorized conversation
            table suitable for terminal display."""
        ),
    ]:
        """Render conversation history as formatted, colorized table, with color-coded message types.

        Notes:
            Requires `self._agent_id` to be set by concrete implementations.

            Message styling:

            - HumanMessage: Green text
            - AIMessage: Magenta text (blue for state summaries)
            - ToolMessage: Red text
            - SystemMessage: Yellow text, truncated to 800 characters

            Tool calls within AIMessage are displayed as separate rows showing
            the function name and arguments.

            Images in HumanMessage content are displayed as the placeholder text
            "<Image>" rather than attempting to render binary data.

        See also:
            history: Provides the messages rendered by this method.
        """
        console = Console(force_terminal=True, legacy_windows=False)
        table = Table(show_header=True)

        table.add_column("Message Type", style="cyan", no_wrap=True)
        table.add_column("Content")

        for message in self.history():
            if isinstance(message, HumanMessage):
                content = message.content
                if not isinstance(content, str):
                    content = "<Image>"

                table.add_row(Text("Human", style="green"), Text(content, style="green"))
            elif isinstance(message, AIMessage):
                if hasattr(message, "metadata") and message.metadata.get("state"):
                    table.add_row(
                        Text("State Summary", style="blue"),
                        Text(message.content, style="blue"),  # type: ignore[arg-type]
                    )
                else:
                    table.add_row(
                        Text("Agent", style="magenta"),
                        Text(message.content, style="magenta"),  # type: ignore[arg-type]
                    )

                for tool_call in message.tool_calls:
                    table.add_row(
                        "Tool Call",
                        Text(
                            f"{tool_call.get('name')}({tool_call.get('args')})",
                            style="bold magenta",
                        ),
                    )
            elif isinstance(message, ToolMessage):
                table.add_row(
                    "Tool Response", Text(f"{message.name}() -> {message.content}"), style="red"
                )
            elif isinstance(message, SystemMessage):
                table.add_row(
                    "System", Text(truncate_display_string(message.content, 800), style="yellow")
                )
            else:
                table.add_row("Unknown", str(message))

        # Render to string with title above
        with console.capture() as capture:
            console.print(Text(f"  Agent ({self._agent_id})", style="bold blue"))  # type: ignore[attr-defined]
            console.print(table)
        return capture.get().strip()
