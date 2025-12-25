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

import threading
from typing import Any

from dimos.agents_deprecated.agent_message import AgentMessage
from dimos.agents_deprecated.agent_types import AgentResponse
from dimos.agents_deprecated.memory.base import AbstractAgentSemanticMemory
from dimos.core import In, Module, Out, rpc
from dimos.skills.skills import AbstractSkill, SkillLibrary
from dimos.utils.logging_config import setup_logger

try:
    from .base import BaseAgent
except ImportError:
    from dimos.agents_deprecated.modules.base import BaseAgent

logger = setup_logger()


class BaseAgentModule(BaseAgent, Module):  # type: ignore[misc]
    """Agent module that inherits from BaseAgent and adds DimOS module interface.

    This provides a thin wrapper around BaseAgent functionality, exposing it
    through the DimOS module system with RPC methods and stream I/O.
    """

    # Module I/O - AgentMessage based communication
    message_in: In[AgentMessage]  # Primary input for AgentMessage
    response_out: Out[AgentResponse]  # Output AgentResponse objects

    def __init__(  # type: ignore[no-untyped-def]
        self,
        model: str = "openai::gpt-4o-mini",
        system_prompt: str | None = None,
        skills: SkillLibrary | list[AbstractSkill] | AbstractSkill | None = None,
        memory: AbstractAgentSemanticMemory | None = None,
        temperature: float = 0.0,
        max_tokens: int = 4096,
        max_input_tokens: int = 128000,
        max_history: int = 20,
        rag_n: int = 4,
        rag_threshold: float = 0.45,
        process_all_inputs: bool = False,
        **kwargs,
    ) -> None:
        """Initialize the agent module.

        Args:
            model: Model identifier (e.g., "openai::gpt-4o", "anthropic::claude-3-haiku")
            system_prompt: System prompt for the agent
            skills: Skills/tools available to the agent
            memory: Semantic memory system for RAG
            temperature: Sampling temperature
            max_tokens: Maximum tokens to generate
            max_input_tokens: Maximum input tokens
            max_history: Maximum conversation history to keep
            rag_n: Number of RAG results to fetch
            rag_threshold: Minimum similarity for RAG results
            process_all_inputs: Whether to process all inputs or drop when busy
            **kwargs: Additional arguments passed to Module
        """
        # Initialize Module first (important for DimOS)
        Module.__init__(self, **kwargs)

        # Initialize BaseAgent with all functionality
        BaseAgent.__init__(
            self,
            model=model,
            system_prompt=system_prompt,
            skills=skills,
            memory=memory,
            temperature=temperature,
            max_tokens=max_tokens,
            max_input_tokens=max_input_tokens,
            max_history=max_history,
            rag_n=rag_n,
            rag_threshold=rag_threshold,
            process_all_inputs=process_all_inputs,
            # Don't pass streams - we'll connect them in start()
            input_query_stream=None,
            input_data_stream=None,
            input_video_stream=None,
        )

        # Track module-specific subscriptions
        self._module_disposables = []  # type: ignore[var-annotated]

        # For legacy stream support
        self._latest_image = None
        self._latest_data = None
        self._image_lock = threading.Lock()
        self._data_lock = threading.Lock()

    @rpc
    def start(self) -> None:
        """Start the agent module and connect streams."""
        super().start()
        logger.info(f"Starting agent module with model: {self.model}")

        # Primary AgentMessage input
        if self.message_in and self.message_in.connection is not None:
            try:
                disposable = self.message_in.observable().subscribe(  # type: ignore[no-untyped-call]
                    lambda msg: self._handle_agent_message(msg)
                )
                self._module_disposables.append(disposable)
            except Exception as e:
                logger.debug(f"Could not connect message_in: {e}")

        # Connect response output
        if self.response_out:
            disposable = self.response_subject.subscribe(
                lambda response: self.response_out.publish(response)
            )
            self._module_disposables.append(disposable)

        logger.info("Agent module started")

    @rpc
    def stop(self) -> None:
        """Stop the agent module."""
        logger.info("Stopping agent module")

        # Dispose module subscriptions
        for disposable in self._module_disposables:
            disposable.dispose()
        self._module_disposables.clear()

        # Dispose BaseAgent resources
        self.base_agent_dispose()

        logger.info("Agent module stopped")
        super().stop()

    @rpc
    def clear_history(self) -> None:
        """Clear conversation history."""
        with self._history_lock:  # type: ignore[attr-defined]
            self.history = []  # type: ignore[var-annotated]
        logger.info("Conversation history cleared")

    @rpc
    def add_skill(self, skill: AbstractSkill) -> None:
        """Add a skill to the agent."""
        self.skills.add(skill)
        logger.info(f"Added skill: {skill.__class__.__name__}")

    @rpc
    def set_system_prompt(self, prompt: str) -> None:
        """Update system prompt."""
        self.system_prompt = prompt
        logger.info("System prompt updated")

    @rpc
    def get_conversation_history(self) -> list[dict[str, Any]]:
        """Get current conversation history."""
        with self._history_lock:  # type: ignore[attr-defined]
            return self.history.copy()

    def _handle_agent_message(self, message: AgentMessage) -> None:
        """Handle AgentMessage from module input."""
        # Process through BaseAgent query method
        try:
            response = self.query(message)
            logger.debug(f"Publishing response: {response}")
            self.response_subject.on_next(response)
        except Exception as e:
            logger.error(f"Agent message processing error: {e}")
            self.response_subject.on_error(e)

    def _handle_module_query(self, query: str) -> None:
        """Handle legacy query from module input."""
        # For simple text queries, just convert to AgentMessage
        agent_msg = AgentMessage()
        agent_msg.add_text(query)

        # Process through unified handler
        self._handle_agent_message(agent_msg)

    def _update_latest_data(self, data: dict[str, Any]) -> None:
        """Update latest data context."""
        with self._data_lock:
            self._latest_data = data  # type: ignore[assignment]

    def _update_latest_image(self, img: Any) -> None:
        """Update latest image."""
        with self._image_lock:
            self._latest_image = img

    def _format_data_context(self, data: dict[str, Any]) -> str:
        """Format data dictionary as context string."""
        # Simple formatting - can be customized
        parts = []
        for key, value in data.items():
            parts.append(f"{key}: {value}")
        return "\n".join(parts)
