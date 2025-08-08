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

"""Base agent class with all features (non-module)."""

import asyncio
import json
import threading
from typing import Any, Dict, List, Optional, Union

from dimos.agents.agent_message import AgentMessage
from dimos.agents.agent_types import AgentResponse, ToolCall
from dimos.agents.memory.base import AbstractAgentSemanticMemory
from dimos.agents.memory.chroma_impl import OpenAISemanticMemory
from dimos.protocol.skill import SkillCoordinator, SkillState
from dimos.utils.logging_config import setup_logger

try:
    from .gateway import UnifiedGatewayClient
except ImportError:
    from dimos.agents.modules.gateway import UnifiedGatewayClient

logger = setup_logger("dimos.agents.modules.base")

# Vision-capable models
VISION_MODELS = {
    "openai::gpt-4o",
    "openai::gpt-4o-mini",
    "openai::gpt-4-turbo",
    "openai::gpt-4-vision-preview",
    "anthropic::claude-3-haiku-20240307",
    "anthropic::claude-3-sonnet-20241022",
    "anthropic::claude-3-opus-20240229",
    "anthropic::claude-3-5-sonnet-20241022",
    "anthropic::claude-3-5-haiku-latest",
    "qwen::qwen-vl-plus",
    "qwen::qwen-vl-max",
}


class BaseAgent:
    """Base agent with all features including memory, skills, and multimodal support.

    This class provides:
    - LLM gateway integration
    - Conversation history
    - Semantic memory (RAG)
    - Skills/tools execution (non-blocking)
    - Multimodal support (text, images, data)
    - Model capability detection
    """

    def __init__(
        self,
        model: str = "openai::gpt-4o-mini",
        system_prompt: Optional[str] = None,
        skills: Optional[SkillCoordinator] = None,
        memory: Optional[AbstractAgentSemanticMemory] = None,
        temperature: float = 0.0,
        max_tokens: int = 4096,
        max_input_tokens: int = 128000,
        max_history: int = 20,
        rag_n: int = 4,
        rag_threshold: float = 0.45,
        # Legacy compatibility
        dev_name: str = "BaseAgent",
        agent_type: str = "LLM",
        **kwargs,
    ):
        """Initialize the base agent with all features.

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
            dev_name: Device/agent name for logging
            agent_type: Type of agent for logging
        """
        self.model = model
        self.system_prompt = system_prompt or "You are a helpful AI assistant."
        self.temperature = temperature
        self.max_tokens = max_tokens
        self.max_input_tokens = max_input_tokens
        self.max_history = max_history
        self.rag_n = rag_n
        self.rag_threshold = rag_threshold
        self.dev_name = dev_name
        self.agent_type = agent_type

        self.skills = skills if skills else SkillCoordinator()

        # Initialize memory - allow None for testing
        if memory is False:  # Explicit False means no memory
            self.memory = None
        else:
            self.memory = memory or OpenAISemanticMemory()

        # Initialize gateway
        self.gateway = UnifiedGatewayClient()

        # Conversation history
        self.history = []
        self._history_lock = threading.Lock()

        # Check model capabilities
        self._supports_vision = self._check_vision_support()

        # Initialize memory with default context
        self._initialize_memory()

    def start(self):
        """Start the agent and its skills."""
        self.skills.start()

    def _check_vision_support(self) -> bool:
        """Check if the model supports vision."""
        return self.model in VISION_MODELS

    def _initialize_memory(self):
        """Initialize memory with default context."""
        try:
            contexts = [
                ("ctx1", "I am an AI assistant that can help with various tasks."),
                ("ctx2", f"I am using the {self.model} model."),
                (
                    "ctx3",
                    "I have access to tools and skills for specific operations."
                    if not self.skills.empty
                    else "I do not have access to external tools.",
                ),
                (
                    "ctx4",
                    "I can process images and visual content."
                    if self._supports_vision
                    else "I cannot process visual content.",
                ),
            ]
            if self.memory:
                for doc_id, text in contexts:
                    self.memory.add_vector(doc_id, text)
        except Exception as e:
            logger.warning(f"Failed to initialize memory: {e}")

    async def aquery(self, agent_msg: AgentMessage) -> AgentResponse:
        """Process query asynchronously and return AgentResponse.

        Args:
            agent_msg: The agent message containing text/images
            skill_results: Optional skill execution results from coordinator

        Returns:
            AgentResponse with content and optional tool calls
        """
        query_text = agent_msg.get_combined_text()
        logger.info(f"Processing query: {query_text}")

        # Get RAG context
        rag_context = self._get_rag_context(query_text)

        # Check if trying to use images with non-vision model
        if agent_msg.has_images() and not self._supports_vision:
            logger.warning(f"Model {self.model} does not support vision. Ignoring image input.")
            # Clear images from message
            agent_msg.images.clear()

        # Build messages including skill results if provided
        messages = self._build_messages(agent_msg, rag_context, skill_results)

        from pprint import pprint

        print("RETURNING", pprint(messages))
        # Get tools if available
        tools = self.skills.get_tools() if not self.skills.empty else None

        # Make inference call
        response = await self.gateway.ainference(
            model=self.model,
            messages=messages,
            tools=tools,
            temperature=self.temperature,
            max_tokens=self.max_tokens,
            stream=False,
        )

        # Extract response
        message = response["choices"][0]["message"]
        content = message.get("content", "")

        # Update history with user message and assistant response
        with self._history_lock:
            # Add user message (last message before assistant)
            if skill_results:
                # If we have skill results, add them to history
                # This includes the original user message + tool results
                self.history.extend(messages[-len(skill_results) - 1 :])
            else:
                # Just add the user message
                self.history.append(messages[-1])

            # Add assistant response
            self.history.append(message)

            # Trim history if needed
            if len(self.history) > self.max_history:
                self.history = self.history[-self.max_history :]

        # Check for tool calls
        tool_calls = None
        if "tool_calls" in message and message["tool_calls"]:
            tool_calls = [
                ToolCall(
                    id=tc["id"],
                    name=tc["function"]["name"],
                    arguments=json.loads(tc["function"]["arguments"]),
                    status="pending",
                )
                for tc in message["tool_calls"]
            ]

            # Return response indicating tools need to be executed
            return AgentResponse(
                content=content,
                role="assistant",
                tool_calls=tool_calls,
                requires_follow_up=True,  # Indicates coordinator should execute tools
                metadata={"model": self.model},
            )

        # No tools, return final response
        return AgentResponse(
            content=content,
            role="assistant",
            tool_calls=None,
            requires_follow_up=False,
            metadata={"model": self.model},
        )

    def _get_rag_context(self, query: str) -> str:
        """Get relevant context from memory."""
        if not self.memory:
            return ""

        try:
            results = self.memory.query(
                query_texts=query, n_results=self.rag_n, similarity_threshold=self.rag_threshold
            )

            if results:
                contexts = [doc.page_content for doc, _ in results]
                return " | ".join(contexts)
        except Exception as e:
            logger.warning(f"RAG query failed: {e}")

        return ""

    def _build_messages(
        self,
        agent_msg: AgentMessage,
        rag_context: str = "",
        skill_results: Optional[Dict[str, SkillState]] = None,
    ) -> List[Dict[str, Any]]:
        """Build messages list from AgentMessage and optional skill results."""
        messages = []

        # System prompt with RAG context if available
        system_content = self.system_prompt
        if rag_context:
            system_content += f"\n\nRelevant context: {rag_context}"
        messages.append({"role": "system", "content": system_content})

        # Add conversation history
        with self._history_lock:
            messages.extend(self.history)

        # If we have skill results, add them as tool messages
        if skill_results:
            for call_id, skill_state in skill_results.items():
                # Extract the return value from skill state
                for msg in skill_state._items:
                    if msg.type.name == "ret":
                        tool_msg = {
                            "role": "tool",
                            "tool_call_id": call_id,
                            "content": str(msg.content),
                            "name": skill_state.name,
                        }
                        messages.append(tool_msg)
                    elif msg.type.name == "error":
                        tool_msg = {
                            "role": "tool",
                            "tool_call_id": call_id,
                            "content": f"Error: {msg.content}",
                            "name": skill_state.name,
                        }
                        messages.append(tool_msg)

        # Build user message content from AgentMessage
        user_content = agent_msg.get_combined_text() if agent_msg.has_text() else ""

        # Handle images for vision models
        if agent_msg.has_images() and self._supports_vision:
            # Build content array with text and images
            content = []
            if user_content:  # Only add text if not empty
                content.append({"type": "text", "text": user_content})

            # Add all images from AgentMessage
            for img in agent_msg.images:
                content.append(
                    {
                        "type": "image_url",
                        "image_url": {"url": f"data:image/jpeg;base64,{img.base64_jpeg}"},
                    }
                )

            logger.debug(f"Building message with {len(content)} content items (vision enabled)")
            messages.append({"role": "user", "content": content})
        else:
            # Text-only message
            messages.append({"role": "user", "content": user_content})

        return messages

    def query(
        self,
        message: Union[str, AgentMessage],
        skill_results: Optional[Dict[str, SkillState]] = None,
    ) -> AgentResponse:
        """Synchronous query method for direct usage.

        Args:
            message: Either a string query or an AgentMessage with text and/or images
            skill_results: Optional skill execution results from coordinator

        Returns:
            AgentResponse object with content and tool information
        """
        # Convert string to AgentMessage if needed
        if isinstance(message, str):
            agent_msg = AgentMessage()
            agent_msg.add_text(message)
        else:
            agent_msg = message

        # Run async method in a new event loop
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            return loop.run_until_complete(self._process_query_async(agent_msg, skill_results))
        finally:
            loop.close()

    def stop(self):
        """Stop the agent and clean up resources."""
        self.skills.stop()
        if self.gateway:
            self.gateway.close()
