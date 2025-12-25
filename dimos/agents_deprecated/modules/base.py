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
from concurrent.futures import ThreadPoolExecutor
import json
from typing import Any

from reactivex.subject import Subject

from dimos.agents_deprecated.agent_message import AgentMessage
from dimos.agents_deprecated.agent_types import AgentResponse, ConversationHistory, ToolCall
from dimos.agents_deprecated.memory.base import AbstractAgentSemanticMemory
from dimos.agents_deprecated.memory.chroma_impl import OpenAISemanticMemory
from dimos.skills.skills import AbstractSkill, SkillLibrary
from dimos.utils.logging_config import setup_logger

try:
    from .gateway import UnifiedGatewayClient
except ImportError:
    from dimos.agents_deprecated.modules.gateway import UnifiedGatewayClient

logger = setup_logger()

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
    - Skills/tools execution
    - Multimodal support (text, images, data)
    - Model capability detection
    """

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
        seed: int | None = None,
        # Legacy compatibility
        dev_name: str = "BaseAgent",
        agent_type: str = "LLM",
        **kwargs,
    ) -> None:
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
            seed: Random seed for deterministic outputs (if supported by model)
            dev_name: Device/agent name for logging
            agent_type: Type of agent for logging
        """
        self.model = model
        self.system_prompt = system_prompt or "You are a helpful AI assistant."
        self.temperature = temperature
        self.max_tokens = max_tokens
        self.max_input_tokens = max_input_tokens
        self._max_history = max_history
        self.rag_n = rag_n
        self.rag_threshold = rag_threshold
        self.seed = seed
        self.dev_name = dev_name
        self.agent_type = agent_type

        # Initialize skills
        if skills is None:
            self.skills = SkillLibrary()
        elif isinstance(skills, SkillLibrary):
            self.skills = skills
        elif isinstance(skills, list):
            self.skills = SkillLibrary()
            for skill in skills:
                self.skills.add(skill)
        elif isinstance(skills, AbstractSkill):
            self.skills = SkillLibrary()
            self.skills.add(skills)
        else:
            self.skills = SkillLibrary()

        # Initialize memory - allow None for testing
        if memory is False:  # type: ignore[comparison-overlap]  # Explicit False means no memory
            self.memory = None
        else:
            self.memory = memory or OpenAISemanticMemory()  # type: ignore[has-type]

        # Initialize gateway
        self.gateway = UnifiedGatewayClient()

        # Conversation history with proper format management
        self.conversation = ConversationHistory(max_size=self._max_history)

        # Thread pool for async operations
        self._executor = ThreadPoolExecutor(max_workers=2)

        # Response subject for emitting responses
        self.response_subject = Subject()  # type: ignore[var-annotated]

        # Check model capabilities
        self._supports_vision = self._check_vision_support()

        # Initialize memory with default context
        self._initialize_memory()

    @property
    def max_history(self) -> int:
        """Get max history size."""
        return self._max_history

    @max_history.setter
    def max_history(self, value: int) -> None:
        """Set max history size and update conversation."""
        self._max_history = value
        self.conversation.max_size = value

    def _check_vision_support(self) -> bool:
        """Check if the model supports vision."""
        return self.model in VISION_MODELS

    def _initialize_memory(self) -> None:
        """Initialize memory with default context."""
        try:
            contexts = [
                ("ctx1", "I am an AI assistant that can help with various tasks."),
                ("ctx2", f"I am using the {self.model} model."),
                (
                    "ctx3",
                    "I have access to tools and skills for specific operations."
                    if len(self.skills) > 0
                    else "I do not have access to external tools.",
                ),
                (
                    "ctx4",
                    "I can process images and visual content."
                    if self._supports_vision
                    else "I cannot process visual content.",
                ),
            ]
            if self.memory:  # type: ignore[has-type]
                for doc_id, text in contexts:
                    self.memory.add_vector(doc_id, text)  # type: ignore[has-type]
        except Exception as e:
            logger.warning(f"Failed to initialize memory: {e}")

    async def _process_query_async(self, agent_msg: AgentMessage) -> AgentResponse:
        """Process query asynchronously and return AgentResponse."""
        query_text = agent_msg.get_combined_text()
        logger.info(f"Processing query: {query_text}")

        # Get RAG context
        rag_context = self._get_rag_context(query_text)

        # Check if trying to use images with non-vision model
        if agent_msg.has_images() and not self._supports_vision:
            logger.warning(f"Model {self.model} does not support vision. Ignoring image input.")
            # Clear images from message
            agent_msg.images.clear()

        # Build messages - pass AgentMessage directly
        messages = self._build_messages(agent_msg, rag_context)

        # Get tools if available
        tools = self.skills.get_tools() if len(self.skills) > 0 else None

        # Debug logging before gateway call
        logger.debug("=== Gateway Request ===")
        logger.debug(f"Model: {self.model}")
        logger.debug(f"Number of messages: {len(messages)}")
        for i, msg in enumerate(messages):
            role = msg.get("role", "unknown")
            content = msg.get("content", "")
            if isinstance(content, str):
                content_preview = content[:100]
            elif isinstance(content, list):
                content_preview = f"[{len(content)} content blocks]"
            else:
                content_preview = str(content)[:100]
            logger.debug(f"  Message {i}: role={role}, content={content_preview}...")
        logger.debug(f"Tools available: {len(tools) if tools else 0}")
        logger.debug("======================")

        # Prepare inference parameters
        inference_params = {
            "model": self.model,
            "messages": messages,
            "tools": tools,
            "temperature": self.temperature,
            "max_tokens": self.max_tokens,
            "stream": False,
        }

        # Add seed if provided
        if self.seed is not None:
            inference_params["seed"] = self.seed

        # Make inference call
        response = await self.gateway.ainference(**inference_params)  # type: ignore[arg-type]

        # Extract response
        message = response["choices"][0]["message"]  # type: ignore[index]
        content = message.get("content", "")

        # Don't update history yet - wait until we have the complete interaction
        # This follows Claude's pattern of locking history until tool execution is complete

        # Check for tool calls
        tool_calls = None
        if message.get("tool_calls"):
            tool_calls = [
                ToolCall(
                    id=tc["id"],
                    name=tc["function"]["name"],
                    arguments=json.loads(tc["function"]["arguments"]),
                    status="pending",
                )
                for tc in message["tool_calls"]
            ]

            # Get the user message for history
            user_message = messages[-1]

            # Handle tool calls (blocking by default)
            final_content = await self._handle_tool_calls(tool_calls, messages, user_message)

            # Return response with tool information
            return AgentResponse(
                content=final_content,
                role="assistant",
                tool_calls=tool_calls,
                requires_follow_up=False,  # Already handled
                metadata={"model": self.model},
            )
        else:
            # No tools, add both user and assistant messages to history
            # Get the user message content from the built message
            user_msg = messages[-1]  # Last message in messages is the user message
            user_content = user_msg["content"]

            # Add to conversation history
            logger.info("=== Adding to history (no tools) ===")
            logger.info(f"  Adding user message: {str(user_content)[:100]}...")
            self.conversation.add_user_message(user_content)
            logger.info(f"  Adding assistant response: {content[:100]}...")
            self.conversation.add_assistant_message(content)
            logger.info(f"  History size now: {self.conversation.size()}")

            return AgentResponse(
                content=content,
                role="assistant",
                tool_calls=None,
                requires_follow_up=False,
                metadata={"model": self.model},
            )

    def _get_rag_context(self, query: str) -> str:
        """Get relevant context from memory."""
        if not self.memory:  # type: ignore[has-type]
            return ""

        try:
            results = self.memory.query(  # type: ignore[has-type]
                query_texts=query, n_results=self.rag_n, similarity_threshold=self.rag_threshold
            )

            if results:
                contexts = [doc.page_content for doc, _ in results]
                return " | ".join(contexts)
        except Exception as e:
            logger.warning(f"RAG query failed: {e}")

        return ""

    def _build_messages(
        self, agent_msg: AgentMessage, rag_context: str = ""
    ) -> list[dict[str, Any]]:
        """Build messages list from AgentMessage."""
        messages = []

        # System prompt with RAG context if available
        system_content = self.system_prompt
        if rag_context:
            system_content += f"\n\nRelevant context: {rag_context}"
        messages.append({"role": "system", "content": system_content})

        # Add conversation history in OpenAI format
        history_messages = self.conversation.to_openai_format()
        messages.extend(history_messages)

        # Debug history state
        logger.info(f"=== Building messages with {len(history_messages)} history messages ===")
        if history_messages:
            for i, msg in enumerate(history_messages):
                role = msg.get("role", "unknown")
                content = msg.get("content", "")
                if isinstance(content, str):
                    preview = content[:100]
                elif isinstance(content, list):
                    preview = f"[{len(content)} content blocks]"
                else:
                    preview = str(content)[:100]
                logger.info(f"  History[{i}]: role={role}, content={preview}")

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
            messages.append({"role": "user", "content": content})  # type: ignore[dict-item]
        else:
            # Text-only message
            messages.append({"role": "user", "content": user_content})

        return messages

    async def _handle_tool_calls(
        self,
        tool_calls: list[ToolCall],
        messages: list[dict[str, Any]],
        user_message: dict[str, Any],
    ) -> str:
        """Handle tool calls from LLM (blocking mode by default)."""
        try:
            # Build assistant message with tool calls
            assistant_msg = {
                "role": "assistant",
                "content": "",
                "tool_calls": [
                    {
                        "id": tc.id,
                        "type": "function",
                        "function": {"name": tc.name, "arguments": json.dumps(tc.arguments)},
                    }
                    for tc in tool_calls
                ],
            }
            messages.append(assistant_msg)

            # Execute tools and collect results
            tool_results = []
            for tool_call in tool_calls:
                logger.info(f"Executing tool: {tool_call.name}")

                try:
                    # Execute the tool
                    result = self.skills.call(tool_call.name, **tool_call.arguments)
                    tool_call.status = "completed"

                    # Format tool result message
                    tool_result = {
                        "role": "tool",
                        "tool_call_id": tool_call.id,
                        "content": str(result),
                        "name": tool_call.name,
                    }
                    tool_results.append(tool_result)

                except Exception as e:
                    logger.error(f"Tool execution failed: {e}")
                    tool_call.status = "failed"

                    # Add error result
                    tool_result = {
                        "role": "tool",
                        "tool_call_id": tool_call.id,
                        "content": f"Error: {e!s}",
                        "name": tool_call.name,
                    }
                    tool_results.append(tool_result)

            # Add tool results to messages
            messages.extend(tool_results)

            # Prepare follow-up inference parameters
            followup_params = {
                "model": self.model,
                "messages": messages,
                "temperature": self.temperature,
                "max_tokens": self.max_tokens,
            }

            # Add seed if provided
            if self.seed is not None:
                followup_params["seed"] = self.seed

            # Get follow-up response
            response = await self.gateway.ainference(**followup_params)  # type: ignore[arg-type]

            # Extract final response
            final_message = response["choices"][0]["message"]  # type: ignore[index]

            # Now add all messages to history in order (like Claude does)
            # Add user message
            user_content = user_message["content"]
            self.conversation.add_user_message(user_content)

            # Add assistant message with tool calls
            self.conversation.add_assistant_message("", tool_calls)

            # Add tool results
            for result in tool_results:
                self.conversation.add_tool_result(
                    tool_call_id=result["tool_call_id"], content=result["content"]
                )

            # Add final assistant response
            final_content = final_message.get("content", "")
            self.conversation.add_assistant_message(final_content)

            return final_message.get("content", "")  # type: ignore[no-any-return]

        except Exception as e:
            logger.error(f"Error handling tool calls: {e}")
            return f"Error executing tools: {e!s}"

    def query(self, message: str | AgentMessage) -> AgentResponse:
        """Synchronous query method for direct usage.

        Args:
            message: Either a string query or an AgentMessage with text and/or images

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
            return loop.run_until_complete(self._process_query_async(agent_msg))
        finally:
            loop.close()

    async def aquery(self, message: str | AgentMessage) -> AgentResponse:
        """Asynchronous query method.

        Args:
            message: Either a string query or an AgentMessage with text and/or images

        Returns:
            AgentResponse object with content and tool information
        """
        # Convert string to AgentMessage if needed
        if isinstance(message, str):
            agent_msg = AgentMessage()
            agent_msg.add_text(message)
        else:
            agent_msg = message

        return await self._process_query_async(agent_msg)

    def base_agent_dispose(self) -> None:
        """Dispose of all resources and close gateway."""
        self.response_subject.on_completed()
        if self._executor:
            self._executor.shutdown(wait=False)
        if self.gateway:
            self.gateway.close()
