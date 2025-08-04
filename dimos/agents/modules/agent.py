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

"""Base agent module following DimOS patterns."""

from __future__ import annotations

import asyncio
import json
import logging
import threading
from typing import TYPE_CHECKING, Any, Dict, List, Optional, Tuple, Union

import reactivex as rx
from reactivex import operators as ops
from reactivex.disposable import CompositeDisposable
from reactivex.subject import Subject

from dimos.core import Module, In, Out, rpc
from dimos.agents.memory.base import AbstractAgentSemanticMemory
from dimos.agents.memory.chroma_impl import OpenAISemanticMemory
from dimos.msgs.sensor_msgs import Image
from dimos.skills.skills import AbstractSkill, SkillLibrary
from dimos.utils.logging_config import setup_logger

try:
    from .gateway import UnifiedGatewayClient
except ImportError:
    # Absolute import for when module is executed remotely
    from dimos.agents.modules.gateway import UnifiedGatewayClient

logger = setup_logger("dimos.agents.modules.agent")


class AgentModule(Module):
    """Base agent module following DimOS patterns.

    This module provides a clean interface for LLM agents that can:
    - Process text queries via query_in
    - Process video frames via video_in
    - Process data streams via data_in
    - Emit responses via response_out
    - Execute skills/tools
    - Maintain conversation history
    - Integrate with semantic memory
    """

    # Module I/O - These are type annotations that will be processed by Module.__init__
    query_in: In[str] = None
    video_in: In[Image] = None
    data_in: In[Dict[str, Any]] = None
    response_out: Out[str] = None

    # Add to class namespace for type hint resolution
    __annotations__["In"] = In
    __annotations__["Out"] = Out
    __annotations__["Image"] = Image
    __annotations__["Dict"] = Dict
    __annotations__["Any"] = Any

    def __init__(
        self,
        model: str,
        skills: Optional[Union[SkillLibrary, List[AbstractSkill], AbstractSkill]] = None,
        memory: Optional[AbstractAgentSemanticMemory] = None,
        system_prompt: Optional[str] = None,
        max_tokens: int = 4096,
        temperature: float = 0.0,
        **kwargs,
    ):
        """Initialize the agent module.

        Args:
            model: Model identifier (e.g., "openai::gpt-4o", "anthropic::claude-3-haiku")
            skills: Skills/tools available to the agent
            memory: Semantic memory system for RAG
            system_prompt: System prompt for the agent
            max_tokens: Maximum tokens to generate
            temperature: Sampling temperature
            **kwargs: Additional parameters passed to Module
        """
        Module.__init__(self, **kwargs)

        self._model = model
        self._system_prompt = system_prompt
        self._max_tokens = max_tokens
        self._temperature = temperature

        # Initialize skills
        if skills is None:
            self._skills = SkillLibrary()
        elif isinstance(skills, SkillLibrary):
            self._skills = skills
        elif isinstance(skills, list):
            self._skills = SkillLibrary()
            for skill in skills:
                self._skills.add(skill)
        elif isinstance(skills, AbstractSkill):
            self._skills = SkillLibrary()
            self._skills.add(skills)
        else:
            self._skills = SkillLibrary()

        # Initialize memory
        self._memory = memory or OpenAISemanticMemory()

        # Gateway will be initialized on start
        self._gateway = None

        # Conversation history
        self._conversation_history = []
        self._history_lock = threading.Lock()

        # Disposables for subscriptions
        self._disposables = CompositeDisposable()

        # Internal subjects for processing
        self._query_subject = Subject()
        self._response_subject = Subject()

        # Processing state
        self._processing = False
        self._processing_lock = threading.Lock()

    @rpc
    def start(self):
        """Initialize gateway and connect streams."""
        logger.info(f"Starting agent module with model: {self._model}")

        # Initialize gateway
        self._gateway = UnifiedGatewayClient()

        # Connect inputs to processing
        if self.query_in:
            self._disposables.add(self.query_in.observable().subscribe(self._handle_query))

        if self.video_in:
            self._disposables.add(self.video_in.observable().subscribe(self._handle_video))

        if self.data_in:
            self._disposables.add(self.data_in.observable().subscribe(self._handle_data))

        # Connect response subject to output
        if self.response_out:
            self._disposables.add(self._response_subject.subscribe(self.response_out.publish))

        logger.info("Agent module started successfully")

    @rpc
    def stop(self):
        """Stop the agent and clean up resources."""
        logger.info("Stopping agent module")
        self._disposables.dispose()
        if self._gateway:
            self._gateway.close()

    @rpc
    def set_system_prompt(self, prompt: str) -> None:
        """Update the system prompt."""
        self._system_prompt = prompt
        logger.info("System prompt updated")

    @rpc
    def add_skill(self, skill: AbstractSkill) -> None:
        """Add a skill to the agent."""
        self._skills.add(skill)
        logger.info(f"Added skill: {skill.__class__.__name__}")

    @rpc
    def clear_history(self) -> None:
        """Clear conversation history."""
        with self._history_lock:
            self._conversation_history = []
        logger.info("Conversation history cleared")

    @rpc
    def get_conversation_history(self) -> List[Dict[str, Any]]:
        """Get the current conversation history."""
        with self._history_lock:
            return self._conversation_history.copy()

    def _handle_query(self, query: str):
        """Handle incoming text query."""
        logger.debug(f"Received query: {query}")

        # Skip if already processing
        with self._processing_lock:
            if self._processing:
                logger.warning("Skipping query - already processing")
                return
            self._processing = True

        try:
            # Process the query
            asyncio.create_task(self._process_query(query))
        except Exception as e:
            logger.error(f"Error handling query: {e}")
            with self._processing_lock:
                self._processing = False

    def _handle_video(self, frame: Image):
        """Handle incoming video frame."""
        logger.debug("Received video frame")

        # Convert to base64 for multimodal processing
        # This is a placeholder - implement actual image encoding
        # For now, just log
        logger.info("Video processing not yet implemented")

    def _handle_data(self, data: Dict[str, Any]):
        """Handle incoming data stream."""
        logger.debug(f"Received data: {data}")

        # Extract query if present
        if "query" in data:
            self._handle_query(data["query"])
        else:
            # Process as context data
            logger.info("Data stream processing not yet implemented")

    async def _process_query(self, query: str):
        """Process a query through the LLM."""
        try:
            # Get RAG context if available
            rag_context = self._get_rag_context(query)

            # Build messages
            messages = self._build_messages(query, rag_context)

            # Get tools if available
            tools = self._skills.get_tools() if len(self._skills) > 0 else None

            # Make inference call
            response = await self._gateway.ainference(
                model=self._model,
                messages=messages,
                tools=tools,
                temperature=self._temperature,
                max_tokens=self._max_tokens,
                stream=False,  # For now, not streaming
            )

            # Extract response
            message = response["choices"][0]["message"]

            # Update conversation history
            with self._history_lock:
                self._conversation_history.append({"role": "user", "content": query})
                self._conversation_history.append(message)

            # Handle tool calls if present
            if "tool_calls" in message and message["tool_calls"]:
                await self._handle_tool_calls(message["tool_calls"], messages)
            else:
                # Emit response
                content = message.get("content", "")
                self._response_subject.on_next(content)

        except Exception as e:
            logger.error(f"Error processing query: {e}")
            self._response_subject.on_next(f"Error: {str(e)}")
        finally:
            with self._processing_lock:
                self._processing = False

    def _get_rag_context(self, query: str) -> str:
        """Get relevant context from memory."""
        try:
            results = self._memory.query(query_texts=query, n_results=4, similarity_threshold=0.45)

            if results:
                context_parts = []
                for doc, score in results:
                    context_parts.append(doc.page_content)
                return " | ".join(context_parts)
        except Exception as e:
            logger.warning(f"Error getting RAG context: {e}")

        return ""

    def _build_messages(self, query: str, rag_context: str) -> List[Dict[str, Any]]:
        """Build messages for the LLM."""
        messages = []

        # Add conversation history
        with self._history_lock:
            messages.extend(self._conversation_history)

        # Add system prompt if not already present
        if self._system_prompt and (not messages or messages[0]["role"] != "system"):
            messages.insert(0, {"role": "system", "content": self._system_prompt})

        # Add current query with RAG context
        if rag_context:
            content = f"{rag_context}\n\nUser query: {query}"
        else:
            content = query

        messages.append({"role": "user", "content": content})

        return messages

    async def _handle_tool_calls(
        self, tool_calls: List[Dict[str, Any]], messages: List[Dict[str, Any]]
    ):
        """Handle tool calls from the LLM."""
        try:
            # Execute each tool
            tool_results = []
            for tool_call in tool_calls:
                tool_id = tool_call["id"]
                tool_name = tool_call["function"]["name"]
                tool_args = json.loads(tool_call["function"]["arguments"])

                logger.info(f"Executing tool: {tool_name} with args: {tool_args}")

                try:
                    result = self._skills.call(tool_name, **tool_args)
                    tool_results.append(
                        {
                            "role": "tool",
                            "tool_call_id": tool_id,
                            "content": str(result),
                            "name": tool_name,
                        }
                    )
                except Exception as e:
                    logger.error(f"Error executing tool {tool_name}: {e}")
                    tool_results.append(
                        {
                            "role": "tool",
                            "tool_call_id": tool_id,
                            "content": f"Error: {str(e)}",
                            "name": tool_name,
                        }
                    )

            # Add tool results to messages
            messages.extend(tool_results)

            # Get follow-up response
            response = await self._gateway.ainference(
                model=self._model,
                messages=messages,
                temperature=self._temperature,
                max_tokens=self._max_tokens,
                stream=False,
            )

            # Extract and emit response
            message = response["choices"][0]["message"]
            content = message.get("content", "")

            # Update history with tool results and response
            with self._history_lock:
                self._conversation_history.extend(tool_results)
                self._conversation_history.append(message)

            self._response_subject.on_next(content)

        except Exception as e:
            logger.error(f"Error handling tool calls: {e}")
            self._response_subject.on_next(f"Error executing tools: {str(e)}")

    def __del__(self):
        """Cleanup on deletion."""
        try:
            self.stop()
        except:
            pass
