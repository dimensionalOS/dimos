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

"""Unified agent module with full features following DimOS patterns."""

import asyncio
import base64
import io
import json
import logging
import threading
from typing import Any, Dict, List, Optional, Union

import numpy as np
from PIL import Image as PILImage
from reactivex import operators as ops
from reactivex.disposable import CompositeDisposable
from reactivex.subject import Subject

from dimos.core import Module, In, Out, rpc
from dimos.agents.memory.base import AbstractAgentSemanticMemory
from dimos.agents.memory.chroma_impl import OpenAISemanticMemory
from dimos.msgs.sensor_msgs import Image
from dimos.skills.skills import AbstractSkill, SkillLibrary
from dimos.utils.logging_config import setup_logger
from dimos.agents.modules.gateway import UnifiedGatewayClient

logger = setup_logger("dimos.agents.modules.unified_agent")


class UnifiedAgentModule(Module):
    """Unified agent module with full features.

    Features:
    - Multi-modal input (text, images, data streams)
    - Tool/skill execution
    - Semantic memory (RAG)
    - Conversation history
    - Multiple LLM provider support
    """

    # Module I/O
    query_in: In[str] = None
    image_in: In[Image] = None
    data_in: In[Dict[str, Any]] = None
    response_out: Out[str] = None

    def __init__(
        self,
        model: str = "openai::gpt-4o-mini",
        system_prompt: str = None,
        skills: Union[SkillLibrary, List[AbstractSkill], AbstractSkill] = None,
        memory: AbstractAgentSemanticMemory = None,
        temperature: float = 0.0,
        max_tokens: int = 4096,
        max_history: int = 20,
        rag_n: int = 4,
        rag_threshold: float = 0.45,
    ):
        """Initialize the unified agent.

        Args:
            model: Model identifier (e.g., "openai::gpt-4o", "anthropic::claude-3-haiku")
            system_prompt: System prompt for the agent
            skills: Skills/tools available to the agent
            memory: Semantic memory system for RAG
            temperature: Sampling temperature
            max_tokens: Maximum tokens to generate
            max_history: Maximum conversation history to keep
            rag_n: Number of RAG results to fetch
            rag_threshold: Minimum similarity for RAG results
        """
        super().__init__()

        self.model = model
        self.system_prompt = system_prompt or "You are a helpful AI assistant."
        self.temperature = temperature
        self.max_tokens = max_tokens
        self.max_history = max_history
        self.rag_n = rag_n
        self.rag_threshold = rag_threshold

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

        # Initialize memory
        self.memory = memory or OpenAISemanticMemory()

        # Gateway and state
        self.gateway = None
        self.history = []
        self.disposables = CompositeDisposable()
        self._processing = False
        self._lock = threading.Lock()

        # Latest image for multimodal
        self._latest_image = None
        self._image_lock = threading.Lock()

        # Latest data context
        self._latest_data = None
        self._data_lock = threading.Lock()

    @rpc
    def start(self):
        """Initialize and start the agent."""
        logger.info(f"Starting unified agent with model: {self.model}")

        # Initialize gateway
        self.gateway = UnifiedGatewayClient()

        # Subscribe to inputs - proper module pattern
        if self.query_in:
            self.disposables.add(self.query_in.subscribe(self._handle_query))

        if self.image_in:
            self.disposables.add(self.image_in.subscribe(self._handle_image))

        if self.data_in:
            self.disposables.add(self.data_in.subscribe(self._handle_data))

        # Add initial context to memory
        try:
            self._initialize_memory()
        except Exception as e:
            logger.warning(f"Failed to initialize memory: {e}")

        logger.info("Unified agent started")

    @rpc
    def stop(self):
        """Stop the agent."""
        logger.info("Stopping unified agent")
        self.disposables.dispose()
        if self.gateway:
            self.gateway.close()

    @rpc
    def clear_history(self):
        """Clear conversation history."""
        self.history = []
        logger.info("Conversation history cleared")

    @rpc
    def add_skill(self, skill: AbstractSkill):
        """Add a skill to the agent."""
        self.skills.add(skill)
        logger.info(f"Added skill: {skill.__class__.__name__}")

    @rpc
    def set_system_prompt(self, prompt: str):
        """Update system prompt."""
        self.system_prompt = prompt
        logger.info("System prompt updated")

    def _initialize_memory(self):
        """Add some initial context to memory."""
        try:
            contexts = [
                ("ctx1", "I am an AI assistant that can help with various tasks."),
                ("ctx2", "I can process images when provided through the image input."),
                ("ctx3", "I have access to tools and skills for specific operations."),
                ("ctx4", "I maintain conversation history for context."),
            ]
            for doc_id, text in contexts:
                self.memory.add_vector(doc_id, text)
        except Exception as e:
            logger.warning(f"Failed to initialize memory: {e}")

    def _handle_query(self, query: str):
        """Handle text query."""
        with self._lock:
            if self._processing:
                logger.warning("Already processing, skipping query")
                return
            self._processing = True

        # Process in thread
        thread = threading.Thread(target=self._run_async_query, args=(query,))
        thread.daemon = True
        thread.start()

    def _handle_image(self, image: Image):
        """Handle incoming image."""
        with self._image_lock:
            self._latest_image = image
        logger.debug("Received new image")

    def _handle_data(self, data: Dict[str, Any]):
        """Handle incoming data."""
        with self._data_lock:
            self._latest_data = data
        logger.debug(f"Received data: {list(data.keys())}")

    def _run_async_query(self, query: str):
        """Run async query in new event loop."""
        asyncio.run(self._process_query(query))

    async def _process_query(self, query: str):
        """Process the query."""
        try:
            logger.info(f"Processing query: {query}")

            # Get RAG context
            rag_context = self._get_rag_context(query)

            # Get latest image if available
            image_b64 = None
            with self._image_lock:
                if self._latest_image:
                    image_b64 = self._encode_image(self._latest_image)

            # Get latest data context
            data_context = None
            with self._data_lock:
                if self._latest_data:
                    data_context = self._format_data_context(self._latest_data)

            # Build messages
            messages = self._build_messages(query, rag_context, data_context, image_b64)

            # Get tools if available
            tools = self.skills.get_tools() if len(self.skills) > 0 else None

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

            # Update history
            self.history.append({"role": "user", "content": query})
            if image_b64:
                self.history.append({"role": "user", "content": "[Image provided]"})
            self.history.append(message)

            # Trim history
            if len(self.history) > self.max_history:
                self.history = self.history[-self.max_history :]

            # Handle tool calls
            if "tool_calls" in message and message["tool_calls"]:
                await self._handle_tool_calls(message["tool_calls"], messages)
            else:
                # Emit response
                content = message.get("content", "")
                if self.response_out:
                    self.response_out.publish(content)

        except Exception as e:
            logger.error(f"Error processing query: {e}")
            import traceback

            traceback.print_exc()
            if self.response_out:
                self.response_out.publish(f"Error: {str(e)}")
        finally:
            with self._lock:
                self._processing = False

    def _get_rag_context(self, query: str) -> str:
        """Get relevant context from memory."""
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

    def _encode_image(self, image: Image) -> str:
        """Encode image to base64."""
        try:
            # Convert to numpy array if needed
            if hasattr(image, "data"):
                img_array = image.data
            else:
                img_array = np.array(image)

            # Convert to PIL Image
            pil_image = PILImage.fromarray(img_array)

            # Convert to RGB if needed
            if pil_image.mode != "RGB":
                pil_image = pil_image.convert("RGB")

            # Encode to base64
            buffer = io.BytesIO()
            pil_image.save(buffer, format="JPEG")
            img_b64 = base64.b64encode(buffer.getvalue()).decode("utf-8")

            return img_b64

        except Exception as e:
            logger.error(f"Failed to encode image: {e}")
            return None

    def _format_data_context(self, data: Dict[str, Any]) -> str:
        """Format data context for inclusion in prompt."""
        try:
            # Simple JSON formatting for now
            return f"Current data context: {json.dumps(data, indent=2)}"
        except:
            return f"Current data context: {str(data)}"

    def _build_messages(
        self, query: str, rag_context: str, data_context: str, image_b64: str
    ) -> List[Dict[str, Any]]:
        """Build messages for LLM."""
        messages = []

        # System prompt
        system_content = self.system_prompt
        if rag_context:
            system_content += f"\n\nRelevant context: {rag_context}"
        messages.append({"role": "system", "content": system_content})

        # Add history
        messages.extend(self.history)

        # Current query
        user_content = query
        if data_context:
            user_content = f"{data_context}\n\n{user_content}"

        # Handle image for different providers
        if image_b64:
            if "anthropic" in self.model:
                # Anthropic format
                messages.append(
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": user_content},
                            {
                                "type": "image",
                                "source": {
                                    "type": "base64",
                                    "media_type": "image/jpeg",
                                    "data": image_b64,
                                },
                            },
                        ],
                    }
                )
            else:
                # OpenAI format
                messages.append(
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": user_content},
                            {
                                "type": "image_url",
                                "image_url": {
                                    "url": f"data:image/jpeg;base64,{image_b64}",
                                    "detail": "auto",
                                },
                            },
                        ],
                    }
                )
        else:
            messages.append({"role": "user", "content": user_content})

        return messages

    async def _handle_tool_calls(
        self, tool_calls: List[Dict[str, Any]], messages: List[Dict[str, Any]]
    ):
        """Handle tool calls from LLM."""
        try:
            # Execute tools
            tool_results = []
            for tool_call in tool_calls:
                tool_id = tool_call["id"]
                tool_name = tool_call["function"]["name"]
                tool_args = json.loads(tool_call["function"]["arguments"])

                logger.info(f"Executing tool: {tool_name}")

                try:
                    result = self.skills.call(tool_name, **tool_args)
                    tool_results.append(
                        {
                            "role": "tool",
                            "tool_call_id": tool_id,
                            "content": str(result),
                            "name": tool_name,
                        }
                    )
                except Exception as e:
                    logger.error(f"Tool execution failed: {e}")
                    tool_results.append(
                        {
                            "role": "tool",
                            "tool_call_id": tool_id,
                            "content": f"Error: {str(e)}",
                            "name": tool_name,
                        }
                    )

            # Add tool results
            messages.extend(tool_results)
            self.history.extend(tool_results)

            # Get follow-up response
            response = await self.gateway.ainference(
                model=self.model,
                messages=messages,
                temperature=self.temperature,
                max_tokens=self.max_tokens,
            )

            # Extract and emit
            message = response["choices"][0]["message"]
            content = message.get("content", "")

            self.history.append(message)

            if self.response_out:
                self.response_out.publish(content)

        except Exception as e:
            logger.error(f"Error handling tool calls: {e}")
            if self.response_out:
                self.response_out.publish(f"Error executing tools: {str(e)}")
