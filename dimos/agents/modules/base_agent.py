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

"""Simple base agent module following exact DimOS patterns."""

import asyncio
import json
import logging
import threading
from typing import Any, Dict, List, Optional

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

logger = setup_logger("dimos.agents.modules.base_agent")


class BaseAgentModule(Module):
    """Simple agent module that follows DimOS patterns exactly."""

    # Module I/O
    query_in: In[str] = None
    response_out: Out[str] = None

    def __init__(self, model: str = "openai::gpt-4o-mini", system_prompt: str = None):
        super().__init__()
        self.model = model
        self.system_prompt = system_prompt or "You are a helpful assistant."
        self.gateway = None
        self.history = []
        self.disposables = CompositeDisposable()
        self._processing = False
        self._lock = threading.Lock()

    @rpc
    def start(self):
        """Initialize and start the agent."""
        logger.info(f"Starting agent with model: {self.model}")

        # Initialize gateway
        self.gateway = UnifiedGatewayClient()

        # Subscribe to input
        if self.query_in:
            self.disposables.add(self.query_in.observable().subscribe(self._handle_query))

        logger.info("Agent started")

    @rpc
    def stop(self):
        """Stop the agent."""
        logger.info("Stopping agent")
        self.disposables.dispose()
        if self.gateway:
            self.gateway.close()

    def _handle_query(self, query: str):
        """Handle incoming query."""
        with self._lock:
            if self._processing:
                logger.warning("Already processing, skipping query")
                return
            self._processing = True

        # Process in a new thread with its own event loop
        thread = threading.Thread(target=self._run_async_query, args=(query,))
        thread.daemon = True
        thread.start()

    def _run_async_query(self, query: str):
        """Run async query in new event loop."""
        asyncio.run(self._process_query(query))

    async def _process_query(self, query: str):
        """Process the query."""
        try:
            logger.info(f"Processing query: {query}")

            # Build messages
            messages = []
            if self.system_prompt:
                messages.append({"role": "system", "content": self.system_prompt})
            messages.extend(self.history)
            messages.append({"role": "user", "content": query})

            # Call LLM
            response = await self.gateway.ainference(
                model=self.model, messages=messages, temperature=0.0, max_tokens=1000
            )

            # Extract content
            content = response["choices"][0]["message"]["content"]

            # Update history
            self.history.append({"role": "user", "content": query})
            self.history.append({"role": "assistant", "content": content})

            # Keep history reasonable
            if len(self.history) > 10:
                self.history = self.history[-10:]

            # Publish response
            if self.response_out:
                self.response_out.publish(content)

        except Exception as e:
            logger.error(f"Error processing query: {e}")
            if self.response_out:
                self.response_out.publish(f"Error: {str(e)}")
        finally:
            with self._lock:
                self._processing = False
