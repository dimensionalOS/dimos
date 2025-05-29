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

"""Cerebras agent implementation for the DIMOS agent framework.

This module provides a CerebrasAgent class that implements the LLMAgent interface
for Cerebras inference API using the official Cerebras Python SDK.
"""

from __future__ import annotations

import os
import threading
import copy
from typing import Any, Dict, List, Optional, Union, Tuple
import logging
import json

from cerebras.cloud.sdk import Cerebras
from dotenv import load_dotenv
from pydantic import BaseModel
from reactivex import Observable
from reactivex.observer import Observer
from reactivex.scheduler import ThreadPoolScheduler
from openai._types import NOT_GIVEN

# Local imports
from dimos.agents.agent import LLMAgent
from dimos.agents.memory.base import AbstractAgentSemanticMemory
from dimos.skills.skills import AbstractSkill, SkillLibrary
from dimos.stream.frame_processor import FrameProcessor
from dimos.utils.logging_config import setup_logger

# Initialize environment variables
load_dotenv()

# Initialize logger for the Cerebras agent
logger = setup_logger("dimos.agents.cerebras")


class CerebrasAgent(LLMAgent):
    """Cerebras agent implementation using the official Cerebras Python SDK.

    This class implements the _send_query method to interact with Cerebras API
    using their official SDK, allowing most of the LLMAgent logic to be reused.
    """

    def __init__(
        self,
        dev_name: str,
        agent_type: str = "Vision",
        query: str = "What do you see?",
        input_query_stream: Optional[Observable] = None,
        input_video_stream: Optional[Observable] = None,
        input_data_stream: Optional[Observable] = None,
        output_dir: str = os.path.join(os.getcwd(), "assets", "agent"),
        agent_memory: Optional[AbstractAgentSemanticMemory] = None,
        system_query: Optional[str] = None,
        max_input_tokens_per_request: int = 128000,
        max_output_tokens_per_request: int = 16384,
        model_name: str = "llama-4-scout-17b-16e-instruct",
        skills: Optional[Union[AbstractSkill, list[AbstractSkill], SkillLibrary]] = None,
        response_model: Optional[BaseModel] = None,
        frame_processor: Optional[FrameProcessor] = None,
        image_detail: str = "low",
        pool_scheduler: Optional[ThreadPoolScheduler] = None,
        process_all_inputs: Optional[bool] = None,
    ):
        """
        Initializes a new instance of the CerebrasAgent.

        Args:
            dev_name (str): The device name of the agent.
            agent_type (str): The type of the agent.
            query (str): The default query text.
            input_query_stream (Observable): An observable for query input.
            input_video_stream (Observable): An observable for video frames.
            input_data_stream (Observable): An observable for data input.
            output_dir (str): Directory for output files.
            agent_memory (AbstractAgentSemanticMemory): The memory system.
            system_query (str): The system prompt to use with RAG context.
            max_input_tokens_per_request (int): Maximum tokens for input.
            max_output_tokens_per_request (int): Maximum tokens for output.
            model_name (str): The Cerebras model name to use. Available options:
                - llama-4-scout-17b-16e-instruct (default, fastest)
                - llama3.1-8b
                - llama-3.3-70b
                - qwen-3-32b
                - deepseek-r1-distill-llama-70b (private preview)
            skills (Union[AbstractSkill, List[AbstractSkill], SkillLibrary]): Skills available to the agent.
            response_model (BaseModel): Optional Pydantic model for structured responses.
            frame_processor (FrameProcessor): Custom frame processor.
            image_detail (str): Detail level for images ("low", "high", "auto").
            pool_scheduler (ThreadPoolScheduler): The scheduler to use for thread pool operations.
            process_all_inputs (bool): Whether to process all inputs or skip when busy.
        """
        # Determine appropriate default for process_all_inputs if not provided
        if process_all_inputs is None:
            # Default to True for text queries, False for video streams
            if input_query_stream is not None and input_video_stream is None:
                process_all_inputs = True
            else:
                process_all_inputs = False

        super().__init__(
            dev_name=dev_name,
            agent_type=agent_type,
            agent_memory=agent_memory,
            pool_scheduler=pool_scheduler,
            process_all_inputs=process_all_inputs,
            system_query=system_query,
            input_query_stream=input_query_stream,
            input_video_stream=input_video_stream,
            input_data_stream=input_data_stream,
        )

        # Initialize Cerebras client
        self.client = Cerebras()

        self.query = query
        self.output_dir = output_dir
        os.makedirs(self.output_dir, exist_ok=True)

        # Initialize conversation history for multi-turn conversations
        self.conversation_history = []
        self._history_lock = threading.Lock()

        # Configure skills
        self.skills = skills
        self.skill_library = None
        if isinstance(self.skills, SkillLibrary):
            self.skill_library = self.skills
        elif isinstance(self.skills, list):
            self.skill_library = SkillLibrary()
            for skill in self.skills:
                self.skill_library.add(skill)
        elif isinstance(self.skills, AbstractSkill):
            self.skill_library = SkillLibrary()
            self.skill_library.add(self.skills)

        self.response_model = response_model
        self.model_name = model_name
        self.image_detail = image_detail
        self.max_output_tokens_per_request = max_output_tokens_per_request
        self.max_input_tokens_per_request = max_input_tokens_per_request

        # Add static context to memory.
        self._add_context_to_memory()

        logger.info("Cerebras Agent Initialized.")

    def _add_context_to_memory(self):
        """Adds initial context to the agent's memory."""
        context_data = [
            (
                "id0",
                "Optical Flow is a technique used to track the movement of objects in a video sequence.",
            ),
            (
                "id1",
                "Edge Detection is a technique used to identify the boundaries of objects in an image.",
            ),
            ("id2", "Video is a sequence of frames captured at regular intervals."),
            (
                "id3",
                "Colors in Optical Flow are determined by the movement of light, and can be used to track the movement of objects.",
            ),
            (
                "id4",
                "Json is a data interchange format that is easy for humans to read and write, and easy for machines to parse and generate.",
            ),
        ]
        for doc_id, text in context_data:
            self.agent_memory.add_vector(doc_id, text)

    def _build_prompt(
        self,
        messages: list,
        base64_image: Optional[Union[str, List[str]]] = None,
        dimensions: Optional[Tuple[int, int]] = None,
        override_token_limit: bool = False,
        condensed_results: str = "",
    ) -> list:
        """Builds a prompt message specifically for Cerebras API.

        Args:
            messages (list): Existing messages list to build upon.
            base64_image (Union[str, List[str]]): Optional Base64-encoded image(s).
            dimensions (Tuple[int, int]): Optional image dimensions.
            override_token_limit (bool): Whether to override token limits.
            condensed_results (str): The condensed RAG context.

        Returns:
            list: Messages formatted for Cerebras API.
        """

        # Add system message if provided and not already in history
        if self.system_query and (not messages or messages[0].get("role") != "system"):
            messages.insert(0, {"role": "system", "content": self.system_query})
            logger.info("Added system message to conversation")

        # Append user query while handling RAG
        if condensed_results:
            user_message = {"role": "user", "content": f"{condensed_results}\n\n{self.query}"}
            logger.info("Created user message with RAG context")
        else:
            user_message = {"role": "user", "content": self.query}

        messages.append(user_message)

        if base64_image is not None:
            # Handle both single image (str) and multiple images (List[str])
            images = [base64_image] if isinstance(base64_image, str) else base64_image

            # For Cerebras, we'll add images inline with text (OpenAI-style format)
            for img in images:
                img_content = [
                    {"type": "text", "text": "Here is an image to analyze:"},
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": f"data:image/jpeg;base64,{img}",
                            "detail": self.image_detail,
                        },
                    },
                ]
                messages.append({"role": "user", "content": img_content})

            logger.info(f"Added {len(images)} image(s) to conversation")

        return messages

    def _send_query(self, messages: list) -> Any:
        """Sends the query to Cerebras API using the official Cerebras SDK.

        Args:
            messages (list): The prompt messages to send.

        Returns:
            The response message from Cerebras.

        Raises:
            Exception: If no response message is returned.
            ConnectionError: If there's an issue connecting to the API.
            ValueError: If the messages or other parameters are invalid.
        """
        try:
            # Prepare API call parameters
            api_params = {
                "model": self.model_name,
                "messages": messages,
                "max_tokens": self.max_output_tokens_per_request,
            }

            # Add tools if available
            if self.skill_library and self.skill_library.get_tools():
                tools = self.skill_library.get_tools()
                api_params["tools"] = tools  # No conversion needed
                api_params["tool_choice"] = "auto"

            # Add response format for structured output if specified
            if self.response_model is not None:
                # Convert Pydantic model to JSON schema for Cerebras
                from pydantic import TypeAdapter

                schema = TypeAdapter(self.response_model).json_schema()

                # Ensure additionalProperties is set to False for strict mode
                if "additionalProperties" not in schema:
                    schema["additionalProperties"] = False

                api_params["response_format"] = {
                    "type": "json_schema",
                    "json_schema": {
                        "name": self.response_model.__name__
                        if hasattr(self.response_model, "__name__")
                        else "response",
                        "strict": True,
                        "schema": schema,
                    },
                }

            # Make the API call
            response = self.client.chat.completions.create(**api_params)

            response_message = response.choices[0].message
            if response_message is None:
                logger.error("Response message does not exist.")
                raise Exception("Response message does not exist.")

            return response_message

        except ConnectionError as ce:
            logger.error(f"Connection error with Cerebras API: {ce}")
            raise
        except ValueError as ve:
            logger.error(f"Invalid parameters for Cerebras API: {ve}")
            raise
        except Exception as e:
            logger.error(f"Unexpected error in Cerebras API call: {e}")
            raise

    def _observable_query(
        self,
        observer: Observer,
        base64_image: Optional[str] = None,
        dimensions: Optional[Tuple[int, int]] = None,
        override_token_limit: bool = False,
        incoming_query: Optional[str] = None,
        reset_conversation: bool = False,
    ):
        """Main query handler that manages conversation history and Cerebras interactions.

        This method follows ClaudeAgent's pattern for efficient conversation history management.

        Args:
            observer (Observer): The observer to emit responses to.
            base64_image (str): Optional Base64-encoded image.
            dimensions (Tuple[int, int]): Optional image dimensions.
            override_token_limit (bool): Whether to override token limits.
            incoming_query (str): Optional query to update the agent's query.
            reset_conversation (bool): Whether to reset the conversation history.
        """
        try:
            # Reset conversation history if requested
            if reset_conversation:
                self.conversation_history = []
                logger.info("Conversation history reset")

            # Create a local copy of conversation history and record its length
            messages = copy.deepcopy(self.conversation_history)
            base_len = len(messages)

            # Update query and get context
            self._update_query(incoming_query)
            _, condensed_results = self._get_rag_context()

            # Build prompt
            messages = self._build_prompt(
                messages, base64_image, dimensions, override_token_limit, condensed_results
            )

            # Send query and get response
            logger.info("Sending Query.")
            response_message = self._send_query(messages)
            logger.info(f"Received Response: {response_message}")

            if response_message is None:
                logger.error("Received None response from Cerebras API")
                observer.on_next("")
                observer.on_completed()
                return

            # Add assistant response to local messages (always)
            assistant_message = {"role": "assistant"}

            if response_message.content:
                assistant_message["content"] = response_message.content
            else:
                assistant_message["content"] = ""  # Ensure content is never None

            if hasattr(response_message, "tool_calls") and response_message.tool_calls:
                assistant_message["tool_calls"] = []
                for tool_call in response_message.tool_calls:
                    assistant_message["tool_calls"].append(
                        {
                            "id": tool_call.id,
                            "type": "function",
                            "function": {
                                "name": tool_call.function.name,
                                "arguments": tool_call.function.arguments,
                            },
                        }
                    )
                logger.info(
                    f"Assistant response includes {len(response_message.tool_calls)} tool call(s)"
                )

            messages.append(assistant_message)

            # Handle tool calls if present (add tool messages to conversation)
            self._handle_tooling(response_message, messages)

            # At the end, append only new messages to the global conversation history under a lock
            if not hasattr(self, "_history_lock"):
                self._history_lock = threading.Lock()
            with self._history_lock:
                for msg in messages[base_len:]:
                    self.conversation_history.append(msg)
                logger.info(
                    f"Updated conversation history (total: {len(self.conversation_history)} messages)"
                )

            # Send response to observers
            result = response_message.content or ""
            observer.on_next(result)
            self.response_subject.on_next(result)
            observer.on_completed()

        except Exception as e:
            logger.error(f"Query failed in {self.dev_name}: {e}")
            observer.on_error(e)
            self.response_subject.on_error(e)

    def _handle_tooling(self, response_message, messages):
        """Executes tools and appends tool-use/result blocks to messages."""
        if not hasattr(response_message, "tool_calls") or not response_message.tool_calls:
            logger.info("No tool calls found in response message")
            return None

        if len(response_message.tool_calls) > 1:
            logger.warning(
                "Multiple tool calls detected in response message. Not a tested feature."
            )

        # Execute all tools and add their results to messages
        for tool_call in response_message.tool_calls:
            logger.info(f"Processing tool call: {tool_call.function.name}")

            # Execute the tool
            args = json.loads(tool_call.function.arguments)
            tool_result = self.skill_library.call(tool_call.function.name, **args)
            logger.info(f"Function Call Results: {tool_result}")

            # Add tool result to conversation history (OpenAI format)
            messages.append(
                {
                    "role": "tool",
                    "tool_call_id": tool_call.id,
                    "content": str(tool_result),
                    "name": tool_call.function.name,
                }
            )
