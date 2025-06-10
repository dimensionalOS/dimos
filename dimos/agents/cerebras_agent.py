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
import re
import time

from cerebras.cloud.sdk import Cerebras
from dotenv import load_dotenv
from pydantic import BaseModel
from reactivex import Observable
from reactivex.observer import Observer
from reactivex.scheduler import ThreadPoolScheduler

# Local imports
from dimos.agents.agent import LLMAgent
from dimos.agents.memory.base import AbstractAgentSemanticMemory
from dimos.agents.prompt_builder.impl import PromptBuilder
from dimos.agents.tokenizer.base import AbstractTokenizer
from dimos.skills.skills import AbstractSkill, SkillLibrary
from dimos.stream.frame_processor import FrameProcessor
from dimos.utils.logging_config import setup_logger
from dimos.agents.tokenizer.openai_tokenizer import OpenAITokenizer

# Initialize environment variables
load_dotenv()

# Initialize logger for the Cerebras agent
logger = setup_logger("dimos.agents.cerebras")


# Response object compatible with LLMAgent
class CerebrasResponseMessage(dict):
    def __init__(
        self,
        content="",
        tool_calls=None,
    ):
        self.content = content
        self.tool_calls = tool_calls or []
        self.parsed = None

        # Initialize as dict with the proper structure
        super().__init__(self.to_dict())

    def __str__(self):
        # Return a string representation for logging
        if self.content:
            return self.content
        elif self.tool_calls:
            # Return JSON representation of the first tool call
            if self.tool_calls:
                tool_call = self.tool_calls[0]
                tool_json = {
                    "name": tool_call.function.name,
                    "arguments": json.loads(tool_call.function.arguments),
                }
                return json.dumps(tool_json)
        return "[No content]"

    def to_dict(self):
        """Convert to dictionary format for JSON serialization."""
        result = {"role": "assistant", "content": self.content or ""}

        if self.tool_calls:
            result["tool_calls"] = []
            for tool_call in self.tool_calls:
                result["tool_calls"].append(
                    {
                        "id": tool_call.id,
                        "type": "function",
                        "function": {
                            "name": tool_call.function.name,
                            "arguments": tool_call.function.arguments,
                        },
                    }
                )

        return result


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
        tokenizer: Optional[AbstractTokenizer] = None,
        prompt_builder: Optional[PromptBuilder] = None,
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
            tokenizer (AbstractTokenizer): The tokenizer for the agent.
            prompt_builder (PromptBuilder): The prompt builder for the agent.
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
        self.max_tokens_per_request = max_input_tokens_per_request + max_output_tokens_per_request

        # Add static context to memory.
        self._add_context_to_memory()

        # Initialize tokenizer and prompt builder
        self.tokenizer = tokenizer or OpenAITokenizer(
            model_name="gpt-4o"
        )  # Use GPT-4 tokenizer for better accuracy
        self.prompt_builder = prompt_builder or PromptBuilder(
            model_name=self.model_name,
            max_tokens=self.max_input_tokens_per_request,
            tokenizer=self.tokenizer,
        )

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

        # Use new truncation function
        messages = self._truncate_messages(messages, override_token_limit)

        return messages

    def _truncate_messages(self, messages: list, override_token_limit: bool = False) -> list:
        """Truncate messages if total tokens exceed 16k using existing truncate_tokens method.

        Args:
            messages (list): List of message dictionaries
            override_token_limit (bool): Whether to skip truncation

        Returns:
            list: Messages with content truncated if needed
        """
        if override_token_limit:
            return messages

        total_tokens = 0
        for message in messages:
            if isinstance(message.get("content"), str):
                total_tokens += self.prompt_builder.tokenizer.token_count(message["content"])
            elif isinstance(message.get("content"), list):
                for item in message["content"]:
                    if item.get("type") == "text":
                        total_tokens += self.prompt_builder.tokenizer.token_count(item["text"])
                    elif item.get("type") == "image_url":
                        total_tokens += 85

        if total_tokens > 16000:
            excess_tokens = total_tokens - 16000
            current_tokens = total_tokens

            # Start from oldest messages and truncate until under 16k
            for i in range(len(messages)):
                if current_tokens <= 16000:
                    break

                msg = messages[i]
                if msg.get("role") == "system":
                    continue

                if isinstance(msg.get("content"), str):
                    original_tokens = self.prompt_builder.tokenizer.token_count(msg["content"])
                    # Calculate how much to truncate from this message
                    tokens_to_remove = min(excess_tokens, original_tokens // 3)
                    new_max_tokens = max(50, original_tokens - tokens_to_remove)

                    msg["content"] = self.prompt_builder.truncate_tokens(
                        msg["content"], new_max_tokens, "truncate_end"
                    )

                    new_tokens = self.prompt_builder.tokenizer.token_count(msg["content"])
                    tokens_saved = original_tokens - new_tokens
                    current_tokens -= tokens_saved
                    excess_tokens -= tokens_saved

            logger.info(
                f"Truncated older messages using truncate_tokens, final tokens: {current_tokens}"
            )
        else:
            logger.info(f"No truncation needed, total tokens: {total_tokens}")

        return messages

    def clean_cerebras_schema(self, schema: dict) -> dict:
        """Simple schema cleaner that removes unsupported fields for Cerebras API."""
        if not isinstance(schema, dict):
            return schema

        # Removing the problematic fields that pydantic generates
        cleaned = {}
        unsupported_fields = {
            "minItems",
            "maxItems",
            "uniqueItems",
            "exclusiveMinimum",
            "exclusiveMaximum",
            "minimum",
            "maximum",
        }

        for key, value in schema.items():
            if key in unsupported_fields:
                continue  # Skip unsupported fields
            elif isinstance(value, dict):
                cleaned[key] = self.clean_cerebras_schema(value)
            elif isinstance(value, list):
                cleaned[key] = [
                    self.clean_cerebras_schema(item) if isinstance(item, dict) else item
                    for item in value
                ]
            else:
                cleaned[key] = value

        return cleaned

    def create_tool_call(
        self, name: str = None, arguments: dict = None, call_id: str = None, content: str = None
    ):
        """Create a tool call object from either direct parameters or JSON content."""
        # If content is provided, parse it as JSON
        if content:
            logger.info(f"Creating tool call from content: {content}")
            try:
                content_json = json.loads(content)
                if (
                    isinstance(content_json, dict)
                    and "name" in content_json
                    and "arguments" in content_json
                ):
                    name = content_json["name"]
                    arguments = content_json["arguments"]
                else:
                    return None
            except json.JSONDecodeError:
                logger.warning("Content appears to be JSON but failed to parse")
                return None

        # Create the tool call object
        if name and arguments is not None:
            timestamp = int(time.time() * 1000000)  # microsecond precision
            tool_id = f"call_{timestamp}"

            logger.info(f"Creating tool call with timestamp ID: {tool_id}")
            return type(
                "ToolCall",
                (),
                {
                    "id": tool_id,
                    "function": type(
                        "Function", (), {"name": name, "arguments": json.dumps(arguments)}
                    ),
                },
            )

        return None

    def _send_query(self, messages: list) -> CerebrasResponseMessage:
        """Sends the query to Cerebras API using the official Cerebras SDK.

        Args:
            messages (list): The prompt messages to send.

        Returns:
            The response message from Cerebras wrapped in our CerebrasResponseMessage class.

        Raises:
            Exception: If no response message is returned from the API.
            ConnectionError: If there's an issue connecting to the API.
            ValueError: If the messages or other parameters are invalid.
        """
        try:
            # Prepare API call parameters
            api_params = {
                "model": self.model_name,
                "messages": messages,
                # "max_tokens": self.max_output_tokens_per_request,
            }

            # Add tools if available
            if self.skill_library and self.skill_library.get_tools():
                tools = self.skill_library.get_tools()
                for tool in tools:
                    if "function" in tool and "parameters" in tool["function"]:
                        tool["function"]["parameters"] = self.clean_cerebras_schema(
                            tool["function"]["parameters"]
                        )
                api_params["tools"] = tools
                api_params["tool_choice"] = "auto"

            if self.response_model is not None:
                api_params["response_format"] = {
                    "type": "json_object",
                    "schema": self.response_model,
                }

            # Make the API call
            response = self.client.chat.completions.create(**api_params)

            raw_message = response.choices[0].message
            if raw_message is None:
                logger.error("Response message does not exist.")
                raise Exception("Response message does not exist.")

            # Process response into final format
            content = raw_message.content
            tool_calls = getattr(raw_message, "tool_calls", None)

            # If no structured tool calls from API, try parsing content as JSON tool call
            if not tool_calls and content and content.strip().startswith("{"):
                parsed_tool_call = self.create_tool_call(content=content)
                if parsed_tool_call:
                    tool_calls = [parsed_tool_call]
                    content = None

            return CerebrasResponseMessage(content=content, tool_calls=tool_calls)

        except ConnectionError as ce:
            logger.error(f"Connection error with Cerebras API: {ce}")
            raise
        except ValueError as ve:
            logger.error(f"Invalid parameters for Cerebras API: {ve}")
            raise
        except Exception as e:
            # Print the raw API parameters when an error occurs
            logger.error(f"Raw API parameters: {json.dumps(api_params, indent=2)}")
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

            # Update query and get context
            self._update_query(incoming_query)
            _, condensed_results = self._get_rag_context()

            # Build prompt
            messages = self._build_prompt(
                messages, base64_image, dimensions, override_token_limit, condensed_results
            )

            while True:
                logger.info("Sending Query.")
                response_message = self._send_query(messages)
                logger.info(f"Received Response: {response_message}")

                if response_message is None:
                    raise Exception("Response message does not exist.")

                # If no skill library or no tool calls, we're done
                if (
                    self.skill_library is None
                    or self.skill_library.get_tools() is None
                    or response_message.tool_calls is None
                ):
                    final_msg = (
                        response_message.parsed
                        if hasattr(response_message, "parsed") and response_message.parsed
                        else (
                            response_message.content
                            if hasattr(response_message, "content")
                            else response_message
                        )
                    )
                    messages.append(response_message)
                    break

                logger.info(f"Assistant requested {len(response_message.tool_calls)} tool call(s)")
                next_response = self._handle_tooling(response_message, messages)

                if next_response is None:
                    final_msg = response_message.content or ""
                    break

                response_message = next_response

            with self._history_lock:
                self.conversation_history = messages
                logger.info(
                    f"Updated conversation history (total: {len(self.conversation_history)} messages)"
                )

            # Emit the final message content to the observer
            observer.on_next(final_msg)
            self.response_subject.on_next(final_msg)
            observer.on_completed()

        except Exception as e:
            logger.error(f"Query failed in {self.dev_name}: {e}")
            observer.on_error(e)
            self.response_subject.on_error(e)
