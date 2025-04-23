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

"""Claude agent implementation for the DIMOS agent framework.

This module provides a ClaudeAgent class that implements the LLMAgent interface
for Anthropic's Claude models. It handles conversion between the DIMOS skill format
and Claude's tools format.
"""

from __future__ import annotations

import json
import os
from typing import Any, Dict, List, Optional, Tuple, Union, cast
import logging

import anthropic
from anthropic.types import ContentBlock, MessageParam, ToolUseBlock
from dotenv import load_dotenv
from httpx._transports import base
from pydantic import BaseModel
from reactivex import Observable
from reactivex.disposable import Disposable
from reactivex.scheduler import ThreadPoolScheduler
from reactivex import create

# Local imports
from dimos.agents.agent import LLMAgent
from dimos.agents.memory.base import AbstractAgentSemanticMemory
from dimos.agents.prompt_builder.impl import PromptBuilder
from dimos.skills.skills import AbstractSkill, SkillLibrary
from dimos.stream.frame_processor import FrameProcessor
from dimos.utils.logging_config import setup_logger
from dimos.utils.threadpool import get_scheduler

# Initialize environment variables
load_dotenv()

# Initialize logger for the Claude agent
logger = setup_logger("dimos.agents.claude", level=logging.DEBUG)

# Response object compatible with LLMAgent
class ResponseMessage:
    def __init__(self, content="", tool_calls=None, thinking_blocks=None):
        self.content = content
        self.tool_calls = tool_calls or []
        self.thinking_blocks = thinking_blocks or []
        self.parsed = None
    
    def __str__(self):
        # Return a string representation for logging
        parts = []
        
        # Include content if available
        if self.content:
            parts.append(self.content)
        
        # Include tool calls if available
        if self.tool_calls:
            tool_names = [tc.function.name for tc in self.tool_calls]
            parts.append(f"[Tools called: {', '.join(tool_names)}]")
        
        return "\n".join(parts) if parts else "[No content]"

class ClaudeAgent(LLMAgent):
    """Claude agent implementation that uses Anthropic's API for processing.

    This class implements the _send_query method to interact with Anthropic's API
    and overrides _build_prompt to create Claude-formatted messages directly.
    """

    def __init__(self,
                 dev_name: str,
                 agent_type: str = "Vision",
                 query: str = "What do you see?",
                 input_query_stream: Optional[Observable] = None,
                 input_video_stream: Optional[Observable] = None,
                 output_dir: str = os.path.join(os.getcwd(), "assets", "agent"),
                 agent_memory: Optional[AbstractAgentSemanticMemory] = None,
                 system_query: Optional[str] = None,
                 max_input_tokens_per_request: int = 128000,
                 max_output_tokens_per_request: int = 16384,
                 model_name: str = "claude-3-7-sonnet-20250219",
                 prompt_builder: Optional[PromptBuilder] = None,
                 rag_query_n: int = 4,
                 rag_similarity_threshold: float = 0.45,
                 skills: Optional[AbstractSkill] = None,
                 response_model: Optional[BaseModel] = None,
                 frame_processor: Optional[FrameProcessor] = None,
                 image_detail: str = "low",
                 pool_scheduler: Optional[ThreadPoolScheduler] = None,
                 process_all_inputs: Optional[bool] = None,
                 thinking_budget_tokens: Optional[int] = 2000):
        """
        Initializes a new instance of the ClaudeAgent.

        Args:
            dev_name (str): The device name of the agent.
            agent_type (str): The type of the agent.
            query (str): The default query text.
            input_query_stream (Observable): An observable for query input.
            input_video_stream (Observable): An observable for video frames.
            output_dir (str): Directory for output files.
            agent_memory (AbstractAgentSemanticMemory): The memory system.
            system_query (str): The system prompt to use with RAG context.
            max_input_tokens_per_request (int): Maximum tokens for input.
            max_output_tokens_per_request (int): Maximum tokens for output.
            model_name (str): The Claude model name to use.
            prompt_builder (PromptBuilder): Custom prompt builder (not used in Claude implementation).
            rag_query_n (int): Number of results to fetch in RAG queries.
            rag_similarity_threshold (float): Minimum similarity for RAG results.
            skills (AbstractSkill): Skills available to the agent.
            response_model (BaseModel): Optional Pydantic model for responses.
            frame_processor (FrameProcessor): Custom frame processor.
            image_detail (str): Detail level for images ("low", "high", "auto").
            pool_scheduler (ThreadPoolScheduler): The scheduler to use for thread pool operations.
            process_all_inputs (bool): Whether to process all inputs or skip when busy.
            thinking_budget_tokens (int): Number of tokens to allocate for Claude's thinking. 0 disables thinking.
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
            system_query=system_query
        )
        
        self.client = anthropic.Anthropic()
        self.query = query
        self.output_dir = output_dir
        os.makedirs(self.output_dir, exist_ok=True)

        # Claude-specific parameters
        self.thinking_budget_tokens = thinking_budget_tokens
        self.claude_api_params = {}  # Will store params for Claude API calls
        
        # Configure skills
        self.skills = skills
        self.skill_library = None # Required for error 'ClaudeAgent' object has no attribute 'skill_library' due to skills refactor
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
        self.rag_query_n = rag_query_n
        self.rag_similarity_threshold = rag_similarity_threshold
        self.image_detail = image_detail
        self.max_output_tokens_per_request = max_output_tokens_per_request
        self.max_input_tokens_per_request = max_input_tokens_per_request
        self.max_tokens_per_request = max_input_tokens_per_request + max_output_tokens_per_request

        # Add static context to memory.
        self._add_context_to_memory()

        self.frame_processor = frame_processor or FrameProcessor(delete_on_init=True)
        self.input_video_stream = input_video_stream
        self.input_query_stream = input_query_stream

        # Ensure only one input stream is provided.
        if self.input_video_stream is not None and self.input_query_stream is not None:
            raise ValueError(
                "More than one input stream provided. Please provide only one input stream."
            )

        if self.input_video_stream is not None:
            logger.info("Subscribing to input video stream...")
            self.disposables.add(
                self.subscribe_to_image_processing(self.input_video_stream))
        if self.input_query_stream is not None:
            logger.info("Subscribing to input query stream...")
            self.disposables.add(
                self.subscribe_to_query_processing(self.input_query_stream))

        logger.info("Claude Agent Initialized.")

    def _add_context_to_memory(self):
        """Adds initial context to the agent's memory."""
        context_data = [
            ("id0",
             "Optical Flow is a technique used to track the movement of objects in a video sequence."
             ),
            ("id1",
             "Edge Detection is a technique used to identify the boundaries of objects in an image."
             ),
            ("id2",
             "Video is a sequence of frames captured at regular intervals."),
            ("id3",
             "Colors in Optical Flow are determined by the movement of light, and can be used to track the movement of objects."
             ),
            ("id4",
             "Json is a data interchange format that is easy for humans to read and write, and easy for machines to parse and generate."
             ),
        ]
        for doc_id, text in context_data:
            self.agent_memory.add_vector(doc_id, text)

    def _convert_tools_to_claude_format(self, tools: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Converts DIMOS tools to Claude format.
        
        Args:
            tools: List of tools in DIMOS format.
            
        Returns:
            List of tools in Claude format.
        """
        if not tools:
            return []
            
        claude_tools = []
        
        for tool in tools:
            # Skip if not a function
            if tool.get('type') != 'function':
                continue
                
            function = tool.get('function', {})
            name = function.get('name')
            description = function.get('description', '')
            parameters = function.get('parameters', {})
            
            claude_tool = {
                "name": name,
                "description": description,
                "input_schema": {
                    "type": "object",
                    "properties": parameters.get('properties', {}),
                    "required": parameters.get('required', []),
                }
            }
            
            claude_tools.append(claude_tool)
            
        return claude_tools

    def _build_prompt(self, messages: list, base64_image: Optional[Union[str, List[str]]] = None,
                      dimensions: Optional[Tuple[int, int]] = None,
                      override_token_limit: bool = False,
                      rag_results: str = "",
                      thinking_budget_tokens: int = None) -> list:
        """Builds a prompt message specifically for Claude API, using local messages copy."""
        """Builds a prompt message specifically for Claude API.

        This method creates messages in Claude's format directly, without using
        any OpenAI-specific formatting or token counting.

        Args:
            base64_image (Union[str, List[str]]): Optional Base64-encoded image(s).
            dimensions (Tuple[int, int]): Optional image dimensions.
            override_token_limit (bool): Whether to override token limits.
            rag_results (str): The condensed RAG context.
            thinking_budget_tokens (int): Number of tokens to allocate for Claude's thinking.

        Returns:
            dict: A dict containing Claude API parameters.
        """
        
        # Append user query to conversation history while handling RAG
        if rag_results:
            messages.append({"role": "user", "content": f"{rag_results}\n\n{self.query}"})
            logger.info(f"Added new user message to conversation history with RAG context (now has {len(messages)} messages)")
        else:   
            messages.append({"role": "user", "content": self.query})
            logger.info(f"Added new user message to conversation history (now has {len(messages)} messages)")
        
        if base64_image is not None:
            # Handle both single image (str) and multiple images (List[str])
            images = [base64_image] if isinstance(base64_image, str) else base64_image
            
            # Add each image as a separate entry in conversation history
            for img in images:
                img_content = [
                    {
                        "type": "image",
                        "source": {
                            "type": "base64",
                            "media_type": "image/jpeg",
                            "data": img
                        }
                    }
                ]
                messages.append({"role": "user", "content": img_content})
            
            if images:
                logger.info(f"Added {len(images)} image(s) as separate entries to conversation history")
    
        # Create Claude parameters with basic settings
        claude_params = {
            "model": self.model_name,
            "max_tokens": self.max_output_tokens_per_request,
            "temperature": 0,  # Add temperature to make responses more deterministic
            "messages": messages
        }
        
        # Add system prompt as a top-level parameter (not as a message)
        if self.system_query:
            claude_params['system'] = self.system_query
        
        # Store the parameters for use in _send_query
        self.claude_api_params = claude_params.copy()
            
        # Add tools if skills are available
        if self.skills and self.skills.get_tools():
            tools = self._convert_tools_to_claude_format(self.skills.get_tools())
            if tools:  # Only add if we have valid tools
                claude_params["tools"] = tools
                # Enable tool calling with proper format
                claude_params["tool_choice"] = {
                    "type": "auto"
                }
            
        # Add thinking if enabled and hard code required temperature = 1
        if thinking_budget_tokens is not None and thinking_budget_tokens != 0:
            claude_params["thinking"] = {
                "type": "enabled",
                "budget_tokens": thinking_budget_tokens
            }
            claude_params["temperature"] = 1  # Required to be 1 when thinking is enabled # Default to 0 for deterministic responses
            
        # Store the parameters for use in _send_query and return them
        self.claude_api_params = claude_params.copy()
        return messages, claude_params
        
    def _send_query(self, messages: list, claude_params: dict) -> Any:
        """Sends the query to Anthropic's API using streaming for better thinking visualization.
        
        Args:
            messages: Dict with 'claude_prompt' key containing Claude API parameters.
        
        Returns:
            The response message in a format compatible with LLMAgent's expectations.
        """
        try:
            # Get Claude parameters
            print("\n\n==== CLAUDE API PARAMETERS ====")
            # print(json.dumps(messages, indent=2, default=str))
            claude_params = (claude_params.get('claude_prompt', None) or self.claude_api_params)
            
            # Log request parameters with truncated base64 data
            print("\n\n==== CLAUDE API REQUEST ====\n")
            logger.debug(self._debug_api_call(claude_params))
            
            print("==== END REQUEST ====\n")
            
            # Initialize response containers
            text_content = ""
            tool_calls = []
            thinking_blocks = []
            
            # Log the start of streaming and the query
            logger.info(f"Sending streaming request to Claude API")
            
            # Log the query to memory.txt
            with open(os.path.join(self.output_dir, "memory.txt"), "a") as f:
                f.write(f"\n\nQUERY: {self.query}\n\n")
                f.flush()
            
            # Stream the response
            with self.client.messages.stream(**claude_params) as stream:
                print("\n==== CLAUDE API RESPONSE STREAM STARTED ====")
                
                # Open the memory file once for the entire stream processing
                with open(os.path.join(self.output_dir, "memory.txt"), "a") as memory_file:
                    # Track the current block being processed
                    current_block = {'type': None, 'id': None, 'content': "", 'signature': None}
                    
                    for event in stream:
                        # Log each event to console
                        # print(f"EVENT: {event.type}")  
                        # print(json.dumps(event.model_dump(), indent=2, default=str))
                        
                        if event.type == "content_block_start":
                            # Initialize a new content block
                            block_type = event.content_block.type
                            current_block = {'type': block_type, 'id': event.index, 'content': "", 'signature': None}
                            logger.debug(f"Starting {block_type} block...")
                        
                        elif event.type == "content_block_delta":
                            if event.delta.type == "thinking_delta":
                                # Accumulate thinking content
                                current_block['content'] = event.delta.thinking
                                memory_file.write(f"{event.delta.thinking}")
                                memory_file.flush()  # Ensure content is written immediately
                            
                            elif event.delta.type == "text_delta":
                                # Accumulate text content
                                text_content += event.delta.text
                                current_block['content'] += event.delta.text
                                memory_file.write(f"{event.delta.text}")
                                memory_file.flush()
                            
                            elif event.delta.type == "signature_delta":
                                # Store signature for thinking blocks
                                current_block['signature'] = event.delta.signature
                                memory_file.write(f"\n[Signature received for block {current_block['id']}]\n")
                                memory_file.flush()
                        
                        elif event.type == "content_block_stop":
                            # Store completed blocks
                            if current_block['type'] == "thinking":
                                # IMPORTANT: Store the complete event.content_block to ensure we preserve
                                # the exact format that Claude expects in subsequent requests
                                if hasattr(event, 'content_block'):
                                    # Use the exact thinking block as provided by Claude
                                    thinking_blocks.append(event.content_block.model_dump())
                                    memory_file.write(f"\nTHINKING COMPLETE: block {current_block['id']}\n")
                                else:
                                    # Fallback to constructed thinking block if content_block missing
                                    thinking_block = {
                                        "type": "thinking",
                                        "thinking": current_block['content'],
                                        "signature": current_block['signature']
                                    }
                                    thinking_blocks.append(thinking_block)
                                memory_file.write(f"\nTHINKING COMPLETE: block {current_block['id']}\n")
                            
                            elif current_block['type'] == "redacted_thinking":
                                # Handle redacted thinking blocks
                                if hasattr(event, 'content_block') and hasattr(event.content_block, 'data'):
                                    redacted_block = {
                                        "type": "redacted_thinking",
                                        "data": event.content_block.data
                                    }
                                    thinking_blocks.append(redacted_block)
                            
                            elif current_block['type'] == "tool_use":
                                # Process tool use blocks when they're complete
                                if hasattr(event, 'content_block'):
                                    tool_block = event.content_block
                                    tool_id = tool_block.id
                                    tool_name = tool_block.name
                                    tool_input = tool_block.input
                                    
                                    # Create a tool call object for LLMAgent compatibility
                                    tool_call_obj = type('ToolCall', (), {
                                        'id': tool_id,
                                        'function': type('Function', (), {
                                            'name': tool_name,
                                            'arguments': json.dumps(tool_input)
                                        })
                                    })
                                    tool_calls.append(tool_call_obj)
                                    
                                    # Write tool call information to memory.txt
                                    memory_file.write(f"\n\nTOOL CALL: {tool_name}\n")
                                    memory_file.write(f"ARGUMENTS: {json.dumps(tool_input, indent=2)}\n")
                            
                            # Reset current block
                            current_block = {'type': None, 'id': None, 'content': "", 'signature': None}
                            memory_file.flush()
                        
                        elif event.type == "message_delta" and event.delta.stop_reason == "tool_use":
                            # When a tool use is detected
                            logger.info(f"Tool use stop reason detected in stream")

                    # Mark the end of the response in memory.txt
                    memory_file.write(f"\n\nRESPONSE COMPLETE\n\n")
                    memory_file.flush()
                
                print("\n==== CLAUDE API RESPONSE STREAM COMPLETED ====")
                
            # Final response
            logger.info(f"Claude streaming complete. Text: {len(text_content)} chars, Tool calls: {len(tool_calls)}, Thinking blocks: {len(thinking_blocks)}")
            
            # Return the complete response with all components
            return ResponseMessage(
                content=text_content, 
                tool_calls=tool_calls if tool_calls else None,
                thinking_blocks=thinking_blocks if thinking_blocks else None
            )
            
        except ConnectionError as ce:
            logger.error(f"Connection error with Anthropic API: {ce}")
            raise
        except ValueError as ve:
            logger.error(f"Invalid parameters for Anthropic API: {ve}")
            raise
        except Exception as e:
            logger.error(f"Unexpected error in Anthropic API call: {e}")
            logger.exception(e)  # This will print the full traceback
            raise
    
    # TODO: Delete, deprecated
    def direct_query(self, query_text: str, base64_image: Optional[str] = None, dimensions: Optional[Tuple[int, int]] = None, rag_results: Optional[str] = None) -> str:
        """Execute a direct query with Claude.
        
        This is a convenience wrapper around _observable_query that returns just the response text.
        It creates a temporary observer to collect the response.
        
        Args:
            query_text (str): The query text to process
            base64_image (Optional[str]): Optional Base64-encoded image
            dimensions (Optional[Tuple[int, int]]): Optional image dimensions
            rag_results (Optional[str]): Optional RAG context
            
        Returns:
            str: The response text from Claude
        """
        # Store the new query text
        self.query = query_text
        logger.info(f"Processing new query: {self.query}")
        
        # Create a temporary observer to collect the response
        class TempObserver(Observer):
            def __init__(self):
                self.response = ""
            def on_next(self, value):
                self.response = value
            def on_error(self, error):
                logger.error(f"Error in direct query: {error}")
            def on_completed(self):
                pass
                
        observer = TempObserver()
        self._observable_query(observer, base64_image, dimensions, False, query_text, False)
        return observer.response

    def _observable_query(self,
                             observer: Observer,
                             base64_image: Optional[str] = None,
                             dimensions: Optional[Tuple[int, int]] = None,
                             override_token_limit: bool = False,
                             incoming_query: Optional[str] = None,
                             reset_conversation: bool = False,
                             thinking_budget_tokens: int = None):
        """Main query handler that manages conversation history and Claude interactions.
        
        This is the primary method for handling all queries, whether they come through
        direct_query or through the observable pattern. It manages the conversation
        history, builds prompts, and handles tool calls.
        
        Args:
            observer (Observer): The observer to emit responses to
            base64_image (Optional[str]): Optional Base64-encoded image
            dimensions (Optional[Tuple[int, int]]): Optional image dimensions
            override_token_limit (bool): Whether to override token limits
            incoming_query (Optional[str]): Optional query to update the agent's query
            reset_conversation (bool): Whether to reset the conversation history
        """
        
        try:
            logger.info("_observable_query called in claude")
            import copy
            # Reset conversation history if requested
            if reset_conversation:
                self.conversation_history = []
            # Create a local copy of conversation history
            messages = copy.deepcopy(self.conversation_history)
            # Update query and get context
            self._update_query(incoming_query)
            _, rag_results = self._get_rag_context()
            # Build prompt and get Claude parameters
            budget = thinking_budget_tokens if thinking_budget_tokens is not None else self.thinking_budget_tokens
            messages, claude_params = self._build_prompt(messages, base64_image, dimensions, override_token_limit, rag_results, budget)
            # Send query and get response
            response_message = self._send_query(messages, claude_params)
            if response_message is None:
                logger.error("Received None response from Claude API")
                observer.on_next("")
                observer.on_completed()
                return
            # Add thinking blocks and text content to conversation history
            content_blocks = []
            if response_message.thinking_blocks:
                content_blocks.extend(response_message.thinking_blocks)
            if response_message.content:
                content_blocks.append({
                    "type": "text",
                    "text": response_message.content
                })
            if content_blocks:
                messages.append({
                    "role": "assistant",
                    "content": content_blocks
                })
            
            # At the end, update the global conversation history under a lock
            import threading
            if not hasattr(self, '_history_lock'):
                self._history_lock = threading.Lock()
            with self._history_lock:
                self.conversation_history = messages

            # Handle tool calls if present
            if response_message.tool_calls:
                self._handle_tooling(response_message, messages)

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
        """Override of LLMAgent._handle_tooling to handle Claude's message format.
        
        This method processes tool calls from Claude sequentially, preserving the
        multi-turn dialogue context including thinking blocks. For each tool call,
        it executes the tool, formats the result, and continues the conversation
        with Claude to get the next instructions.
        
        Args:
            response_message: The response message containing tool calls
            messages: Dict containing Claude API parameters
            
        Returns:
            The final response after handling all tools, or None if no further processing needed
        """
        if not hasattr(response_message, 'tool_calls') or not response_message.tool_calls:
            logger.info("No tool calls found in response message")
            return None
            
        if len(response_message.tool_calls) > 1:
            logger.warning("Multiple tool calls detected in response message. Not a tested feature.")
        # Execute all tools first and collect their results
        for tool_call in response_message.tool_calls:
            logger.info(f"Processing tool call: {tool_call.function.name}")
            
            # Add tool call to conversation history
            tool_use_block = {
                "type": "tool_use",
                "id": tool_call.id,
                "name": tool_call.function.name,
                "input": json.loads(tool_call.function.arguments)
            }
            messages.append({
                "role": "assistant",
                "content": [tool_use_block]
            })
            
            # Execute the tool
            args = json.loads(tool_call.function.arguments)
            tool_result = self.skills.call(tool_call.function.name, **args)
            
            # Add tool result to conversation history
            if tool_result:
                messages.append({
                    "role": "user",
                    "content": [{
                        "type": "tool_result",
                        "tool_use_id": tool_call.id,
                        "content": f"{tool_result}"
                    }]
                })
        
        # Get final response from Claude with all tool results in context
        self.run_observable_query(
            query_text=f"Tool {tool_call.function.name} execution complete. Please summarize the results and continue.",
            thinking_budget_tokens=0
        ).run()

    def _debug_api_call(self, claude_params: dict):
        """Debugging function to log API calls with truncated base64 data."""
        # Remove tools to reduce verbosity
        import copy
        log_params = copy.deepcopy(claude_params)
        if 'tools' in log_params:
            del log_params['tools']
            
        # Truncate base64 data in images - much cleaner approach
        if 'messages' in log_params:
            for msg in log_params['messages']:
                if 'content' in msg:
                    for content in msg['content']:
                        if isinstance(content, dict) and content.get('type') == 'image':
                            source = content.get('source', {})
                            if source.get('type') == 'base64' and 'data' in source:
                                data = source['data']
                                source['data'] = f"{data[:50]}..."
        return json.dumps(log_params, indent=2, default=str) 