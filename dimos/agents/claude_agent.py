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
from pydantic import BaseModel
from reactivex import Observable
from reactivex.disposable import Disposable
from reactivex.scheduler import ThreadPoolScheduler
from reactivex import create

# Local imports
from dimos.agents.agent import LLMAgent
from dimos.agents.memory.base import AbstractAgentSemanticMemory
from dimos.agents.prompt_builder.impl import PromptBuilder
from dimos.robot.skills import AbstractSkill
from dimos.stream.frame_processor import FrameProcessor
from dimos.utils.logging_config import setup_logger
from dimos.utils.threadpool import get_scheduler

# Initialize environment variables
load_dotenv()

# Initialize logger for the Claude agent
logger = setup_logger("dimos.agents.claude", level=logging.DEBUG)


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
                 thinking_budget_tokens: Optional[int] = None):
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
            thinking_budget_tokens (int): Number of tokens to allocate for Claude's thinking.
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
        
        self.response_model = response_model
        self.model_name = model_name
        self.rag_query_n = rag_query_n
        self.rag_similarity_threshold = rag_similarity_threshold
        self.image_detail = image_detail
        self.max_output_tokens_per_request = max_output_tokens_per_request
        self.max_input_tokens_per_request = max_input_tokens_per_request
        self.max_tokens_per_request = max_input_tokens_per_request + max_output_tokens_per_request
        self.logger = logger

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
            self.logger.info("Subscribing to input video stream...")
            self.disposables.add(
                self.subscribe_to_image_processing(self.input_video_stream))
        if self.input_query_stream is not None:
            self.logger.info("Subscribing to input query stream...")
            self.disposables.add(
                self.subscribe_to_query_processing(self.input_query_stream))

        self.logger.info("Claude Agent Initialized.")

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
            
        self.logger.info(f"Converted tools to Claude format: {json.dumps(claude_tools, indent=2)}")
        return claude_tools

    def _build_prompt(self, base64_image: Optional[str],
                      dimensions: Optional[Tuple[int, int]],
                      override_token_limit: bool,
                      condensed_results: str) -> dict:
        """Builds a prompt message specifically for Claude API.

        This method creates messages in Claude's format directly, without using
        any OpenAI-specific formatting or token counting.

        Args:
            base64_image (str): Optional Base64-encoded image.
            dimensions (Tuple[int, int]): Optional image dimensions.
            override_token_limit (bool): Whether to override token limits.
            condensed_results (str): The condensed RAG context.

        Returns:
            dict: A dict containing Claude API parameters.
        """
        # Build Claude message content
        claude_content = []
        
        # Add RAG context and query as text
        text_content = ""
        if condensed_results:
            text_content += f"{condensed_results}\n\n"
        text_content += self.query
        
        claude_content.append({"type": "text", "text": text_content})
        
        # Add image if present
        if base64_image:
            claude_content.append({
                "type": "image",
                "source": {
                    "type": "base64",
                    "media_type": "image/jpeg",
                    "data": base64_image
                }
            })
        
        # Create Claude messages
        claude_messages = [{"role": "user", "content": claude_content}]
        
        # Build complete Claude parameters
        claude_params = {
            "messages": claude_messages,
            "model": self.model_name,
            "max_tokens": self.max_output_tokens_per_request,
            "temperature": 0  # Add temperature to make responses more deterministic
        }
        
        # Add system prompt if present
        if self.system_query:
            claude_params["system"] = self.system_query
            
        # Add tools if skills are available
        if self.skills and self.skills.get_tools():
            tools = self._convert_tools_to_claude_format(self.skills.get_tools())
            if tools:  # Only add if we have valid tools
                claude_params["tools"] = tools
                # Enable tool calling with proper format
                claude_params["tool_choice"] = {
                    "type": "auto"
                }
                self.logger.info(f"Added tools to Claude params: {json.dumps(claude_params['tools'], indent=2)}")
            
        # Add thinking if enabled
        if self.thinking_budget_tokens is not None:
            claude_params["thinking"] = {
                "type": "enabled",
                "budget_tokens": self.thinking_budget_tokens
            }
            
        # Store the parameters for use in _send_query
        self.claude_api_params = claude_params
        self.logger.info(f"Final Claude parameters: {json.dumps(claude_params, indent=2)}")
            
        return {'claude_prompt': claude_params}

    def _send_query(self, messages: dict) -> Any:
        """Sends the query to Anthropic's API.

        Args:
            messages: Dict with 'claude_prompt' key containing Claude API parameters.

        Returns:
            The response message in a format compatible with LLMAgent's expectations.
        """
        try:
            # Get Claude parameters
            claude_params = (messages.get('claude_prompt', None) or 
                           self.claude_api_params)
            
            # Make the API call
            self.logger.info(f"Sending request to Claude API with params: {json.dumps(claude_params, indent=2)}")
            response = self.client.messages.create(**claude_params)
            self.logger.info(f"Raw Claude response: {response}")
            
            # Create a response object compatible with LLMAgent
            class ResponseMessage:
                def __init__(self, content, tool_calls=None):
                    self.content = content
                    self.tool_calls = tool_calls
                    self.parsed = None
            
            # Extract text content and tool calls from response content array
            text_content = ""
            tool_calls = []
            
            for content_block in response.content:
                if content_block.type == 'text':
                    text_content += content_block.text
                elif content_block.type == 'tool_use':
                    self.logger.info(f"Found tool use block: {content_block}")
                    # Create a tool call object that matches OpenAI's format
                    tool_call_obj = type('ToolCall', (), {
                        'id': content_block.id,
                        'function': type('Function', (), {
                            'name': content_block.name,
                            'arguments': json.dumps(content_block.input)
                        })
                    })
                    tool_calls.append(tool_call_obj)
                    self.logger.info(f"Converted tool call object: id={tool_call_obj.id}, name={tool_call_obj.function.name}, args={tool_call_obj.function.arguments}")
            
            response_message = ResponseMessage(content=text_content, tool_calls=tool_calls if tool_calls else None)
            self.logger.info(f"Created response message: content={text_content}, has_tool_calls={bool(tool_calls)}")
            
            # If we have tool calls, use LLMAgent's _handle_tooling
            if tool_calls:
                self.logger.info("Processing tool calls with _handle_tooling")
                # Log the messages we're passing to _handle_tooling
                self.logger.info(f"Messages being passed to _handle_tooling: {json.dumps(messages, indent=2)}")
                
                handled_response = self._handle_tooling(response_message, messages)
                if handled_response is not None:
                    self.logger.info(f"Got handled response from _handle_tooling: {handled_response}")
                    return handled_response
                else:
                    self.logger.info("_handle_tooling returned None")
                    
            return response_message
            
        except ConnectionError as ce:
            self.logger.error(f"Connection error with Anthropic API: {ce}")
            raise
        except ValueError as ve:
            self.logger.error(f"Invalid parameters for Anthropic API: {ve}")
            raise
        except Exception as e:
            self.logger.error(f"Unexpected error in Anthropic API call: {e}")
            self.logger.exception(e)  # This will print the full traceback
            raise

    def _handle_tooling(self, response_message, messages):
        """Override of LLMAgent._handle_tooling to handle Claude's message format.
        
        Args:
            response_message: The response message containing tool calls
            messages: Dict containing Claude API parameters
            
        Returns:
            The final response after handling tools, or None if no further processing needed
        """
        self.logger.info("Entering Claude's _handle_tooling override")
        
        if not hasattr(response_message, 'tool_calls') or not response_message.tool_calls:
            self.logger.info("No tool calls found in response message")
            return None
            
        self.logger.info(f"Processing {len(response_message.tool_calls)} tool calls")
        
        has_called_tools = False
        tool_results = []
        
        # First collect all tool results
        for tool_call in response_message.tool_calls:
            has_called_tools = True
            name = tool_call.function.name
            args = json.loads(tool_call.function.arguments)
            result = self.skills.call_function(name, **args)
            self.logger.info(f"Function Call Results: {result}")
            tool_results.append(f"Tool '{name}' returned: {result}")
        
        if has_called_tools:
            # Add all tool results as a single assistant message
            messages['claude_prompt']['messages'].append({
                "role": "assistant",
                "content": [{"type": "text", "text": response_message.content}]
            })
            messages['claude_prompt']['messages'].append({
                "role": "user",
                "content": [{"type": "text", "text": "Tool execution results:\n" + "\n".join(tool_results)}]
            })
            
            self.logger.info("Sending follow-up query with tool results")
            return self._send_query(messages)
        
        return None