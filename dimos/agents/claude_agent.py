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
from dimos.skills.skills import AbstractSkill
from dimos.stream.frame_processor import FrameProcessor
from dimos.utils.logging_config import setup_logger
from dimos.utils.threadpool import get_scheduler

# Initialize environment variables
load_dotenv()

# Initialize logger for the Claude agent
logger = setup_logger("dimos.agents.claude")

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
            
        # Add thinking if enabled and hard code required temperature = 1
        if self.thinking_budget_tokens is not None:
            claude_params["thinking"] = {
                "type": "enabled",
                "budget_tokens": self.thinking_budget_tokens
            }
            claude_params["temperature"] = 1  # Required to be 1 when thinking is enabled # Default to 0 for deterministic responses
            
        # Store the parameters for use in _send_query
        self.claude_api_params = claude_params
            
        return {'claude_prompt': claude_params}
        
    def _send_query(self, messages: dict) -> Any:
        """Sends the query to Anthropic's API using streaming for better thinking visualization.
        
        Args:
            messages: Dict with 'claude_prompt' key containing Claude API parameters.
        
        Returns:
            The response message in a format compatible with LLMAgent's expectations.
        """
        try:
            # Get Claude parameters
            claude_params = (messages.get('claude_prompt', None) or 
                           self.claude_api_params)
            
            # Log full request parameters to console
            print("\n\n==== CLAUDE API REQUEST ====")
            print(json.dumps(claude_params, indent=2, default=str))
            
            print("==== END REQUEST ====")
            
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
                        print(f"EVENT: {event.type}")  
                        print(json.dumps(event.model_dump(), indent=2, default=str))
                        
                        if event.type == "content_block_start":
                            # Initialize a new content block
                            block_type = event.content_block.type
                            current_block = {'type': block_type, 'id': event.index, 'content': "", 'signature': None}
                            logger.debug(f"Starting {block_type} block...")
                        
                        elif event.type == "content_block_delta":
                            if event.delta.type == "thinking_delta":
                                # Accumulate thinking content
                                current_block['content'] = event.delta.thinking
                                memory_file.write(f"THINKING: {event.delta.thinking}")
                                memory_file.flush()  # Ensure content is written immediately
                            
                            elif event.delta.type == "text_delta":
                                # Accumulate text content
                                text_content += event.delta.text
                                current_block['content'] += event.delta.text
                                memory_file.write(f"RESPONSE: {event.delta.text}")
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
                                    logger.debug(f"Tool call complete: {tool_name}")
                            
                            # Reset current block
                            current_block = {'type': None, 'id': None, 'content': "", 'signature': None}
                            memory_file.flush()
                        
                        elif event.type == "message_delta" and event.delta.stop_reason == "tool_use":
                            # When a tool use is detected
                            logger.debug(f"Tool use stop reason detected in stream")
                    
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
            
    def direct_query(self, query_text: str, base64_image: Optional[str] = None, dimensions: Optional[Tuple[int, int]] = None):
        """Execute a direct streaming query with Claude that handles multi-turn tool calling.
        
        Unlike run_streaming_query, this method completely bypasses the LLMAgent pipeline
        and directly interacts with the Claude API. This allows it to work even when the
        parent LLMAgent._observable_query method would exit early due to thinking blocks.
        
        Args:
            query_text (str): The query text to process
            base64_image (Optional[str]): Optional Base64-encoded image to include in the query
            dimensions (Optional[Tuple[int, int]]): Optional image dimensions (width, height)
            
        Returns:
            The final text response from Claude after all tool calls are complete
        """
        # Start with the user query
        self.query = query_text
        
        # Get RAG context
        _, condensed_results = self._get_rag_context()
        
        # Build the initial prompt including image if provided
        messages = self._build_prompt(base64_image, dimensions, False, condensed_results)
        claude_params = messages.get('claude_prompt', {}).copy()
        
        # Initialize conversation history
        conversation = claude_params.get('messages', []).copy()
        final_response = ""
        
        # Main processing loop that continues until all tool calls are complete
        still_processing = True
        while still_processing:
            # Send query with current conversation state
            claude_params['messages'] = conversation
            response_message = self._send_query({'claude_prompt': claude_params})
            
            if response_message is None:
                logger.error("Received None response from Claude API")
                break
                
            # Add response text to final output
            if response_message.content:
                final_response += ("\n" if final_response else "") + response_message.content
            
            # Process any tool calls
            if response_message.tool_calls and len(response_message.tool_calls) > 0:
                logger.info(f"Processing {len(response_message.tool_calls)} tool calls")
                
                # Process each tool call individually
                for i, tool_call in enumerate(response_message.tool_calls):
                    logger.info(f"Processing tool call {i+1}/{len(response_message.tool_calls)}: {tool_call.function.name}")
                    
                    # Build assistant message with content and thinking blocks
                    assistant_content = []
                    
                    # Add thinking blocks if available
                    if hasattr(response_message, 'thinking_blocks') and response_message.thinking_blocks:
                        assistant_content.extend(response_message.thinking_blocks)
                    
                    # Add text content if present
                    if response_message.content and response_message.content.strip():
                        assistant_content.append({"type": "text", "text": response_message.content})
                    
                    # Add tool use block
                    assistant_content.append({
                        "type": "tool_use",
                        "id": tool_call.id,
                        "name": tool_call.function.name,
                        "input": json.loads(tool_call.function.arguments)
                    })
                    
                    # Add assistant message to conversation
                    conversation.append({
                        "role": "assistant",
                        "content": assistant_content
                    })
                    
                    # Execute the tool
                    name = tool_call.function.name
                    args = json.loads(tool_call.function.arguments)
                    result = self.skills.call(name, **args)
                    logger.info(f"Tool '{name}' executed with args {args} and returned: {result}")
                    
                    # Add tool result to conversation
                    conversation.append({
                        "role": "user",
                        "content": [{
                            "type": "tool_result",
                            "tool_use_id": tool_call.id,
                            "content": f"{result}"
                        }]
                    })
                
                # Continue processing with updated conversation
            else:
                # No more tool calls - we're done
                still_processing = False
        
        logger.info(f"Direct query complete. Final response: {final_response}")
        return final_response
    # TODO: Delete, not used
    def run_streaming_query(self, query_text: str, base64_image: Optional[str] = None, dimensions: Optional[Tuple[int, int]] = None):
        """Run a streaming query with Claude that handles multi-turn tool calling.
        
        This method creates a continuous streaming session that:
        1. Sends the initial query to Claude
        2. Processes streaming responses with thinking blocks
        3. Uses _handle_tooling to execute any tool calls that Claude makes
        4. Continues the conversation with tool results
        5. Repeats until Claude completes its response without tool calls
        
        Args:
            query_text (str): The query text to process
            base64_image (Optional[str]): Optional Base64-encoded image to include in the query
            dimensions (Optional[Tuple[int, int]]): Optional image dimensions (width, height)
            
        Returns:
            The final text response from Claude after all tool calls are complete
        """
        # Use the direct query method instead since it works better with the early exit in LLMAgent
        return self.direct_query(query_text, base64_image, dimensions)
        
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
            
        # Make deep copy of messages to avoid modifying the original
        messages_copy = messages.copy()
        claude_params = messages_copy.get('claude_prompt', {}).copy()
        
        # Initialize conversation history if not already present
        conversation = claude_params.get('messages', []).copy()
        
        # Store the current state of the dialogue
        current_response = response_message
        final_response = current_response.content
        
        # Process tool calls one by one, maintaining conversation state
        for i, tool_call in enumerate(response_message.tool_calls):
            logger.info(f"Processing tool call {i+1}/{len(response_message.tool_calls)}: {tool_call.function.name}")
            
            # Build assistant message with text content and thinking blocks
            assistant_content = []
            
            # First add all thinking blocks
            if hasattr(current_response, 'thinking_blocks') and current_response.thinking_blocks:
                assistant_content.extend(current_response.thinking_blocks)
            
            # Then add text content if present
            if current_response.content.strip():
                assistant_content.append({"type": "text", "text": current_response.content})
            
            # Add just the current tool call
            tool_use_block = {
                "type": "tool_use",
                "id": tool_call.id,
                "name": tool_call.function.name,
                "input": json.loads(tool_call.function.arguments)
            }
            assistant_content.append(tool_use_block)
            
            # Add assistant response to conversation
            conversation.append({
                "role": "assistant",
                "content": assistant_content
            })
            
            # Execute this tool
            name = tool_call.function.name
            args = json.loads(tool_call.function.arguments)
            result = self.skills.call(name, **args)
            logger.info(f"Tool '{name}' returned: {result}")
            # Format the tool result
            tool_result = {
                "type": "tool_result",
                "tool_use_id": tool_call.id,
                "content": f"{result}"
            }
            
            # Add tool result to conversation
            conversation.append({
                "role": "user",
                "content": [tool_result]
            })
            
            # Log the conversation state
            print("\n\n==== CLAUDE API TOOL CONTINUATION STATE ====")
            print(json.dumps(conversation, indent=2, default=str))
            print("==== END TOOL CONTINUATION STATE ====")
            
            # If this isn't the last tool, follow up with Claude to get next instruction
            if i < len(response_message.tool_calls) - 1:
                # Send continuation query with updated conversation
                claude_params['messages'] = conversation
                next_response = self._send_query({'claude_prompt': claude_params})
                
                if next_response is None:
                    logger.error("Received None response from Claude API in tool continuation")
                    break
                    
                # Update current response and add to final output
                current_response = next_response
                final_response += "\n" + next_response.content
                
                # Check if the next response has new tool calls
                if hasattr(next_response, 'tool_calls') and next_response.tool_calls:
                    # Recursive call to handle new tool calls
                    logger.info(f"Found additional {len(next_response.tool_calls)} tool calls in continuation")
                    claude_params['messages'] = conversation
                    recursive_response = self._handle_tooling(next_response, {'claude_prompt': claude_params})
                    if recursive_response:
                        final_response += "\n" + recursive_response.content
                    return ResponseMessage(content=final_response)
        
        # After all tools are processed, get final response from Claude
        claude_params['messages'] = conversation
        final_claude_response = self._send_query({'claude_prompt': claude_params})
        
        if final_claude_response:
            final_response += "\n" + final_claude_response.content
            
        return ResponseMessage(content=final_response)
