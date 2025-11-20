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

"""Agent framework for LLM-based autonomous systems.

This module provides a flexible foundation for creating agents that can:
- Process image and text inputs through LLM APIs
- Store and retrieve contextual information using semantic memory
- Handle tool/function calling
- Process streaming inputs asynchronously

The module offers base classes (Agent, LLMAgent) and concrete implementations
like OpenAIAgent that connect to specific LLM providers.
"""

from __future__ import annotations

# Standard library imports
import ast
import json
import os
import threading
import logging
from typing import Any, Dict, List, Tuple, Optional, Union

# Third-party imports
from dotenv import load_dotenv
from openai import NOT_GIVEN, OpenAI
from pydantic import BaseModel
import reactivex
from reactivex import Observer, create, Observable, empty, operators as RxOps, throw, just
from reactivex.disposable import CompositeDisposable, Disposable
from reactivex.scheduler import ThreadPoolScheduler
from reactivex.subject import Subject

# Local imports
from dimos.agents.memory.base import AbstractAgentSemanticMemory
from dimos.agents.memory.chroma_impl import AgentSemanticMemory
from dimos.agents.prompt_builder.impl import PromptBuilder
from dimos.agents.tokenizer.base import AbstractTokenizer
from dimos.agents.tokenizer.openai_tokenizer import OpenAITokenizer
from dimos.skills.skills import AbstractSkill, SkillLibrary
from dimos.stream.frame_processor import FrameProcessor
from dimos.stream.video_operators import Operators as MyOps, VideoOperators as MyVidOps
from dimos.types.constants import Colors
from dimos.utils.threadpool import get_scheduler
from dimos.utils.logging_config import setup_logger

# Initialize environment variables
load_dotenv()

# Initialize logger for the agent module
logger = setup_logger("dimos.agents")

# Constants
_TOKEN_BUDGET_PARTS = 4  # Number of parts to divide token budget
_MAX_SAVED_FRAMES = 100  # Maximum number of frames to save


# -----------------------------------------------------------------------------
# region Agent Base Class
# -----------------------------------------------------------------------------
class Agent:
    """Base agent that manages memory and subscriptions."""

    def __init__(self,
                 dev_name: str = "NA",
                 agent_type: str = "Base",
                 agent_memory: Optional[AbstractAgentSemanticMemory] = None,
                 pool_scheduler: Optional[ThreadPoolScheduler] = None):
        """
        Initializes a new instance of the Agent.

        Args:
            dev_name (str): The device name of the agent.
            agent_type (str): The type of the agent (e.g., 'Base', 'Vision').
            agent_memory (AbstractAgentSemanticMemory): The memory system for the agent.
            pool_scheduler (ThreadPoolScheduler): The scheduler to use for thread pool operations.
                If None, the global scheduler from get_scheduler() will be used.
        """
        self.dev_name = dev_name
        self.agent_type = agent_type
        self.agent_memory = agent_memory or AgentSemanticMemory()
        self.disposables = CompositeDisposable()
        self.pool_scheduler = pool_scheduler if pool_scheduler else get_scheduler()

    def dispose_all(self):
        """Disposes of all active subscriptions managed by this agent."""
        if self.disposables:
            self.disposables.dispose()
        else:
            logger.info("No disposables to dispose.")


# endregion Agent Base Class


# -----------------------------------------------------------------------------
# region LLMAgent Base Class (Generic LLM Agent)
# -----------------------------------------------------------------------------
class LLMAgent(Agent):
    """Generic LLM agent containing common logic for LLM-based agents.

    This class implements functionality for:
      - Updating the query
      - Querying the agent's memory (for RAG)
      - Building prompts via a prompt builder
      - Handling tooling callbacks in responses
      - Subscribing to image and query streams
      - Emitting responses as an observable stream

    Subclasses must implement the `_send_query` method, which is responsible
    for sending the prompt to a specific LLM API.
    
    Attributes:
        query (str): The current query text to process.
        prompt_builder (PromptBuilder): Handles construction of prompts.
        system_query (str): System prompt for RAG context situations.
        image_detail (str): Detail level for image processing ('low','high','auto').
        max_input_tokens_per_request (int): Maximum input token count.
        max_output_tokens_per_request (int): Maximum output token count.
        max_tokens_per_request (int): Total maximum token count.
        rag_query_n (int): Number of results to fetch from memory.
        rag_similarity_threshold (float): Minimum similarity for RAG results.
        frame_processor (FrameProcessor): Processes video frames.
        output_dir (str): Directory for output files.
        response_subject (Subject): Subject that emits agent responses.
        process_all_inputs (bool): Whether to process every input emission (True) or 
            skip emissions when the agent is busy processing a previous input (False).
    """
    logging_file_memory_lock = threading.Lock()

    def __init__(self,
                 dev_name: str = "NA",
                 agent_type: str = "LLM",
                 agent_memory: Optional[AbstractAgentSemanticMemory] = None,
                 pool_scheduler: Optional[ThreadPoolScheduler] = None, 
                 process_all_inputs: bool = False,
                 system_query: Optional[str] = None,
                 max_output_tokens_per_request: int = 16384,
                 max_input_tokens_per_request: int = 128000):
        """
        Initializes a new instance of the LLMAgent.

        Args:
            dev_name (str): The device name of the agent.
            agent_type (str): The type of the agent.
            agent_memory (AbstractAgentSemanticMemory): The memory system for the agent.
            pool_scheduler (ThreadPoolScheduler): The scheduler to use for thread pool operations.
                If None, the global scheduler from get_scheduler() will be used.
            process_all_inputs (bool): Whether to process every input emission (True) or 
                skip emissions when the agent is busy processing a previous input (False).
        """
        super().__init__(dev_name, agent_type, agent_memory or
                         AgentSemanticMemory(), pool_scheduler)
        # These attributes can be configured by a subclass if needed.
        self.query: Optional[str] = None
        self.prompt_builder: Optional[PromptBuilder] = None
        self.system_query: Optional[str] = system_query
        self.image_detail: str = "low"
        self.max_input_tokens_per_request: int = max_input_tokens_per_request
        self.max_output_tokens_per_request: int = max_output_tokens_per_request
        self.max_tokens_per_request: int = (self.max_input_tokens_per_request +
                                            self.max_output_tokens_per_request)
        self.rag_query_n: int = 4
        self.rag_similarity_threshold: float = 0.45
        self.frame_processor: Optional[FrameProcessor] = None
        self.output_dir: str = os.path.join(os.getcwd(), "assets", "agent")
        self.process_all_inputs: bool = process_all_inputs
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Subject for emitting responses
        self.response_subject = Subject()

    def _update_query(self, incoming_query: Optional[str]) -> None:
        """Updates the query if an incoming query is provided.

        Args:
            incoming_query (str): The new query text.
        """
        if incoming_query is not None:
            self.query = incoming_query

    def _get_rag_context(self) -> Tuple[str, str]:
        """Queries the agent memory to retrieve RAG context.

        Returns:
            Tuple[str, str]: A tuple containing the formatted results (for logging)
            and condensed results (for use in the prompt).
        """
        results = self.agent_memory.query(
            query_texts=self.query,
            n_results=self.rag_query_n,
            similarity_threshold=self.rag_similarity_threshold)
        formatted_results = "\n".join(
            f"Document ID: {doc.id}\nMetadata: {doc.metadata}\nContent: {doc.page_content}\nScore: {score}\n"
            for (doc, score) in results)
        condensed_results = " | ".join(
            f"{doc.page_content}" for (doc, _) in results)
        logger.info(f"Agent Memory Query Results:\n{formatted_results}")
        logger.info("=== Results End ===")
        return formatted_results, condensed_results

    def _build_prompt(self, base64_image: Optional[str],
                      dimensions: Optional[Tuple[int, int]],
                      override_token_limit: bool,
                      condensed_results: str) -> list:
        """Builds a prompt message using the prompt builder.

        Args:
            base64_image (str): Optional Base64-encoded image.
            dimensions (Tuple[int, int]): Optional image dimensions.
            override_token_limit (bool): Whether to override token limits.
            condensed_results (str): The condensed RAG context.

        Returns:
            list: A list of message dictionaries to be sent to the LLM.
        """
        # Budget for each component of the prompt
        budgets = {
            "system_prompt":
                self.max_input_tokens_per_request // _TOKEN_BUDGET_PARTS,
            "user_query":
                self.max_input_tokens_per_request // _TOKEN_BUDGET_PARTS,
            "image":
                self.max_input_tokens_per_request // _TOKEN_BUDGET_PARTS,
            "rag":
                self.max_input_tokens_per_request // _TOKEN_BUDGET_PARTS,
        }

        # Define truncation policies for each component
        policies = {
            "system_prompt": "truncate_end",
            "user_query": "truncate_middle",
            "image": "do_not_truncate",
            "rag": "truncate_end",
        }

        return self.prompt_builder.build(
            user_query=self.query,
            override_token_limit=override_token_limit,
            base64_image=base64_image,
            image_width=dimensions[0] if dimensions is not None else None,
            image_height=dimensions[1] if dimensions is not None else None,
            image_detail=self.image_detail,
            rag_context=condensed_results,
            system_prompt=self.system_query,
            budgets=budgets,
            policies=policies,
        )

    def _handle_tooling(self, response_message, messages):
        """Handles tooling callbacks in the response message.

        If tool calls are present, the corresponding functions are executed and
        a follow-up query is sent.

        Args:
            response_message: The response message containing tool calls.
            messages (list): The original list of messages sent.

        Returns:
            The final response message after processing tool calls, if any.
        """

        # TODO: Make this more generic or move implementation to OpenAIAgent.
        # This is presently OpenAI-specific.
        def _tooling_callback(message, messages, response_message,
                              skill_library: SkillLibrary):
            has_called_tools = False
            new_messages = []
            for tool_call in message.tool_calls:
                has_called_tools = True
                name = tool_call.function.name
                args = json.loads(tool_call.function.arguments)
                result = skill_library.call(name, **args)
                logger.info(f"Function Call Results: {result}")
                new_messages.append({
                    "role": "tool",
                    "tool_call_id": tool_call.id,
                    "content": str(result),
                    "name": name
                })
            if has_called_tools:
                logger.info("Sending Another Query.")
                messages.append(response_message)
                messages.extend(new_messages)
                # Delegate to sending the query again.
                return self._send_query(messages)
            else:
                logger.info("No Need for Another Query.")
                return None

        if response_message.tool_calls is not None:
            return _tooling_callback(response_message, messages,
                                     response_message, self.skill_library)
        return None

    def _observable_query(self,
                          observer: Observer,
                          base64_image: Optional[str] = None,
                          dimensions: Optional[Tuple[int, int]] = None,
                          override_token_limit: bool = False,
                          incoming_query: Optional[str] = None):
        """Prepares and sends a query to the LLM, emitting the response to the observer.

        Args:
            observer (Observer): The observer to emit responses to.
            base64_image (str): Optional Base64-encoded image.
            dimensions (Tuple[int, int]): Optional image dimensions.
            override_token_limit (bool): Whether to override token limits.
            incoming_query (str): Optional query to update the agent's query.

        Raises:
            Exception: Propagates any exceptions encountered during processing.
        """
        try:
            self._update_query(incoming_query)
            _, condensed_results = self._get_rag_context()
            messages = self._build_prompt(base64_image, dimensions,
                                          override_token_limit,
                                          condensed_results)
            # logger.debug(f"Sending Query: {messages}")
            logger.info("Sending Query.")
            response_message = self._send_query(messages)
            logger.info(f"Received Response: {response_message}")
            if response_message is None:
                raise Exception("Response message does not exist.")

            # TODO: Make this more generic. The parsed tag and tooling handling may be OpenAI-specific.
            # If no skill library is provided or there are no tool calls, emit the response directly.
            if (self.skill_library is None or
                    self.skill_library.get_tools() in (None, NOT_GIVEN) or
                    response_message.tool_calls is None):
                final_msg = (response_message.parsed
                             if hasattr(response_message, 'parsed') and
                             response_message.parsed else
                             (response_message.content if hasattr(response_message, 'content') else response_message))
                observer.on_next(final_msg)
                self.response_subject.on_next(final_msg)
            else:
                response_message_2 = self._handle_tooling(
                    response_message, messages)
                final_msg = response_message_2 if response_message_2 is not None else response_message
                if isinstance(final_msg, BaseModel): # TODO: Test
                    final_msg = str(final_msg.content)
                observer.on_next(final_msg)
                self.response_subject.on_next(final_msg)
            observer.on_completed()
        except Exception as e:
            logger.error(f"Query failed in {self.dev_name}: {e}")
            observer.on_error(e)
            self.response_subject.on_error(e)

    def _send_query(self, messages: list) -> Any:
        """Sends the query to the LLM API.

        This method must be implemented by subclasses with specifics of the LLM API.

        Args:
            messages (list): The prompt messages to be sent.

        Returns:
            Any: The response message from the LLM.

        Raises:
            NotImplementedError: Always, unless overridden.
        """
        raise NotImplementedError(
            "Subclasses must implement _send_query method.")

    def _log_response_to_file(self, response, output_dir: str = None):
        """Logs the LLM response to a file.

        Args:
            response: The response message to log.
            output_dir (str): The directory where the log file is stored.
        """
        if output_dir is None:
            output_dir = self.output_dir
        if response is not None:
            with self.logging_file_memory_lock:
                log_path = os.path.join(output_dir, 'memory.txt')
                with open(log_path, 'a') as file:
                    file.write(f"{self.dev_name}: {response}\n")
                logger.info(f"LLM Response [{self.dev_name}]: {response}")

    def subscribe_to_image_processing(
            self, frame_observable: Observable) -> Disposable:
        """Subscribes to a stream of video frames for processing.

        This method sets up a subscription to process incoming video frames.
        Each frame is encoded and then sent to the LLM by directly calling the
        _observable_query method. The response is then logged to a file.

        Args:
            frame_observable (Observable): An observable emitting video frames.

        Returns:
            Disposable: A disposable representing the subscription.
        """
        # Initialize frame processor if not already set
        if self.frame_processor is None:
            self.frame_processor = FrameProcessor(delete_on_init=True)

        print_emission_args = {
            "enabled": True,
            "dev_name": self.dev_name,
            "counts": {}
        }

        def _process_frame(frame) -> Observable:
            """
            Processes a single frame by:
            - Logging the receipt
            - Exporting the frame as a JPEG
            - Encoding the image
            - Filtering out invalid results
            - Sending the encoded image to the LLM via _observable_query
            
            Returns:
                An observable that emits the response from _observable_query.
            """
            return just(frame).pipe(
                MyOps.print_emission(id='B', **print_emission_args),
                RxOps.observe_on(self.pool_scheduler),
                MyOps.print_emission(id='C', **print_emission_args),
                RxOps.subscribe_on(self.pool_scheduler),
                MyOps.print_emission(id='D', **print_emission_args),
                MyVidOps.with_jpeg_export(self.frame_processor,
                                          suffix=f"{self.dev_name}_frame_",
                                          save_limit=_MAX_SAVED_FRAMES),
                MyOps.print_emission(id='E', **print_emission_args),
                MyVidOps.encode_image(),
                MyOps.print_emission(id='F', **print_emission_args),
                RxOps.filter(lambda base64_and_dims: base64_and_dims is not None
                             and base64_and_dims[0] is not None and
                             base64_and_dims[1] is not None),
                MyOps.print_emission(id='G', **print_emission_args),
                RxOps.flat_map(lambda base64_and_dims: create(
                    lambda observer, _: self._observable_query(
                        observer,
                        base64_image=base64_and_dims[0],
                        dimensions=base64_and_dims[1],
                        incoming_query=self.system_query))),
                MyOps.print_emission(id='H', **print_emission_args),
            )

        # Use a mutable flag to ensure only one frame is processed at a time.
        is_processing = [False]

        def process_if_free(frame):
            if not self.process_all_inputs and is_processing[0]:
                # Drop frame if a request is in progress and process_all_inputs is False
                return empty()
            else:
                is_processing[0] = True
                return _process_frame(frame).pipe(
                    MyOps.print_emission(id='I', **print_emission_args),
                    RxOps.observe_on(self.pool_scheduler),
                    MyOps.print_emission(id='J', **print_emission_args),
                    RxOps.subscribe_on(self.pool_scheduler),
                    MyOps.print_emission(id='K', **print_emission_args),
                    RxOps.do_action(
                        on_completed=lambda: is_processing.__setitem__(
                            0, False),
                        on_error=lambda e: is_processing.__setitem__(0, False)),
                    MyOps.print_emission(id='L', **print_emission_args),
                )

        observable = frame_observable.pipe(
            MyOps.print_emission(id='A', **print_emission_args),
            RxOps.flat_map(process_if_free),
            MyOps.print_emission(id='M', **print_emission_args),
        )

        disposable = observable.subscribe(
            on_next=lambda response: self._log_response_to_file(
                response, self.output_dir),
            on_error=lambda e: logger.error(f"Error encountered: {e}"),
            on_completed=lambda: logger.info(
                f"Stream processing completed for {self.dev_name}"))
        self.disposables.add(disposable)
        return disposable

    def subscribe_to_query_processing(
            self, query_observable: Observable) -> Disposable:
        """Subscribes to a stream of queries for processing.

        This method sets up a subscription to process incoming queries by directly
        calling the _observable_query method. The responses are logged to a file.

        Args:
            query_observable (Observable): An observable emitting queries.

        Returns:
            Disposable: A disposable representing the subscription.
        """
        print_emission_args = {
            "enabled": True,
            "dev_name": self.dev_name,
            "counts": {}
        }

        def _process_query(query) -> Observable:
            """
            Processes a single query by logging it and passing it to _observable_query.
            Returns an observable that emits the LLM response.
            """
            return just(query).pipe(
                MyOps.print_emission(id='Pr A', **print_emission_args),
                RxOps.flat_map(lambda query: create(
                    lambda observer, _: self._observable_query(
                        observer, incoming_query=query))),
                MyOps.print_emission(id='Pr B', **print_emission_args),
            )

        # A mutable flag indicating whether a query is currently being processed.
        is_processing = [False]

        def process_if_free(query):
            logger.info(f"Processing Query: {query}")
            if not self.process_all_inputs and is_processing[0]:
                # Drop query if a request is already in progress and process_all_inputs is False
                return empty()
            else:
                is_processing[0] = True
                logger.info("Processing Query.")
                return _process_query(query).pipe(
                    MyOps.print_emission(id='B', **print_emission_args),
                    RxOps.observe_on(self.pool_scheduler),
                    MyOps.print_emission(id='C', **print_emission_args),
                    RxOps.subscribe_on(self.pool_scheduler),
                    MyOps.print_emission(id='D', **print_emission_args),
                    RxOps.do_action(
                        on_completed=lambda: is_processing.__setitem__(
                            0, False),
                        on_error=lambda e: is_processing.__setitem__(0, False)),
                    MyOps.print_emission(id='E', **print_emission_args),
                )

        observable = query_observable.pipe(
            MyOps.print_emission(id='A', **print_emission_args),
            RxOps.flat_map(lambda query: process_if_free(query)),
            MyOps.print_emission(id='F', **print_emission_args))

        disposable = observable.subscribe(
            on_next=lambda response: self._log_response_to_file(
                response, self.output_dir),
            on_error=lambda e: logger.error(
                f"Error processing query for {self.dev_name}: {e}"),
            on_completed=lambda: logger.info(
                f"Stream processing completed for {self.dev_name}"))
        self.disposables.add(disposable)
        return disposable

    def get_response_observable(self) -> Observable:
        """Gets an observable that emits responses from this agent.
        
        Returns:
            Observable: An observable that emits string responses from the agent.
        """
        return self.response_subject.pipe(
            RxOps.observe_on(self.pool_scheduler), 
            RxOps.subscribe_on(self.pool_scheduler),
            RxOps.share())

    def run_observable_query(self, query_text: str) -> Observable:
        """Creates an observable that processes a one-off text query to Agent and emits the response.
        
        This method provides a simple way to send a text query and get an observable
        stream of the response. It's designed for one-off queries rather than
        continuous processing of input streams. Useful for testing and development.
        
        Args:
            query_text (str): The query text to process.
            
        Returns:
            Observable: An observable that emits the response as a string.
        """
        return create(lambda observer, _: self._observable_query(
            observer, incoming_query=query_text)) 

    def dispose_all(self):
        """Disposes of all active subscriptions managed by this agent."""
        super().dispose_all()
        self.response_subject.on_completed()


# endregion LLMAgent Base Class (Generic LLM Agent)


# -----------------------------------------------------------------------------
# region OpenAIAgent Subclass (OpenAI-Specific Implementation)
# -----------------------------------------------------------------------------
class OpenAIAgent(LLMAgent):
    """OpenAI agent implementation that uses OpenAI's API for processing.

    This class implements the _send_query method to interact with OpenAI's API.
    It also sets up OpenAI-specific parameters, such as the client, model name,
    tokenizer, and response model.
    """

    def __init__(self,
                 dev_name: str,
                 agent_type: str = "Vision",
                 query: str = "What do you see?",
                 input_query_stream: Optional[Observable] = None,
                 input_video_stream: Optional[Observable] = None,
                 output_dir: str = os.path.join(os.getcwd(), "assets",
                                                "agent"),
                 agent_memory: Optional[AbstractAgentSemanticMemory] = None,
                 system_query: Optional[str] = None,
                 max_input_tokens_per_request: int = 128000,
                 max_output_tokens_per_request: int = 16384,
                 model_name: str = "gpt-4o",
                 prompt_builder: Optional[PromptBuilder] = None,
                 tokenizer: Optional[AbstractTokenizer] = None,
                 rag_query_n: int = 4,
                 rag_similarity_threshold: float = 0.45,
                 skills: Optional[Union[AbstractSkill, list[AbstractSkill], SkillLibrary]] = None,
                 response_model: Optional[BaseModel] = None,
                 frame_processor: Optional[FrameProcessor] = None,
                 image_detail: str = "low",
                 pool_scheduler: Optional[ThreadPoolScheduler] = None,
                 process_all_inputs: Optional[bool] = None,
                 openai_client: Optional[OpenAI] = None):
        """
        Initializes a new instance of the OpenAIAgent.

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
            model_name (str): The OpenAI model name to use.
            prompt_builder (PromptBuilder): Custom prompt builder.
            tokenizer (AbstractTokenizer): Custom tokenizer for token counting.
            rag_query_n (int): Number of results to fetch in RAG queries.
            rag_similarity_threshold (float): Minimum similarity for RAG results.
            skills (Union[AbstractSkill, List[AbstractSkill], SkillLibrary]): Skills available to the agent.
            response_model (BaseModel): Optional Pydantic model for responses.
            frame_processor (FrameProcessor): Custom frame processor.
            image_detail (str): Detail level for images ("low", "high", "auto").
            pool_scheduler (ThreadPoolScheduler): The scheduler to use for thread pool operations.
                If None, the global scheduler from get_scheduler() will be used.
            process_all_inputs (bool): Whether to process all inputs or skip when busy.
                If None, defaults to True for text queries, False for video streams.
            openai_client (OpenAI): The OpenAI client to use. This can be used to specify
                a custom OpenAI client if targetting another provider.
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
        self.client = openai_client or OpenAI()
        self.query = query
        self.output_dir = output_dir
        os.makedirs(self.output_dir, exist_ok=True)

        # Configure skill library.
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

        self.response_model = response_model if response_model is not None else NOT_GIVEN
        self.model_name = model_name
        self.tokenizer = tokenizer or OpenAITokenizer(
            model_name=self.model_name)
        self.prompt_builder = prompt_builder or PromptBuilder(
            self.model_name, tokenizer=self.tokenizer)
        self.rag_query_n = rag_query_n
        self.rag_similarity_threshold = rag_similarity_threshold
        self.image_detail = image_detail
        self.max_output_tokens_per_request = max_output_tokens_per_request
        self.max_input_tokens_per_request = max_input_tokens_per_request
        self.max_tokens_per_request = max_input_tokens_per_request + max_output_tokens_per_request

        # Add static context to memory.
        self._add_context_to_memory()

        self.frame_processor = frame_processor or FrameProcessor(
            delete_on_init=True)
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

        logger.info("OpenAI Agent Initialized.")

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

    def _send_query(self, messages: list) -> Any:
        """Sends the query to OpenAI's API.

        Depending on whether a response model is provided, the appropriate API
        call is made.

        Args:
            messages (list): The prompt messages to send.

        Returns:
            The response message from OpenAI.

        Raises:
            Exception: If no response message is returned.
            ConnectionError: If there's an issue connecting to the API.
            ValueError: If the messages or other parameters are invalid.
        """
        try:
            if self.response_model is not NOT_GIVEN:
                response = self.client.beta.chat.completions.parse(
                    model=self.model_name,
                    messages=messages,
                    response_format=self.response_model,
                    tools=(self.skill_library.get_tools() if self.skill_library is not None else NOT_GIVEN),
                    max_tokens=self.max_output_tokens_per_request,
                )
            else:
                response = self.client.chat.completions.create(
                    model=self.model_name,
                    messages=messages,
                    max_tokens=self.max_output_tokens_per_request,
                    tools=(self.skill_library.get_tools()
                           if self.skill_library is not None else NOT_GIVEN),
                )
            response_message = response.choices[0].message
            if response_message is None:
                logger.error("Response message does not exist.")
                raise Exception("Response message does not exist.")
            return response_message
        except ConnectionError as ce:
            logger.error(f"Connection error with API: {ce}")
            raise
        except ValueError as ve:
            logger.error(f"Invalid parameters: {ve}")
            raise
        except Exception as e:
            logger.error(f"Unexpected error in API call: {e}")
            raise

    def stream_query(self, query_text: str) -> Observable:
        """Creates an observable that processes a text query and emits the response.
        
        This method provides a simple way to send a text query and get an observable
        stream of the response. It's designed for one-off queries rather than
        continuous processing of input streams.
        
        Args:
            query_text (str): The query text to process.
            
        Returns:
            Observable: An observable that emits the response as a string.
        """
        return create(lambda observer, _: self._observable_query(
            observer, incoming_query=query_text))


# endregion OpenAIAgent Subclass (OpenAI-Specific Implementation)