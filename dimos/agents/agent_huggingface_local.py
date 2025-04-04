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

from __future__ import annotations

# Standard library imports
import json
import logging
import os
from typing import Any, Optional

# Third-party imports
from dotenv import load_dotenv
from reactivex import Observable, create
from reactivex.scheduler import ThreadPoolScheduler
from reactivex.subject import Subject
import torch
from transformers import AutoModelForCausalLM, AutoTokenizer

# Local imports
from dimos.agents.agent import LLMAgent
from dimos.agents.memory.base import AbstractAgentSemanticMemory
from dimos.agents.prompt_builder.impl import PromptBuilder
from dimos.agents.tokenizer.base import AbstractTokenizer
from dimos.agents.tokenizer.huggingface_tokenizer import HuggingFaceTokenizer
from dimos.utils.logging_config import setup_logger

# Initialize environment variables
load_dotenv()

# Initialize logger for the agent module
logger = setup_logger("dimos.agents", level=logging.DEBUG)

# HuggingFaceLLMAgent Class
class HuggingFaceLocalAgent(LLMAgent):
    def __init__(self,
                 dev_name: str,
                 agent_type: str = "HF-LLM",
                 model_name: str = "Qwen/Qwen2.5-3B",
                 device: str = "auto",
                 query: str = "How many r's are in the word 'strawberry'?",
                 input_query_stream: Optional[Observable] = None,
                 input_video_stream: Optional[Observable] = None,
                 output_dir: str = os.path.join(os.getcwd(), "assets",
                                                "agent"),
                 agent_memory: Optional[AbstractAgentSemanticMemory] = None,
                 system_query: Optional[str] = None,
                 max_output_tokens_per_request: int = None,
                 max_input_tokens_per_request: int = None,
                 prompt_builder: Optional[PromptBuilder] = None,
                 tokenizer: Optional[AbstractTokenizer] = None,
                 image_detail: str = "low",
                 pool_scheduler: Optional[ThreadPoolScheduler] = None,
                 process_all_inputs: Optional[bool] = None,):

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

        self.query = query
        self.output_dir = output_dir
        os.makedirs(self.output_dir, exist_ok=True)

        self.model_name = model_name
        self.device = device
        if self.device == "auto":
            self.device = "cuda" if torch.cuda.is_available() else "cpu"
            if self.device == "cuda":
                print(f"Using GPU: {torch.cuda.get_device_name(0)}")
            else:
                print("GPU not available, using CPU")
        print(f"Device: {self.device}")

        self.tokenizer = tokenizer or HuggingFaceTokenizer(self.model_name)

        self.prompt_builder = prompt_builder or PromptBuilder(
            self.model_name,
            tokenizer=self.tokenizer
        )

        self.model = AutoModelForCausalLM.from_pretrained(
            model_name,
            torch_dtype=torch.float16 if self.device == "cuda" else torch.float32,
            device_map=self.device
        )

        self.max_output_tokens_per_request = max_output_tokens_per_request

        # self.stream_query(self.query).subscribe(lambda x: print(x))

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


    def _send_query(self, messages: list) -> Any:
        _BLUE_PRINT_COLOR: str = "\033[34m"
        _RESET_COLOR: str = "\033[0m"
        
        try:
            # Log the incoming messages
            print(f"{_BLUE_PRINT_COLOR}Messages: {str(messages)}{_RESET_COLOR}")
            
            # Process with chat template
            try:
                print("Applying chat template...")
                prompt_text = self.tokenizer.tokenizer.apply_chat_template(
                    conversation=[{"role": "user", "content": str(messages)}],
                    tokenize=False,
                    add_generation_prompt=True
                )
                print("Chat template applied.")
                
                # Tokenize the prompt
                print("Preparing model inputs...")
                model_inputs = self.tokenizer.tokenizer([prompt_text], return_tensors="pt").to(self.model.device)
                print("Model inputs prepared.")
                
                # Generate the response
                print("Generating response...")
                generated_ids = self.model.generate(
                    **model_inputs,
                    max_new_tokens=self.max_output_tokens_per_request
                )
                
                # Extract the generated tokens (excluding the input prompt tokens)
                print("Processing generated output...")
                generated_ids = [
                    output_ids[len(input_ids):]
                    for input_ids, output_ids in zip(model_inputs.input_ids, generated_ids)
                ]
                
                # Convert tokens back to text
                response = self.tokenizer.tokenizer.batch_decode(generated_ids, skip_special_tokens=True)[0]
                print("Response successfully generated.")
                
                return response
                
            except AttributeError as e:
                # Handle case where tokenizer doesn't have the expected methods
                logger.warning(f"Chat template not available: {e}. Using simple format.")
                # Continue with execution and use simple format
                
            except Exception as e:
                # Log any other errors but continue execution
                logger.warning(f"Error in chat template processing: {e}. Falling back to simple format.")
            
            # Fallback approach for models without chat template support
            # This code runs if the try block above raises an exception
            print("Using simple prompt format...")
            
            # Convert messages to a simple text format
            if isinstance(messages, list) and messages and isinstance(messages[0], dict) and "content" in messages[0]:
                prompt_text = messages[0]["content"]
            else:
                prompt_text = str(messages)
                
            # Tokenize the prompt
            model_inputs = self.tokenizer.tokenize_text(prompt_text)
            model_inputs = torch.tensor([model_inputs], device=self.model.device)
            
            # Generate the response
            generated_ids = self.model.generate(
                input_ids=model_inputs,
                max_new_tokens=self.max_output_tokens_per_request
            )
            
            # Extract the generated tokens
            generated_ids = generated_ids[0][len(model_inputs[0]):]
            
            # Convert tokens back to text
            response = self.tokenizer.detokenize_text(generated_ids.tolist())
            print("Response generated using simple format.")
            
            return response
            
        except Exception as e:
            # Catch all other errors
            logger.error(f"Error during query processing: {e}", exc_info=True)
            return f"Error processing request. Please try again."

    def stream_query(self, query_text: str) -> Subject:
        """
        Creates an observable that processes a text query and emits the response.
        """
        return create(lambda observer, _: self._observable_query(
            observer, incoming_query=query_text))

# endregion HuggingFaceLLMAgent Subclass (HuggingFace-Specific Implementation)
