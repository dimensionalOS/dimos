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

from ctransformers import AutoModelForCausalLM as CTransformersModel

class CTransformersTokenizerAdapter:
    def __init__(self, model):
        self.model = model

    def encode(self, text, **kwargs):
        return self.model.tokenize(text)

    def decode(self, token_ids, **kwargs):
        return self.model.detokenize(token_ids)

    def token_count(self, text):
        return len(self.tokenize_text(text)) if text else 0

    def tokenize_text(self, text):
        return self.model.tokenize(text)

    def detokenize_text(self, tokenized_text):
        try:
            return self.model.detokenize(tokenized_text)
        except Exception as e:
            raise ValueError(f"Failed to detokenize text. Error: {str(e)}")

    def apply_chat_template(self, conversation, tokenize=False, add_generation_prompt=True):
        prompt = ""
        for message in conversation:
            role = message["role"]
            content = message["content"]
            if role == "system":
                prompt += f"<|system|>\n{content}\n"
            elif role == "user":
                prompt += f"<|user|>\n{content}\n"
            elif role == "assistant":
                prompt += f"<|assistant|>\n{content}\n"
        if add_generation_prompt:
            prompt += "<|assistant|>\n"
        return prompt


# CTransformers Agent Class
class CTransformersGGUFAgent(LLMAgent):
    def __init__(self,
                 dev_name: str,
                 agent_type: str = "HF-LLM",
                 model_name: str = "TheBloke/Llama-2-7B-GGUF",
                 model_file: str = "llama-2-7b.Q4_K_M.gguf",
                 model_type: str = "llama",
                 gpu_layers: int = 50,
                 device: str = "auto",
                 query: str = "How many r's are in the word 'strawberry'?",
                 input_query_stream: Optional[Observable] = None,
                 input_video_stream: Optional[Observable] = None,
                 output_dir: str = os.path.join(os.getcwd(), "assets", "agent"),
                 agent_memory: Optional[AbstractAgentSemanticMemory] = None,
                 system_query: Optional[str] = "You are a robot with the following functions. Move(), Reverse(), Left(), Right(), Stop(). Given the following command return the correct function. ONLY RETURN THE FUNCTION NAME, NOTHING ELSE.",
                 max_output_tokens_per_request: int = 10,
                 max_input_tokens_per_request: int = 250,
                 prompt_builder: Optional[PromptBuilder] = None,
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
            system_query=system_query,
            max_output_tokens_per_request=max_output_tokens_per_request,
            max_input_tokens_per_request=max_input_tokens_per_request
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

        self.model = CTransformersModel.from_pretrained(
            model_name,
            model_file=model_file,
            model_type=model_type,
            gpu_layers=gpu_layers
        )

        self.tokenizer = CTransformersTokenizerAdapter(self.model)

        self.prompt_builder = prompt_builder or PromptBuilder(
            self.model_name,
            tokenizer=self.tokenizer
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
        try:
            _BLUE_PRINT_COLOR: str = "\033[34m"
            _RESET_COLOR: str = "\033[0m"

            # === FIX: Flatten message content ===
            flat_messages = []
            for msg in messages:
                role = msg["role"]
                content = msg["content"]
                if isinstance(content, list):
                    # Assume it's a list of {'type': 'text', 'text': ...}
                    text_parts = [c["text"] for c in content if isinstance(c, dict) and "text" in c]
                    content = " ".join(text_parts)
                flat_messages.append({"role": role, "content": content})

            print(f"{_BLUE_PRINT_COLOR}Messages: {flat_messages}{_RESET_COLOR}")

            print("Applying chat template...")
            prompt_text = self.tokenizer.apply_chat_template(
                conversation=flat_messages,
                tokenize=False,
                add_generation_prompt=True
            )
            print("Chat template applied.")
            print(f"Prompt text:\n{prompt_text}")

            response = self.model(prompt_text, max_new_tokens=self.max_output_tokens_per_request)
            print("Model response received.")
            return response

        except Exception as e:
            logger.error(f"Error during HuggingFace query: {e}")
            return "Error processing request."

    def stream_query(self, query_text: str) -> Subject:
        """
        Creates an observable that processes a text query and emits the response.
        """
        return create(lambda observer, _: self._observable_query(
            observer, incoming_query=query_text))

# endregion HuggingFaceLLMAgent Subclass (HuggingFace-Specific Implementation)
