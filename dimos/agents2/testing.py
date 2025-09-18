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

"""Testing utilities for agents."""

import json
import os
from pathlib import Path
from typing import Any, Dict, Iterator, List, Optional, Sequence, Union

from langchain.chat_models import init_chat_model
from langchain_core.callbacks.manager import CallbackManagerForLLMRun
from langchain_core.language_models.chat_models import SimpleChatModel
from langchain_core.messages import (
    AIMessage,
    AIMessageChunk,
    BaseMessage,
)
from langchain_core.outputs import ChatGeneration, ChatGenerationChunk, ChatResult
from langchain_core.runnables import Runnable


class MockModel(SimpleChatModel):
    """Custom fake chat model that supports tool calls for testing.

    Can operate in two modes:
    1. Playback mode (default): Reads responses from a JSON file or list
    2. Record mode: Uses a real LLM and saves responses to a JSON file
    """

    responses: List[Union[str, AIMessage]] = []
    i: int = 0
    json_path: Optional[Path] = None
    record: bool = False
    real_model: Optional[Any] = None
    recorded_messages: List[Dict[str, Any]] = []

    def __init__(self, **kwargs):
        # Extract custom parameters before calling super().__init__
        responses = kwargs.pop("responses", [])
        json_path = kwargs.pop("json_path", None)
        model_provider = kwargs.pop("model_provider", "openai")
        model_name = kwargs.pop("model_name", "gpt-4o")

        super().__init__(**kwargs)

        self.json_path = Path(json_path) if json_path else None
        self.record = bool(os.getenv("RECORD"))
        self.i = 0
        self._bound_tools: Optional[Sequence[Any]] = None
        self.recorded_messages = []

        if self.record:
            # Initialize real model for recording
            self.real_model = init_chat_model(model_provider=model_provider, model=model_name)
            self.responses = []  # Initialize empty for record mode
        elif self.json_path:
            self.responses = self._load_responses_from_json()
        elif responses:
            self.responses = responses
        else:
            raise ValueError("no responses")

    @property
    def _llm_type(self) -> str:
        return "tool-call-fake-chat-model"

    def _load_responses_from_json(self) -> List[AIMessage]:
        with open(self.json_path, "r") as f:
            data = json.load(f)

        responses = []
        for item in data.get("responses", []):
            if isinstance(item, str):
                responses.append(AIMessage(content=item))
            else:
                # Reconstruct AIMessage from dict
                msg = AIMessage(
                    content=item.get("content", ""), tool_calls=item.get("tool_calls", [])
                )
                responses.append(msg)
        return responses

    def _save_responses_to_json(self):
        if not self.json_path:
            return

        self.json_path.parent.mkdir(parents=True, exist_ok=True)

        data = {
            "responses": [
                {"content": msg.content, "tool_calls": getattr(msg, "tool_calls", [])}
                if isinstance(msg, AIMessage)
                else msg
                for msg in self.recorded_messages
            ]
        }

        with open(self.json_path, "w") as f:
            json.dump(data, f, indent=2, default=str)

    def _call(
        self,
        messages: List[BaseMessage],
        stop: Optional[List[str]] = None,
        run_manager: Optional[CallbackManagerForLLMRun] = None,
        **kwargs: Any,
    ) -> str:
        """Not used in _generate."""
        return ""

    def _generate(
        self,
        messages: List[BaseMessage],
        stop: Optional[List[str]] = None,
        run_manager: Optional[CallbackManagerForLLMRun] = None,
        **kwargs: Any,
    ) -> ChatResult:
        if self.record:
            # Recording mode - use real model and save responses
            if not self.real_model:
                raise ValueError("Real model not initialized for recording")

            # Bind tools if needed
            model = self.real_model
            if self._bound_tools:
                model = model.bind_tools(self._bound_tools)

            result = model.invoke(messages)
            self.recorded_messages.append(result)
            self._save_responses_to_json()

            generation = ChatGeneration(message=result)
            return ChatResult(generations=[generation])
        else:
            # Playback mode - use predefined responses
            if not self.responses:
                raise ValueError(f"No responses available for playback. ")

            if self.i >= len(self.responses):
                self.i = 0  # Wrap around

            response = self.responses[self.i]
            self.i += 1

            # if self.i >= len(self.responses):
            #     # Don't wrap around - stay at last response
            #     response = self.responses[-1]
            # else:
            #     response = self.responses[self.i]
            #     self.i += 1

            if isinstance(response, AIMessage):
                message = response
            else:
                message = AIMessage(content=str(response))

            generation = ChatGeneration(message=message)
            return ChatResult(generations=[generation])

    def _stream(
        self,
        messages: List[BaseMessage],
        stop: Optional[List[str]] = None,
        run_manager: Optional[CallbackManagerForLLMRun] = None,
        **kwargs: Any,
    ) -> Iterator[ChatGenerationChunk]:
        """Stream not implemented for testing."""
        result = self._generate(messages, stop, run_manager, **kwargs)
        message = result.generations[0].message
        chunk = AIMessageChunk(content=message.content)
        yield ChatGenerationChunk(message=chunk)

    def bind_tools(
        self,
        tools: Sequence[Union[dict[str, Any], type, Any]],
        *,
        tool_choice: Optional[str] = None,
        **kwargs: Any,
    ) -> Runnable:
        """Store tools and return self."""
        self._bound_tools = tools
        if self.record and self.real_model:
            # Also bind tools to the real model
            self.real_model = self.real_model.bind_tools(tools, tool_choice=tool_choice, **kwargs)
        return self

    @property
    def tools(self) -> Optional[Sequence[Any]]:
        """Get bound tools for inspection."""
        return self._bound_tools
