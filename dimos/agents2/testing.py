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

from typing import Any, Dict, Iterator, List, Optional, Sequence, Union

from langchain_core.callbacks.manager import CallbackManagerForLLMRun
from langchain_core.language_models.chat_models import SimpleChatModel
from langchain_core.messages import AIMessage, AIMessageChunk, BaseMessage
from langchain_core.outputs import ChatGeneration, ChatGenerationChunk, ChatResult
from langchain_core.runnables import Runnable


class MockModel(SimpleChatModel):
    """Custom fake chat model that supports tool calls for testing."""

    responses: List[Union[str, AIMessage]] = []
    i: int = 0

    def __init__(self, **kwargs):
        # Extract responses before calling super().__init__
        responses = kwargs.pop("responses", [])
        super().__init__(**kwargs)
        self.responses = responses
        self.i = 0
        self._bound_tools: Optional[Sequence[Any]] = None

    @property
    def _llm_type(self) -> str:
        return "tool-call-fake-chat-model"

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
        """Generate a response using predefined responses."""
        if self.i >= len(self.responses):
            self.i = 0  # Wrap around

        response = self.responses[self.i]
        self.i += 1

        # Handle different response types
        if isinstance(response, AIMessage):
            message = response
        else:
            # It's a string
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
        return self

    @property
    def tools(self) -> Optional[Sequence[Any]]:
        """Get bound tools for inspection."""
        return self._bound_tools
