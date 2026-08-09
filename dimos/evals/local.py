# Copyright 2026 Dimensional Inc.
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

"""Keyless local eval model: MoondreamVlModel behind the chat-model interface.

Lets ``EvalRunner(chat_model=MoondreamChat())`` run passive evals with zero API
keys on any GPU box. Moondream is single-image, so multi-image contexts are
tiled into one contact sheet; text blocks concatenate into the question.
"""

from __future__ import annotations

import base64
from functools import cached_property
from typing import Any

from langchain_core.callbacks import CallbackManagerForLLMRun
from langchain_core.language_models.chat_models import BaseChatModel
from langchain_core.messages import AIMessage, BaseMessage
from langchain_core.outputs import ChatGeneration, ChatResult
import numpy as np

from dimos.msgs.sensor_msgs.Image import Image


class MoondreamChat(BaseChatModel):
    """Chat-model adapter over the local moondream2 VLM (dimos MoondreamVlModel)."""

    tile_columns: int = 3

    @property
    def _llm_type(self) -> str:
        return "moondream-local"

    @cached_property
    def _vl(self) -> Any:
        from dimos.models.vl.moondream import MoondreamVlModel

        model = MoondreamVlModel()
        model.start()
        return model

    def _generate(
        self,
        messages: list[BaseMessage],
        stop: list[str] | None = None,
        run_manager: CallbackManagerForLLMRun | None = None,
        **kwargs: Any,
    ) -> ChatResult:
        texts: list[str] = []
        frames: list[np.ndarray[Any, Any]] = []
        for message in messages:
            content = message.content
            if isinstance(content, str):
                texts.append(content)
                continue
            for block in content:
                if not isinstance(block, dict):
                    texts.append(str(block))
                elif block.get("type") == "text":
                    texts.append(str(block["text"]))
                elif block.get("type") == "image_url":
                    frames.append(_decode_data_uri(str(block["image_url"]["url"])))

        image = Image.from_numpy(_tile(frames) if frames else _BLANK)
        answer = self._vl.query(image, "\n".join(texts))
        return ChatResult(generations=[ChatGeneration(message=AIMessage(content=str(answer)))])


_BLANK = np.full((64, 64, 3), 128, dtype=np.uint8)


def _decode_data_uri(uri: str) -> np.ndarray[Any, Any]:
    import cv2

    payload = uri.split(",", 1)[1]
    buffer = np.frombuffer(base64.b64decode(payload), dtype=np.uint8)
    frame = cv2.imdecode(buffer, cv2.IMREAD_COLOR)
    if frame is None:
        raise ValueError("undecodable image data URI")
    return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)


def _tile(frames: list[np.ndarray[Any, Any]], columns: int = 3) -> np.ndarray[Any, Any]:
    """Grid-tile frames into one contact sheet (moondream is single-image)."""
    if len(frames) == 1:
        return frames[0]
    height = min(f.shape[0] for f in frames)
    width = min(f.shape[1] for f in frames)
    import cv2

    resized = [cv2.resize(f, (width, height)) for f in frames]
    rows = [np.hstack(resized[i : i + columns]) for i in range(0, len(resized), columns)]
    max_w = max(r.shape[1] for r in rows)
    rows = [np.pad(r, ((0, 0), (0, max_w - r.shape[1]), (0, 0))) for r in rows]
    return np.vstack(rows)
