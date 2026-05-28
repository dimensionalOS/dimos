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

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Literal


@dataclass(frozen=True)
class EncodedImageArtifact:
    """Small image payload that can safely cross the RPC/MCP boundary."""

    data: str
    mime_type: Literal["image/jpeg", "image/png"] = "image/jpeg"
    width: int | None = None
    height: int | None = None
    frame_id: str = ""

    def agent_encode(self) -> list[dict[str, Any]]:
        return [
            {
                "type": "image_url",
                "image_url": {"url": f"data:{self.mime_type};base64,{self.data}"},
            }
        ]

    def __str__(self) -> str:
        size = len(self.data)
        dims = "unknown"
        if self.width is not None and self.height is not None:
            dims = f"{self.width}x{self.height}"
        return (
            f"EncodedImageArtifact(mime_type={self.mime_type}, "
            f"dims={dims}, base64_chars={size}, frame_id='{self.frame_id}')"
        )
