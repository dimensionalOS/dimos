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

"""One model call over the encoded recording. The only place in
:mod:`dimos.evals` that calls ``agent_encode()`` — the surface under test."""

from __future__ import annotations

import json
from typing import TYPE_CHECKING, Any

from pydantic import Field

from dimos.evals.agents.lib.single_call import (
    Blocks,
    SingleCallAgent,
    SingleCallAgentConfig,
)
from dimos.evals.types import RunningEnvironment

if TYPE_CHECKING:
    from dimos.memory.type.observation import Observation


def _observation_blocks(obs: Observation[Any], stamp: str) -> Blocks:
    """One observation as ``agent_encode()`` renders it. ``str(data)`` where a
    type has no encoder: an encoding gap the eval surfaces by design."""
    data = obs.data
    encoded = data.agent_encode() if hasattr(data, "agent_encode") else None
    if isinstance(encoded, list):  # e.g. Image -> image_url blocks
        return [{"type": "text", "text": stamp}, *encoded]
    if encoded is not None:  # e.g. PointCloud2 -> dict
        return [{"type": "text", "text": f"{stamp} {json.dumps(encoded, default=str)}"}]
    return [{"type": "text", "text": f"{stamp} {data}"}]


def _legend_block(obs: Observation[Any]) -> Blocks:
    """A type that describes its encoding once, as a class constant, gets that
    description delivered once rather than per frame."""
    legend = getattr(type(obs.data), "AGENT_ENCODE_LEGEND", None)
    return [{"type": "text", "text": f"format: {legend}"}] if isinstance(legend, str) else []


class QuestionAnswerConfig(SingleCallAgentConfig):
    frames_per_stream: int = Field(default=8, ge=1)


class QuestionAnswer(SingleCallAgent):
    """One model call: ``agent_encode()`` of everything in the recording, then
    the instruction. At most ``frames_per_stream`` observations per stream,
    spread evenly over the stream."""

    config: QuestionAnswerConfig

    def _observation_blocks(self, env: RunningEnvironment) -> Blocks:
        blocks: Blocks = []
        for stream in env.streams:
            observations = list(stream)
            if not observations:
                continue
            n = self.config.frames_per_stream
            if n == 1:
                observations = observations[:1]
            elif len(observations) > n:
                observations = [
                    observations[round(i * (len(observations) - 1) / (n - 1))] for i in range(n)
                ]
            t0 = observations[0].ts
            blocks.append(
                {
                    "type": "text",
                    "text": f"observations from stream {stream.name!r} (t is seconds from the first shown):",
                }
            )
            blocks += _legend_block(observations[0])
            for obs in observations:
                blocks += _observation_blocks(obs, f"[t={obs.ts - t0:.1f}s]")
        if not blocks:
            raise ValueError("nothing in the recording to encode; the run would be blind")
        return blocks
