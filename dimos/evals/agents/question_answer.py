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

from dataclasses import dataclass
import json
from pathlib import Path
from typing import TYPE_CHECKING, Any

from dimos.evals.agents.lib.chat import Blocks, ChatAgent, single_call
from dimos.evals.types import Environment, RunningEnvironment, Trajectory

if TYPE_CHECKING:
    from dimos.memory.store.base import Store
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


@dataclass
class QuestionAnswer(ChatAgent):
    """One model call: ``agent_encode()`` of everything in the recording, then
    the instruction. At most ``frames_per_stream`` observations per stream,
    spread evenly over the stream."""

    frames_per_stream: int = 8

    def preflight(self, environment: Environment) -> None:
        """Any environment with a recording."""

    def run(self, inputs: str, env: RunningEnvironment, run_dir: Path) -> Trajectory:
        blocks = self._encode(env.recording)
        if not blocks:
            raise RuntimeError("nothing in the recording to encode; the run would be blind")
        return single_call(self, blocks, inputs, run_dir)

    def _encode(self, recording: Store) -> Blocks:
        blocks: Blocks = []
        for name, stream in recording.streams.items():
            observations = list(stream)
            if not observations:
                continue
            n = self.frames_per_stream
            if len(observations) > n:
                observations = [
                    observations[round(i * (len(observations) - 1) / (n - 1))] for i in range(n)
                ]
            t0 = observations[0].ts
            blocks.append(
                {
                    "type": "text",
                    "text": f"observations from stream {name!r} (t is seconds from the first shown):",
                }
            )
            blocks += _legend_block(observations[0])
            for obs in observations:
                blocks += _observation_blocks(obs, f"[t={obs.ts - t0:.1f}s]")
        return blocks
