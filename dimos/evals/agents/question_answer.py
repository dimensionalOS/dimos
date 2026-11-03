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
from dimos.msgs.sensor_msgs.Image import Image

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


def _is_camera_observation(obs: Observation[Any]) -> bool:
    """Camera wording applies only to image streams, not odom / lidar / etc."""
    return isinstance(obs.data, Image)


class QuestionAnswerConfig(SingleCallAgentConfig):
    frames_per_stream: int = Field(default=3, ge=1)
    system_prompt: str = (
        "Answer using only the provided observations. "
        "When multi-frame camera clips are shown, earlier frames are history and the last frame is "
        "question time; judge spatial relations in the robot body frame at that last frame. "
        "Copy one Choice token exactly as written (e.g. left not turn left). Do not refuse."
    )


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
            multi = len(observations) > 1
            camera = _is_camera_observation(observations[0])
            if camera:
                header = (
                    f"observations from stream {stream.name!r}: robot forward-camera frames "
                    f"in time order (t is absolute recording time)."
                )
                if multi:
                    header += (
                        " The last frame below is question time (current robot pose); "
                        "answer spatial questions relative to that frame."
                    )
            else:
                header = (
                    f"observations from stream {stream.name!r} "
                    f"(t is absolute recording time):"
                )
            blocks.append({"type": "text", "text": header})
            blocks += _legend_block(observations[0])
            last_i = len(observations) - 1
            for i, obs in enumerate(observations):
                stamp = f"[t={obs.ts:.3f}s]"
                if camera:
                    stamp += " robot forward-camera"
                    if multi and i == last_i:
                        stamp += " at question time (current pose)"
                    elif multi:
                        stamp += " (earlier in the clip)"
                blocks += _observation_blocks(obs, stamp)
        if not blocks:
            raise ValueError("nothing in the recording to encode; the run would be blind")
        return blocks
