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

"""Construct generated video-health messages from browser JSON."""

import time
from typing import Any

from dimos.message_utils import time as message_time
from dimos.messages.dimos_msgs.msg import VideoStats
from dimos.messages.std_msgs.msg import Header


def from_dict(d: dict[str, Any]) -> VideoStats:
    """Accept missing and null browser metrics before getStats has populated."""
    return VideoStats(
        header=Header(
            stamp=message_time.time_from_seconds(float(d.get("ts") or time.time())),
            frame_id=str(d.get("frame_id") or "video"),
        ),
        fps=float(d.get("fps") or 0.0),
        kbps=float(d.get("kbps") or 0.0),
        width=int(d.get("width") or 0),
        height=int(d.get("height") or 0),
        loss_pct=float(d.get("loss_pct") or 0.0),
        jitter_buffer_ms=float(d.get("jitter_buffer_ms") or 0.0),
        decode_ms=float(d.get("decode_ms") or 0.0),
        frames_dropped=int(d.get("frames_dropped") or 0),
        freezes=int(d.get("freezes") or 0),
        e2e_latency_ms=float(d.get("e2e_latency_ms") or 0.0),
    )
