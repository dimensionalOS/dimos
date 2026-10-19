# Copyright 2025-2026 Dimensional Inc.
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

"""Convert browser video-health JSON into an explicit generated CDR message."""

from typing import Any

from dimos_generated.dimos_msgs.msg import VideoStats
from dimos_generated.std_msgs.msg import Header

from dimos.msgs.time import header_now, time_from_seconds


def _unsigned_metric(payload: dict[str, Any], field: str, bits: int) -> int:
    value = int(payload.get(field) or 0)
    if not 0 <= value < 1 << bits:
        raise ValueError(f"{field} must fit uint{bits}")
    return value


def video_stats_from_dict(payload: dict[str, Any]) -> VideoStats:
    """Translate browser getStats fields; null/missing metrics use zero defaults."""
    timestamp = payload.get("ts")
    frame = str(payload.get("frame_id") or "video")
    header = (
        header_now(frame)
        if timestamp is None
        else Header(stamp=time_from_seconds(float(timestamp)), frame_id=frame)
    )
    return VideoStats(
        header=header,
        fps=float(payload.get("fps") or 0),
        kbps=float(payload.get("kbps") or 0),
        width=_unsigned_metric(payload, "width", 32),
        height=_unsigned_metric(payload, "height", 32),
        loss_pct=float(payload.get("loss_pct") or 0),
        jitter_buffer_ms=float(payload.get("jitter_buffer_ms") or 0),
        decode_ms=float(payload.get("decode_ms") or 0),
        frames_dropped=_unsigned_metric(payload, "frames_dropped", 64),
        freezes=_unsigned_metric(payload, "freezes", 64),
        e2e_latency_ms=float(payload.get("e2e_latency_ms") or 0),
    )
