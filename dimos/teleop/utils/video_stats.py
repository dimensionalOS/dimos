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

"""Operator-side video health snapshot, sampled in the browser via getStats().

Same trick as ``Buttons`` (which rides on ``UInt32`` with bit packing): we
don't need a new LCM type, we ride on ``sensor_msgs.Joy`` with positional
``axes[]`` slots for each metric. That keeps the message wire-compatible with
the existing dimos_lcm stack while exposing named fields in Python.

This is a teleop-only construct (not a general sensor message), so it lives
beside the other teleop helpers in ``teleop/utils`` rather than in
``dimos.msgs.sensor_msgs``.
"""

from __future__ import annotations

import math
import time
from typing import BinaryIO

from dimos_lcm.sensor_msgs import Joy as LCMJoy

from dimos.types.timestamped import Timestamped


def _sec_nsec(ts: float) -> list[int]:
    s = int(ts)
    return [s, int((ts - s) * 1_000_000_000)]


# Positional layout inside the carrier's axes[] array. Add new metrics at the
# end so existing recordings stay readable.
_AXES = (
    "fps",
    "kbps",
    "width",
    "height",
    "loss_pct",
    "jitter_buffer_ms",
    "decode_ms",
    "frames_dropped",
    "freezes",
)


class VideoStats(Timestamped):
    """One snapshot of operator-side video health.

    Sampled by the operator's browser from ``pc.getStats()`` once per second
    and shipped to the robot, where ``HostedTeleopModule`` publishes it on
    an ``Out[VideoStats]`` port. The recorder captures it like any other
    typed stream — no sidecar file needed.

    All fields are floats on the wire (Joy ``axes[]`` is ``float32[]``);
    ``width``, ``height``, ``frames_dropped``, ``freezes`` are integer-valued
    but stored as floats. float32 is exact for integers up to 2^24.
    """

    msg_name = "sensor_msgs.VideoStats"

    ts: float
    frame_id: str
    fps: float
    kbps: float
    width: int
    height: int
    loss_pct: float
    jitter_buffer_ms: float
    decode_ms: float
    frames_dropped: int
    freezes: int

    def __init__(
        self,
        ts: float = 0.0,
        frame_id: str = "video",
        fps: float = 0.0,
        kbps: float = 0.0,
        width: int = 0,
        height: int = 0,
        loss_pct: float = 0.0,
        jitter_buffer_ms: float = 0.0,
        decode_ms: float = 0.0,
        frames_dropped: int = 0,
        freezes: int = 0,
    ) -> None:
        self.ts = ts if ts != 0 else time.time()
        self.frame_id = frame_id
        self.fps = float(fps)
        self.kbps = float(kbps)
        self.width = int(width)
        self.height = int(height)
        self.loss_pct = float(loss_pct)
        self.jitter_buffer_ms = float(jitter_buffer_ms)
        self.decode_ms = float(decode_ms)
        self.frames_dropped = int(frames_dropped)
        self.freezes = int(freezes)

    @classmethod
    def from_dict(cls, d: dict[str, float | int]) -> VideoStats:
        """Build from the JSON payload the browser ships (over state_reliable)."""
        return cls(
            ts=float(d.get("ts", 0.0)),
            frame_id=str(d.get("frame_id", "video")),
            fps=float(d.get("fps", 0.0)),
            kbps=float(d.get("kbps", 0.0)),
            width=int(d.get("width", 0)),
            height=int(d.get("height", 0)),
            loss_pct=float(d.get("loss_pct", 0.0)),
            jitter_buffer_ms=float(d.get("jitter_buffer_ms", 0.0)),
            decode_ms=float(d.get("decode_ms", 0.0)),
            frames_dropped=int(d.get("frames_dropped", 0)),
            freezes=int(d.get("freezes", 0)),
        )

    def _as_axes(self) -> list[float]:
        # Order must match _AXES.
        return [
            self.fps,
            self.kbps,
            float(self.width),
            float(self.height),
            self.loss_pct,
            self.jitter_buffer_ms,
            self.decode_ms,
            float(self.frames_dropped),
            float(self.freezes),
        ]

    def lcm_encode(self) -> bytes:
        lcm_msg = LCMJoy()
        [lcm_msg.header.stamp.sec, lcm_msg.header.stamp.nsec] = _sec_nsec(self.ts)
        lcm_msg.header.frame_id = self.frame_id
        axes = self._as_axes()
        lcm_msg.axes_length = len(axes)
        lcm_msg.axes = axes
        lcm_msg.buttons_length = 0
        lcm_msg.buttons = []
        return lcm_msg.lcm_encode()  # type: ignore[no-any-return]

    @classmethod
    def lcm_decode(cls, data: bytes | BinaryIO) -> VideoStats:
        lcm_msg = LCMJoy.lcm_decode(data)
        axes = list(lcm_msg.axes) if lcm_msg.axes else []
        # Pad short arrays so older recordings (fewer metrics) still decode.
        axes += [0.0] * (len(_AXES) - len(axes))
        return cls(
            ts=lcm_msg.header.stamp.sec + (lcm_msg.header.stamp.nsec / 1_000_000_000),
            frame_id=lcm_msg.header.frame_id or "video",
            fps=axes[0],
            kbps=axes[1],
            width=int(axes[2]),
            height=int(axes[3]),
            loss_pct=axes[4],
            jitter_buffer_ms=axes[5],
            decode_ms=axes[6],
            frames_dropped=int(axes[7]),
            freezes=int(axes[8]),
        )

    def __str__(self) -> str:
        return (
            f"VideoStats({self.width}x{self.height} {self.fps:.1f}fps "
            f"{self.kbps:.0f}kbps loss={self.loss_pct:.2f}% "
            f"jbuf={self.jitter_buffer_ms:.0f}ms decode={self.decode_ms:.1f}ms "
            f"dropped={self.frames_dropped} freezes={self.freezes})"
        )

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, VideoStats):
            return False
        return (
            math.isclose(self.ts, other.ts, rel_tol=1e-4, abs_tol=1e-5)
            and self.frame_id == other.frame_id
            and math.isclose(self.fps, other.fps, rel_tol=1e-4, abs_tol=1e-5)
            and math.isclose(self.kbps, other.kbps, rel_tol=1e-4, abs_tol=1e-5)
            and self.width == other.width
            and self.height == other.height
            and math.isclose(self.loss_pct, other.loss_pct, rel_tol=1e-4, abs_tol=1e-5)
            and math.isclose(
                self.jitter_buffer_ms, other.jitter_buffer_ms, rel_tol=1e-4, abs_tol=1e-5
            )
            and math.isclose(self.decode_ms, other.decode_ms, rel_tol=1e-4, abs_tol=1e-5)
            and self.frames_dropped == other.frames_dropped
            and self.freezes == other.freezes
        )


__all__ = ["VideoStats"]
