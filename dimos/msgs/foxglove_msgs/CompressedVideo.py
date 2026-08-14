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

"""foxglove_msgs::msg::CompressedVideo — one encoded video packet.

Just the encoded bytes + codec name. Inter-frame codecs (h264) can't be decoded
one packet at a time — feed the ordered stream through
:class:`~dimos.robot.unitree.go2.dds.video.H264Decoder` to get ``Image`` frames,
or let the rerun bridge log it as a ``VideoStream`` and decode in-viewer.
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING

from dimos_lcm.foxglove_msgs import CompressedVideo as LCMCompressedVideo
import numpy as np

from dimos.types.timestamped import Timestamped

if TYPE_CHECKING:
    from dimos.visualization.rerun.bridge import RerunData


class CompressedVideo(Timestamped):
    """One encoded video packet (Annex-B for h264)."""

    msg_name = "foxglove_msgs.CompressedVideo"

    def __init__(
        self,
        data: np.ndarray | bytes,
        format: str = "h264",
        frame_id: str = "",
        ts: float | None = None,
    ) -> None:
        """Initialize a CompressedVideo.

        Args:
            data: The encoded packet, u8
            format: Codec name, e.g. "h264"
            frame_id: Frame ID
            ts: Timestamp in seconds
        """
        self.data = data if isinstance(data, np.ndarray) else np.frombuffer(data, dtype=np.uint8)
        self.format = format
        self.frame_id = frame_id
        self.ts = ts if ts is not None else time.time()

    def lcm_encode(self) -> bytes:
        msg = LCMCompressedVideo()
        msg.timestamp.sec = int(self.ts)
        msg.timestamp.nanosec = int((self.ts - int(self.ts)) * 1e9)
        msg.frame_id = self.frame_id
        msg.data = self.data.tobytes()
        msg.data_length = len(msg.data)
        msg.format = self.format
        return msg.lcm_encode()  # type: ignore[no-any-return]

    @classmethod
    def lcm_decode(cls, data: bytes) -> CompressedVideo:
        msg = LCMCompressedVideo.lcm_decode(data)
        return cls(
            data=bytes(msg.data),
            format=msg.format,
            frame_id=msg.frame_id,
            ts=msg.timestamp.sec + msg.timestamp.nanosec * 1e-9,
        )

    def to_rerun(self) -> RerunData:
        """Log the encoded packet as a rerun ``VideoStream`` sample (viewer decodes).

        rerun decodes the stream in-viewer, so this stays per-packet and cheap —
        no server-side decode, and the .rrd holds the compressed bytes. Iterate
        from the start (or a keyframe) so the first sample the viewer sees is one.
        """
        import rerun as rr

        codecs = {
            "h264": rr.VideoCodec.H264,
            "h265": rr.VideoCodec.H265,
            "av1": rr.VideoCodec.AV1,
            "vp8": rr.VideoCodec.VP8,
            "vp9": rr.VideoCodec.VP9,
        }
        codec = codecs.get(self.format.lower())
        if codec is None:
            raise ValueError(f"no rerun VideoCodec for format {self.format!r}")
        return rr.VideoStream(codec, sample=self.data.tobytes())

    def __repr__(self) -> str:
        return (
            f"CompressedVideo(format='{self.format}', bytes={self.data.size}, "
            f"frame_id='{self.frame_id}', ts={self.ts})"
        )
