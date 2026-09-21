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

"""foxglove_msgs::msg::CompressedVideo_ — one encoded video packet (rt/frontvideo/h264).

Just the encoded bytes + codec name. Inter-frame codecs (h264) can't be decoded
one packet at a time — feed the ordered stream through
:class:`~dimos.robot.unitree.go2.dds.video.H264Decoder` to get ``Image`` frames.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass
class CompressedVideo:
    data: np.ndarray  # u8[], the encoded packet (Annex-B for h264)
    format: str  # codec name, e.g. "h264"
    frame_id: str

    def to_rerun(self) -> Any:
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
        }
        codec = codecs.get(self.format.lower())
        if codec is None:
            raise ValueError(f"no rerun VideoCodec for format {self.format!r}")
        return rr.VideoStream(codec, sample=self.data.tobytes())
