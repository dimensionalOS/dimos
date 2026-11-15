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

"""Browser audio container -> PCM AudioEvent, via PyAV.

The one decoder for recorded uploads: the FFmpeg bundled in the PyAV wheel
sniffs whatever container the browser produced (webm/opus, ogg, mp4/aac,
wav) and absorbs the MediaRecorder variance, so callers never transcode
client-side.
"""

import io
import time

import av
import numpy as np

from dimos.stream.audio.base import AudioEvent
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def decode_audio_bytes(raw: bytes) -> AudioEvent | None:
    """One whole recording -> a mono 16-kHz float32 AudioEvent (the shape
    WhisperNode assumes), or None when FFmpeg cannot decode it."""
    try:
        with av.open(io.BytesIO(raw), "r", metadata_errors="ignore") as container:
            stream = container.streams.audio[0]
            resampler = av.AudioResampler(format="s16", layout="mono", rate=16_000)
            frames: list[av.AudioFrame] = []
            for packet in container.demux(stream):
                try:
                    decoded = stream.decode(packet)
                except av.error.InvalidDataError as exc:
                    # Keep good audio on either side of a damaged packet.
                    logger.warning("skipping invalid audio packet", error=str(exc))
                    continue
                for frame in decoded:
                    frames.extend(resampler.resample(frame))
            frames += resampler.resample(None)  # flush
    except Exception:
        logger.exception("audio decode failed")
        return None
    if not frames:
        return None
    pcm = np.concatenate([frame.to_ndarray()[0] for frame in frames])
    return AudioEvent(
        data=pcm.astype(np.float32) / 32768, sample_rate=16_000, timestamp=time.time()
    )
