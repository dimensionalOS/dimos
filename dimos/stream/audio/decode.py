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

"""Browser audio container -> PCM AudioEvent, via ffmpeg.

The one decoder for recorded uploads: ffmpeg sniffs whatever container the
browser produced (webm/opus, ogg, mp4/aac, wav) and absorbs the
MediaRecorder variance, so callers never transcode client-side.
"""

import io
import shutil
import time

import ffmpeg  # type: ignore[import-untyped]
import numpy as np
import soundfile as sf  # type: ignore[import-untyped]

from dimos.stream.audio.base import AudioEvent
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def ffmpeg_requirement() -> str | None:
    """Return a startup error when the ffmpeg executable is unavailable."""
    if shutil.which("ffmpeg") is None:
        return "ffmpeg is required for browser audio; install the ffmpeg system package"
    return None


def decode_audio_bytes(raw: bytes) -> AudioEvent | None:
    """One whole recording -> a mono 16-kHz float32 AudioEvent (the shape
    WhisperNode assumes), or None when ffmpeg cannot decode it."""
    try:
        out, _ = (
            ffmpeg.input("pipe:0")
            .output(
                "pipe:1",
                format="wav",
                acodec="pcm_s16le",
                ac=1,
                ar="16000",
                loglevel="quiet",
            )
            .run(input=raw, capture_stdout=True, capture_stderr=True)
        )
        audio, sample_rate = sf.read(io.BytesIO(out), dtype="float32")
    except Exception:
        logger.exception("audio decode failed")
        return None
    if audio.ndim > 1:
        audio = audio[:, 0]
    return AudioEvent(
        data=np.asarray(audio), sample_rate=sample_rate, timestamp=time.time(), channels=1
    )
