#!/usr/bin/env python3
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

"""Microphone input source for audio pipeline."""

from typing import Optional

import gi
from pydantic import Field

gi.require_version("Gst", "1.0")

from reactivex import Observable

from dimos.stream.audio2.base import GStreamerSourceBase
from dimos.stream.audio2.gstreamer import GStreamerNodeConfig
from dimos.stream.audio2.types import AudioEvent
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.stream.audio2.input.microphone")


class MicrophoneInputConfig(GStreamerNodeConfig):
    """Configuration for microphone input."""

    device: Optional[str] = Field(
        default=None, description="Audio device name (None = default device)"
    )


class MicrophoneInputNode(GStreamerSourceBase):
    """GStreamer-based microphone input that emits AudioEvents."""

    def __init__(self, config: MicrophoneInputConfig):
        super().__init__(config)
        self.config: MicrophoneInputConfig = config  # Type hint for better IDE support

    def _get_pipeline_string(self) -> str:
        """Get the microphone source pipeline string."""
        parts = ["autoaudiosrc name=source"]

        # Apply device if specified
        if self.config.device:
            parts[0] = f"autoaudiosrc device={self.config.device} name=source"

        # Add audio conversion for format flexibility
        parts.extend(["!", "audioconvert", "!", "audioresample"])

        return " ".join(parts)

    def _get_source_name(self) -> str:
        """Get a descriptive name including device if specified."""
        device_str = f"[{self.config.device}]" if self.config.device else ""
        return f"MicrophoneInput{device_str}"

    def _configure_appsink(self):
        """Configure appsink with realtime option."""
        # Call base implementation with realtime parameter
        super()._configure_appsink(sync=True)

    def _configure_source(self):
        """Configure the audio source element with buffer/latency settings."""
        # Try to get the actual source element
        source = self._pipeline.get_by_name("source")
        if source:
            # Set buffer-time and latency-time if supported
            if self.config.buffer_time is not None:
                try:
                    source.set_property("buffer-time", self.config.buffer_time)
                    logger.debug(f"Set buffer-time={self.config.buffer_time}")
                except Exception:
                    pass  # Not all sources support this

            if self.config.latency_time is not None:
                try:
                    source.set_property("latency-time", self.config.latency_time)
                    logger.debug(f"Set latency-time={self.config.latency_time}")
                except Exception:
                    pass  # Not all sources support this

    def _create_pipeline(self):
        """Create and configure the pipeline."""
        super()._create_pipeline()
        # Configure source element after pipeline creation
        self._configure_source()


def microphone(
    device: Optional[str] = None,
    buffer_time: Optional[int] = None,
    latency_time: Optional[int] = None,
    realtime: bool = True,
    **kwargs,
) -> Observable[AudioEvent]:
    """Create a microphone input source.

    Args:
        device: Audio device name (None = default device)
        buffer_time: Buffer time in microseconds (None = auto)
        latency_time: Latency time in microseconds (None = auto)
        realtime: Emit at real-time speed (default: True for microphone)
        **kwargs: Additional arguments passed to MicrophoneInputConfig:
            - output: Output audio specification (default: Vorbis compressed)
            - properties: GStreamer element properties

    Returns:
        Observable that emits AudioEvents

    Examples:
        # Use default microphone with default settings
        microphone().subscribe(speaker())

        # Use specific device with low latency
        microphone(
            device="hw:1,0",
            buffer_time=10000,  # 10ms buffer
            latency_time=5000   # 5ms latency
        ).subscribe(speaker())

        # Get raw PCM audio from microphone
        microphone(
            output=AudioSpec(
                format=AudioFormat.PCM_F32LE,
                sample_rate=48000,
                channels=1
            )
        ).subscribe(speaker())
    """
    config = MicrophoneInputConfig(
        device=device, buffer_time=buffer_time, latency_time=latency_time, **kwargs
    )
    return MicrophoneInputNode(config).create_observable()
