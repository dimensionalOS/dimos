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

"""File input source for audio pipeline."""

from pathlib import Path

import gi
from pydantic import Field, validator

gi.require_version("Gst", "1.0")
from gi.repository import Gst

from reactivex import Observable

from dimos.stream.audio2.base import GStreamerSourceBase
from dimos.stream.audio2.gstreamer import GStreamerNodeConfig
from dimos.stream.audio2.types import AudioEvent
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.stream.audio2.input.file")


class FileInputConfig(GStreamerNodeConfig):
    """Configuration for file input."""

    file_path: str = Field(description="Path to audio file")
    loop: bool = Field(default=False, description="Whether to loop the file")
    realtime: bool = Field(
        default=True, description="Emit audio at real-time speed instead of as fast as possible"
    )

    @validator("file_path")
    def validate_file_exists(cls, v):
        """Validate that the file exists."""
        if not Path(v).exists():
            raise ValueError(f"Audio file not found: {v}")
        return v


class FileInputNode(GStreamerSourceBase):
    """GStreamer-based file input that emits AudioEvents."""

    def __init__(self, config: FileInputConfig):
        super().__init__(config)
        self.config: FileInputConfig = config  # Type hint for better IDE support

    def _get_pipeline_string(self) -> str:
        """Get the file source pipeline string."""
        # Basic pipeline: filesrc -> decodebin
        pipeline = f"filesrc location={self.config.file_path} ! decodebin"

        # For realtime playback, we'll rely on the appsink sync property
        # The decodebin will connect directly to appsink via dynamic pads
        return pipeline

    def _get_source_name(self) -> str:
        """Get a descriptive name including the file."""
        return f"FileInput[{Path(self.config.file_path).name}]"

    def _create_pipeline(self):
        """Create pipeline and ensure it has a clock for realtime playback."""
        super()._create_pipeline()

        # For realtime playback, ensure the pipeline uses the system clock
        if self.config.realtime:
            # Force the pipeline to use the system clock
            clock = Gst.SystemClock.obtain()
            self._pipeline.use_clock(clock)
            logger.info(f"{self._get_source_name()}: Using system clock for realtime playback")

    def _configure_appsink(self):
        """Configure appsink with realtime option."""
        # Call base implementation with realtime parameter
        super()._configure_appsink(sync=self.config.realtime)

    def _handle_eos(self):
        """Handle end of stream with looping support."""
        if self.config.loop:
            # Seek back to beginning
            logger.info(f"{self._get_source_name()}: Looping")
            self._pipeline.seek_simple(
                Gst.Format.TIME, Gst.SeekFlags.FLUSH | Gst.SeekFlags.KEY_UNIT, 0
            )
        else:
            # Use parent implementation for completion
            super()._handle_eos()


def file_input(file_path: str, **kwargs) -> Observable[AudioEvent]:
    """Create a file input source.

    Args:
        file_path: Path to audio file
        **kwargs: Additional arguments passed to FileInputConfig:
            - loop: Whether to loop the file (default: False)
            - realtime: Emit at real-time speed (default: True)
            - output: Output audio specification (default: Vorbis compressed)
            - properties: GStreamer element properties

    Returns:
        Observable that emits AudioEvents
    """
    config = FileInputConfig(file_path=file_path, **kwargs)
    return FileInputNode(config).create_observable()
