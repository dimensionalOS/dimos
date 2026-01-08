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

"""GStreamer integration for audio pipeline."""

import atexit
import threading
from typing import Any, Optional, Union

import gi

gi.require_version("Gst", "1.0")
from gi.repository import GLib, Gst
from pydantic import BaseModel, Field, field_validator

from dimos.stream.audio2.types import AudioFormat, AudioSpec
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.stream.audio2.gstreamer")


# Global GLib MainLoop management
_mainloop_lock = threading.Lock()
_mainloop_thread = None
_mainloop = None
_mainloop_refs = 0


def ensure_gstreamer_init():
    """Initialize GStreamer if not already initialized."""
    if not Gst.is_initialized():
        Gst.init(None)


def ensure_mainloop_running():
    """Ensure the GLib MainLoop is running for GStreamer.

    This uses reference counting so multiple components can share
    the same MainLoop safely.
    """
    global _mainloop_thread, _mainloop, _mainloop_refs

    with _mainloop_lock:
        if _mainloop_refs == 0:
            # First user - start the mainloop
            ensure_gstreamer_init()

            _mainloop = GLib.MainLoop()

            def run_mainloop():
                logger.debug("Starting GLib MainLoop thread")
                try:
                    _mainloop.run()
                except Exception as e:
                    logger.error(f"MainLoop error: {e}")
                finally:
                    logger.debug("GLib MainLoop thread stopped")

            _mainloop_thread = threading.Thread(
                target=run_mainloop, daemon=True, name="GStreamerMainLoop"
            )
            _mainloop_thread.start()
            logger.info("Started GStreamer MainLoop")

        _mainloop_refs += 1
        logger.info(f"MainLoop reference acquired: {_mainloop_refs}")


def release_mainloop():
    """Release a reference to the mainloop.

    When the last reference is released, the mainloop stops.
    """
    global _mainloop_thread, _mainloop, _mainloop_refs

    import threading
    import traceback

    calling_thread = threading.current_thread().name

    with _mainloop_lock:
        if _mainloop_refs > 0:
            _mainloop_refs -= 1
            logger.info(
                f"MainLoop reference released by {calling_thread}: {_mainloop_refs} remaining"
            )

            if _mainloop_refs == 0 and _mainloop:
                # Last user - stop the mainloop
                logger.info(
                    f"Stopping GStreamer MainLoop (last reference released by {calling_thread})"
                )
                logger.debug(f"Release stack trace:\n{''.join(traceback.format_stack())}")
                _mainloop.quit()

                # Wait for the thread to finish to ensure clean shutdown
                # But don't try to join if we're in the MainLoop thread itself
                if (
                    _mainloop_thread
                    and _mainloop_thread.is_alive()
                    and threading.current_thread() != _mainloop_thread
                ):
                    _mainloop_thread.join(timeout=2.0)
                    if _mainloop_thread.is_alive():
                        logger.warning("MainLoop thread did not stop cleanly")

                _mainloop = None
                _mainloop_thread = None
        else:
            logger.warning(
                f"Trying to release mainloop with no references (called by {calling_thread})"
            )


def _cleanup_mainloop_at_exit():
    """Ensure mainloop is stopped at exit."""
    global _mainloop_refs, _mainloop

    with _mainloop_lock:
        if _mainloop_refs > 0:
            logger.debug(f"Cleaning up GStreamer MainLoop at exit (refs={_mainloop_refs})")
            _mainloop_refs = 0
            if _mainloop and _mainloop.is_running():
                _mainloop.quit()


# Register cleanup function
atexit.register(_cleanup_mainloop_at_exit)


class GStreamerNodeConfig(BaseModel):
    """Configuration for GStreamer nodes with format control.

    Default output is raw PCM F32LE - the standard internal format for passing audio
    between pipeline stages. Compressed formats (Vorbis, Opus) need container/framing
    headers to work through appsrc→decodebin, making raw audio more reliable for
    internal use. Compression should happen at final outputs (network, file).

    The output field accepts either:
    - AudioSpec object: AudioSpec(format=AudioFormat.PCM_F32LE, sample_rate=48000, channels=2)
    - String shortcuts: "raw", "vorbis", "opus", "mp3", "flac", etc.
    - AudioFormat enum names: "PCM_F32LE", "PCM_S16LE", "OPUS", etc.
    """

    output: Union[str, AudioSpec] = Field(
        default_factory=lambda: AudioSpec(format=AudioFormat.PCM_F32LE),
        description="Output audio specification (str or AudioSpec)",
    )
    properties: dict[str, Any] = Field(
        default_factory=dict, description="GStreamer element properties"
    )
    buffer_time: Optional[int] = Field(
        default=None, description="Buffer time in microseconds (None = auto)"
    )
    latency_time: Optional[int] = Field(
        default=None, description="Latency time in microseconds (None = auto)"
    )

    @field_validator("output", mode="before")
    @classmethod
    def convert_output_string(cls, v):
        """Convert string shortcuts to AudioSpec."""
        if isinstance(v, str):
            # Shortcut mappings
            shortcuts = {
                "raw": AudioFormat.PCM_F32LE,
                "vorbis": AudioFormat.VORBIS,
                "opus": AudioFormat.OPUS,
                "mp3": AudioFormat.MP3,
                "aac": AudioFormat.AAC,
                "flac": AudioFormat.FLAC,
                "webm": AudioFormat.WEBM,
            }

            # Try lowercase shortcut first
            lower = v.lower()
            if lower in shortcuts:
                return AudioSpec(format=shortcuts[lower])

            # Try as AudioFormat enum name (e.g., "PCM_F32LE", "VORBIS")
            try:
                audio_format = AudioFormat[v.upper()]
                return AudioSpec(format=audio_format)
            except KeyError:
                pass

            # Try as AudioFormat enum value (e.g., "S16LE", "audio/mpeg")
            try:
                audio_format = AudioFormat(v)
                return AudioSpec(format=audio_format)
            except ValueError:
                raise ValueError(
                    f"Invalid output format: '{v}'. "
                    f"Use shortcuts like 'raw', 'vorbis', 'opus', or AudioFormat enum names."
                )

        return v

    class Config:
        """Pydantic config."""

        arbitrary_types_allowed = True  # Allow AudioSpec and other custom types

    def get_caps_string(self) -> str:
        """Get GStreamer caps string for the output format."""
        return self.output.to_gst_caps_string()

    def get_encoder_string(self) -> Optional[str]:
        """Get GStreamer encoder element for the output format.

        Returns None for raw formats.
        """
        encoders = {
            AudioFormat.OPUS: "opusenc bitrate=64000",  # 64kbps - good quality, low CPU
            AudioFormat.MP3: "lamemp3enc target=bitrate bitrate=128",
            AudioFormat.AAC: "faac bitrate=96000",
            AudioFormat.VORBIS: "vorbisenc quality=0.3",
            AudioFormat.FLAC: "flacenc",  # Lossless
        }

        if self.output.format.is_raw:
            return None

        encoder = encoders.get(self.output.format)
        if not encoder:
            raise ValueError(f"No encoder configured for format: {self.output.format}")

        return encoder


class GStreamerPipelineBase:
    """Base class for GStreamer pipeline components.

    Handles mainloop lifecycle and common pipeline operations.
    """

    def __init__(self):
        self._pipeline = None
        self._running = False
        self._mainloop_acquired = False

    def _ensure_pipeline_ready(self):
        """Ensure mainloop is running and pipeline is created."""
        if not self._running:
            logger.debug(f"{self.__class__.__name__}: Ensuring pipeline ready")
            ensure_mainloop_running()
            self._running = True
            self._mainloop_acquired = True

    def _cleanup_pipeline(self):
        """Clean up pipeline and release mainloop reference."""
        logger.debug(
            f"{self.__class__.__name__}: Cleaning up pipeline (running={self._running}, mainloop_acquired={self._mainloop_acquired})"
        )
        if self._pipeline:
            self._pipeline.set_state(Gst.State.NULL)
            self._pipeline = None

        if self._mainloop_acquired:
            logger.debug(f"{self.__class__.__name__}: Releasing mainloop reference")
            release_mainloop()
            self._mainloop_acquired = False
        else:
            logger.debug(f"{self.__class__.__name__}: No mainloop reference to release")

        self._running = False

    def _on_bus_message(self, bus, message):
        """Handle messages from the GStreamer bus.

        Subclasses can override this for custom message handling.
        """
        # Debug log all message types
        if message.type != Gst.MessageType.STATE_CHANGED:  # Skip noisy state change messages
            logger.debug(
                f"Bus message: {message.type} from {message.src.get_name() if message.src else 'unknown'}"
            )

        if message.type == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            logger.error(f"Pipeline error: {err}, {debug}")
            self._handle_error(err)
        elif message.type == Gst.MessageType.WARNING:
            err, debug = message.parse_warning()
            logger.warning(f"Pipeline warning: {err}, {debug}")
        elif message.type == Gst.MessageType.EOS:
            logger.info(
                f"End of stream received from {message.src.get_name() if message.src else 'unknown'}"
            )
            self._handle_eos()

    def _handle_error(self, error):
        """Handle pipeline errors. Override in subclasses."""
        pass

    def _handle_eos(self):
        """Handle end of stream. Override in subclasses."""
        pass

    def _setup_bus(self, pipeline):
        """Set up message bus for the pipeline."""
        bus = pipeline.get_bus()
        # Add signal watch - this processes messages in the GLib MainLoop
        bus.add_signal_watch()
        bus.connect("message", self._on_bus_message)

    def __del__(self):
        """Ensure cleanup on deletion."""
        self._cleanup_pipeline()


# Common GStreamer pipeline strings for audio processing
PIPELINE_TEMPLATES = {
    "microphone": "autoaudiosrc ! audioconvert ! audioresample",
    "test_tone": "audiotestsrc wave=sine freq=440",
    "file_input": "filesrc location={path} ! decodebin ! audioconvert ! audioresample",
    "network_input": "udpsrc port={port} ! application/x-rtp ! rtpopusdepay ! opusdec",
    # Output templates
    "speaker": "audioconvert ! audioresample ! autoaudiosink",
    "file_output": "audioconvert ! audioresample ! {encoder} ! filesink location={path}",
    "network_output": "audioconvert ! audioresample ! opusenc ! rtpopuspay ! udpsink host={host} port={port}",
}


def create_caps_filter(sample_rate: int, channels: int, format_str: str) -> str:
    """Create a GStreamer caps filter string."""
    return (
        f"audio/x-raw,format={format_str},rate={sample_rate},channels={channels},layout=interleaved"
    )
