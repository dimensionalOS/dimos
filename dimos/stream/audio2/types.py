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

"""Type definitions for the audio pipeline system."""

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Literal, Optional, Protocol, TypeVar, Union

import numpy as np
from reactivex.observable import Observable
from reactivex import Observer
from reactivex.abc import DisposableBase


class AudioFormat(Enum):
    """Supported audio formats in the pipeline."""

    # Uncompressed formats (GStreamer audio/x-raw)
    PCM_S16LE = "S16LE"  # 16-bit signed little-endian
    PCM_S32LE = "S32LE"  # 32-bit signed little-endian
    PCM_F32LE = "F32LE"  # 32-bit float little-endian
    PCM_F64LE = "F64LE"  # 64-bit float little-endian

    # Compressed formats
    MP3 = "audio/mpeg"
    AAC = "audio/aac"
    OPUS = "audio/x-opus"
    VORBIS = "audio/x-vorbis"
    FLAC = "audio/x-flac"
    WEBM = "audio/webm"

    @property
    def is_compressed(self) -> bool:
        """Check if this is a compressed format."""
        return self.value.startswith("audio/")

    @property
    def is_raw(self) -> bool:
        """Check if this is a raw PCM format."""
        return not self.is_compressed

    def to_gst_caps_string(self) -> str:
        """Get the GStreamer caps string for this format."""
        if self.is_raw:
            return f"audio/x-raw,format={self.value}"
        else:
            return self.value


@dataclass(frozen=True)
class AudioSpec:
    """Complete audio specification including format, rate, and channels.

    This can generate complete GStreamer caps strings.
    """

    format: AudioFormat
    sample_rate: Optional[int] = None
    channels: Optional[int] = None

    def to_gst_caps_string(self) -> str:
        """Generate complete GStreamer caps string."""
        caps = self.format.to_gst_caps_string()

        if self.format.is_raw:
            # Add rate and channels for raw formats
            if self.sample_rate:
                caps += f",rate={self.sample_rate}"
            if self.channels:
                caps += f",channels={self.channels}"
            caps += ",layout=interleaved"

        return caps


@dataclass(frozen=True)
class AudioEvent:
    """
    Base class for all audio events in the pipeline.

    This is the parent type for both raw and compressed audio data.
    """

    sample_rate: int  # Sample rate in Hz
    channels: int  # Number of channels
    timestamp: float  # Unix timestamp

    @property
    def format(self) -> AudioFormat:
        """Get the audio format of this event."""
        raise NotImplementedError("Subclasses must implement format property")


@dataclass(frozen=True)
class RawAudioEvent(AudioEvent):
    """
    Raw (uncompressed) audio data event.

    This is what most transforms will work with. Data is always
    in numpy array format with shape (n_samples,) for mono or
    (n_samples, n_channels) for multi-channel audio.
    """

    data: np.ndarray  # Audio samples

    @property
    def format(self) -> AudioFormat:
        """Get the audio format based on numpy dtype."""
        if self.data.dtype == np.int16:
            return AudioFormat.PCM_S16LE
        elif self.data.dtype == np.int32:
            return AudioFormat.PCM_S32LE
        elif self.data.dtype == np.float32:
            return AudioFormat.PCM_F32LE
        elif self.data.dtype == np.float64:
            return AudioFormat.PCM_F64LE
        else:
            raise ValueError(f"Unsupported dtype: {self.data.dtype}")

    @property
    def dtype(self) -> np.dtype:
        """Get the numpy dtype of the audio data."""
        return self.data.dtype

    @property
    def duration(self) -> float:
        """Calculate duration from sample count."""
        return len(self.data) / self.sample_rate

    def to_float32(self) -> "RawAudioEvent":
        """Convert to float32 normalized to [-1.0, 1.0]."""
        if self.dtype == np.float32:
            return self

        new_data = self.data.astype(np.float32)
        if self.dtype == np.int16:
            new_data /= 32768.0
        elif self.dtype == np.int32:
            new_data /= 2147483648.0

        return RawAudioEvent(
            data=new_data,
            sample_rate=self.sample_rate,
            channels=self.channels,
            timestamp=self.timestamp,
        )

    def to_int16(self) -> "RawAudioEvent":
        """Convert to int16 format."""
        if self.dtype == np.int16:
            return self

        new_data = self.data
        if self.dtype == np.float32 or self.dtype == np.float64:
            new_data = np.clip(new_data * 32767, -32768, 32767).astype(np.int16)

        return RawAudioEvent(
            data=new_data,
            sample_rate=self.sample_rate,
            channels=self.channels,
            timestamp=self.timestamp,
        )


@dataclass(frozen=True)
class CompressedAudioEvent(AudioEvent):
    """
    Compressed audio data event.

    Used for encoded audio streams that need to be decoded or can
    be passed through directly to compatible sinks.
    """

    data: bytes  # Compressed audio data
    format_type: AudioFormat  # The compression format
    duration: Optional[float] = None  # Duration if known from metadata

    @property
    def format(self) -> AudioFormat:
        """Get the compression format."""
        return self.format_type


# Type aliases for the reactive audio pipeline
# Sources now return Observables directly (not factory functions)
AudioSource = Observable[AudioEvent]
AudioSink = Observer[AudioEvent]

# For raw audio only (pure Python transforms)
RawAudioSource = Observable[RawAudioEvent]
RawAudioSink = Observer[RawAudioEvent]

# Transform types
AudioTransform = Callable[[AudioEvent], AudioEvent]
RawAudioTransform = Callable[[RawAudioEvent], RawAudioEvent]

AudioDecoder = Callable[[AudioEvent], RawAudioEvent]  # Decodes any format to raw
AudioEncoder = Callable[[RawAudioEvent], CompressedAudioEvent]  # Encodes to compressed


class AudioSourceProtocol(Protocol):
    """Protocol for audio source factories."""

    def __call__(self) -> Observable[AudioEvent]:
        """Create an observable that emits audio events."""
        ...


class AudioTransformProtocol(Protocol):
    """Protocol for audio transform functions."""

    def __call__(self, event: RawAudioEvent) -> RawAudioEvent:
        """Transform a raw audio event."""
        ...


class AudioSinkProtocol(Protocol):
    """Protocol for audio sink observers."""

    def on_next(self, event: AudioEvent) -> None:
        """Handle the next audio event."""
        ...

    def on_error(self, error: Exception) -> None:
        """Handle an error in the stream."""
        ...

    def on_completed(self) -> None:
        """Handle stream completion."""
        ...


# Configuration types for nodes
@dataclass
class GStreamerPipelineConfig:
    """Configuration for custom GStreamer pipeline nodes."""

    pipeline: str  # GStreamer pipeline string
    caps: Optional[str] = None  # Optional caps filter
    properties: Optional[dict[str, Any]] = None  # Element properties


@dataclass
class MicrophoneConfig:
    """Configuration for microphone input."""

    device: Optional[str] = None  # None = default device
    sample_rate: int = 16000
    channels: int = 1
    chunk_size: int = 1024
    format: AudioFormat = AudioFormat.PCM_F32LE


@dataclass
class SpeakerConfig:
    """Configuration for speaker output."""

    device: Optional[str] = None  # None = default device
    buffer_size_ms: int = 200


@dataclass
class FileOutputConfig:
    """Configuration for file output."""

    file_path: str
    format: AudioFormat = AudioFormat.FLAC  # Default to lossless
    quality: Optional[float] = None  # Format-specific quality setting


@dataclass
class NormalizerConfig:
    """Configuration for audio normalization."""

    target_level: float = 0.7
    min_threshold: float = 0.01
    max_gain: float = 10.0
    adapt_speed: float = 0.05
    method: Literal["peak", "rms"] = "peak"


@dataclass
class VolumeMonitorConfig:
    """Configuration for volume monitoring."""

    method: Literal["peak", "rms"] = "peak"
    window_size: Optional[int] = None
    callback: Optional[Callable[[float], None]] = None


@dataclass
class EncoderConfig:
    """Configuration for audio encoding."""

    format: AudioFormat
    bitrate: Optional[int] = None  # In bits per second
    quality: Optional[float] = None  # 0.0-1.0 for VBR codecs


@dataclass
class DecoderConfig:
    """Configuration for audio decoding."""

    output_format: AudioFormat = AudioFormat.PCM_F32LE
    output_sample_rate: Optional[int] = None  # None = keep original


# Helper type for pipeline management
class AudioPipeline:
    """Represents a running audio pipeline that can be disposed."""

    def __init__(self, subscription: DisposableBase):
        self._subscription: Optional[DisposableBase] = subscription

    def dispose(self) -> None:
        """Stop the audio pipeline and clean up resources."""
        if self._subscription:
            self._subscription.dispose()
            self._subscription = None

    def __enter__(self) -> "AudioPipeline":
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.dispose()


# Type variable for generic audio processing
T = TypeVar("T", bound=AudioEvent)

# Union type for flexible node inputs
AudioNode = Union[AudioSource, AudioTransform, AudioSink]
