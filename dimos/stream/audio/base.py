from abc import ABC, abstractmethod
from reactivex import Observable
import numpy as np


class AbstractAudioEmitter(ABC):
    """Base class for components that emit audio."""

    @abstractmethod
    def emit_audio(self) -> Observable:
        """Create an observable that emits audio frames.

        Returns:
            Observable emitting audio frames
        """
        pass


class AbstractAudioConsumer(ABC):
    """Base class for components that consume audio."""

    @abstractmethod
    def consume_audio(self, audio_observable: Observable) -> "AbstractAudioConsumer":
        """Set the audio observable to consume.

        Args:
            audio_observable: Observable emitting audio frames

        Returns:
            Self for method chaining
        """
        pass


class AbstractAudioTransform(AbstractAudioConsumer, AbstractAudioEmitter):
    """Base class for components that both consume and emit audio.

    This represents a transform in an audio processing pipeline.
    """

    pass


class AudioEvent:
    """Class to represent an audio frame event with metadata."""

    def __init__(
        self, data: np.ndarray, sample_rate: int, timestamp: float, channels: int = 1
    ):
        """
        Initialize an AudioEvent.

        Args:
            data: Audio data as numpy array
            sample_rate: Audio sample rate in Hz
            timestamp: Unix timestamp when the audio was captured
            channels: Number of audio channels
        """
        self.data = data
        self.sample_rate = sample_rate
        self.timestamp = timestamp
        self.channels = channels
        self.dtype = data.dtype
        self.shape = data.shape

    def to_float32(self) -> "AudioEvent":
        """Convert audio data to float32 format normalized to [-1.0, 1.0]."""
        if self.data.dtype == np.float32:
            return self

        new_data = self.data.astype(np.float32)
        if self.data.dtype == np.int16:
            new_data /= 32768.0

        return AudioEvent(
            data=new_data,
            sample_rate=self.sample_rate,
            timestamp=self.timestamp,
            channels=self.channels,
        )

    def to_int16(self) -> "AudioEvent":
        """Convert audio data to int16 format."""
        if self.data.dtype == np.int16:
            return self

        new_data = self.data
        if self.data.dtype == np.float32:
            new_data = (new_data * 32767).astype(np.int16)

        return AudioEvent(
            data=new_data,
            sample_rate=self.sample_rate,
            timestamp=self.timestamp,
            channels=self.channels,
        )

    def __repr__(self) -> str:
        return (
            f"AudioEvent(shape={self.shape}, dtype={self.dtype}, "
            f"sample_rate={self.sample_rate}, channels={self.channels})"
        )
