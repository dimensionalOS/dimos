#!/usr/bin/env python3
from audio.abstract import AbstractAudioEmitter, AudioEvent
import numpy as np
from typing import Optional, List, Dict, Any
from reactivex import Observable, create, disposable
import time
import logging
import sounddevice as sd

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class SounddeviceAudioSource(AbstractAudioEmitter):
    """Audio source implementation using the sounddevice library."""

    def __init__(
        self,
        device_index: Optional[int] = None,
        sample_rate: int = 16000,
        channels: int = 1,
        block_size: int = 1024,
        dtype: np.dtype = np.float32,
    ):
        """
        Initialize SounddeviceAudioSource.

        Args:
            device_index: Audio device index (None for default)
            sample_rate: Audio sample rate in Hz
            channels: Number of audio channels (1=mono, 2=stereo)
            block_size: Number of samples per audio frame
            dtype: Data type for audio samples (np.float32 or np.int16)
        """
        self.device_index = device_index
        self.sample_rate = sample_rate
        self.channels = channels
        self.block_size = block_size
        self.dtype = dtype

        self._stream = None
        self._running = False

    def emit_audio(self) -> Observable:
        """
        Create an observable that emits audio frames.

        Returns:
            Observable emitting AudioEvent objects
        """

        def on_subscribe(observer, scheduler):
            # Callback function to process audio data
            def audio_callback(indata, frames, time_info, status):
                if status:
                    logger.warning(f"Audio callback status: {status}")

                # Create audio event
                audio_event = AudioEvent(
                    data=indata.copy(),
                    sample_rate=self.sample_rate,
                    timestamp=time.time(),
                    channels=self.channels,
                )

                observer.on_next(audio_event)

            # Start the audio stream
            try:
                self._stream = sd.InputStream(
                    device=self.device_index,
                    samplerate=self.sample_rate,
                    channels=self.channels,
                    blocksize=self.block_size,
                    dtype=self.dtype,
                    callback=audio_callback,
                )
                self._stream.start()
                self._running = True

                logger.info(
                    f"Started audio capture: {self.sample_rate}Hz, "
                    f"{self.channels} channels, {self.block_size} samples per frame"
                )

            except Exception as e:
                logger.error(f"Error starting audio stream: {e}")
                observer.on_error(e)

            # Return a disposable to clean up resources
            def dispose():
                logger.info("Stopping audio capture")
                self._running = False
                if self._stream:
                    self._stream.stop()
                    self._stream.close()
                    self._stream = None

            return disposable.Disposable(dispose)

        return create(on_subscribe)

    def get_available_devices(self) -> List[Dict[str, Any]]:
        """Get a list of available audio input devices."""
        return sd.query_devices()


if __name__ == "__main__":
    from audio.node_volume_monitor import monitor
    from utils import keepalive
    monitor(SounddeviceAudioSource().emit_audio())
    keepalive()
