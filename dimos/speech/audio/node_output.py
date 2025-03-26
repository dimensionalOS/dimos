#!/usr/bin/env python3
from typing import Optional, List, Dict, Any
import numpy as np
import time
import logging
import sounddevice as sd
from reactivex import Observable, create, disposable

from audio.abstract import AbstractAudioTransform, AudioEvent

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class SounddeviceAudioOutput(AbstractAudioTransform):
    """
    Audio output implementation using the sounddevice library.
    
    This class implements AbstractAudioTransform so it can both play audio and
    optionally pass audio events through to other components (for example, to
    record audio while playing it, or to visualize the waveform while playing).
    """

    def __init__(
        self,
        device_index: Optional[int] = None,
        sample_rate: int = 16000,
        channels: int = 1,
        block_size: int = 1024,
        dtype: np.dtype = np.float32,
    ):
        """
        Initialize SounddeviceAudioOutput.

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
        self._subscription = None
        self.audio_observable = None

    def consume_audio(self, audio_observable: Observable) -> 'SounddeviceAudioOutput':
        """
        Subscribe to an audio observable and play the audio through the speakers.

        Args:
            audio_observable: Observable emitting AudioEvent objects
        
        Returns:
            Self for method chaining
        """
        self.audio_observable = audio_observable
        
        # Create and start the output stream
        try:
            self._stream = sd.OutputStream(
                device=self.device_index,
                samplerate=self.sample_rate,
                channels=self.channels,
                blocksize=self.block_size,
                dtype=self.dtype,
            )
            self._stream.start()
            self._running = True

            logger.info(
                f"Started audio output: {self.sample_rate}Hz, "
                f"{self.channels} channels, {self.block_size} samples per frame"
            )

        except Exception as e:
            logger.error(f"Error starting audio output stream: {e}")
            raise e

        # Subscribe to the observable
        self._subscription = audio_observable.subscribe(
            on_next=self._play_audio_event,
            on_error=self._handle_error,
            on_completed=self._handle_completion
        )
        
        return self
    
    def emit_audio(self) -> Observable:
        """
        Pass through the audio observable to allow chaining with other components.
        
        Returns:
            The same Observable that was provided to consume_audio
        """
        if self.audio_observable is None:
            raise ValueError("No audio source provided. Call consume_audio() first.")
            
        return self.audio_observable
        
    def stop(self):
        """Stop audio output and clean up resources."""
        logger.info("Stopping audio output")
        self._running = False
        
        if self._subscription:
            self._subscription.dispose()
            self._subscription = None
            
        if self._stream:
            self._stream.stop()
            self._stream.close()
            self._stream = None

    def _play_audio_event(self, audio_event):
        """Play audio from an AudioEvent."""
        if not self._running or not self._stream:
            return

        try:
            # Ensure data type matches our stream
            if audio_event.dtype != self.dtype:
                if self.dtype == np.float32:
                    audio_event = audio_event.to_float32()
                elif self.dtype == np.int16:
                    audio_event = audio_event.to_int16()

            # Write audio data to the stream
            self._stream.write(audio_event.data)
        except Exception as e:
            logger.error(f"Error playing audio: {e}")

    def _handle_error(self, error):
        """Handle errors from the observable."""
        logger.error(f"Error in audio observable: {error}")

    def _handle_completion(self):
        """Handle completion of the observable."""
        logger.info("Audio observable completed")
        self._running = False
        if self._stream:
            self._stream.stop()
            self._stream.close()
            self._stream = None

    def get_available_devices(self) -> List[Dict[str, Any]]:
        """Get a list of available audio output devices."""
        return sd.query_devices()


if __name__ == "__main__":
    from audio.node_microphone import SounddeviceAudioSource
    from audio.node_normalizer import AudioNormalizer
    from utils import keepalive

    # Create microphone source, normalizer and audio output
    mic = SounddeviceAudioSource()
    normalizer = AudioNormalizer()
    speaker = SounddeviceAudioOutput()

    # Connect the components in a pipeline
    normalizer.consume_audio(mic.emit_audio())
    speaker.consume_audio(normalizer.emit_audio())

    keepalive()
