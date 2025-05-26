from dimos.stream.audio.abstract import (
    AbstractAudioEmitter,
    AudioEvent,
)
import numpy as np
from reactivex import Observable, create, disposable
import threading
import time

from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.stream.audio.node_simulated")


class SimulatedAudioSource(AbstractAudioEmitter):
    """Audio source that generates simulated audio for testing."""

    def __init__(
        self,
        sample_rate: int = 16000,
        frame_length: int = 1024,
        channels: int = 1,
        dtype: np.dtype = np.float32,
        frequency: float = 440.0,  # A4 note
        waveform: str = "sine",  # Type of waveform
        modulation_rate: float = 0.5,  # Modulation rate in Hz
        volume_oscillation: bool = True,  # Enable sinusoidal volume changes
        volume_oscillation_rate: float = 0.2,  # Volume oscillation rate in Hz
    ):
        """
        Initialize SimulatedAudioSource.

        Args:
            sample_rate: Audio sample rate in Hz
            frame_length: Number of samples per frame
            channels: Number of audio channels
            dtype: Data type for audio samples
            frequency: Frequency of the sine wave in Hz
            waveform: Type of waveform ("sine", "square", "triangle", "sawtooth")
            modulation_rate: Frequency modulation rate in Hz
            volume_oscillation: Whether to oscillate volume sinusoidally
            volume_oscillation_rate: Rate of volume oscillation in Hz
        """
        self.sample_rate = sample_rate
        self.frame_length = frame_length
        self.channels = channels
        self.dtype = dtype
        self.frequency = frequency
        self.waveform = waveform.lower()
        self.modulation_rate = modulation_rate
        self.volume_oscillation = volume_oscillation
        self.volume_oscillation_rate = volume_oscillation_rate
        self.phase = 0.0
        self.volume_phase = 0.0

        self._running = False
        self._thread = None

    def _generate_sine_wave(self, time_points: np.ndarray) -> np.ndarray:
        """Generate a waveform based on selected type."""
        # Generate base time points with phase
        t = time_points + self.phase

        # Add frequency modulation for more interesting sounds
        if self.modulation_rate > 0:
            # Modulate frequency between 0.5x and 1.5x the base frequency
            freq_mod = self.frequency * (
                1.0 + 0.5 * np.sin(2 * np.pi * self.modulation_rate * t)
            )
        else:
            freq_mod = np.ones_like(t) * self.frequency

        # Create phase argument for oscillators
        phase_arg = 2 * np.pi * np.cumsum(freq_mod / self.sample_rate)

        # Generate waveform based on selection
        if self.waveform == "sine":
            wave = np.sin(phase_arg)
        elif self.waveform == "square":
            wave = np.sign(np.sin(phase_arg))
        elif self.waveform == "triangle":
            wave = (
                2
                * np.abs(
                    2
                    * (
                        phase_arg / (2 * np.pi)
                        - np.floor(phase_arg / (2 * np.pi) + 0.5)
                    )
                )
                - 1
            )
        elif self.waveform == "sawtooth":
            wave = 2 * (
                phase_arg / (2 * np.pi) - np.floor(0.5 + phase_arg / (2 * np.pi))
            )
        else:
            # Default to sine wave
            wave = np.sin(phase_arg)

        # Apply sinusoidal volume oscillation if enabled
        if self.volume_oscillation:
            # Current time points for volume calculation
            vol_t = t + self.volume_phase

            # Volume oscillates between 0.0 and 1.0 using a sine wave (complete silence to full volume)
            volume_factor = 0.5 + 0.5 * np.sin(
                2 * np.pi * self.volume_oscillation_rate * vol_t
            )

            # Apply the volume factor
            wave *= volume_factor * 0.7

            # Update volume phase for next frame
            self.volume_phase += (
                time_points[-1] - time_points[0] + (time_points[1] - time_points[0])
            )

        # Update phase for next frame
        self.phase += (
            time_points[-1] - time_points[0] + (time_points[1] - time_points[0])
        )

        # Add a second channel if needed
        if self.channels == 2:
            wave = np.column_stack((wave, wave))
        elif self.channels > 2:
            wave = np.tile(wave.reshape(-1, 1), (1, self.channels))

        # Convert to int16 if needed
        if self.dtype == np.int16:
            wave = (wave * 32767).astype(np.int16)

        return wave

    def _audio_thread(self, observer, interval: float):
        """Thread function for simulated audio generation."""
        try:
            sample_index = 0
            self._running = True

            while self._running:
                # Calculate time points for this frame
                time_points = (
                    np.arange(sample_index, sample_index + self.frame_length)
                    / self.sample_rate
                )

                # Generate audio data
                audio_data = self._generate_sine_wave(time_points)

                # Create audio event
                audio_event = AudioEvent(
                    data=audio_data,
                    sample_rate=self.sample_rate,
                    timestamp=time.time(),
                    channels=self.channels,
                )

                observer.on_next(audio_event)

                # Update sample index for next frame
                sample_index += self.frame_length

                # Sleep to simulate real-time audio
                time.sleep(interval)

        except Exception as e:
            logger.error(f"Error in simulated audio thread: {e}")
            observer.on_error(e)
        finally:
            self._running = False
            observer.on_completed()

    def emit_audio(self, fps: int = 30) -> Observable:
        """
        Create an observable that emits simulated audio frames.

        Args:
            fps: Frames per second to emit

        Returns:
            Observable emitting AudioEvent objects
        """

        def on_subscribe(observer, scheduler):
            # Calculate interval based on fps
            interval = 1.0 / fps

            # Start the audio generation thread
            self._thread = threading.Thread(
                target=self._audio_thread, args=(observer, interval), daemon=True
            )
            self._thread.start()

            logger.info(
                f"Started simulated audio source: {self.sample_rate}Hz, "
                f"{self.channels} channels, {self.frame_length} samples per frame"
            )

            # Return a disposable to clean up
            def dispose():
                logger.info("Stopping simulated audio")
                self._running = False
                if self._thread and self._thread.is_alive():
                    self._thread.join(timeout=1.0)

            return disposable.Disposable(dispose)

        return create(on_subscribe)


if __name__ == "__main__":
    from dimos.stream.audio.utils import keepalive
    from dimos.stream.audio.node_volume_monitor import monitor
    from dimos.stream.audio.node_output import SounddeviceAudioOutput

    source = SimulatedAudioSource()
    speaker = SounddeviceAudioOutput()
    speaker.consume_audio(source.emit_audio())
    monitor(speaker.emit_audio())

    keepalive()
