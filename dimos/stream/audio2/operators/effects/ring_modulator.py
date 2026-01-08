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

"""Ring modulator effect for creating robotic/metallic sounds."""

import numpy as np
from reactivex import create
from reactivex.abc import ObservableBase

from dimos.stream.audio2.types import AudioEvent, RawAudioEvent
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.stream.audio2.operators.effects.ring_modulator")


def ring_modulator(
    carrier_freq: float = 30.0,
    carrier_waveform: str = "square",
    mix: float = 0.5,
):
    """Create a ring modulator effect operator.

    Ring modulation multiplies the audio signal with a carrier wave, creating
    metallic, robotic, or bell-like timbres. This is the classic robot voice effect.

    Args:
        carrier_freq: Carrier frequency in Hz (default: 30.0)
                     Lower = deeper/growly, Higher = more metallic
                     Typical range: 20-200 Hz
        carrier_waveform: Waveform type: "sine", "square", "saw", "triangle" (default: "square")
        mix: Wet/dry mix 0.0-1.0 (default: 0.5)
             0.0 = original signal, 1.0 = fully modulated

    Returns:
        An operator function that can be used with pipe()

    Examples:
        # Classic robot voice
        file_input("voice.wav").pipe(
            ring_modulator(carrier_freq=30, carrier_waveform="square"),
            speaker()
        ).run()

        # Metallic high frequency
        file_input("voice.wav").pipe(
            ring_modulator(carrier_freq=100, carrier_waveform="sine", mix=0.8),
            speaker()
        ).run()
    """

    # Carrier phase accumulator (captured in closure)
    state = {"phase": 0.0}

    def modulate_audio(event: AudioEvent) -> AudioEvent:
        """Apply ring modulation to audio event."""
        # Only process raw audio
        if not isinstance(event, RawAudioEvent):
            logger.warning("Ring modulator received non-raw audio - passing through unchanged")
            return event

        audio_data = event.data.astype(np.float32) if event.data.dtype != np.float32 else event.data
        num_samples = len(audio_data) if audio_data.ndim == 1 else audio_data.shape[0]

        # Generate carrier wave
        sample_rate = event.sample_rate
        phase_increment = 2.0 * np.pi * carrier_freq / sample_rate

        # Generate carrier samples
        phases = state["phase"] + np.arange(num_samples) * phase_increment

        if carrier_waveform == "sine":
            carrier = np.sin(phases)
        elif carrier_waveform == "square":
            carrier = np.sign(np.sin(phases))
        elif carrier_waveform == "saw":
            carrier = 2.0 * (phases / (2.0 * np.pi) % 1.0) - 1.0
        elif carrier_waveform == "triangle":
            saw = 2.0 * (phases / (2.0 * np.pi) % 1.0) - 1.0
            carrier = 2.0 * np.abs(saw) - 1.0
        else:
            carrier = np.sign(np.sin(phases))

        # Update phase for next buffer
        state["phase"] = (phases[-1] + phase_increment) % (2.0 * np.pi)

        # Apply ring modulation (multiplication)
        if audio_data.ndim == 2:
            # Stereo - apply to both channels
            carrier_stereo = np.column_stack([carrier, carrier])
            modulated = audio_data * carrier_stereo
        else:
            # Mono
            modulated = audio_data * carrier

        # Mix wet/dry
        output = (1.0 - mix) * audio_data + mix * modulated

        return RawAudioEvent(
            data=output,
            sample_rate=event.sample_rate,
            channels=event.channels,
            timestamp=event.timestamp,
        )

    def _ring_modulator(source: ObservableBase) -> ObservableBase:
        """The actual operator function."""

        def subscribe(observer, scheduler=None):
            logger.info(
                f"Started ring modulator (freq={carrier_freq}Hz, wave={carrier_waveform}, mix={mix})"
            )

            def on_next(event):
                try:
                    modulated = modulate_audio(event)
                    observer.on_next(modulated)
                except Exception as e:
                    logger.error(f"Error in ring modulator: {e}")
                    observer.on_error(e)

            def on_completed():
                logger.info("Ring modulator completed")
                observer.on_completed()

            return source.subscribe(on_next, observer.on_error, on_completed, scheduler=scheduler)

        return create(subscribe)

    return _ring_modulator
