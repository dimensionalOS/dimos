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

"""Robotize effect - combines pitch shift and ring modulation for robotic voice."""

from reactivex import operators as ops
from reactivex.abc import ObservableBase

from .pitch_shift import pitch_shift
from .ring_modulator import ring_modulator


def robotize(
    pitch: float = 1,
    carrier_freq: float = 80.0,
    carrier_waveform: str = "sine",
    ring_mix: float = 0.7,
):
    """Create a robotic voice effect by combining pitch shift and ring modulation.

    This operator transforms audio into a robotic/synthetic voice by:
    1. Shifting the pitch (typically upward)
    2. Applying ring modulation with a low-frequency carrier wave

    Works with raw PCM audio (standard internal format).

    Args:
        pitch: Pitch shift multiplier (default: 1.2). Values > 1.0 shift pitch up.
        carrier_freq: Ring modulator carrier frequency in Hz (default: 30.0)
                     Lower = deeper robot voice, Higher = more metallic
        carrier_waveform: Carrier waveform: "sine", "square", "saw", "triangle" (default: "square")
        ring_mix: Ring modulation wet/dry mix 0.0-1.0 (default: 0.7)

    Returns:
        An operator function that can be used with pipe()

    Examples:
        # Classic robot voice
        file_input("voice.wav").pipe(
            robotize(),
            speaker()
        ).run()

        # Deep robotic voice
        file_input("voice.wav").pipe(
            robotize(pitch=1.0, carrier_freq=20, ring_mix=0.8),
            speaker()
        ).run()

        # High-pitched metallic voice
        file_input("voice.wav").pipe(
            robotize(pitch=1.5, carrier_freq=50, carrier_waveform="sine"),
            speaker()
        ).run()
    """

    def _robotize(source: ObservableBase) -> ObservableBase:
        """Apply pitch shift followed by ring modulation."""
        return source.pipe(
            pitch_shift(pitch),
            ring_modulator(carrier_freq, carrier_waveform, ring_mix),
        )

    return _robotize
