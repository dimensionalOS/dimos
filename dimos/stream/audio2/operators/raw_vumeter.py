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

"""Raw audio VU meter operator for visualizing audio levels in the terminal.

This operator only works with raw audio (RawAudioEvent). For compressed audio,
use the GStreamer-based vumeter operator instead.
"""

import sys
from typing import Callable

import numpy as np
from reactivex import create
from reactivex.abc import ObservableBase

from dimos.stream.audio2.types import AudioEvent, RawAudioEvent
from dimos.stream.audio2.operators.utils import calculate_peak_volume
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.stream.audio2.operators.vumeter")


def raw_vumeter(
    threshold: float = 0.01,
    bar_length: int = 50,
    volume_func: Callable[[np.ndarray], float] = calculate_peak_volume,
    show_percentage: bool = True,
    show_activity: bool = True,
):
    """Create a raw audio VU meter operator that displays audio levels.

    This operator prints a visual representation of audio volume to the console
    while passing audio events through unchanged.

    NOTE: This operator only works with RawAudioEvent. Compressed audio is passed
    through unchanged without displaying levels. For a version that handles any
    format, use the GStreamer-based vumeter operator.

    Args:
        threshold: Threshold for considering audio as active (default: 0.01)
        bar_length: Length of the volume bar in characters (default: 50)
        volume_func: Function to calculate volume (default: calculate_peak_volume)
        show_percentage: Show percentage value (default: True)
        show_activity: Show activity status (active/silent) (default: True)

    Returns:
        An operator function that can be used with pipe()

    Examples:
        # Monitor raw audio levels from microphone
        microphone().pipe(
            raw_vumeter(),
            speaker()
        ).run()

        # Use RMS volume with custom bar length
        from dimos.stream.audio2.operators.utils import calculate_rms_volume

        file_input("audio.wav").pipe(
            raw_vumeter(bar_length=80, volume_func=calculate_rms_volume),
            speaker()
        ).run()

        # Minimal meter without percentage and activity
        signal(frequency=440).pipe(
            raw_vumeter(show_percentage=False, show_activity=False)
        ).run()
    """

    def create_volume_bar(volume: float) -> str:
        """Create a text representation of the volume level.

        Args:
            volume: Volume level between 0.0 and 1.0

        Returns:
            String representation of the volume
        """
        # Calculate number of filled segments
        filled = int(volume * bar_length)
        filled = max(0, min(bar_length, filled))  # Clamp to valid range

        # Create the bar
        bar = "█" * filled + "░" * (bar_length - filled)

        # Build the output string
        parts = [bar]

        if show_percentage:
            percentage = int(volume * 100)
            parts.append(f"{percentage:3d}%")

        if show_activity:
            active = volume >= threshold
            activity = "active" if active else "silent"
            parts.append(activity)

        return " ".join(parts)

    def _vumeter(source: ObservableBase) -> ObservableBase:
        """The actual operator function."""

        def subscribe(observer, scheduler=None):
            func_name = volume_func.__name__.replace("calculate_", "").replace("_volume", "")
            logger.info(f"Started VU meter (method: {func_name})")

            def on_next(event: AudioEvent):
                try:
                    # Only process raw audio events
                    if isinstance(event, RawAudioEvent):
                        # Calculate volume
                        volume = volume_func(event.data)

                        # Create and print the volume bar
                        bar_text = create_volume_bar(volume)

                        # Print with carriage return to overwrite previous line
                        print(f"\r{bar_text}", end="", file=sys.stderr, flush=True)

                    # Pass the event through unchanged
                    observer.on_next(event)
                except Exception as e:
                    logger.error(f"Error in VU meter: {e}")
                    observer.on_error(e)

            def on_completed():
                # Print newline to clear the meter line
                print(file=sys.stderr)
                logger.info("VU meter completed")
                observer.on_completed()

            def on_error(error):
                # Print newline to clear the meter line
                print(file=sys.stderr)
                observer.on_error(error)

            return source.subscribe(on_next, on_error, on_completed, scheduler=scheduler)

        return create(subscribe)

    return _vumeter
