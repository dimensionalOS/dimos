#!/usr/bin/env python3
# Copyright 2025 Dimensional Inc.
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

from collections.abc import Callable

from reactivex import Observable, create, disposable

from dimos.stream.audio.base import AbstractAudioConsumer, AudioEvent
from dimos.stream.audio.text.base import AbstractTextEmitter
from dimos.stream.audio.text.node_stdout import TextPrinterNode
from dimos.stream.audio.volume import calculate_peak_volume
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class VolumeMonitorNode(AbstractAudioConsumer, AbstractTextEmitter):
    """
    A node that monitors audio volume and emits text descriptions.
    """

    def __init__(
        self,
        threshold: float = 0.01,
        bar_length: int = 50,
        volume_func: Callable = calculate_peak_volume,  # type: ignore[type-arg]
    ) -> None:
        """
        Initialize VolumeMonitorNode.

        Args:
            threshold: Threshold for considering audio as active
            bar_length: Length of the volume bar in characters
            volume_func: Function to calculate volume (defaults to peak volume)
        """
        self.threshold = threshold
        self.bar_length = bar_length
        self.volume_func = volume_func
        self.func_name = volume_func.__name__.replace("calculate_", "")
        self.audio_observable = None

    def create_volume_text(self, volume: float) -> str:
        """
        Create a text representation of the volume level.

        Args:
            volume: Volume level between 0.0 and 1.0

        Returns:
            String representation of the volume
        """
        # Calculate number of filled segments
        filled = int(volume * self.bar_length)

        # Create the bar
        bar = "█" * filled + "░" * (self.bar_length - filled)

        # Determine if we're above threshold
        active = volume >= self.threshold

        # Format the text
        percentage = int(volume * 100)
        activity = "active" if active else "silent"
        return f"{bar} {percentage:3d}% {activity}"

    def consume_audio(self, audio_observable: Observable) -> "VolumeMonitorNode":  # type: ignore[type-arg]
        """
        Set the audio source observable to consume.

        Args:
            audio_observable: Observable emitting AudioEvent objects

        Returns:
            Self for method chaining
        """
        self.audio_observable = audio_observable  # type: ignore[assignment]
        return self

    def emit_text(self) -> Observable:  # type: ignore[type-arg]
        """
        Create an observable that emits volume text descriptions.

        Returns:
            Observable emitting text descriptions of audio volume
        """
        if self.audio_observable is None:
            raise ValueError("No audio source provided. Call consume_audio() first.")

        def on_subscribe(observer, scheduler):
            logger.info(f"Starting volume monitor (method: {self.func_name})")

            # Subscribe to the audio source
            def on_audio_event(event: AudioEvent) -> None:
                try:
                    # Calculate volume
                    volume = self.volume_func(event.data)

                    # Create text representation
                    text = self.create_volume_text(volume)

                    # Emit the text
                    observer.on_next(text)
                except Exception as e:
                    logger.error(f"Error processing audio event: {e}")
                    observer.on_error(e)

            # Set up subscription to audio source
            subscription = self.audio_observable.subscribe(
                on_next=on_audio_event,
                on_error=lambda e: observer.on_error(e),
                on_completed=lambda: observer.on_completed(),
            )

            # Return a disposable to clean up resources
            def dispose() -> None:
                logger.info("Stopping volume monitor")
                subscription.dispose()

            return disposable.Disposable(dispose)

        return create(on_subscribe)


def monitor(
    audio_source: Observable,  # type: ignore[type-arg]
    threshold: float = 0.01,
    bar_length: int = 50,
    volume_func: Callable = calculate_peak_volume,  # type: ignore[type-arg]
) -> VolumeMonitorNode:
    """
    Create a volume monitor node connected to a text output node.

    Args:
        audio_source: The audio source to monitor
        threshold: Threshold for considering audio as active
        bar_length: Length of the volume bar in characters
        volume_func: Function to calculate volume

    Returns:
        The configured volume monitor node
    """
    # Create the volume monitor node with specified parameters
    volume_monitor = VolumeMonitorNode(
        threshold=threshold, bar_length=bar_length, volume_func=volume_func
    )

    # Connect the volume monitor to the audio source
    volume_monitor.consume_audio(audio_source)

    # Create and connect the text printer node
    text_printer = TextPrinterNode()
    text_printer.consume_text(volume_monitor.emit_text())

    # Return the volume monitor node
    return volume_monitor


if __name__ == "__main__":
    from audio.node_simulated import SimulatedAudioSource  # type: ignore[import-not-found]
    from utils import keepalive  # type: ignore[import-not-found]

    # Use the monitor function to create and connect the nodes
    volume_monitor = monitor(SimulatedAudioSource().emit_audio())

    keepalive()
