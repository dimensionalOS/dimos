#!/usr/bin/env python3
import numpy as np
import time
import logging
from typing import Optional, Callable
from reactivex import Observable, create, disposable

from audio.abstract import AudioEvent, AbstractAudioConsumer, AbstractAudioEmitter
from text.abstract import AbstractTextEmitter
from text.node_stdout import TextPrinterNode

from audio.volume import calculate_peak_volume


# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class VolumeMonitorNode(AbstractAudioConsumer, AbstractTextEmitter):
    """
    A node that monitors audio volume and emits text descriptions.
    """

    def __init__(
        self,
        threshold: float = 0.01,
        bar_length: int = 50,
        volume_func: Callable = calculate_peak_volume,
    ):
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
        activity = "ACTIVE" if active else "silent"
        return f"[{bar}] {percentage:3d}% {activity}"

    def consume_audio(self, audio_observable: Observable) -> 'VolumeMonitorNode':
        """
        Set the audio source observable to consume.

        Args:
            audio_observable: Observable emitting AudioEvent objects

        Returns:
            Self for method chaining
        """
        self.audio_observable = audio_observable
        return self

    def emit_text(self) -> Observable:
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
            def on_audio_event(event: AudioEvent):
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
                on_completed=lambda: observer.on_completed()
            )
            
            # Return a disposable to clean up resources
            def dispose():
                logger.info("Stopping volume monitor")
                subscription.dispose()
            
            return disposable.Disposable(dispose)
        
        return create(on_subscribe)


def monitor(audio_source: Observable,
          threshold: float = 0.01, 
          bar_length: int = 50,
          volume_func: Callable = calculate_peak_volume) -> VolumeMonitorNode:
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
        threshold=threshold,
        bar_length=bar_length,
        volume_func=volume_func
    )
    
    # Connect the volume monitor to the audio source
    volume_monitor.consume_audio(audio_source)
    
    # Create and connect the text printer node
    text_printer = TextPrinterNode()
    text_printer.consume_text(volume_monitor.emit_text())
    
    # Return the volume monitor node
    return volume_monitor


if __name__ == "__main__":
    from utils import keepalive
    from audio.node_simulated import SimulatedAudioSource

    # Use the monitor function to create and connect the nodes
    volume_monitor = monitor(SimulatedAudioSource().emit_audio())

    keepalive()
