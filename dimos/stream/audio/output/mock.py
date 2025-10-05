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

from typing import List, Optional, Tuple
import time

import numpy as np
from reactivex import Observable

from dimos.stream.audio.base import AbstractAudioTransform, AudioEvent
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.stream.audio.output.mock")


class MockAudioOutput(AbstractAudioTransform):
    """
    Mock audio output for testing purposes.

    This class collects all received audio events for inspection and verification
    in tests. It implements AbstractAudioTransform to fit into the audio pipeline.
    """

    def __init__(self):
        """Initialize MockAudioOutput."""
        self.received_events: List[AudioEvent] = []
        self.receive_times: List[float] = []
        self._subscription = None
        self.audio_observable = None
        self._started = False
        self._stopped = False

    def consume_audio(self, audio_observable: Observable) -> "MockAudioOutput":
        """
        Subscribe to an audio observable and collect events.

        Args:
            audio_observable: Observable emitting AudioEvent objects

        Returns:
            Self for method chaining
        """
        self.audio_observable = audio_observable
        self._started = True

        # Subscribe to the observable
        self._subscription = audio_observable.subscribe(
            on_next=self._collect_audio_event,
            on_error=self._handle_error,
            on_completed=self._handle_completion,
        )

        logger.info("Started mock audio output")
        return self

    def emit_audio(self) -> Observable:
        """
        Pass through the audio observable to allow chaining.

        Returns:
            The same Observable that was provided to consume_audio
        """
        if self.audio_observable is None:
            raise ValueError("No audio source provided. Call consume_audio() first.")

        return self.audio_observable

    def _collect_audio_event(self, audio_event: AudioEvent):
        """Collect an audio event for testing."""
        self.received_events.append(audio_event)
        self.receive_times.append(time.time())
        logger.debug(f"Collected audio event: {audio_event}")

    def _handle_error(self, error):
        """Handle errors from the observable."""
        logger.error(f"Error in audio observable: {error}")
        self._stopped = True

    def _handle_completion(self):
        """Handle completion of the observable."""
        logger.info("Audio observable completed")
        self._stopped = True

    def stop(self):
        """Stop collecting audio and clean up."""
        logger.info("Stopping mock audio output")
        self._stopped = True

        if self._subscription:
            self._subscription.dispose()
            self._subscription = None

    def get_received_events(self) -> List[AudioEvent]:
        """Get all received audio events."""
        return self.received_events

    def get_total_samples(self) -> int:
        """Get the total number of audio samples received."""
        return sum(len(event.data) for event in self.received_events)

    def get_audio_duration(self) -> float:
        """Calculate total audio duration in seconds."""
        if not self.received_events:
            return 0.0

        total_samples = self.get_total_samples()
        # Assume all events have same sample rate (use first event)
        sample_rate = self.received_events[0].sample_rate
        return total_samples / sample_rate

    def verify_continuous(self, tolerance_ms: float = 50.0) -> bool:
        """
        Verify that audio events are continuous without gaps.

        Args:
            tolerance_ms: Maximum allowed gap between events in milliseconds

        Returns:
            True if audio is continuous, False if there are gaps
        """
        if len(self.received_events) < 2:
            return True

        for i in range(1, len(self.received_events)):
            # Calculate expected time for this event based on previous event
            prev_event = self.received_events[i - 1]
            expected_time = (
                self.receive_times[i - 1] + len(prev_event.data) / prev_event.sample_rate
            )

            # Check if actual receive time is within tolerance
            actual_time = self.receive_times[i]
            gap_ms = (actual_time - expected_time) * 1000

            if gap_ms > tolerance_ms:
                logger.warning(f"Gap detected: {gap_ms:.1f}ms at event {i}")
                return False

        return True

    def get_format_info(self) -> Optional[Tuple[int, int, np.dtype]]:
        """
        Get format information from received audio.

        Returns:
            Tuple of (sample_rate, channels, dtype) or None if no events
        """
        if not self.received_events:
            return None

        first_event = self.received_events[0]
        return (first_event.sample_rate, first_event.channels, first_event.dtype)

    def clear(self):
        """Clear all collected events."""
        self.received_events.clear()
        self.receive_times.clear()

    def wait_for_events(self, count: int, timeout: float = 5.0) -> bool:
        """
        Wait for a specific number of events to be received.

        Args:
            count: Number of events to wait for
            timeout: Maximum time to wait in seconds

        Returns:
            True if events received, False if timeout
        """
        start_time = time.time()
        while len(self.received_events) < count:
            if time.time() - start_time > timeout:
                return False
            time.sleep(0.01)
        return True
