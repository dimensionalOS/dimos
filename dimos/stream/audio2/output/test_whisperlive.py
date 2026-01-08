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

"""Test WhisperLive STT output with microphone input.

Prerequisites:
1. Start WhisperLive server in Docker:
   docker run -it --gpus all -p 9090:9090 ghcr.io/collabora/whisperlive-gpu:latest

2. Run this test:
   python -m dimos.stream.audio2.output.test_whisperlive            # Microphone input
   python -m dimos.stream.audio2.output.test_whisperlive file       # File input (callback API)
   python -m dimos.stream.audio2.output.test_whisperlive reactive   # File input (reactive Observable API)
   python -m dimos.stream.audio2.output.test_whisperlive passthrough # Microphone with speaker passthrough
"""

import time

import pytest

from dimos.stream.audio2 import (
    TranscriptionEvent,
    file_input,
    microphone,
    resample,
    whisperlive_stt,
)
from dimos.utils.data import get_data


def on_transcription(event: TranscriptionEvent):
    """Callback for transcription results."""
    prefix = "[FINAL]" if event.is_final else "[PARTIAL]"
    print(f"{prefix} {event.text}")


def test_file_to_whisperlive():
    """Stream audio file to WhisperLive using reactive Observable API."""
    print("Starting file -> WhisperLive STT (reactive API)...")
    print("Transcribing audio file...")
    print("-" * 60)

    try:
        # Use the test audio file
        audio_file = get_data("audio_bender") / "out_of_date.wav"

        # Create STT operator
        stt = whisperlive_stt(
            host="localhost",
            port=9090,
            model="small",
            lang="en",
            use_vad=True,
        )

        # Create audio pipeline: file -> resample -> WhisperLive
        audio_pipeline = file_input(str(audio_file), realtime=True).pipe(
            resample(target_sample_rate=16000, target_channels=1), stt
        )

        # Subscribe to transcription stream (reactive) - must happen before running audio
        stt.transcriptions.subscribe(
            on_next=lambda event: print(
                f"[{'FINAL' if event.is_final else 'PARTIAL'}] {event.text}"
            ),
            on_error=lambda e: print(f"Transcription error: {e}"),
            on_completed=lambda: print("Transcription stream completed"),
        )

        audio_pipeline.run()

        # Allow cleanup threads to finish
        time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nStopped by user")
    except Exception as e:
        print(f"Error: {e}")
        import traceback

        traceback.print_exc()


@pytest.mark.tool
def test_microphone_to_whisperlive():
    """Stream audio file to WhisperLive using reactive Observable API."""
    print("Starting file -> WhisperLive STT (reactive API)...")
    print("Transcribing audio file...")
    print("-" * 60)

    # Create STT operator
    stt = whisperlive_stt(
        host="localhost",
        port=9090,
        model="small",
        lang="en",
        use_vad=True,
    )

    # Create audio pipeline: file -> resample -> WhisperLive
    audio_pipeline = microphone().pipe(resample(target_sample_rate=16000, target_channels=1), stt)

    # Subscribe to transcription stream (reactive) - must happen before running audio
    stt.transcriptions.subscribe(
        on_next=lambda event: print(f"[{'FINAL' if event.is_final else 'PARTIAL'}] {event.text}"),
        on_error=lambda e: print(f"Transcription error: {e}"),
        on_completed=lambda: print("Transcription stream completed"),
    )

    audio_pipeline.run()

    # Allow cleanup threads to finish
    time.sleep(0.1)
