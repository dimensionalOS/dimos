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

"""Tests for pyttsx3 TTS input."""

import time

import pytest

from dimos.stream.audio2.input.tts_pyttsx3 import PYTTSX3_AVAILABLE, pyttsx3_tts
from dimos.stream.audio2.operators import robotize
from dimos.stream.audio2.output.soundcard import speaker

# Skip all tests if pyttsx3 is not available
pytestmark = pytest.mark.skipif(not PYTTSX3_AVAILABLE, reason="pyttsx3 not installed")


def test_pyttsx3_tts_basic():
    """Test basic pyttsx3 TTS."""
    pyttsx3_tts("Hello from e-speak").pipe(speaker()).run()

    # Give cleanup threads time to finish
    time.sleep(1.0)


def test_pyttsx3_tts_with_rate():
    """Test pyttsx3 TTS with custom rate."""
    pyttsx3_tts("This is slow speech", rate=150).pipe(speaker()).run()

    # Give cleanup threads time to finish
    time.sleep(1.0)


def test_pyttsx3_tts_with_robotize():
    """Test pyttsx3 TTS with robotize effect."""
    pyttsx3_tts("I am a robot").pipe(robotize(), speaker()).run()

    # Give cleanup threads time to finish
    time.sleep(1.0)
