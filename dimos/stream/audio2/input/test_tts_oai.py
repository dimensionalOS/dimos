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

"""Tests for OpenAI TTS input."""

import os

import pytest

from dimos.stream.audio2.input.tts_oai import Voice, openai_tts
from dimos.stream.audio2.operators import robotize
from dimos.stream.audio2.output.soundcard import speaker

# Skip all TTS tests if no OpenAI API key is available
pytestmark = pytest.mark.skipif(
    not os.environ.get("OPENAI_API_KEY"), reason="OPENAI_API_KEY not set"
)


def test_openai_tts_basic():
    """Test basic OpenAI TTS."""
    openai_tts("Hello from OpenAI").pipe(speaker()).run()


def test_openai_tts_with_voice():
    """Test OpenAI TTS with custom voice."""
    openai_tts("Testing the Nova voice", voice=Voice.NOVA).pipe(speaker()).run()


def test_openai_tts_with_speed():
    """Test OpenAI TTS with custom speed."""
    openai_tts("This is slow speech", speed=0.7).pipe(speaker()).run()


def test_openai_tts_with_robotize():
    """Test OpenAI TTS with custom speed."""
    openai_tts("This is a robotic speech").pipe(robotize(), speaker()).run()
