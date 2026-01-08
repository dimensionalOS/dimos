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

"""Example usage of the audio2 streaming API.

All examples use the simplified import API from dimos.stream.audio2.
"""

from dimos.stream.audio2 import (
    Voice,
    WaveformType,
    file_input,
    microphone,
    network_output,
    normalizer,
    openai_tts,
    pitch_shift,
    pyttsx3_tts,
    ring_modulator,
    robotize,
    signal,
    speaker,
    vumeter,
)

# Play a test signal through speakers
signal(frequency=440.0, duration=2.0).pipe(speaker()).run()

# Play an audio file (requires audio.wav to exist)
file_input("audio.wav").pipe(speaker()).run()

# Microphone passthrough to speakers (uncomment to test)
microphone().pipe(speaker()).run()

# OpenAI TTS (high quality, requires API key)
openai_tts("Hello world").pipe(speaker()).run()
openai_tts("I am a robot", voice=Voice.ONYX, speed=0.9).pipe(speaker()).run()

# Local TTS (offline, lower quality)
pyttsx3_tts("Hello from espeak", rate=150).pipe(speaker()).run()

# Pitch shift
signal(frequency=440, duration=1.0).pipe(pitch_shift(1.5), speaker()).run()

# Ring modulation (metallic/robotic sound)
signal(frequency=440, duration=2.0).pipe(
    ring_modulator(carrier_freq=50, carrier_waveform="sine"), speaker()
).run()

# Robotize effect (pitch shift + ring modulation)
openai_tts("I am a robot").pipe(robotize(), speaker()).run()

# Custom robotize parameters
openai_tts("Deep robot voice").pipe(
    robotize(pitch=0.8, carrier_freq=20, ring_mix=0.8), speaker()
).run()

# Normalize audio levels (requires file)
file_input("quiet_audio.wav").pipe(normalizer(target=0.8), speaker()).run()

# Monitor audio levels with VU meter (requires microphone)
microphone().pipe(vumeter(), speaker()).run()

# TTS with multiple effects
openai_tts("Multiple effects demo").pipe(
    pitch_shift(1.2), robotize(carrier_freq=40), normalizer(), speaker()
).run()

# File with effects chain (requires file)
file_input("music.mp3").pipe(
    vumeter(), pitch_shift(0.8), ring_modulator(carrier_freq=100), speaker()
).run()

# Stream to remote speaker
openai_tts("Remote robot voice").pipe(
    robotize(), network_output(host="192.168.1.100", port=5002, codec="opus")
).run()

# Generate different waveforms
signal(waveform=WaveformType.SQUARE, frequency=200, duration=1.0).pipe(speaker()).run()
signal(waveform=WaveformType.SAW, frequency=200, duration=1.0).pipe(speaker()).run()
signal(waveform=WaveformType.WHITE_NOISE, volume=0.3, duration=2.0).pipe(speaker()).run()
signal(waveform=WaveformType.PINK_NOISE, volume=0.3, duration=2.0).pipe(speaker()).run()
