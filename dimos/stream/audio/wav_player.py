# Copyright 2026 Dimensional Inc.
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

"""Nonblocking desktop playback of PCM16 WAV prompts."""

import importlib
import io
import wave

import numpy as np


class WavPlayer:
    """Play on the default output device, interrupting the previous prompt."""

    def __init__(self) -> None:
        self._output = importlib.import_module("sounddevice")
        self._playing = False

    def play(self, audio: bytes) -> None:
        with wave.open(io.BytesIO(audio)) as wav:
            if wav.getsampwidth() != 2:
                raise ValueError("Desktop speech requires PCM16 WAV audio")
            samples = np.frombuffer(wav.readframes(wav.getnframes()), dtype="<i2")
            samples = samples.reshape(-1, wav.getnchannels())
            # sounddevice.play stops previous playback and returns immediately.
            self._playing = True
            self._output.play(samples, samplerate=wav.getframerate(), blocking=False)

    def stop(self) -> None:
        if self._playing:
            self._output.stop()
            self._playing = False
