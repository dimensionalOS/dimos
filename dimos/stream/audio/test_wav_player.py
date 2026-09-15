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

import io
import wave

import numpy as np

from dimos.stream.audio.wav_player import WavPlayer


def test_wav_playback_is_nonblocking_and_stop_is_idempotent(mocker):
    output = mocker.patch("dimos.stream.audio.wav_player.importlib.import_module").return_value
    buffer = io.BytesIO()
    with wave.open(buffer, "wb") as wav:
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(24000)
        wav.writeframes(np.array([0, 100, -100], dtype="<i2").tobytes())
    player = WavPlayer()
    try:
        player.play(buffer.getvalue())
        player.play(buffer.getvalue())
        assert output.play.call_count == 2
        args, kwargs = output.play.call_args
        assert args[0].tolist() == [[0], [100], [-100]]
        assert kwargs == {"samplerate": 24000, "blocking": False}
    finally:
        player.stop()
        player.stop()
    output.stop.assert_called_once_with()
