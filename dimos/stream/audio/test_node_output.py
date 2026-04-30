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

import pytest
import reactivex as rx

from dimos.stream.audio.node_output import SounddeviceAudioOutput


@pytest.fixture
def sound_device():
    out = SounddeviceAudioOutput()
    yield out
    out.stop()


def test_consume_audio_survives_missing_device(mocker, sound_device) -> None:
    mocker.patch(
        "dimos.stream.audio.node_output.sd.OutputStream",
        side_effect=Exception("Error querying device -1"),
    )

    sound_device.consume_audio(rx.empty())

    assert sound_device._stream is None
    assert not sound_device._running.is_set()
    assert sound_device._subscription is not None
