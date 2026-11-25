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

import numpy as np

from dimos.protocol.encode.cbor import decode, encode
from dimos.stream.audio.base import AudioEvent


def test_audio_event_round_trips_through_the_codec() -> None:
    event = AudioEvent(np.arange(8, dtype=np.int16), sample_rate=16000, timestamp=1.5, channels=2)

    restored = decode(encode(event))

    assert isinstance(restored, AudioEvent)
    np.testing.assert_array_equal(restored.data, event.data)
    assert (restored.sample_rate, restored.timestamp, restored.channels) == (16000, 1.5, 2)
    assert restored.dtype == np.int16
    assert restored.shape == (8,)
