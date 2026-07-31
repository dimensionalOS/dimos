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

from collections.abc import Callable
from unittest.mock import MagicMock

import numpy as np

from dimos.core.transport import WebRTCAudioTransport
from dimos.stream.audio.base import AudioEvent


def test_webrtc_audio_transport_delivers_pcm_metadata() -> None:
    provider = MagicMock()
    provider.is_connected = True
    unsubscribe = MagicMock()
    provider.subscribe_audio_frames.return_value = unsubscribe
    config = MagicMock()
    config.provider.return_value = provider
    transport = WebRTCAudioTransport(config=config)
    received = []

    returned_unsubscribe = transport.subscribe(received.append)
    audio_callback = provider.subscribe_audio_frames.call_args.args[0]
    audio_callback(np.array([1, -2, 3], dtype=np.int16).tobytes(), 48000, 1)

    assert len(received) == 1
    assert received[0].sample_rate == 48000
    assert received[0].channels == 1
    np.testing.assert_array_equal(received[0].data, np.array([1, -2, 3], dtype=np.int16))

    returned_unsubscribe()
    unsubscribe.assert_called_once_with()


def test_webrtc_audio_transport_keeps_other_subscribers_after_unsubscribe() -> None:
    provider = MagicMock()
    provider.is_connected = True
    callbacks: list[Callable[[bytes, int, int], None]] = []

    def subscribe(callback: Callable[[bytes, int, int], None]) -> Callable[[], None]:
        callbacks.append(callback)
        return lambda: callbacks.remove(callback)

    provider.subscribe_audio_frames.side_effect = subscribe
    config = MagicMock()
    config.provider.return_value = provider
    transport = WebRTCAudioTransport(config=config)
    first: list[AudioEvent] = []
    second: list[AudioEvent] = []

    unsubscribe_first = transport.subscribe(first.append)
    transport.subscribe(second.append)
    for callback in list(callbacks):
        callback(np.array([1], dtype=np.int16).tobytes(), 48000, 1)
    unsubscribe_first()
    for callback in list(callbacks):
        callback(np.array([2], dtype=np.int16).tobytes(), 48000, 1)

    assert [event.data.item() for event in first] == [1]
    assert [event.data.item() for event in second] == [1, 2]
