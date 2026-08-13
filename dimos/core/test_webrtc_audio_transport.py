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

from unittest.mock import MagicMock

import numpy as np

from dimos.core.transport import WebRTCAudioTransport
from dimos.protocol.pubsub.impl.webrtc.providers.spec import AudioProvider


def test_webrtc_audio_transport_delivers_pcm_metadata() -> None:
    provider = MagicMock(spec=AudioProvider)
    provider.is_connected = True
    config = MagicMock()
    config.provider.return_value = provider
    transport = WebRTCAudioTransport(config=config)
    received = []

    unsubscribe = transport.subscribe(received.append)
    try:
        audio_callback = provider.set_audio_frame_callback.call_args.args[0]
        audio_callback(np.array([1, -2, 3], dtype=np.int16).tobytes(), 48000, 1)

        assert len(received) == 1
        assert received[0].sample_rate == 48000
        assert received[0].channels == 1
        np.testing.assert_array_equal(received[0].data, np.array([1, -2, 3], dtype=np.int16))
    finally:
        unsubscribe()

    provider.set_audio_frame_callback.assert_called_with(None)
