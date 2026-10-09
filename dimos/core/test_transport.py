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

import pickle

import pytest

from dimos.core.transport import (
    JpegLcmTransport,
    LCMTransport,
    pLCMTransport,
)
from dimos.msgs.sensor_msgs.Image import Image


@pytest.mark.parametrize("transport_class", [pLCMTransport, LCMTransport, JpegLcmTransport])
def test_lcm_transport_pickle_preserves_connection_settings(transport_class):
    args = () if transport_class is pLCMTransport else (Image,)
    transport = transport_class("/roundtrip", *args, url="udpm://239.255.76.68:7798?ttl=2", ttl=2)
    try:
        restored = pickle.loads(pickle.dumps(transport))
        try:
            assert type(restored) is transport_class
            assert restored.topic == transport.topic
            assert restored.lcm.config == transport.lcm.config
        finally:
            restored.stop()
    finally:
        transport.stop()
