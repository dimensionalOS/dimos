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

import threading

from dimos_cdrgen.registry import encode

from dimos.messages.std_msgs.msg import Int32, String
from dimos.protocol.pubsub.impl.lcmpubsub import LCM, LCMPubSubBase, Topic


def test_wildcard_decodes_only_explicit_schemas_and_retains_unknown_bytes():
    bus = LCM(url="memq://")
    received = []
    ready = threading.Event()
    bus.start()

    def receive(value, topic):
        received.append((value, topic))
        if len(received) == 2:
            ready.set()

    declarations = {}
    unsubscribe = bus.subscribe_all(receive, message_types=declarations)
    declarations["known"] = Int32
    try:
        LCMPubSubBase.publish(bus, Topic("known"), encode(String("invalid width")))
        bus.publish(Topic("known", message_type=Int32), Int32(42))
        LCMPubSubBase.publish(bus, Topic("unknown"), b"opaque")
        assert ready.wait(2)
        assert received[0][0] == Int32(42)
        assert received[0][1].message.message_type is Int32
        assert received[1][0] == b"opaque"
        assert received[1][1].message is None
    finally:
        unsubscribe()
        bus.stop()
