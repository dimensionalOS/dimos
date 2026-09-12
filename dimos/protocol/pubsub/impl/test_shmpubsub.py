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

import time
from typing import Any

from dimos.protocol.pubsub.impl.shmpubsub import PickleSharedMemory


def test_new_subscriber_ignores_frame_left_in_segment(wait_until: Any) -> None:
    topic = "/shm_stale_frame"
    owner = PickleSharedMemory(prefer="cpu")
    owner.start()
    owner.publish(topic, b"stale")

    reader = PickleSharedMemory(prefer="cpu")
    reader.start()
    got: list[bytes] = []
    reader.subscribe(topic, lambda msg, _topic: got.append(msg))
    try:
        time.sleep(0.2)
        assert got == []

        owner.publish(topic, b"fresh")
        wait_until(lambda: got == [b"fresh"], timeout=2.0)
    finally:
        reader.stop()
        owner.stop()
