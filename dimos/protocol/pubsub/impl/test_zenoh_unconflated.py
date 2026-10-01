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

import threading
import time

from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.protocol.pubsub.impl.zenohpubsub import Topic, Zenoh
from dimos.protocol.service.zenohservice import ZenohSessionPool


def test_subscribe_all_keeps_every_message_of_a_keyed_channel_in_order() -> None:
    pool = ZenohSessionPool()
    pubsub = Zenoh(session_pool=pool)
    pubsub.start()
    keyed = Topic(topic="dimos/keyed", lcm_type=Vector3)
    plain = Topic(topic="dimos/plain", lcm_type=Vector3)
    got: dict[str, list[float]] = {"dimos/keyed": [], "dimos/plain": []}
    done = threading.Event()

    def collect(msg: Vector3, topic: Topic) -> None:
        got[topic.topic].append(msg.x)
        if len(got["dimos/keyed"]) == 30:
            done.set()

    try:
        unsub = pubsub.subscribe_all(collect, unconflated=("keyed",))
        time.sleep(0.5)
        for i in range(30):
            pubsub.publish(keyed, Vector3(float(i), 0.0, 0.0))
            pubsub.publish(plain, Vector3(float(i), 0.0, 0.0))
        assert done.wait(5.0), got
        time.sleep(0.2)
        unsub()
    finally:
        pubsub.stop()
        pool.close_all()

    assert got["dimos/keyed"] == [float(i) for i in range(30)]
    assert got["dimos/plain"] and got["dimos/plain"][-1] == 29.0
    assert pubsub.unconflated_dropped == 0
