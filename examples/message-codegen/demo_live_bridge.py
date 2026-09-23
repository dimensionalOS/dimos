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

"""Send a generated image over LCM or Zenoh into a running headless Rerun bridge."""

import argparse
from pathlib import Path
import socket
from threading import Event
from typing import Any
from uuid import uuid4

from dimos_generated.sensor_msgs.msg import Image
from dimos_generated.std_msgs.msg import Header
import numpy as np
import rerun as rr

from dimos.core.global_config import GlobalConfig
from dimos.msgs.image import image_from_array
from dimos.protocol.pubsub.impl.lcmpubsub import LCM, Topic
from dimos.protocol.pubsub.impl.zenohpubsub import Zenoh
from dimos.protocol.service.zenohservice import ZenohSessionPool
from dimos.visualization.rerun.bridge import RerunBridgeModule


class ObservedBridge(RerunBridgeModule):
    def __init__(self, expected_topic: str, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.expected_topic = expected_topic
        self.received = Event()

    def _on_message(self, msg: Any, topic: Any) -> None:
        super()._on_message(msg, topic)
        if getattr(topic, "topic", None) == self.expected_topic and isinstance(msg, Image):
            assert msg.width == 160 and msg.height == 120
            assert msg.header.frame_id == "camera_optical"
            self.received.set()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--transport", choices=("lcm", "zenoh"), default="lcm")
    backend = parser.parse_args().transport
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    channel = ("dimos/vb/" if backend == "zenoh" else "/vb/") + uuid4().hex[:8]
    pools = [ZenohSessionPool(), ZenohSessionPool()]
    if backend == "zenoh":
        with socket.socket() as sock:
            sock.bind(("127.0.0.1", 0))
            endpoint = f"tcp/127.0.0.1:{sock.getsockname()[1]}"
        receiver = Zenoh(
            session_pool=pools[0],
            listen=[endpoint],
            connect=[],
            scouting=False,
            multicast=False,
            gossip=False,
        )
        sender = Zenoh(
            session_pool=pools[1],
            listen=[],
            connect=[endpoint],
            scouting=False,
            multicast=False,
            gossip=False,
        )
    else:
        receiver = LCM()
        sender = LCM()
    bridge = ObservedBridge(
        channel,
        g=GlobalConfig(transport=backend),
        pubsubs=[receiver],
        rerun_open="none",
        rerun_web=False,
        connect_url=f"rerun+http://127.0.0.1:{port}/proxy",
    )
    output = Path(f"build/message-codegen/demo/evidence/live-bridge-{backend}.rrd")
    output.parent.mkdir(parents=True, exist_ok=True)
    try:
        bridge.start()
        rr.save(str(output))
        sender.start()
        pixels = np.zeros((120, 160, 3), dtype=np.uint8)
        pixels[:, :80, 0] = 255
        pixels[:, 80:, 2] = 255
        message = image_from_array(
            pixels, encoding="rgb8", header=Header(frame_id="camera_optical")
        )
        for _ in range(10):
            sender.publish(Topic(channel, Image), message)
            if bridge.received.wait(0.2):
                break
        assert bridge.received.is_set(), (
            f"bridge did not receive the generated image over {backend}"
        )
        print(f"{backend.upper()} CDR image received and rendered on {channel}: 160x120 rgb8")
    finally:
        bridge.stop()
        sender.stop()
        rr.disconnect()
        for pool in pools:
            pool.close_all()
    assert output.stat().st_size > 0
    print(f"Headless live bridge recording: {output}")


if __name__ == "__main__":
    main()
