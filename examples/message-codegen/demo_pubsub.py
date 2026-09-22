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

"""Exchange generated messages through DimOS LCM, Zenoh TCP, and shared memory."""

import argparse
from contextlib import ExitStack
import socket
import threading
from typing import Any
from urllib.parse import urlsplit
import uuid

from dimos_generated.dimos_msgs.msg import LineSegment3D, LineSegments3D
from dimos_generated.geometry_msgs.msg import Point
from dimos_generated.sensor_msgs.msg import Image
import numpy as np

from dimos.core.transport import LCMTransport, PubSubTransport, SHMTransport, ZenohTransport
from dimos.msgs.protocol import DimosMsg
from dimos.msgs.time import time_from_nanoseconds, to_nanoseconds
from dimos.protocol.service.lcmservice import LCMConfig
from dimos.protocol.service.zenohservice import ZenohSessionPool
from dimos.utils.testing.waiting import retry_until


def free_port(kind: socket.SocketKind) -> int:
    with socket.socket(socket.AF_INET, kind) as listener:
        listener.bind(("127.0.0.1", 0))
        return int(listener.getsockname()[1])


def exchange(
    publisher: PubSubTransport[Any], subscriber: PubSubTransport[Any], message: DimosMsg
) -> DimosMsg:
    received: list[DimosMsg] = []
    ready = threading.Event()

    def receive(value: DimosMsg) -> None:
        received.append(value)
        ready.set()

    unsubscribe = subscriber.subscribe(receive)
    try:
        retry_until(ready, lambda: publisher.broadcast(None, message), timeout=10)
        return received[0]
    finally:
        unsubscribe()


def demonstrate(backend: str) -> None:
    image = Image(
        width=640,
        height=480,
        encoding="rgb8",
        step=1920,
        data=np.arange(640 * 480 * 3, dtype=np.uint8),
    )
    image.header.frame_id = "camera"
    image.header.stamp = time_from_nanoseconds(1_700_000_000_123_456_789)
    lines = LineSegments3D(segments=[LineSegment3D(start=Point(x=1), end=Point(y=2), weight=4)])
    lines.header.frame_id = "map"
    with ExitStack() as stack:
        pools = [ZenohSessionPool(), ZenohSessionPool()]
        for pool in pools:
            stack.callback(pool.close_all)
        tcp_endpoint = f"tcp/127.0.0.1:{free_port(socket.SOCK_STREAM)}"
        group = urlsplit(LCMConfig().url).hostname
        lcm_url = f"udpm://{group}:{free_port(socket.SOCK_DGRAM)}?ttl=0&recv_buf_size=4194304"
        for message in (lines, image):
            topic = f"dimos/demo/{uuid.uuid4().hex[:8]}"
            peers = []
            for index in range(2):
                peer: PubSubTransport[Any]
                if backend == "lcm":
                    peer = LCMTransport(topic, type(message), url=lcm_url)
                elif backend == "zenoh":
                    peer = ZenohTransport(
                        topic,
                        type(message),
                        session_pool=pools[index],
                        scouting=False,
                        multicast=False,
                        gossip=False,
                        listen=[tcp_endpoint] if index == 0 else [],
                        connect=[] if index == 0 else [tcp_endpoint],
                    )
                else:
                    peer = SHMTransport(topic, type(message), prefer="cpu")
                stack.callback(peer.stop)
                peer.start()
                peers.append(peer)
            decoded = exchange(peers[0], peers[1], message)
            assert decoded.encode() == message.encode()
            if isinstance(decoded, LineSegments3D):
                print(
                    f"{backend}: {decoded.msg_name}, frame={decoded.header.frame_id}, "
                    f"segment weight={decoded.segments[0].weight}"
                )
            else:
                assert isinstance(decoded, Image)
                print(
                    f"{backend}: {decoded.msg_name}, {len(decoded.data):,} pixel bytes match, "
                    f"source nanoseconds={to_nanoseconds(decoded.header.stamp)}"
                )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=["lcm", "zenoh", "shm", "all"], default="all")
    args = parser.parse_args()
    for backend in ("lcm", "zenoh", "shm") if args.backend == "all" else (args.backend,):
        demonstrate(backend)


if __name__ == "__main__":
    main()
