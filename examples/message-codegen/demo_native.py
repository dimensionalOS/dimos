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

# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

"""Run Python → C++ → Rust → Python through the native SDKs on LCM and Zenoh."""

import argparse
from contextlib import ExitStack
import json
import os
from pathlib import Path
import socket
import subprocess
import threading
from typing import Any
from urllib.parse import urlsplit
import uuid

from dimos_generated.dimos_msgs.msg import LineSegment3D, LineSegments3D
from dimos_generated.geometry_msgs.msg import Point
from dimos_generated.sensor_msgs.msg import Image
import numpy as np

from dimos.core.transport import LCMTransport, PubSubTransport, ZenohTransport
from dimos.msgs.time import time_from_nanoseconds, to_nanoseconds
from dimos.protocol.service.lcmservice import LCMConfig
from dimos.protocol.service.zenohservice import ZenohConfig, ZenohSessionPool


def free_port(kind: socket.SocketKind) -> int:
    with socket.socket(socket.AF_INET, kind) as listener:
        listener.bind(("127.0.0.1", 0))
        return int(listener.getsockname()[1])


def stop_process(process: subprocess.Popen[bytes]) -> None:
    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)


def demonstrate(backend: str, cpp: Path, rust: Path, evidence: Path) -> None:
    base = f"demo/{uuid.uuid4().hex[:8]}"
    endpoint = f"tcp/127.0.0.1:{free_port(socket.SOCK_STREAM)}"
    group = urlsplit(LCMConfig().url).hostname
    lcm_url = f"udpm://{group}:{free_port(socket.SOCK_DGRAM)}?ttl=0&recv_buf_size=4194304"
    stamp = 1_700_000_000_123_456_789
    lines = LineSegments3D(segments=[LineSegment3D(start=Point(x=1), end=Point(y=2), weight=4)])
    lines.header.frame_id = "map"
    lines.header.stamp = time_from_nanoseconds(stamp)
    image = Image(
        width=640,
        height=480,
        encoding="rgb8",
        step=1920,
        data=np.arange(640 * 480 * 3, dtype=np.uint8),
    )
    image.header.frame_id = "camera"
    image.header.stamp = time_from_nanoseconds(stamp)
    with ExitStack() as stack:
        pool = ZenohSessionPool()
        stack.callback(pool.close_all)
        channels: dict[str, list[PubSubTransport[Any]]] = {}
        for name, message in (("lines", lines), ("image", image)):
            channels[name] = []
            for index in range(3):
                topic = f"{base}/{name[0]}{index}"
                peer: PubSubTransport[Any]
                if backend == "lcm":
                    peer = LCMTransport(topic, type(message), url=lcm_url)
                else:
                    peer = ZenohTransport(
                        topic,
                        type(message),
                        session_pool=pool,
                        scouting=False,
                        multicast=False,
                        gossip=False,
                        listen=[endpoint],
                        connect=[],
                    )
                stack.callback(peer.stop)
                peer.start()
                channels[name].append(peer)
        received: dict[str, list[Any]] = {"lines": [], "image": []}
        ready = {name: threading.Event() for name in received}
        for name, peers in channels.items():

            def receive(value: Any, name: str = name) -> None:
                received[name].append(value)
                ready[name].set()

            stack.callback(peers[-1].subscribe(receive))
        native_session = ZenohConfig(
            mode="client",
            connect=[endpoint],
            listen=[],
            multicast=False,
            gossip=False,
            scouting=False,
            connect_timeout=10,
        ).to_wire()
        processes = []
        for index, executable in enumerate((cpp, rust)):
            log = stack.enter_context((evidence / f"native-{backend}-{index}.log").open("wb"))
            process = subprocess.Popen(
                [str(executable.resolve())],
                stdin=subprocess.PIPE,
                stdout=log,
                stderr=subprocess.STDOUT,
                env={
                    **os.environ,
                    "DIMOS_TRANSPORT": backend,
                    "LCM_DEFAULT_URL": lcm_url,
                    "RUST_LOG": "info",
                },
            )
            stack.callback(stop_process, process)
            processes.append(process)
            topics = {
                f"{name}_{direction}": peers[index + offset].channel
                for name, peers in channels.items()
                for direction, offset in (("in", 0), ("out", 1))
            }
            launch = {
                "topics": topics,
                "config": None,
                "session": native_session if backend == "zenoh" else None,
            }
            assert process.stdin is not None
            process.stdin.write(json.dumps(launch).encode() + b"\n")
            process.stdin.close()
        for name, message in (("lines", lines), ("image", image)):
            for _ in range(60):
                assert all(p.poll() is None for p in processes), (
                    f"Native process exited; see {evidence}"
                )
                channels[name][0].broadcast(None, message)
                if ready[name].wait(0.25):
                    break
            assert received[name], f"No {backend} {name} reply; see {evidence}"
        decoded_lines = received["lines"][0]
        assert decoded_lines.segments[0].weight == 6
        assert to_nanoseconds(decoded_lines.header.stamp) == stamp
        assert decoded_lines.header.frame_id == "map"
        assert decoded_lines.segments[0].start.x == 1
        assert decoded_lines.segments[0].end.y == 2
        assert received["image"][0].encode() == image.encode()
        print(f"{backend}: Python weight=4 → C++ weight=5 → Rust weight=6 → Python verified")
        print(f"{backend}: {len(image.data):,} image bytes and source nanoseconds={stamp} match")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=["lcm", "zenoh", "all"], default="all")
    parser.add_argument("--cpp", type=Path, default=Path("build/native-cpp-examples/cdr_relay"))
    parser.add_argument("--rust", type=Path, default=Path("target/debug/cdr_relay"))
    parser.add_argument(
        "--evidence", type=Path, default=Path("build/message-codegen/demo/evidence")
    )
    args = parser.parse_args()
    args.evidence.mkdir(parents=True, exist_ok=True)
    for backend in ("lcm", "zenoh") if args.backend == "all" else (args.backend,):
        demonstrate(backend, args.cpp, args.rust, args.evidence)


if __name__ == "__main__":
    main()
