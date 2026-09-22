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

"""Exchange changing custom messages and images through a three-language blueprint."""

from __future__ import annotations

import argparse
from contextlib import ExitStack
from pathlib import Path
import socket
import threading
from typing import Literal
import uuid

from dimos_generated.dimos_msgs.msg import LineSegment3D, LineSegments3D
from dimos_generated.geometry_msgs.msg import Point
from dimos_generated.sensor_msgs.msg import Image
import numpy as np

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.native_module import NativeModule
from dimos.core.stream import In, Out
from dimos.msgs.time import time_from_nanoseconds, to_nanoseconds
from dimos.protocol.service.zenohservice import ZenohConfig, ZenohSessionPool

_ROOT = Path(__file__).resolve().parents[2]
_STAMP = 1_700_000_000_123_456_789


class PythonRelayDemo(Module):
    """Publish a sample and validate the replies from both native modules."""

    l0: Out[LineSegments3D]
    l2: In[LineSegments3D]
    i0: Out[Image]
    i2: In[Image]

    @rpc
    def start(self) -> None:
        super().start()
        self._stamp = -1
        self._lines: LineSegments3D | None = None
        self._image: Image | None = None
        self._received = threading.Event()
        self.l2.subscribe(self._receive_lines)
        self.i2.subscribe(self._receive_image)

    def _receive_lines(self, message: LineSegments3D) -> None:
        if to_nanoseconds(message.header.stamp) == self._stamp:
            self._lines = message
            if self._image is not None:
                self._received.set()

    def _receive_image(self, message: Image) -> None:
        if to_nanoseconds(message.header.stamp) == self._stamp:
            self._image = message
            if self._lines is not None:
                self._received.set()

    @rpc
    def exchange(self, sample: int) -> str:
        self._stamp = _STAMP + sample
        self._lines = None
        self._image = None
        self._received.clear()
        lines = LineSegments3D(
            segments=[LineSegment3D(start=Point(x=sample), end=Point(y=2), weight=sample)]
        )
        lines.header.stamp = time_from_nanoseconds(self._stamp)
        lines.header.frame_id = "map"
        image = Image(
            width=640,
            height=480,
            step=1920,
            encoding="rgb8",
            data=np.full(640 * 480 * 3, sample % 256, dtype=np.uint8),
        )
        image.header.stamp = time_from_nanoseconds(self._stamp)
        image.header.frame_id = "camera"
        for _ in range(60):
            self.l0.publish(lines)
            self.i0.publish(image)
            if self._received.wait(0.25):
                break
        if self._lines is None or self._image is None:
            raise TimeoutError(
                f"Missing native relay reply for sample {sample}: "
                f"lines={self._lines is not None}, image={self._image is not None}"
            )
        expected = LineSegments3D.decode(lines.encode())
        expected.segments = [
            LineSegment3D(start=Point(x=sample), end=Point(y=2), weight=sample + 2)
        ]
        if self._lines.encode() != expected.encode():
            raise ValueError("Native custom message reply differs from the expected edits")
        if self._image.encode() != image.encode():
            raise ValueError("Native image reply changed pixels or metadata")
        return (
            f"sample {sample}: Python weight={sample} → C++ {sample + 1} → Rust {sample + 2}; "
            f"Python verified {len(image.data):,} image bytes and stamp {self._stamp}"
        )


class CppRelay(NativeModule):
    lines_in: In[LineSegments3D]
    lines_out: Out[LineSegments3D]
    image_in: In[Image]
    image_out: Out[Image]


class RustRelay(NativeModule):
    lines_in: In[LineSegments3D]
    lines_out: Out[LineSegments3D]
    image_in: In[Image]
    image_out: Out[Image]


def blueprint(backend: Literal["lcm", "zenoh"], cpp: Path, rust: Path) -> Blueprint:
    return (
        autoconnect(
            PythonRelayDemo.blueprint(),
            CppRelay.blueprint(executable=str(cpp.resolve()), stdin_config=True),
            RustRelay.blueprint(executable=str(rust.resolve()), stdin_config=True),
        )
        .remappings(
            [
                (CppRelay, "lines_in", "l0"),
                (CppRelay, "lines_out", "l1"),
                (RustRelay, "lines_in", "l1"),
                (RustRelay, "lines_out", "l2"),
                (CppRelay, "image_in", "i0"),
                (CppRelay, "image_out", "i1"),
                (RustRelay, "image_in", "i1"),
                (RustRelay, "image_out", "i2"),
            ]
        )
        .namespace(f"cdr{uuid.uuid4().hex[:8]}")
        .global_config(viewer="none", transport=backend)
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--transport", choices=("lcm", "zenoh"), default="zenoh")
    parser.add_argument("--samples", type=int, default=5)
    parser.add_argument("--cpp", type=Path, default=_ROOT / "build/native-cpp-examples/cdr_relay")
    parser.add_argument("--rust", type=Path, default=_ROOT / "target/debug/cdr_relay")
    args = parser.parse_args()
    if args.samples < 1:
        parser.error("--samples must be positive")
    for executable in (args.cpp, args.rust):
        if not executable.is_file():
            parser.error(f"Build the native relay first: {executable}")
    with ExitStack() as stack:
        bp = blueprint(args.transport, args.cpp, args.rust)
        if args.transport == "zenoh":
            with socket.socket() as reservation:
                reservation.bind(("127.0.0.1", 0))
                endpoint = f"tcp/127.0.0.1:{reservation.getsockname()[1]}"
            pool = ZenohSessionPool()
            stack.callback(pool.close_all)
            pool.acquire(
                ZenohConfig(
                    mode="router", listen=[endpoint], connect=[], multicast=False, gossip=False
                )
            )
            bp = bp.global_config(
                zenoh_mode="client", zenoh_connect=endpoint, zenoh_multicast=False
            )
        coordinator = ModuleCoordinator.build(bp)
        stack.callback(coordinator.stop)
        producer = coordinator.get_instance(PythonRelayDemo)
        for sample in range(args.samples):
            print(producer.exchange(sample), flush=True)
        print(f"{args.transport}: three-language coordinator blueprint verified", flush=True)


if __name__ == "__main__":
    main()
