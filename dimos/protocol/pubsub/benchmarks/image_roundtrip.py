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

"""Compare raw images and JPEG over local LCM and Zenoh transports."""

from __future__ import annotations

import argparse
import statistics
import threading
import time
from typing import Any

import numpy as np

from dimos.core.transport import LCMTransport, PubSubTransport, ZenohTransport
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.protocol.pubsub.benchmarks.codec import CompressedCodec
from dimos.protocol.service.zenohservice import ZenohSessionPool


def make_image(width: int = 1280, height: int = 720) -> Image:
    """720p-ish frame: gradient + noise, roughly what a real camera compresses like."""
    rng = np.random.RandomState(7)
    gradient = np.broadcast_to(np.linspace(0, 255, width, dtype=np.uint8), (height, width))
    noise = rng.randint(0, 60, (height, width), dtype=np.uint8)
    data = np.stack([gradient, np.minimum(gradient, 128) + noise // 2, noise], axis=-1)
    return Image(data=data, format=ImageFormat.RGB, frame_id="cam", ts=42.125)


def measure(
    transport: PubSubTransport[Any], frame: Image, n: int, wire_bytes: int
) -> dict[str, float] | None:
    """Round-trip n frames one at a time; per-frame latency includes encode+wire+decode.

    Large raw frames fragment over UDP and can genuinely drop — drops are
    counted and reported, not failed on. Returns None when nothing ever
    arrives (raw 720p over LCM can be entirely undeliverable).
    """
    received: list[Image] = []
    event = threading.Event()

    def callback(msg: Image) -> None:
        received.append(msg)
        event.set()

    latencies = []
    drops = 0
    unsubscribe = transport.subscribe(callback)
    try:
        # warmup (first delivery also proves subscription is live); real
        # deliveries arrive in ms — 3s only gets burned by undeliverable paths
        deadline = time.monotonic() + 3
        while not event.is_set() and time.monotonic() < deadline:
            transport.broadcast(None, frame)
            event.wait(0.2)
        if not event.is_set():
            return None

        for _ in range(n):
            count = len(received)
            start = time.perf_counter()
            transport.broadcast(None, frame)
            deadline = time.monotonic() + 2
            while len(received) <= count and time.monotonic() < deadline:
                time.sleep(0.0005)
            if len(received) > count:
                latencies.append(time.perf_counter() - start)
            else:
                drops += 1
    finally:
        unsubscribe()
        transport.stop()
    if not latencies:
        return None
    lat = statistics.median(latencies)
    return {
        "latency_ms": lat * 1000,
        "max_ms": max(latencies) * 1000,
        "wire_bytes": wire_bytes,
        "fps": 1 / lat,
        "drops": drops,
        "n": n,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--transport", choices=["lcm", "zenoh"], action="append")
    parser.add_argument("--frames", type=int, default=15)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    args = parser.parse_args()
    if min(args.frames, args.width, args.height) <= 0:
        parser.error("frames, width, and height must be positive")
    frame = make_image(args.width, args.height)
    raw_wire = len(frame.lcm_encode())
    jpeg_wire = len(CompressedImage.from_image(frame).lcm_encode())
    print(
        f"{'transport':<10}{'mode':<8}{'wire/frame':>12}{'median ms':>12}{'max ms':>12}{'fps':>10}{'drops':>8}"
    )
    for protocol in args.transport or ["lcm", "zenoh"]:
        pool = ZenohSessionPool()

        def inner(
            topic: str,
            msg_type: type[Any],
            protocol: str = protocol,
            pool: ZenohSessionPool = pool,
        ) -> PubSubTransport[Any]:
            if protocol == "lcm":
                return LCMTransport(topic, msg_type)
            return ZenohTransport(topic, msg_type, session_pool=pool)

        try:
            for mode, wire_size in [("raw", raw_wire), ("jpeg", jpeg_wire)]:
                transport = (
                    inner(f"dimos/bench/{protocol}_{mode}", Image)
                    if mode == "raw"
                    else CompressedCodec(inner(f"dimos/bench/{protocol}_{mode}", CompressedImage))
                )
                result = measure(transport, frame, args.frames, wire_size)
                if result is None:
                    print(f"{protocol:<10}{mode:<8}UNDELIVERABLE")
                else:
                    print(
                        f"{protocol:<10}{mode:<8}{wire_size:>12}{result['latency_ms']:>12.2f}"
                        f"{result['max_ms']:>12.2f}{result['fps']:>10.1f}{result['drops']:>8.0f}"
                    )
        finally:
            pool.close_all()
    print("Latency includes encoding, transport, and decoding on localhost.")


if __name__ == "__main__":
    main()
