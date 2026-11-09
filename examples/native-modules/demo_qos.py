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

"""Run one image stream through a slow native handler under each QoS preset.

Run with:
    python examples/native-modules/demo_qos.py --qos never_drop
    python examples/native-modules/demo_qos.py --qos latest_wins
    python examples/native-modules/demo_qos.py --transport lcm

A Python publisher sends 64 KB images at 500 Hz. A Rust module sleeps 10 ms per
image and republishes it. A Python sink reports what arrived and how old it was.

What to expect on one machine:
    never_drop   every image arrives, in order. The publisher is held: its publish
                 call p99 is in the hundreds of milliseconds and the 5 s stream
                 takes about 30 s.
    latest_wins  publish never blocks. Zenoh drops the newest image once its
                 queues fill, so about a quarter arrive and they are 1.5 s old.
    lcm          the handler's input drops the newest image itself and logs a
                 throttled "input full" warning. The images that arrive are about
                 one input buffer old, around 100 ms.
"""

from __future__ import annotations

import argparse
from itertools import pairwise
import threading
import time

import numpy as np

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.core.transport import ZenohTransport
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.protocol.pubsub.impl.zenohpubsub import (
    QOS_LATEST_WINS,
    QOS_NEVER_DROP,
    Topic as ZenohTopic,
)

_BUILD = "nix build -L path:.#dimos-native-module-examples"
_FRAMES_TOPIC = "dimos/qos_demo/frames"
_PRESETS = {"never_drop": QOS_NEVER_DROP, "latest_wins": QOS_LATEST_WINS}


class FramePublisherConfig(ModuleConfig):
    rate_hz: float = 500.0
    size_bytes: int = 65536
    count: int = 2500


class FramePublisher(Module):
    """Publishes numbered images at a fixed rate and times each publish call."""

    config: FramePublisherConfig
    frames: Out[Image]

    _publish_s: list[float]
    _done: bool = False

    @rpc
    def start(self) -> None:
        super().start()
        self._publish_s = []
        threading.Thread(target=self._run, daemon=True).start()

    @rpc
    def stop(self) -> None:
        super().stop()

    def _run(self) -> None:
        side = max(1, int((self.config.size_bytes / 3) ** 0.5))
        pixels = np.zeros((side, side, 3), dtype=np.uint8)
        period = 1.0 / self.config.rate_hz
        time.sleep(1.0)
        next_at = time.perf_counter()
        for seq in range(self.config.count):
            next_at += period
            image = Image(data=pixels, format=ImageFormat.RGB, frame_id=str(seq), ts=time.time())
            t0 = time.perf_counter()
            self.frames.publish(image)
            self._publish_s.append(time.perf_counter() - t0)
            delay = next_at - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
        self._done = True

    @rpc
    def report(self) -> dict[str, float]:
        """Publish call percentiles in milliseconds, plus whether the stream has ended."""
        return {"done": float(self._done), **_percentiles_ms(self._publish_s)}


class FrameSink(Module):
    """Records the sequence number and age of every image the handler forwarded."""

    handled: In[Image]

    _seen: list[tuple[int, float]]

    @rpc
    def start(self) -> None:
        super().start()
        self._seen = []
        self.handled.subscribe(self._on_handled)

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_handled(self, image: Image) -> None:
        self._seen.append((int(image.frame_id), time.time() - image.ts))

    @rpc
    def report(self) -> dict[str, float]:
        """Received count, gap runs, and age percentiles in milliseconds."""
        seqs = [seq for seq, _ in self._seen]
        gaps = sum(1 for a, b in pairwise(seqs) if b != a + 1)
        return {
            "received": float(len(seqs)),
            "gap_runs": float(gaps),
            **_percentiles_ms([age for _, age in self._seen]),
        }


class SlowHandlerConfig(NativeModuleConfig):
    executable: str = "result/bin/slow_handler"
    build_command: str = _BUILD
    source_dir: str = "examples/native-modules/rust"
    stdin_config: bool = True
    handler_delay_ms: int = 10


class SlowHandlerModule(NativeModule):
    """Sleeps handler_delay_ms per image, then republishes it."""

    config: SlowHandlerConfig
    frames: In[Image]
    handled: Out[Image]


def _percentiles_ms(samples: list[float]) -> dict[str, float]:
    if not samples:
        return {"p50_ms": 0.0, "p99_ms": 0.0, "max_ms": 0.0}
    arr = np.asarray(samples) * 1000.0
    return {
        "p50_ms": float(np.percentile(arr, 50)),
        "p99_ms": float(np.percentile(arr, 99)),
        "max_ms": float(arr.max()),
    }


def blueprint(args: argparse.Namespace) -> Blueprint:
    bp = autoconnect(
        FramePublisher.blueprint(rate_hz=args.rate, size_bytes=args.size, count=args.count),
        SlowHandlerModule.blueprint(handler_delay_ms=args.handler_delay_ms),
        FrameSink.blueprint(),
    ).global_config(viewer="none", transport=args.transport)
    if args.transport == "zenoh":
        topic = ZenohTopic(_FRAMES_TOPIC, Image, qos=_PRESETS[args.qos])
        bp = bp.transports({("frames", Image): ZenohTransport.spec(topic)})
    return bp


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--qos", choices=sorted(_PRESETS), default="latest_wins")
    parser.add_argument("--transport", choices=["lcm", "zenoh"], default="zenoh")
    parser.add_argument("--rate", type=float, default=500.0)
    parser.add_argument("--size", type=int, default=65536)
    parser.add_argument("--count", type=int, default=2500)
    parser.add_argument("--handler-delay-ms", type=int, default=10)
    args = parser.parse_args()

    coordinator = ModuleCoordinator.build(blueprint(args))
    try:
        publisher = coordinator.get_instance(FramePublisher)
        sink = coordinator.get_instance(FrameSink)
        started = time.perf_counter()
        while not publisher.report()["done"]:
            time.sleep(0.5)
        time.sleep(2.0)
        pub = publisher.report()
        got = sink.report()
    finally:
        coordinator.stop()

    label = args.transport if args.transport == "lcm" else f"zenoh {args.qos}"
    print(
        f"{label}: {args.count} images, {args.size} bytes, {args.rate:g} Hz, handler {args.handler_delay_ms} ms"
    )
    print(f"  received {got['received']:.0f}/{args.count} in {got['gap_runs']:.0f} gap runs")
    print(
        f"  age      p50 {got['p50_ms']:.0f} ms  p99 {got['p99_ms']:.0f} ms  max {got['max_ms']:.0f} ms"
    )
    print(
        f"  publish  p50 {pub['p50_ms']:.2f} ms  p99 {pub['p99_ms']:.2f} ms  max {pub['max_ms']:.0f} ms"
    )
    print(f"  stream took {time.perf_counter() - started - 2.0:.1f} s")


if __name__ == "__main__":
    main()
