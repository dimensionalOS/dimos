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

"""End-to-end replay benchmark for native CompressedImage blueprints (#2831).

Runs a real blueprint in replay mode for a fixed duration, with optional
BenchSink consumer modules, and records per-frame freshness plus host
CPU/RSS/loopback traffic. Sinks consume the CompressedImage stream either
decoding every frame (--sink decode, a busy detector) or storing bytes
without ever decoding (--sink bytes, the save-latest / VLM / relay pattern).
One process per run:

    python -m dimos.protocol.pubsub.benchmark.tool_replay_bench \
        --blueprint unitree-go2 --sink decode --sinks 4 --work-ms 20 \
        --duration 60 --out /tmp/bench/go2_decode_0

Wrap with `taskset -c 0-3` / `tc netem` for constrained profiles. Raw/codec
baseline cells predate the option-5 migration — run them from a worktree at
3f2fc05ad, where this tool still has --mode raw|codec.
"""

from __future__ import annotations

import argparse
from collections.abc import Callable
import json
import os
from pathlib import Path
import threading
import time
from typing import Any, TypeVar

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out, Stream
from dimos.core.transport import PubSubTransport
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

T = TypeVar("T")


class CompressedCodec(PubSubTransport[T]):
    """Image→CompressedImage jpeg codec over any inner transport.

    Bench-only utility: rejected as public API in #2831 (transports shouldn't
    peek into messages), kept here so raw-vs-codec cells stay comparable.
    The wire carries a typed sensor_msgs.CompressedImage; ts/frame_id survive.
    Subscribers always receive a decoded Image.
    """

    def __init__(self, inner: PubSubTransport[Any], quality: int = 75) -> None:
        super().__init__(inner.topic)
        self.inner = inner
        self.quality = quality

    def __reduce__(self):  # type: ignore[no-untyped-def]
        return (CompressedCodec, (self.inner, self.quality))

    def broadcast(self, stream: Out[T] | None, msg: T) -> None:
        from dimos.msgs.sensor_msgs.CompressedImage import (
            CompressedImage,
        )  # deferred to avoid pulling in cv2/rerun

        if not isinstance(msg, CompressedImage):
            msg = CompressedImage.from_image(msg, quality=self.quality)  # type: ignore[assignment, arg-type]
        self.inner.broadcast(stream, msg)

    def subscribe(
        self, callback: Callable[[T], Any], selfstream: Stream[T] | None = None
    ) -> Callable[[], None]:
        return self.inner.subscribe(  # type: ignore[no-any-return]
            lambda m: callback(m.decode()), selfstream
        )

    def start(self) -> None:
        self.inner.start()

    def stop(self) -> None:
        self.inner.stop()


class BenchSink(Module):
    """Records (recv_time, msg.ts, age, wire bytes) per frame.

    sink_mode "decode" decodes every frame (and burns work_ms of real cv2 cpu
    on the pixels, like a busy detector); "bytes" never decodes.
    """

    color_image: In[CompressedImage]

    def __init__(
        self,
        out_path: str = "",
        work_ms: float = 0.0,
        sink_mode: str = "decode",
        **kwargs: Any,
    ) -> None:
        super().__init__(**kwargs)
        self._out_path = out_path
        self._work_ms = work_ms
        self._sink_mode = sink_mode
        self._buf: list[str] = []
        self._count = 0
        self._lock = threading.Lock()

    @rpc
    def start(self) -> None:
        super().start()
        self.color_image.subscribe(self._on_image)

    @rpc
    def stop(self) -> None:
        self._flush()
        super().stop()

    def _on_image(self, msg: CompressedImage) -> None:
        now = time.time()
        self._count += 1
        rec = {
            "t": now,
            "ts": msg.ts,
            "age": now - msg.ts,
            "i": self._count,
            "bytes": len(msg.data),
        }
        if self._sink_mode == "decode":
            img = msg.decode()
            if self._work_ms > 0:
                import cv2

                deadline = time.perf_counter() + self._work_ms / 1000
                while time.perf_counter() < deadline:
                    cv2.GaussianBlur(img.data, (31, 31), 5)
        rec["done_t"] = time.time()
        with self._lock:
            self._buf.append(json.dumps(rec))
            if len(self._buf) >= 20:
                self._flush()

    def _flush(self) -> None:
        if not self._out_path or not self._buf:
            return
        with open(self._out_path, "a") as f:
            f.write("\n".join(self._buf) + "\n")
        self._buf = []


# distinct classes so several sinks can live in one blueprint (fan-out)
class BenchSink1(BenchSink): ...


class BenchSink2(BenchSink): ...


class BenchSink3(BenchSink): ...


SINK_CLASSES = [BenchSink, BenchSink1, BenchSink2, BenchSink3]


def _sample_host(out_path: str, stop: threading.Event) -> None:
    import psutil

    me = psutil.Process()
    procs: dict[int, psutil.Process] = {}
    # multicast may ride any interface (lo lacks MULTICAST on some boxes)
    nic0 = psutil.net_io_counters(pernic=True)
    last = ({k: v.bytes_sent for k, v in nic0.items()}, time.time())
    with open(out_path, "a") as f:
        while not stop.wait(1.0):
            tree = [me, *me.children(recursive=True)]
            cpu = rss = 0.0
            for p in tree:
                try:
                    if p.pid not in procs:
                        procs[p.pid] = p
                        p.cpu_percent(None)  # prime
                        continue
                    cpu += p.cpu_percent(None)
                    rss += p.memory_info().rss
                except psutil.NoSuchProcess:
                    procs.pop(p.pid, None)
            nics = psutil.net_io_counters(pernic=True)
            now = time.time()
            dt = now - last[1]
            rates = {k: (v.bytes_sent - last[0].get(k, 0)) / dt * 8 / 1e6 for k, v in nics.items()}
            last = ({k: v.bytes_sent for k, v in nics.items()}, now)
            f.write(
                json.dumps(
                    {
                        "t": now,
                        "cpu_pct": cpu,
                        "rss_mb": rss / 1e6,
                        "lo_mbps": rates.get("lo", 0),
                        "net_mbps": {k: round(r, 3) for k, r in rates.items() if r > 0.1},
                    }
                )
                + "\n"
            )
            f.flush()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--blueprint", default="unitree-go2")
    ap.add_argument("--sink", choices=["decode", "bytes"], default="decode")
    ap.add_argument("--sinks", type=int, default=1)
    ap.add_argument("--work-ms", type=float, default=0.0)
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "meta.json").write_text(json.dumps(vars(args)))

    from dimos.core.coordination.blueprints import autoconnect
    from dimos.core.coordination.module_coordinator import ModuleCoordinator
    from dimos.robot.get_all_blueprints import get_blueprint_by_name
    from dimos.visualization.rerun.bridge import RerunBridgeModule

    # no viewer bridge: it serves the shared viewer gRPC port and hijacks any
    # open viewer on this box; also run benchmarks with a dedicated LCM_DEFAULT_URL
    bp = get_blueprint_by_name(args.blueprint).disabled_modules(RerunBridgeModule)
    sinks = [
        cls.blueprint(
            out_path=str(out / f"sink{i}.jsonl"), work_ms=args.work_ms, sink_mode=args.sink
        )
        for i, cls in enumerate(SINK_CLASSES[: args.sinks])
    ]
    bp = autoconnect(bp, *sinks)

    coordinator = ModuleCoordinator.build(bp, {"g": {"replay": True, "viewer": "none"}})
    logger.info("benchmark run started", sink=args.sink, blueprint=args.blueprint)

    stop = threading.Event()
    sampler = threading.Thread(target=_sample_host, args=(str(out / "host.jsonl"), stop))
    sampler.start()
    try:
        time.sleep(args.duration)
    finally:
        stop.set()
        sampler.join(timeout=5)
        stopper = threading.Thread(target=coordinator.stop, daemon=True)
        stopper.start()
        stopper.join(timeout=30)
        (out / "done").write_text("ok")
        if stopper.is_alive():
            logger.warning("coordinator.stop() hung; hard exit")
            os._exit(0)


if __name__ == "__main__":
    main()
