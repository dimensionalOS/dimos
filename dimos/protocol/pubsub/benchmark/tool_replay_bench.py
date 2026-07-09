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

"""End-to-end replay benchmark: raw Image blueprints vs CompressedCodec.

Runs a real blueprint in replay mode for a fixed duration, with optional
BenchSink consumer modules (configurable synthetic per-frame work, like a
busy detector), and records per-frame freshness plus host CPU/RSS/loopback
traffic. One process per run:

    python -m dimos.protocol.pubsub.benchmark.tool_replay_bench \
        --blueprint unitree-go2 --mode codec --sinks 4 --work-ms 20 \
        --duration 60 --out /tmp/bench/go2_codec_0

Wrap with `taskset -c 0-3` / `tc netem` for constrained profiles.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import threading
import time

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In
from dimos.msgs.sensor_msgs.Image import Image
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class BenchSink(Module):
    """Records (recv_time, msg.ts, age) per frame; burns work_ms of real cpu."""

    color_image: In[Image]

    def __init__(self, out_path: str = "", work_ms: float = 0.0, **kwargs) -> None:
        super().__init__(**kwargs)
        self._out_path = out_path
        self._work_ms = work_ms
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

    def _on_image(self, msg: Image) -> None:
        now = time.time()
        self._count += 1
        rec = {"t": now, "ts": msg.ts, "age": now - msg.ts, "i": self._count}
        if self._work_ms > 0:
            import cv2

            deadline = time.perf_counter() + self._work_ms / 1000
            while time.perf_counter() < deadline:
                cv2.GaussianBlur(msg.data, (31, 31), 5)
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
    lo0 = psutil.net_io_counters(pernic=True).get("lo")
    last_lo = (lo0.bytes_sent if lo0 else 0, time.time())
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
            lo = psutil.net_io_counters(pernic=True).get("lo")
            now = time.time()
            lo_rate = (lo.bytes_sent - last_lo[0]) / (now - last_lo[1]) if lo else 0
            last_lo = (lo.bytes_sent if lo else 0, now)
            f.write(
                json.dumps(
                    {"t": now, "cpu_pct": cpu, "rss_mb": rss / 1e6, "lo_mbps": lo_rate * 8 / 1e6}
                )
                + "\n"
            )
            f.flush()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--blueprint", default="unitree-go2")
    ap.add_argument("--mode", choices=["raw", "codec"], default="raw")
    ap.add_argument("--quality", type=int, default=75)
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
    from dimos.core.transport import CompressedCodec, LCMTransport
    from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
    from dimos.robot.get_all_blueprints import get_blueprint_by_name

    bp = get_blueprint_by_name(args.blueprint)
    sinks = [
        cls.blueprint(out_path=str(out / f"sink{i}.jsonl"), work_ms=args.work_ms)
        for i, cls in enumerate(SINK_CLASSES[: args.sinks])
    ]
    bp = autoconnect(bp, *sinks)
    if args.mode == "codec":
        bp = bp.transports(
            {
                ("color_image", Image): CompressedCodec(
                    LCMTransport("/color_image", CompressedImage), quality=args.quality
                )
            }
        )

    coordinator = ModuleCoordinator.build(bp, {"g": {"replay": True, "viewer": "none"}})
    logger.info("benchmark run started", mode=args.mode, blueprint=args.blueprint)

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
