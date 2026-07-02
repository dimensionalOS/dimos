#!/usr/bin/env python3
# Benchmark load generator for the in-browser bench: a configurable dimos Module emitting timestamped,
# seq-tagged streams so the browser can measure latency/throughput/jitter/loss across transports.
# Tuned by env (BENCH_HZ, BENCH_MID_HZ, BENCH_SLOW_HZ, BENCH_GRID_HZ, BENCH_IMG_HZ, BENCH_IMG_BYTES);
# also a blueprint. Ultra-light (no coordinator) twin of the `load`/`go2-load` blueprint
# (dimos/robot/benchmark/go2_load.py): same /load/* wire, env-tuned. For interactive start_bench
# control use `dimos run load` / `deno task dog`.
#
# Topics (names match the STREAM_PROFILES in packages/web/src/bench.ts and go2_load's lanes; leading
# "/" stripped on zenoh). Each stamps `ts` (publish wall-clock, seconds) → one-way latency, and
# `frame_id=str(seq)` (per-topic counter) → drop/gap detection:
#   /load/fast|mid|slow  PoseStamped   @ fast/mid/slow_hz  small lanes at distinct rates
#   /load/grid           OccupancyGrid @ grid_hz           medium (~3.7 KB)
#   /load/img            Image         @ img_hz            large (~img_bytes)
import os
import time

import numpy as np
import reactivex as rx

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat

TRANSPORT = os.environ.get("DIMOS_TRANSPORT", "lcm")


def _env_f(key: str, default: float) -> float:
    return float(os.environ.get(key, default))


def _env_i(key: str, default: int) -> int:
    return int(os.environ.get(key, default))


class BenchSourceConfig(ModuleConfig):
    # Defaults read from env so the standalone harness and `dimos run --option` agree
    # (an explicit --option still overrides; the env value is only the default). Lane rates
    # default to the go2_load spread the STREAM_PROFILES hints describe (0 = lane off).
    fast_hz: float = _env_f("BENCH_HZ", 100)  # /load/fast PoseStamped
    mid_hz: float = _env_f("BENCH_MID_HZ", 20)  # /load/mid PoseStamped
    slow_hz: float = _env_f("BENCH_SLOW_HZ", 2)  # /load/slow PoseStamped
    grid_hz: float = _env_f("BENCH_GRID_HZ", 5)  # OccupancyGrid
    img_hz: float = _env_f("BENCH_IMG_HZ", 0)  # large Image
    img_bytes: int = _env_i("BENCH_IMG_BYTES", 1_000_000)  # ~payload size when img_hz > 0


def make_image(nbytes: int) -> Image:
    """An RGB Image of ~nbytes bytes. Built once, then reused (only `ts`/`frame_id` restamped per
    publish). RANDOM bytes, not a pattern: WS permessage-deflate compresses regular payloads ~1000:1,
    which turns any throughput measurement into fiction (random ≈ real camera/lidar entropy)."""
    px = max(1, nbytes // 3)
    h = max(1, int(px**0.5))
    w = max(1, px // h)
    data = np.random.default_rng(0).integers(0, 256, (h, w, 3), dtype=np.uint8)
    return Image(data=data, format=ImageFormat.RGB)


GRID = ((np.random.rand(60, 60) > 0.85).astype(np.int8)) * 100
IDENT = Quaternion.from_euler(Vector3(0.0, 0.0, 0.0))


class BenchSource(Module):
    """Configurable high-rate load generator for transport/mechanism benchmarking."""

    config: BenchSourceConfig
    fast: Out[PoseStamped]
    mid: Out[PoseStamped]
    slow: Out[PoseStamped]
    grid: Out[OccupancyGrid]
    img: Out[Image]

    @rpc
    def start(self) -> None:
        c = self.config
        self._seq: dict[str, int] = {}

        def seq(topic: str) -> str:
            n = self._seq.get(topic, 0)
            self._seq[topic] = n + 1
            return str(n)

        def pose_lane(port: Out[PoseStamped], topic: str, hz: float, y: float) -> None:
            if hz <= 0:
                return

            def tick(_: int) -> None:
                port.publish(
                    PoseStamped(
                        ts=time.time(),
                        frame_id=seq(topic),
                        position=(0.0, y, 0.0),
                        orientation=IDENT,
                    )
                )

            self.register_disposable(rx.interval(1.0 / hz).subscribe(tick))

        pose_lane(self.fast, "/load/fast", c.fast_hz, 0.0)
        pose_lane(self.mid, "/load/mid", c.mid_hz, 1.0)
        pose_lane(self.slow, "/load/slow", c.slow_hz, 2.0)

        if c.grid_hz > 0:
            self.register_disposable(
                rx.interval(1.0 / c.grid_hz).subscribe(
                    lambda _: self.grid.publish(
                        OccupancyGrid(
                            grid=GRID,
                            resolution=0.1,
                            origin=Pose(-3.0, -3.0, 0.0),
                            frame_id=seq("/load/grid"),
                            ts=time.time(),
                        )
                    )
                )
            )

        if c.img_hz > 0:
            self._img = make_image(c.img_bytes)

            def tick_img(_: int) -> None:
                self._img.ts = time.time()
                self._img.frame_id = seq("/load/img")
                self.img.publish(self._img)

            self.register_disposable(rx.interval(1.0 / c.img_hz).subscribe(tick_img))


# Blueprint export (launch via `dimos run bench-source --option ...` once registered).
bench_source = BenchSource.blueprint()


def _tn(topic: str) -> str:
    # Zenoh key-exprs cannot start with "/"; LCM channels keep it. The Zenoh gateway
    # re-adds the leading "/" so browser topic names match either way.
    return topic[1:] if (TRANSPORT == "zenoh" and topic.startswith("/")) else topic


def _mk(topic: str, typ: type):  # type: ignore[no-untyped-def]
    if TRANSPORT == "zenoh":
        from dimos.core.transport import ZenohTransport

        return ZenohTransport(topic, typ)
    from dimos.core.transport import LCMTransport

    return LCMTransport(topic, typ)


if __name__ == "__main__":
    mod = BenchSource()
    c = mod.config
    mod.fast.transport = _mk(_tn("/load/fast"), PoseStamped)
    mod.mid.transport = _mk(_tn("/load/mid"), PoseStamped)
    mod.slow.transport = _mk(_tn("/load/slow"), PoseStamped)
    mod.grid.transport = _mk(_tn("/load/grid"), OccupancyGrid)
    mod.img.transport = _mk(_tn("/load/img"), Image)
    mod.start()
    print(
        f"bench_source: /load/fast@{c.fast_hz} mid@{c.mid_hz} slow@{c.slow_hz} "
        f"+ grid@{c.grid_hz}Hz + img({c.img_bytes}B)@{c.img_hz}Hz over {TRANSPORT}",
        flush=True,
    )
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        mod.stop()
