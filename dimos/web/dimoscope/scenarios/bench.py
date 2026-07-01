#!/usr/bin/env python3
# Benchmark load generator for the in-browser bench: a configurable dimos Module emitting timestamped,
# seq-tagged streams so the browser can measure latency/throughput/jitter/loss across transports.
# Tuned by env (BENCH_HZ, BENCH_IMG_HZ, BENCH_IMG_BYTES, BENCH_POSE_TOPICS, ...); also a blueprint.
# Ultra-light (no coordinator) twin of the `load`/`go2-load` blueprint (dimos/robot/benchmark/go2_load.py):
# same /load/* wire, env-tuned. For interactive start_bench control use `dimos run load` / `deno task dog`.
#
# Topics (names match @dimos/web/bench; leading "/" stripped on zenoh). Each stamps `ts` (publish
# wall-clock, seconds) → one-way latency, and `frame_id=str(seq)` (per-topic counter) → drop/gap detection:
#   /load/p0../pN  PoseStamped   @ rate_hz   small, high-rate
#   /load/grid     OccupancyGrid @ grid_hz   medium (~3.7 KB)
#   /load/img      Image         @ img_hz    large (~img_bytes)
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
    # (an explicit --option still overrides; the env value is only the default).
    rate_hz: float = _env_f("BENCH_HZ", 100)  # each PoseStamped topic
    n_pose_topics: int = _env_i("BENCH_POSE_TOPICS", 4)  # 1..4
    grid_hz: float = _env_f("BENCH_GRID_HZ", 20)  # OccupancyGrid (0 = off)
    img_hz: float = _env_f("BENCH_IMG_HZ", 0)  # large Image (0 = off)
    img_bytes: int = _env_i("BENCH_IMG_BYTES", 1_000_000)  # ~payload size when img_hz > 0


def make_image(nbytes: int) -> Image:
    """An RGB Image of ~nbytes bytes. Built once with a fast numpy fill, then reused
    (only `ts`/`frame_id` are restamped per publish) so generation cost never dominates."""
    px = max(1, nbytes // 3)
    h = max(1, int(px**0.5))
    w = max(1, px // h)
    data = (np.arange(h * w * 3, dtype=np.int64) % 256).astype(np.uint8).reshape(h, w, 3)
    return Image(data=data, format=ImageFormat.RGB)


GRID = ((np.random.rand(60, 60) > 0.85).astype(np.int8)) * 100
IDENT = Quaternion.from_euler(Vector3(0.0, 0.0, 0.0))


class BenchSource(Module):
    """Configurable high-rate load generator for transport/mechanism benchmarking."""

    config: BenchSourceConfig
    p0: Out[PoseStamped]
    p1: Out[PoseStamped]
    p2: Out[PoseStamped]
    p3: Out[PoseStamped]
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

        n_pose = max(1, min(4, c.n_pose_topics))
        pose_ports = [self.p0, self.p1, self.p2, self.p3][:n_pose]

        def tick_pose(_: int) -> None:
            now = time.time()
            for i, port in enumerate(pose_ports):
                port.publish(
                    PoseStamped(
                        ts=now,
                        frame_id=seq(f"/load/p{i}"),
                        position=(0.0, float(i), 0.0),
                        orientation=IDENT,
                    )
                )

        if c.rate_hz > 0:  # rate_hz=0 → pose off (heavy-stream-only profiles)
            self.register_disposable(rx.interval(1.0 / c.rate_hz).subscribe(tick_pose))

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
    mod.p0.transport = _mk(_tn("/load/p0"), PoseStamped)
    mod.p1.transport = _mk(_tn("/load/p1"), PoseStamped)
    mod.p2.transport = _mk(_tn("/load/p2"), PoseStamped)
    mod.p3.transport = _mk(_tn("/load/p3"), PoseStamped)
    mod.grid.transport = _mk(_tn("/load/grid"), OccupancyGrid)
    mod.img.transport = _mk(_tn("/load/img"), Image)
    mod.start()
    print(
        f"bench_source: {max(1, min(4, c.n_pose_topics))}×PoseStamped @ {c.rate_hz}Hz "
        f"+ grid @ {c.grid_hz}Hz + img({c.img_bytes}B) @ {c.img_hz}Hz over {TRANSPORT}",
        flush=True,
    )
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        mod.stop()
