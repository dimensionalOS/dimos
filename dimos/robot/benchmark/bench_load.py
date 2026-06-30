#!/usr/bin/env python3

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

"""Bench-load: a synthetic high-rate publisher you can trigger from the browser SDK.

This is the coordinator-wired, RPC-controllable version of bench/bench_source.py.
It floods user-defined topics with timestamped load so the @dimos/topics SDK (and the
headless bench harness) can measure latency / throughput / bandwidth end-to-end:

  /bench/p0../p3  PoseStamped  @ hz       (small, high-rate, ts-stamped for e2e latency)
  /bench/grid     OccupancyGrid @ grid_hz (large ~3.7KB)

It runs under the normal coordinator (lifecycle + RPC), and works over either transport
(DIMOS_TRANSPORT=lcm | zenoh). It pins the /bench/* channels so the wire names match the
existing bench tooling on both backends (see _BENCH_TRANSPORTS below).

Blueprints (registered in dimos/robot/all_blueprints.py):
  bench-load   standalone flood            DIMOS_TRANSPORT=zenoh uv run dimos run bench-load
  go2-bench    unitree-go2 + bench load     DIMOS_TRANSPORT=zenoh uv run dimos --simulation dimsim run go2-bench

The flood is idle until triggered (unless autostart=True). Trigger from the browser SDK:
  client.call("BenchLoad", "start_bench", [50])   # 50 Hz
  client.call("BenchLoad", "stop_bench")
(whitelisted in servers/gateway_zenoh.py). Or autostart via
  uv run dimos run bench-load --option benchload.autostart=true
"""

import time

import numpy as np
import reactivex as rx

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.global_config import global_config
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.core.transport import LCMTransport, ZenohTransport
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import unitree_go2

_GRID = ((np.random.rand(60, 60) > 0.85).astype(np.int8)) * 100
_IDENT = Quaternion.from_euler(Vector3(0.0, 0.0, 0.0))

# Per-frame size hard cap. The "large stream" axis is sustained THROUGHPUT (heavy_hz * heavy_bytes),
# not one giant packet -- keep individual frames realistic (<= a raw 4K-ish frame) and reach high MB/s
# via the rate, so we exercise the pipeline, not a single-message pathology.
_MAX_HEAVY_BYTES = 12_000_000


def _make_image(nbytes: int) -> Image:
    """A raw RGB Image of ~nbytes. Built memory-safe: a single uint8 buffer (NOT
    np.arange(int64)%256, which allocs 8x and would blow up at large sizes)."""
    nbytes = max(3, min(int(nbytes), _MAX_HEAVY_BYTES))
    px = max(1, nbytes // 3)
    h = max(1, int(px**0.5))
    w = max(1, px // h)
    data = np.zeros((h, w, 3), dtype=np.uint8)
    return Image(data=data, format=ImageFormat.RGB)


def _make_cloud(nbytes: int) -> PointCloud2:
    """A PointCloud2 of ~nbytes -- the realistic lidar/depth firehose. Sized by point
    count (xyz float32 = 12 B/point; this is what the wire encode serializes). Built once."""
    import open3d as o3d  # lazy: only the cloud path needs Open3D

    nbytes = max(12, min(int(nbytes), _MAX_HEAVY_BYTES))
    n = max(1, nbytes // 12)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.zeros((n, 3), dtype=np.float64))
    return PointCloud2(pointcloud=pcd)


class BenchLoadConfig(ModuleConfig):
    hz: float = 100.0  # PoseStamped rate (per topic)
    grid_hz: float = 20.0  # OccupancyGrid rate
    heavy_hz: float = 0.0  # large-stream rate (0 = off); throughput = heavy_hz * heavy_bytes
    heavy_bytes: int = 1_000_000  # per-frame size of the large stream (<= _MAX_HEAVY_BYTES)
    heavy_kind: str = "image"  # "image" (raw RGB frame) | "cloud" (PointCloud2 lidar)
    autostart: bool = False  # start flooding on module start (else wait for start_bench)


class BenchLoad(Module):
    """Synthetic load generator with start_bench/stop_bench RPC controls."""

    config: BenchLoadConfig

    p0: Out[PoseStamped]
    p1: Out[PoseStamped]
    p2: Out[PoseStamped]
    p3: Out[PoseStamped]
    grid: Out[OccupancyGrid]
    img: Out[Image]  # large stream -- raw RGB frame (heavy_kind="image")
    cloud: Out[PointCloud2]  # large stream -- lidar/depth cloud (heavy_kind="cloud")

    @rpc
    def start(self) -> None:
        super().start()
        self._seq = 0
        self._grid_seq = 0
        self._bench_disp: list = []
        if self.config.autostart:
            self.start_bench()

    @rpc
    def stop(self) -> None:
        self.stop_bench()
        super().stop()

    @rpc
    def start_bench(
        self,
        hz: float | None = None,
        grid_hz: float | None = None,
        heavy_hz: float | None = None,
        heavy_bytes: int | None = None,
        heavy_kind: str | None = None,
    ) -> str:
        """(Re)start the synthetic flood, optionally overriding rates/sizes at runtime.

        The large stream (`heavy_*`) is OFF unless `heavy_hz > 0`. Throughput is
        `heavy_hz * heavy_bytes` -- keep frames realistic (<= _MAX_HEAVY_BYTES) and reach
        high MB/s via the rate. `heavy_kind`: "image" (raw RGB frame) | "cloud" (PointCloud2).
        """
        self.stop_bench()  # idempotent — never double-subscribe
        h = float(hz) if hz else self.config.hz
        gh = float(grid_hz) if grid_hz else self.config.grid_hz
        hh = float(heavy_hz) if heavy_hz is not None else self.config.heavy_hz
        hb = int(heavy_bytes) if heavy_bytes else self.config.heavy_bytes
        hk = (heavy_kind or self.config.heavy_kind).lower()
        ports = [self.p0, self.p1, self.p2, self.p3]

        def tick(_: int) -> None:
            now = time.time()
            self._seq += 1
            for i, p in enumerate(ports):
                p.publish(
                    PoseStamped(
                        ts=now,  # source timestamp → end-to-end latency
                        frame_id=str(
                            self._seq
                        ),  # per-topic seq → SDK loss detection (matches bench/bench_source.py)
                        position=(self._seq * 0.001, float(i), 0.0),
                        orientation=_IDENT,
                    )
                )

        def grid_tick(_: int) -> None:
            self._grid_seq += 1
            self.grid.publish(
                OccupancyGrid(
                    grid=_GRID,
                    resolution=0.1,
                    origin=Pose(-3.0, -3.0, 0.0),
                    frame_id=str(self._grid_seq),
                )
            )

        self._bench_disp = [
            rx.interval(1.0 / h).subscribe(tick),
            rx.interval(1.0 / gh).subscribe(grid_tick),
        ]

        heavy_msg = ""
        if hh > 0:
            # Build the frame ONCE; only restamp ts/frame_id per publish so generation
            # cost never dominates (mirrors bench/bench_source.py).
            self._heavy_seq = 0
            if hk == "cloud":
                cloud = _make_cloud(hb)
                port = self.cloud

                def heavy_tick(_: int) -> None:
                    self._heavy_seq += 1
                    cloud.ts = time.time()
                    cloud.frame_id = str(self._heavy_seq)
                    port.publish(cloud)
            else:
                hk = "image"
                image = _make_image(hb)
                port = self.img

                def heavy_tick(_: int) -> None:
                    self._heavy_seq += 1
                    image.ts = time.time()
                    image.frame_id = str(self._heavy_seq)
                    port.publish(image)

            self._bench_disp.append(rx.interval(1.0 / hh).subscribe(heavy_tick))
            mb_s = (hh * min(hb, _MAX_HEAVY_BYTES)) / 1e6
            heavy_msg = f" + {hk} @ {hh}Hz x {hb}B (~{mb_s:.1f} MB/s)"

        return f"bench started: 4x PoseStamped @ {h}Hz + grid @ {gh}Hz{heavy_msg}"

    @rpc
    def stop_bench(self) -> str:
        """Halt the flood (dispose the interval subscriptions)."""
        for d in getattr(self, "_bench_disp", []):
            d.dispose()
        self._bench_disp = []
        return "bench stopped"


# Pin the /bench/* wire names the SDK + bench harness + zenoh gateway (default ZENOH_KEY=bench/**)
# expect. Mirror bench/bench_source.py exactly: zenoh key "bench/x" (NO dimos/ namespace), LCM
# channel "/bench/x". Pinning the transport that matches the active backend keeps the coordinator's
# _coerce_transport_to_backend from re-adding the dimos/ prefix via the factory.
def _bench_transport(topic: str, typ: type) -> object:
    if global_config.transport == "zenoh":
        return ZenohTransport(topic.lstrip("/"), typ)  # key: bench/x
    return LCMTransport(topic, typ)  # channel: /bench/x


_BENCH_TRANSPORTS = {
    ("p0", PoseStamped): _bench_transport("/bench/p0", PoseStamped),
    ("p1", PoseStamped): _bench_transport("/bench/p1", PoseStamped),
    ("p2", PoseStamped): _bench_transport("/bench/p2", PoseStamped),
    ("p3", PoseStamped): _bench_transport("/bench/p3", PoseStamped),
    ("grid", OccupancyGrid): _bench_transport("/bench/grid", OccupancyGrid),
    ("img", Image): _bench_transport("/bench/img", Image),
    ("cloud", PointCloud2): _bench_transport("/bench/cloud", PointCloud2),
}

# Standalone flood.
bench_load = BenchLoad.blueprint().transports(_BENCH_TRANSPORTS)

# unitree-go2 sim + bench load on the same transport (custom topics alongside the real robot).
go2_bench = autoconnect(unitree_go2, bench_load)
