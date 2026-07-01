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

"""GO2Load — the one synthetic load module behind the dimoscope dog demo.

Merges the former ScopeBench (fixed multi-rate lanes; the QoS / on-demand demo) and BenchLoad (a
crankable heavy flood; the transport crash-ladder benchmark) into ONE module on ONE /load/* namespace:

  /load/fast   PoseStamped   100 Hz   light, high-rate      ┐
  /load/mid    PoseStamped    20 Hz   mid                   │ fixed lanes — a spread of QoS lanes the
  /load/slow   PoseStamped     2 Hz   slow (obvious)        │ Streams tab shows at distinct Hz; each
  /load/grid   OccupancyGrid   5 Hz   medium (~3.7 KB)      │ on-demand-toggleable while teleoperating
  /load/cloud  PointCloud2    10 Hz   heavy (PointCloud2)   ┘
  /load/img    Image           off    the overload flood → firehose (start_bench)

Two RPC surfaces, both browser-driven (Commands buttons / client.call), both over the gateway whitelist:
  start_all / stop_all / enable(t) / disable(t) / set_rate(t, hz) / status()   — the fixed lanes
  start_bench(heavy_hz, heavy_bytes, heavy_kind) / stop_bench()                — the /load/img flood

Composed with unitree_go2 into the SINGLE registered dog blueprint `go2-load`; also exported standalone
as `load` (no sim) for benchmarking without the dimsim/dog stack:

  DIMOS_TRANSPORT=zenoh uv run dimos --simulation dimsim run go2-load   # the dog (deno task dog)
  DIMOS_TRANSPORT=zenoh uv run dimos run load                          # standalone flood, no sim

Wire names are pinned the same way on both backends (zenoh key "load/x" — NO dimos/ namespace — vs LCM
channel "/load/x"), so the browser sees identical topic names. The @dimos/web bench harness
(STREAM_PROFILES) + the Streams-tab `load` preset key off these names.
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

_IDENT = Quaternion.from_euler(Vector3(0.0, 0.0, 0.0))
_GRID = ((np.random.rand(60, 60) > 0.85).astype(np.int8)) * 100

# Per-frame size hard cap. The overload axis is sustained THROUGHPUT (heavy_hz * heavy_bytes), not one
# giant packet -- keep individual frames realistic (<= a raw 4K-ish frame) and reach high MB/s via rate.
_MAX_HEAVY_BYTES = 12_000_000


def _make_cloud(n_points: int) -> PointCloud2:
    """A PointCloud2 of ~n_points (xyz float32). Built once; only ts/frame_id restamped per publish."""
    import open3d as o3d  # lazy: only the cloud path needs Open3D

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.zeros((max(1, int(n_points)), 3), dtype=np.float64))
    return PointCloud2(pointcloud=pcd)


def _make_image(nbytes: int) -> Image:
    """A raw RGB Image of ~nbytes. Built memory-safe: a single uint8 buffer (NOT
    np.arange(int64)%256, which allocs 8x and would blow up at large sizes)."""
    nbytes = max(3, min(int(nbytes), _MAX_HEAVY_BYTES))
    px = max(1, nbytes // 3)
    h = max(1, int(px**0.5))
    w = max(1, px // h)
    data = np.zeros((h, w, 3), dtype=np.uint8)
    return Image(data=data, format=ImageFormat.RGB)


class GO2LoadConfig(ModuleConfig):
    # Fixed lanes (the QoS / on-demand demo).
    fast_hz: float = 100.0  # PoseStamped, high-rate light
    mid_hz: float = 20.0  # PoseStamped, mid
    slow_hz: float = 2.0  # PoseStamped, slow (obvious in the metrics)
    grid_hz: float = 5.0  # OccupancyGrid, medium payload
    cloud_hz: float = 10.0  # PointCloud2, heavy
    cloud_points: int = 20_000
    autostart: bool = True  # run the fixed lanes on start (else wait for start_all)
    # The overload flood (the crash-ladder benchmark) -- OFF until start_bench(heavy_hz > 0).
    heavy_hz: float = 0.0  # /load/img rate; throughput = heavy_hz * heavy_bytes
    heavy_bytes: int = 1_000_000  # per-frame size of the flood (<= _MAX_HEAVY_BYTES)
    heavy_kind: str = "image"  # "image" (raw RGB → /load/img) | "cloud" (PointCloud2 → /load/cloud)


class GO2Load(Module):
    """Fixed multi-rate /load/* lanes (QoS demo) + a crankable /load/img flood (benchmark)."""

    config: GO2LoadConfig

    fast: Out[PoseStamped]  # 100 Hz
    mid: Out[PoseStamped]  # 20 Hz
    slow: Out[PoseStamped]  # 2 Hz
    grid: Out[OccupancyGrid]  # 5 Hz
    cloud: Out[PointCloud2]  # 10 Hz (heavy lane; also the cloud-kind flood target)
    img: Out[Image]  # the overload flood (raw RGB frame), off until start_bench

    @rpc
    def start(self) -> None:
        super().start()
        self._seq: dict[str, int] = {}
        self._active: dict = {}  # name -> rx Disposable (the live lane interval)
        self._rate: dict[str, float] = {}  # name -> live Hz
        self._cloud = _make_cloud(self.config.cloud_points)
        self._heavy_disp = None  # the /load/img flood disposable (or None)
        self._bench_kind: str | None = None  # "image" | "cloud" | None
        # name -> (port, default_hz, tick-factory)
        self._streams = {
            "fast": (self.fast, self.config.fast_hz, self._pose_tick),
            "mid": (self.mid, self.config.mid_hz, self._pose_tick),
            "slow": (self.slow, self.config.slow_hz, self._pose_tick),
            "grid": (self.grid, self.config.grid_hz, self._grid_tick),
            "cloud": (self.cloud, self.config.cloud_hz, self._cloud_tick),
        }
        if self.config.autostart:
            self.start_all()
            if self.config.heavy_hz > 0:  # config-driven flood (headless / tests); else wait for start_bench
                self.start_bench()

    @rpc
    def stop(self) -> None:
        self.stop_bench()
        self.stop_all()
        super().stop()

    # ── tick factories (each restamps ts + frame_id=seq for e2e latency / loss detection) ──
    def _next(self, name: str) -> int:
        self._seq[name] = self._seq.get(name, 0) + 1
        return self._seq[name]

    def _pose_tick(self, name: str, port):  # type: ignore[no-untyped-def]
        def tick(_: int) -> None:
            n = self._next(name)
            port.publish(
                PoseStamped(
                    ts=time.time(),
                    frame_id=str(n),
                    position=(n * 0.001, 0.0, 0.0),
                    orientation=_IDENT,
                )
            )

        return tick

    def _grid_tick(self, name: str, port):  # type: ignore[no-untyped-def]
        def tick(_: int) -> None:
            port.publish(
                OccupancyGrid(
                    grid=_GRID,
                    resolution=0.1,
                    origin=Pose(-3.0, -3.0, 0.0),
                    frame_id=str(self._next(name)),
                )
            )

        return tick

    def _cloud_tick(self, name: str, port):  # type: ignore[no-untyped-def]
        def tick(_: int) -> None:
            self._cloud.ts = time.time()
            self._cloud.frame_id = str(self._next(name))
            port.publish(self._cloud)

        return tick

    # ── per-lane start/stop (self-managed disposables) ──
    def _start_topic(self, name: str, hz: float) -> None:
        self._stop_topic(name)
        if hz <= 0:
            return
        port, _, factory = self._streams[name]
        self._active[name] = rx.interval(1.0 / hz).subscribe(factory(name, port))
        self._rate[name] = hz

    def _stop_topic(self, name: str) -> None:
        d = self._active.pop(name, None)
        if d is not None:
            d.dispose()
        self._rate.pop(name, None)

    def _norm(self, topic: str) -> str:
        name = str(topic).rsplit("/", 1)[-1]
        if name not in self._streams:
            raise ValueError(f"unknown load topic: {topic}")
        return name

    # ── fixed-lane RPC surface (the QoS / on-demand demo) ──
    @rpc
    def start_all(self) -> str:
        for name, (_, hz, _) in self._streams.items():
            self._start_topic(name, hz)
        return f"load streams started: {', '.join(self._active)}"

    @rpc
    def stop_all(self) -> str:
        for name in list(self._active):
            self._stop_topic(name)
        return "load streams stopped"

    @rpc
    def enable(self, topic: str) -> str:
        name = self._norm(topic)
        self._start_topic(name, self._streams[name][1])
        return f"enabled {name}"

    @rpc
    def disable(self, topic: str) -> str:
        name = self._norm(topic)
        self._stop_topic(name)
        return f"disabled {name}"

    @rpc
    def set_rate(self, topic: str, hz: float) -> str:
        name = self._norm(topic)
        self._start_topic(name, float(hz))
        return f"{name} @ {hz} Hz"

    @rpc
    def status(self) -> dict:
        return {name: self._rate.get(name, 0.0) for name in self._streams}

    # ── the overload flood (the transport crash-ladder benchmark) ──
    @rpc
    def start_bench(
        self,
        heavy_hz: float | None = None,
        heavy_bytes: int | None = None,
        heavy_kind: str | None = None,
    ) -> str:
        """(Re)start the heavy flood. OFF unless heavy_hz > 0. Throughput = heavy_hz * heavy_bytes;
        keep frames realistic (<= _MAX_HEAVY_BYTES) and reach high MB/s via the rate.
        heavy_kind: "image" (raw RGB → /load/img) | "cloud" (PointCloud2 → /load/cloud lane)."""
        self.stop_bench()  # idempotent — never double-subscribe
        hh = float(heavy_hz) if heavy_hz is not None else self.config.heavy_hz
        hb = int(heavy_bytes) if heavy_bytes else self.config.heavy_bytes
        hk = (heavy_kind or self.config.heavy_kind).lower()
        if hh <= 0:
            return "bench off (heavy_hz=0)"
        mb_s = (hh * min(hb, _MAX_HEAVY_BYTES)) / 1e6
        if hk == "cloud":
            # Crank the /load/cloud lane: bigger cloud at the flood rate (stop_bench restores the default).
            self._cloud = _make_cloud(max(1, min(hb, _MAX_HEAVY_BYTES) // 12))
            self._start_topic("cloud", hh)
            self._bench_kind = "cloud"
            return f"bench: cloud @ {hh}Hz x {hb}B (~{mb_s:.1f} MB/s) on /load/cloud"
        # image (default): the dedicated /load/img firehose port.
        self._img = _make_image(hb)
        self._heavy_seq = 0

        def heavy_tick(_: int) -> None:
            self._heavy_seq += 1
            self._img.ts = time.time()
            self._img.frame_id = str(self._heavy_seq)
            self.img.publish(self._img)

        self._heavy_disp = rx.interval(1.0 / hh).subscribe(heavy_tick)
        self._bench_kind = "image"
        return f"bench: image @ {hh}Hz x {hb}B (~{mb_s:.1f} MB/s) on /load/img"

    @rpc
    def stop_bench(self) -> str:
        """Halt the flood (dispose the /load/img interval; restore the /load/cloud lane if cranked)."""
        d = getattr(self, "_heavy_disp", None)
        if d is not None:
            d.dispose()
        self._heavy_disp = None
        if getattr(self, "_bench_kind", None) == "cloud":
            self._cloud = _make_cloud(self.config.cloud_points)
            self._start_topic("cloud", self.config.cloud_hz)  # restore the fixed lane
        self._bench_kind = None
        return "bench stopped"


# Pin the /load/* wire names so the browser sees the same names on both backends: zenoh key "load/x"
# (NO dimos/ namespace — pinning the matching-backend transport keeps the coordinator's
# _coerce_transport_to_backend from re-adding the dimos/ prefix), LCM channel "/load/x".
def _load_transport(topic: str, typ: type) -> object:
    if global_config.transport == "zenoh":
        return ZenohTransport(topic.lstrip("/"), typ)  # key: load/x
    return LCMTransport(topic, typ)  # channel: /load/x


_LOAD_TRANSPORTS = {
    ("fast", PoseStamped): _load_transport("/load/fast", PoseStamped),
    ("mid", PoseStamped): _load_transport("/load/mid", PoseStamped),
    ("slow", PoseStamped): _load_transport("/load/slow", PoseStamped),
    ("grid", OccupancyGrid): _load_transport("/load/grid", OccupancyGrid),
    ("cloud", PointCloud2): _load_transport("/load/cloud", PointCloud2),
    ("img", Image): _load_transport("/load/img", Image),
}

# THE dog: teleoperable go2 dimsim + our multi-rate /load/* lanes + the crankable flood, one name.
go2_load = autoconnect(unitree_go2, GO2Load.blueprint().transports(_LOAD_TRANSPORTS))

# Standalone (no sim) — the lighter interactive-benchmark source (coordinator-wired, RPC-controllable).
load = GO2Load.blueprint().transports(_LOAD_TRANSPORTS)
