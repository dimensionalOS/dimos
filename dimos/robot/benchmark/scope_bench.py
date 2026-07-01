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

"""ScopeBench — a multi-rate synthetic stream source for the `go2-scope` demo blueprint.

Publishes several /scope/* topics at DISTINCT rates (2-100 Hz, light to heavy) so the browser Streams
shows a spread of QoS lanes + live metrics, and you can toggle each on-demand while teleoperating the
robot. Per-topic RPCs drive the source from the browser (Commands buttons / client.call) or a script:

  start_all / stop_all            (arg-less → browser buttons)
  enable(t) / disable(t)          per-topic source on/off ("fast" or "/scope/fast")
  set_rate(t, hz)                 change a topic's rate live
  status() -> {name: hz}          which topics are active + at what rate

Composed with unitree_go2 into the SINGLE registered blueprint `go2-scope` (mirrors go2-bench):

  DIMOS_TRANSPORT=zenoh uv run dimos --simulation dimsim run go2-scope

Modeled on dimos/robot/benchmark/bench_load.py (same /bench-style transport pinning) and the multi-rate
scenario modules (dimos/web/dimoscope/scenarios/nav.py).
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
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import unitree_go2

_IDENT = Quaternion.from_euler(Vector3(0.0, 0.0, 0.0))
_GRID = ((np.random.rand(60, 60) > 0.85).astype(np.int8)) * 100


def _make_cloud(n_points: int) -> PointCloud2:
    """A PointCloud2 of ~n_points (xyz float32). Built once; only ts/frame_id restamped per publish."""
    import open3d as o3d  # lazy: only the cloud path needs Open3D

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.zeros((max(1, int(n_points)), 3), dtype=np.float64))
    return PointCloud2(pointcloud=pcd)


class ScopeBenchConfig(ModuleConfig):
    fast_hz: float = 100.0  # PoseStamped, high-rate light
    mid_hz: float = 20.0  # PoseStamped, mid
    slow_hz: float = 2.0  # PoseStamped, slow (obvious in the metrics)
    grid_hz: float = 5.0  # OccupancyGrid, medium payload
    cloud_hz: float = 10.0  # PointCloud2, heavy
    cloud_points: int = 20_000
    autostart: bool = True  # publish on module start (else wait for start_all)


class ScopeBench(Module):
    """Multi-rate /scope/* synthetic source with per-topic RPC controls."""

    config: ScopeBenchConfig

    fast: Out[PoseStamped]  # 100 Hz
    mid: Out[PoseStamped]  # 20 Hz
    slow: Out[PoseStamped]  # 2 Hz
    grid: Out[OccupancyGrid]  # 5 Hz
    cloud: Out[PointCloud2]  # 10 Hz (heavy)

    @rpc
    def start(self) -> None:
        super().start()
        self._seq: dict[str, int] = {}
        self._active: dict = {}  # name -> rx Disposable (the live interval)
        self._rate: dict[str, float] = {}  # name -> live Hz
        self._cloud = _make_cloud(self.config.cloud_points)
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

    @rpc
    def stop(self) -> None:
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

    # ── per-topic start/stop (self-managed disposables, like bench_load) ──
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
            raise ValueError(f"unknown scope topic: {topic}")
        return name

    # ── RPC surface ──
    @rpc
    def start_all(self) -> str:
        for name, (_, hz, _) in self._streams.items():
            self._start_topic(name, hz)
        return f"scope streams started: {', '.join(self._active)}"

    @rpc
    def stop_all(self) -> str:
        for name in list(self._active):
            self._stop_topic(name)
        return "scope streams stopped"

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


# Pin the /scope/* wire names so the browser sees the same names on both backends (mirror bench_load.py:
# zenoh key "scope/x" — NO dimos/ namespace — vs LCM channel "/scope/x").
def _scope_transport(topic: str, typ: type) -> object:
    if global_config.transport == "zenoh":
        return ZenohTransport(topic.lstrip("/"), typ)  # key: scope/x
    return LCMTransport(topic, typ)  # channel: /scope/x


_SCOPE_TRANSPORTS = {
    ("fast", PoseStamped): _scope_transport("/scope/fast", PoseStamped),
    ("mid", PoseStamped): _scope_transport("/scope/mid", PoseStamped),
    ("slow", PoseStamped): _scope_transport("/scope/slow", PoseStamped),
    ("grid", OccupancyGrid): _scope_transport("/scope/grid", OccupancyGrid),
    ("cloud", PointCloud2): _scope_transport("/scope/cloud", PointCloud2),
}

# THE special blueprint: teleoperable go2 dimsim + our multi-rate scope streams, one registered name.
go2_scope = autoconnect(unitree_go2, ScopeBench.blueprint().transports(_SCOPE_TRANSPORTS))
