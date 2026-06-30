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
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import unitree_go2

_GRID = ((np.random.rand(60, 60) > 0.85).astype(np.int8)) * 100
_IDENT = Quaternion.from_euler(Vector3(0.0, 0.0, 0.0))


class BenchLoadConfig(ModuleConfig):
    hz: float = 100.0  # PoseStamped rate (per topic)
    grid_hz: float = 20.0  # OccupancyGrid rate
    autostart: bool = False  # start flooding on module start (else wait for start_bench)


class BenchLoad(Module):
    """Synthetic load generator with start_bench/stop_bench RPC controls."""

    config: BenchLoadConfig

    p0: Out[PoseStamped]
    p1: Out[PoseStamped]
    p2: Out[PoseStamped]
    p3: Out[PoseStamped]
    grid: Out[OccupancyGrid]

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
    def start_bench(self, hz: float | None = None, grid_hz: float | None = None) -> str:
        """(Re)start the synthetic flood, optionally overriding rates at runtime."""
        self.stop_bench()  # idempotent — never double-subscribe
        h = float(hz) if hz else self.config.hz
        gh = float(grid_hz) if grid_hz else self.config.grid_hz
        ports = [self.p0, self.p1, self.p2, self.p3]

        def tick(_: int) -> None:
            now = time.time()
            self._seq += 1
            for i, p in enumerate(ports):
                p.publish(
                    PoseStamped(
                        ts=now,  # source timestamp → end-to-end latency
                        frame_id=str(self._seq),  # per-topic seq → SDK loss detection (matches bench/bench_source.py)
                        position=(self._seq * 0.001, float(i), 0.0),
                        orientation=_IDENT,
                    )
                )

        def grid_tick(_: int) -> None:
            self._grid_seq += 1
            self.grid.publish(
                OccupancyGrid(
                    grid=_GRID, resolution=0.1, origin=Pose(-3.0, -3.0, 0.0), frame_id=str(self._grid_seq)
                )
            )

        self._bench_disp = [
            rx.interval(1.0 / h).subscribe(tick),
            rx.interval(1.0 / gh).subscribe(grid_tick),
        ]
        return f"bench started: 4x PoseStamped @ {h}Hz + grid @ {gh}Hz"

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
}

# Standalone flood.
bench_load = BenchLoad.blueprint().transports(_BENCH_TRANSPORTS)

# unitree-go2 sim + bench load on the same transport (custom topics alongside the real robot).
go2_bench = autoconnect(unitree_go2, bench_load)
