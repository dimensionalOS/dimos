#!/usr/bin/env python3
# Benchmark publisher — feeds the gateway a controlled, timestamped load so the
# headless bench client can measure latency / throughput / bandwidth / on-demand.
#
# Publishes (over the active transport — set DIMOS_TRANSPORT=lcm|zenoh):
#   /bench/p0../p3  PoseStamped @ BENCH_HZ   (small, high-rate, ts-stamped)
#   /bench/grid     OccupancyGrid @ BENCH_GRID_HZ  (large ~3.7KB)
#
# RUN:  DIMOS_TRANSPORT=lcm ../../../.venv/bin/python bench/bench_publisher.py
import os
import time

import numpy as np
import reactivex as rx

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid

TRANSPORT = os.environ.get("DIMOS_TRANSPORT", "lcm")
PUB_HZ = float(os.environ.get("BENCH_HZ", 100))
GRID_HZ = float(os.environ.get("BENCH_GRID_HZ", 20))

GRID = ((np.random.rand(60, 60) > 0.85).astype(np.int8)) * 100
IDENT = Quaternion.from_euler(Vector3(0.0, 0.0, 0.0))


def mk(topic, typ):
    if TRANSPORT == "zenoh":
        from dimos.core.transport import ZenohTransport

        return ZenohTransport(topic, typ)
    from dimos.core.transport import LCMTransport

    return LCMTransport(topic, typ)


class BenchPub(Module):
    config: ModuleConfig
    p0: Out[PoseStamped]
    p1: Out[PoseStamped]
    p2: Out[PoseStamped]
    p3: Out[PoseStamped]
    grid: Out[OccupancyGrid]

    @rpc
    def start(self) -> None:
        self._seq = 0
        ports = [self.p0, self.p1, self.p2, self.p3]

        def tick(_):
            now = time.time()
            self._seq += 1
            for i, p in enumerate(ports):
                p.publish(
                    PoseStamped(
                        ts=now,  # source timestamp → end-to-end latency
                        frame_id="bench",
                        position=(self._seq * 0.001, float(i), 0.0),
                        orientation=IDENT,
                    )
                )

        self.register_disposable(rx.interval(1.0 / PUB_HZ).subscribe(tick))
        self.register_disposable(
            rx.interval(1.0 / GRID_HZ).subscribe(
                lambda _: self.grid.publish(
                    OccupancyGrid(grid=GRID, resolution=0.1, origin=Pose(-3.0, -3.0, 0.0), frame_id="bench")
                )
            )
        )


if __name__ == "__main__":
    mod = BenchPub()
    mod.p0.transport = mk("/bench/p0", PoseStamped)
    mod.p1.transport = mk("/bench/p1", PoseStamped)
    mod.p2.transport = mk("/bench/p2", PoseStamped)
    mod.p3.transport = mk("/bench/p3", PoseStamped)
    mod.grid.transport = mk("/bench/grid", OccupancyGrid)
    mod.start()
    print(f"bench_publisher: 4×PoseStamped @ {PUB_HZ}Hz + grid @ {GRID_HZ}Hz over {TRANSPORT}")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        mod.stop()
