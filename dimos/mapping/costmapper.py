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

from dataclasses import asdict, dataclass, field
import queue
import threading
import time

from reactivex import operators as ops
import rerun as rr
import rerun.blueprint as rrb

from dimos.core import In, Module, Out, rpc
from dimos.core.global_config import GlobalConfig
from dimos.core.module import ModuleConfig
from dimos.dashboard.rerun_init import connect_rerun
from dimos.mapping.pointclouds.occupancy import (
    OCCUPANCY_ALGOS,
    HeightCostConfig,
    OccupancyConfig,
)
from dimos.msgs.nav_msgs import OccupancyGrid
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass
class Config(ModuleConfig):
    algo: str = "height_cost"
    config: OccupancyConfig = field(default_factory=HeightCostConfig)


class CostMapper(Module):
    default_config = Config
    config: Config

    global_map: In[PointCloud2]
    global_costmap: Out[OccupancyGrid]

    # Background Rerun logging (decouples viz from data pipeline)
    _rerun_queue: queue.Queue[tuple[OccupancyGrid, float, float] | None]
    _rerun_thread: threading.Thread | None = None

    @classmethod
    def rerun_views(cls):  # type: ignore[no-untyped-def]
        """Return Rerun view blueprints for costmap visualization."""
        return [
            rrb.TimeSeriesView(
                name="Costmap (ms)",
                origin="/metrics/costmap",
                contents=["+ /metrics/costmap/calc_ms"],
            ),
        ]

    def __init__(self, global_config: GlobalConfig | None = None, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._global_config = global_config or GlobalConfig()
        self._rerun_queue = queue.Queue(maxsize=2)

    def _rerun_worker(self) -> None:
        """Background thread: pull from queue and log to Rerun (non-blocking)."""
        while True:
            try:
                item = self._rerun_queue.get(timeout=1.0)
                if item is None:  # Shutdown signal
                    break

                grid, calc_time_ms, rx_monotonic = item

                # Generate mesh + log to Rerun (blocks in background, not on data path)
                try:
                    # 2D image panel
                    rr.log(
                        "world/nav/costmap/image",
                        grid.to_rerun(
                            mode="image",
                            colormap="RdBu_r",
                        ),
                    )
                    # 3D floor overlay (expensive mesh generation)
                    rr.log(
                        "world/nav/costmap/floor",
                        grid.to_rerun(
                            mode="mesh",
                            colormap=None,  # Grayscale: free=white, occupied=black
                            z_offset=0.02,
                        ),
                    )

                    # Log timing metrics
                    rr.log("metrics/costmap/calc_ms", rr.Scalars(calc_time_ms))
                    latency_ms = (time.monotonic() - rx_monotonic) * 1000
                    rr.log("metrics/costmap/latency_ms", rr.Scalars(latency_ms))
                except Exception as e:
                    logger.warning(f"Rerun logging error: {e}")
            except queue.Empty:
                continue

    @rpc
    def start(self) -> None:
        super().start()

        # Only start Rerun logging if Rerun backend is selected
        if self._global_config.viewer_backend.startswith("rerun"):
            connect_rerun(global_config=self._global_config)

            # Start background Rerun logging thread
            self._rerun_thread = threading.Thread(target=self._rerun_worker, daemon=True)
            self._rerun_thread.start()
            logger.info("CostMapper: started async Rerun logging thread")

        def _publish_costmap(grid: OccupancyGrid, calc_time_ms: float, rx_monotonic: float) -> None:
            # Publish to downstream FIRST (fast, not blocked by Rerun)
            self.global_costmap.publish(grid)

            # Queue for async Rerun logging (non-blocking, drops if queue full)
            if self._rerun_thread and self._rerun_thread.is_alive():
                try:
                    self._rerun_queue.put_nowait((grid, calc_time_ms, rx_monotonic))
                except queue.Full:
                    pass  # Drop viz frame, data pipeline continues

        def _calculate_and_time(
            msg: PointCloud2,
        ) -> tuple[OccupancyGrid, float, float]:
            rx_monotonic = time.monotonic()  # Capture receipt time
            start = time.perf_counter()
            grid = self._calculate_costmap(msg)
            elapsed_ms = (time.perf_counter() - start) * 1000
            return grid, elapsed_ms, rx_monotonic

        self._disposables.add(
            self.global_map.observable()  # type: ignore[no-untyped-call]
            .pipe(ops.map(_calculate_and_time))
            .subscribe(lambda result: _publish_costmap(result[0], result[1], result[2]))
        )

    @rpc
    def stop(self) -> None:
        # Shutdown background Rerun thread
        if self._rerun_thread and self._rerun_thread.is_alive():
            self._rerun_queue.put(None)  # Shutdown signal
            self._rerun_thread.join(timeout=2.0)

        super().stop()

    # @timed()  # TODO: fix thread leak in timed decorator
    def _calculate_costmap(self, msg: PointCloud2) -> OccupancyGrid:
        fn = OCCUPANCY_ALGOS[self.config.algo]
        return fn(msg, **asdict(self.config.config))


cost_mapper = CostMapper.blueprint
