# Copyright 2025 Dimensional Inc.
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
import time

from reactivex import operators as ops
import rerun as rr

from dimos.core import In, Module, Out, rpc
from dimos.core.module import ModuleConfig
from dimos.dashboard.rerun_init import connect_rerun
from dimos.mapping.pointclouds.occupancy import (
    OCCUPANCY_ALGOS,
    HeightCostConfig,
    OccupancyConfig,
)
from dimos.msgs.nav_msgs import OccupancyGrid
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.utils.metrics import timed


@dataclass
class Config(ModuleConfig):
    algo: str = "height_cost"
    config: OccupancyConfig = field(default_factory=HeightCostConfig)


class CostMapper(Module):
    default_config = Config
    config: Config

    global_map: In[PointCloud2]
    global_costmap: Out[OccupancyGrid]

    @rpc
    def start(self) -> None:
        super().start()
        connect_rerun()

        def _publish_costmap(
            grid: OccupancyGrid, calc_time_ms: float, rx_monotonic: float
        ) -> None:
            self.global_costmap.publish(grid)

            # Log BOTH 2D image panel AND 3D floor overlay to Rerun
            # 2D image panel (for costmap visualization)
            rr.log(
                "world/nav/costmap/image",
                grid.to_rerun(
                    mode="image",
                    colormap="RdBu_r",
                ),
            )
            # 3D floor overlay (textured mesh, slightly above floor)
            rr.log(
                "world/nav/costmap/floor",
                grid.to_rerun(
                    mode="mesh",
                    colormap="RdBu_r",
                    z_offset=0.02,
                ),
            )

            # Log timing metrics to Rerun
            rr.log("metrics/costmap/calc_ms", rr.Scalars(calc_time_ms))

            # Log pipeline latency (time from message receipt to publish complete)
            latency_ms = (time.monotonic() - rx_monotonic) * 1000
            rr.log("metrics/costmap/latency_ms", rr.Scalars(latency_ms))

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
        super().stop()

    # @timed()  # TODO: fix thread leak in timed decorator
    def _calculate_costmap(self, msg: PointCloud2) -> OccupancyGrid:
        fn = OCCUPANCY_ALGOS[self.config.algo]
        return fn(msg, **asdict(self.config.config))


cost_mapper = CostMapper.blueprint
