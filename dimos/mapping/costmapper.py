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

from dataclasses import dataclass

from reactivex import operators as ops

from dimos.core import In, Module, Out, rpc
from dimos.core.module import ModuleConfig
from dimos.mapping.pointclouds.occupancy import simple_occupancy
from dimos.msgs.nav_msgs import OccupancyGrid
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.utils.reactive import backpressure


@dataclass
class Config(ModuleConfig):
    resolution: float = 0.05
    can_pass_under: float = 0.6
    can_climb: float = 0.15


class CostMapper(Module):
    default_config = Config
    config: Config

    global_map: In[LidarMessage]
    global_costmap: Out[OccupancyGrid]

    @rpc
    def start(self) -> None:
        super().start()

        self._disposables.add(
            backpressure(
                self.global_map.observable()  # type: ignore[no-untyped-call]
            )
            .pipe(ops.map(self._calculate_costmap))
            .subscribe(
                self.global_costmap.publish,
            )
        )

    @rpc
    def stop(self) -> None:
        return super().stop()

    def _calculate_costmap(self, msg: LidarMessage) -> OccupancyGrid:
        return height_cost_occupancy(msg, **self.config)


cost_mapper = CostMapper.blueprint
