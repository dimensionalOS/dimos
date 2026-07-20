# Copyright 2026 Dimensional Inc.
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

"""NavStack: the ``example_hk_nav`` DAG as a real :class:`~dimos.pure.PureGraph`.

    lidar ─▶ VoxelMapper ─▶ PureCostMapper ─▶ Planner ─▶ path
       odom ─▶ OdomTf ──────────────────────▶ (planner position, via tf=)
                            goal ────────────▶ (planner goal)

The hand-wired ``tee`` fan-out of ``example_hk_nav`` becomes explicit wiring in
``wire()``; ``over()`` runs it in one process (bounded streaming), ``blueprint()``
lowers it to the coordinator (Phase B). The rerun sink stays out of the graph — a
recording is attached to the run's exported streams, not baked into the DAG.
"""

from dimos import pure as pm
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.pure.graph import PureGraph
from dimos.pure.modules.costmapper import PureCostMapper
from dimos.pure.modules.odom_tf import OdomTf
from dimos.pure.modules.planner import Planner
from dimos.pure.modules.voxel_mapper import VoxelMapper


class NavStack(PureGraph):
    """Compose the hk pure-nav modules into one validated navigation DAG."""

    voxel_size: float = 0.1
    emit_every: int = 20

    class In(pm.In):
        lidar: PointCloud2
        odom: PoseStamped
        goal: PoseStamped

    class Out(pm.Out):
        path: Path
        costmap: OccupancyGrid
        map: PointCloud2

    def wire(self, i: "NavStack.In") -> "NavStack.Out":
        """Map scans, cost them, plan robot->goal; odom drives the planner's tf."""
        odom_tf = OdomTf()(odom=i.odom)
        mapped = VoxelMapper(voxel_size=self.voxel_size, emit_every=self.emit_every)(scan=i.lidar)
        cost = PureCostMapper()(global_map=mapped.global_map)
        plan = Planner()(costmap=cost.global_costmap, goal=i.goal, tf=odom_tf.tf)
        return NavStack.Out(
            path=plan.path,
            costmap=cost.global_costmap,
            map=mapped.global_map,
        )
