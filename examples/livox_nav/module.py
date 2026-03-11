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

import math
import pickle
import time

from dimos.core import In, Module, Out, rpc
from dimos.core.blueprints import autoconnect
from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2
from dimos.mapping.costmapper import cost_mapper
from dimos.mapping.voxels import voxel_mapper
from dimos.msgs.geometry_msgs import Quaternion, Transform, Vector3
from dimos.msgs.nav_msgs import Odometry
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.navigation.frontier_exploration import wavefront_frontier_explorer
from dimos.navigation.replanning_a_star.module import replanning_a_star_planner
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import unitree_go2_basic
from dimos.utils.logging_config import setup_logger
from dimos.visualization.rerun.bridge import rerun_bridge

logger = setup_logger()

voxel_size = 0.05


class RecordMid360Module(Module):
    lidar: In[PointCloud2]
    odometry: In[Odometry]

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.file = None

    @rpc
    def start(self):
        self.file = open("lidar.pkl", "wb")
        self.file2 = open("odo.pkl", "wb")
        logger.info("Started recording lidar data to lidar.pkl")

        def save_lidar(msg):
            pickle.dump(msg, self.file)
            logger.info(f"Saved pointcloud at ts={msg.ts}")

        def save_odom(msg):
            pickle.dump(msg, self.file2)
            logger.info(f"Saved odometry at ts={msg.ts}")

        self.lidar.subscribe(save_lidar)
        self.odometry.subscribe(save_odom)

    @rpc
    def stop(self):
        if self.file:
            self.file.close()
            logger.info("Recording stopped.")
        super().stop()


class ReplayMid360Module(Module):
    lidar: Out[PointCloud2]

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.file = None

    @rpc
    def start(self):
        import threading

        self.file = open("lidar.pkl", "rb")
        self._running = True
        self._thread = threading.Thread(target=self._replay_loop, daemon=True)
        self._thread.start()

    def _replay_loop(self):
        floor_orientation = Transform(
            translation=Vector3(0, 0, 0),
            rotation=Quaternion.from_euler(Vector3(0, math.radians(24), 0)),
        )
        try:
            logger.info("Starting replay from lidar.pkl")
            while self._running:
                pcd: PointCloud2 = pickle.load(self.file)
                logger.info(f"Replaying pointcloud at ts={pcd.ts}")
                self.lidar.publish(pcd.transform(floor_orientation))
                time.sleep(0.1)  # Add small delay between frames
        except EOFError:
            logger.info("Replay finished - reached end of file")
            self._running = False

    @rpc
    def stop(self):
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        if self.file:
            self.file.close()


unitree_go2 = autoconnect(
    unitree_go2_basic,  # robot connection + visualization
    voxel_mapper(voxel_size=0.05),  # 3D voxel mapping
    cost_mapper(),  # 2D costmap generation
    replanning_a_star_planner(),  # path planning
    wavefront_frontier_explorer(),  # exploration
).global_config(n_workers=6, robot_model="unitree_go2")


record_mid360 = autoconnect(
    FastLio2.blueprint(voxel_size=voxel_size, map_voxel_size=voxel_size, map_freq=-1),
    rerun_bridge(
        visual_override={
            "world/lidar": lambda grid: grid.to_rerun(voxel_size=voxel_size, mode="boxes"),
        }
    ),
    RecordMid360Module.blueprint(),
)

replay_mid360 = autoconnect(
    ReplayMid360Module.blueprint(),
    rerun_bridge(
        visual_override={
            "world/lidar": lambda grid: grid.to_rerun(voxel_size=voxel_size, mode="boxes"),
        }
    ),
)

replay_mid360_voxel_mapper = autoconnect(
    ReplayMid360Module.blueprint(), voxel_mapper(voxel_size=voxel_size), rerun_bridge()
)

if __name__ == "__main__":
    # record_mid360.build().loop()
    # replay_mid360.build().loop()
    replay_mid360_voxel_mapper.build().loop()
