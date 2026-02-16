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
from reactivex.disposable import Disposable

from dimos.core.blueprints import autoconnect
from dimos.mapping.costmapper import cost_mapper
from dimos.mapping.voxels import VoxelGridMapper
from dimos.navigation.frontier_exploration import wavefront_frontier_explorer
from dimos.navigation.replanning_a_star.module import replanning_a_star_planner
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import unitree_go2_basic
from dimos.utils.logging_config import setup_logger
from dimos.core import Module, In, Out, rpc
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.msgs.geometry_msgs import Vector3, Quaternion, Transform, PoseStamped
from dimos.msgs.nav_msgs import Odometry
from dimos.visualization.rerun.bridge import rerun_bridge
from dimos.mapping.pointclouds.occupancy import HeightCostConfig
from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2
from dimos.robot.unitree.go2.connection import GO2Connection

import pickle
import math
import time
from pathlib import Path


logger = setup_logger()

voxel_size = 0.05

# data_path = Path(__file__).parent.parent.parent.parent.parent.parent / "data" / "livox_nav_recording"
# lidar_path = data_path / "lidar.pkl"


class ReplayMid360Module(Module):
    """Module that replays Mid360 lidar data from pickle file."""

    lidar: Out[PointCloud2]

    def __init__(self) -> None:
        super().__init__()
        # TODO: clean up (use get_data function)
        data_path = Path(__file__).parent.parent.parent.parent.parent.parent.parent / "data" / "livox_nav_recording"
        lidar_path = data_path / "lidar.pkl"

        self.lidar_path = lidar_path
        self.file = None
        self._running = False
        self._thread = None

    @rpc
    def start(self) -> None:
        """Start replaying lidar data."""
        import threading

        self.file = open(self.lidar_path, "rb")
        self._running = True
        self._thread = threading.Thread(target=self._replay_loop, daemon=True)
        self._thread.start()
        logger.info(f"ReplayMid360Module started, replaying from {self.lidar_path}")

    def _replay_loop(self):
        floor_orientation = Transform(
            translation=Vector3(0, 0, 0.5),
            rotation=Quaternion.from_euler(Vector3(0, math.radians(24), 0)),
        )
        try:
            logger.info(f"Starting replay from {self.lidar_path}")
            frame_count = 0
            while self._running:
                pcd: PointCloud2 = pickle.load(self.file)
                logger.info(f"Replaying pointcloud at ts={pcd.ts}")
                self.lidar.publish(pcd.transform(floor_orientation).filter_by_height(max_height=2))
                frame_count += 1
                time.sleep(0.1)  # Add small delay between frames
        except EOFError:
            logger.info(f"Replay finished - reached end of file after {frame_count} frames")
            self._running = False
        except Exception as e:
            logger.error(f"Error during replay: {e}")
            self._running = False

    @rpc
    def stop(self) -> None:
        """Stop replaying lidar data."""
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        if self.file:
            self.file.close()
        logger.info("ReplayMid360Module stopped")

ANGLE_OF_MID_360_ON_ROBOT = 24 # degre
class TransformToRobot(Module):
    lidar_untrans: In[PointCloud2]
    lidar_trans: Out[PointCloud2]

    def __init__(self) -> None:
        super().__init__()
        self.floor_orientation = Transform(
            translation=Vector3(0, 0, 0.5),
            rotation=Quaternion.from_euler(Vector3(0, math.radians(ANGLE_OF_MID_360_ON_ROBOT), 0)),
        )

    @rpc
    def start(self):
        unsub = self.lidar_untrans.subscribe(self._on_lidar)
        self._disposables.add(Disposable(unsub))

    def _on_lidar(self, pcd: PointCloud2):
        #TODO: add straight to costmapper
        """Apply transform and filter by height before publishing."""
        transformed = pcd.transform(self.floor_orientation).filter_by_height(max_height=2)
        self.lidar_trans.publish(transformed)

    @rpc
    def stop(self):
        pass

class OdometryToOdom(Module):
    """Module that converts nav_msgs.Odometry to geometry_msgs.PoseStamped."""

    odometry: In[Odometry]
    odom: Out[PoseStamped]

    def __init__(self) -> None:
        super().__init__()

    @rpc
    def start(self):
        unsub = self.odometry.subscribe(self._on_odom)
        self._disposables.add(Disposable(unsub))

    def _on_odom(self, odometry: Odometry):
        """Convert Odometry to PoseStamped."""
        self.odom.publish(PoseStamped(
            ts=odometry.ts,
            position=odometry.pose.position,
            orientation=odometry.pose.orientation,
        ))

    @rpc
    def stop(self):
        pass

unitree_go2_fastlio = autoconnect(
    unitree_go2_basic,
    FastLio2.blueprint(voxel_size=voxel_size, map_voxel_size=voxel_size, map_freq=-1),
    TransformToRobot.blueprint(),
    OdometryToOdom.blueprint(),
    VoxelGridMapper.blueprint(voxel_size=voxel_size),
    cost_mapper(),
    replanning_a_star_planner(),
    wavefront_frontier_explorer(),
    rerun_bridge()
).remappings([
    # TODO: maybe call lidar in fastlio raw lidar
    # turn off go2 lidar (basically)
    (GO2Connection, 'lidar', 'lidar_onboard'),
    (GO2Connection, 'odom', 'odom_null'),
    #
    # (FastLio2, 'odometry', 'odometry_nav'),
    # (FastLio2, 'lidar', 'lidar_fastlio'),
    (TransformToRobot, 'lidar_untrans', 'lidar'),
    # Convert nav_msgs.Odometry to geometry_msgs.PoseStamped
    # (OdometryToOdom, 'odometry_nav', 'odom'),
    # (cost_mapper, 'odom', 'odometry'),
    (VoxelGridMapper, 'lidar', 'lidar_trans'),
]).global_config(n_dask_workers=6, robot_model="unitree_go2")



__all__ = ["unitree_go2_fastlio"]

if __name__ == "__main__":
    unitree_go2_fastlio.build().loop()