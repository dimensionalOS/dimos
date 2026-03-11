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
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.core.core import rpc
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.msgs.geometry_msgs import Vector3, Quaternion, Transform, PoseStamped
from dimos.msgs.nav_msgs import Odometry
from dimos.protocol.service.system_configurator import ClockSyncConfigurator
from dimos.web.websocket_vis.websocket_vis_module import websocket_vis
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import (
    with_vis,
    rerun_config,
    _transports_base,
)
from dimos.visualization.rerun.bridge import rerun_bridge
from dimos.core.global_config import global_config
from dimos.mapping.pointclouds.occupancy import HeightCostConfig
from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2
from dimos.robot.unitree.go2.connection import GO2Connection, go2_connection
from dimos.utils.data import get_data
from dimos.core.module import ModuleConfig
from dataclasses import dataclass

import pickle
import math
import time
from pathlib import Path


logger = setup_logger()

VOXEL_SIZE = 0.05
ANGLE_OF_MID_360_ON_ROBOT = 24  # degree
INITIAL_HEIGHT_OF_MID_360_ON_ROBOT = 0.2  # meter


LIDAR_MAX_HEIGHT = global_config.ceiling_height


class ReplayMid360Module(Module):
    """Module that replays Mid360 lidar data from pickle file."""

    lidar: Out[PointCloud2]

    def __init__(self) -> None:
        super().__init__()
        self.lidar_path = get_data("livox_go2_recording/lidar.pkl")
        self.file = None
        self._running = False
        self._thread = None

    @rpc
    def start(self) -> None:
        """Start replaying lidar data."""
        import threading

        self.file = self.lidar_path.open("rb")
        self._running = True
        self._thread = threading.Thread(target=self._replay_loop, daemon=True)
        self._thread.start()

        logger.info(f"ReplayMid360Module started, replaying from {self.lidar_path}")

    def _replay_loop(self):
        frame_count = 0
        try:
            if not self.file:
                return
            logger.info(f"Starting replay from {self.lidar_path}")
            while self._running:
                pcd: PointCloud2 = pickle.load(self.file)
                logger.info(f"Replaying pointcloud at ts={pcd.ts}")
                self.lidar.publish(pcd)
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


@dataclass
class Config(ModuleConfig):
    angle_of_mid_360_on_robot: float = 0
    height_of_mid_360_on_robot: float = 0.5
    lidar_max_height: float = 0.2


class TransformToRobot(Module):
    default_config = Config
    config: Config  # type: ignore[assignment]
    lidar_untrans: In[PointCloud2]
    lidar_trans: Out[PointCloud2]

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.floor_orientation = Transform(
            translation=Vector3(0, 0, self.config.height_of_mid_360_on_robot),
            rotation=Quaternion.from_euler(
                Vector3(0, math.radians(self.config.angle_of_mid_360_on_robot), 0)
            ),
        )

    @rpc
    def start(self):
        self.lidar_untrans.subscribe(self._on_lidar)

    def _on_lidar(self, pcd: PointCloud2):
        """Apply transform and filter by height before publishing."""
        transformed = pcd.transform(self.floor_orientation).filter_by_height(
            max_height=self.config.lidar_max_height
        )
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
        self.odometry.subscribe(self._on_odom)

    def _on_odom(self, odometry: Odometry):
        """Convert Odometry to PoseStamped."""

        visual_pose = Quaternion.from_euler(Vector3(0, 0, odometry.pose.orientation.to_euler().z))
        self.odom.publish(
            PoseStamped(
                ts=odometry.ts,
                position=odometry.pose.position,
                orientation=visual_pose,
            )
        )

    @rpc
    def stop(self):
        pass


@dataclass
class OdometryTFPublisherConfig(ModuleConfig):
    camera_link_x: float = 0.3
    camera_link_y: float = 0.0
    camera_link_z: float = 0.0
    camera_link_qx: float = 0.0
    camera_link_qy: float = 0.0
    camera_link_qz: float = 0.0
    camera_link_qw: float = 1.0


class OdometryTFPublisher(Module):
    """Publishes base_link and camera TFs derived from FastLIO2 odometry.

    Subscribes to the ``odom`` stream (a :class:`PoseStamped` produced by
    :class:`OdometryToOdom`) and emits three TF transforms on every tick:

    * ``base_link`` — built from the odometry pose via
      :meth:`Transform.from_pose`, giving the robot's world-frame
      position/orientation.
    * ``camera_link`` — configurable offset/rotation from ``base_link``
      (defaults to 0.3 m forward, no rotation).
    * ``camera_optical`` — fixed -90°/+90° rotation from ``camera_link``
      matching the GO2 camera convention.

    The ``camera_link`` transform is controlled by config fields:
    ``camera_link_x/y/z`` (translation) and ``camera_link_qx/y/z/w``
    (rotation quaternion).
    """

    default_config = OdometryTFPublisherConfig
    config: OdometryTFPublisherConfig  # type: ignore[assignment]

    odom: In[PoseStamped]

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self._camera_link_translation = Vector3(
            self.config.camera_link_x,
            self.config.camera_link_y,
            self.config.camera_link_z,
        )
        self._camera_link_rotation = Quaternion(
            self.config.camera_link_qx,
            self.config.camera_link_qy,
            self.config.camera_link_qz,
            self.config.camera_link_qw,
        )

    @rpc
    def start(self) -> None:
        self.odom.subscribe(self._on_odom)

    def _on_odom(self, odom: PoseStamped) -> None:
        """Convert FastLIO2 odometry pose to TF tree and publish."""
        base_link = Transform.from_pose("base_link", odom)

        camera_link = Transform(
            translation=self._camera_link_translation,
            rotation=self._camera_link_rotation,
            frame_id="base_link",
            child_frame_id="camera_link",
            ts=odom.ts,
        )

        camera_optical = Transform(
            translation=Vector3(0.0, 0.0, 0.0),
            rotation=Quaternion(-0.5, 0.5, -0.5, 0.5),
            frame_id="camera_link",
            child_frame_id="camera_optical",
            ts=odom.ts,
        )

        self.tf.publish(base_link, camera_link, camera_optical)

    @rpc
    def stop(self) -> None:
        pass


fastlio_livox_replay = autoconnect(
    ReplayMid360Module.blueprint(),
    TransformToRobot.blueprint(
        angle_of_mid_360_on_robot=ANGLE_OF_MID_360_ON_ROBOT,
        height_of_mid_360_on_robot=INITIAL_HEIGHT_OF_MID_360_ON_ROBOT,
        lidar_max_height=LIDAR_MAX_HEIGHT,
    ),
    OdometryToOdom.blueprint(),
    VoxelGridMapper.blueprint(voxel_size=VOXEL_SIZE),
    cost_mapper(),
    rerun_bridge(),
).remappings(
    [
        (TransformToRobot, "lidar_untrans", "lidar"),
        (VoxelGridMapper, "lidar", "lidar_trans"),
    ]
)

if global_config.viewer.startswith("rerun"):
    from dimos.visualization.rerun.bridge import _resolve_viewer_mode

    rerun_config["visual_override"] = {
        **rerun_config["visual_override"],
        "world/lidar_null": None,
        "world/odom_null": None,
        "world/lidar": None,
    }
    _fastlio_rerun_config = rerun_config
    _with_vis_fastlio = autoconnect(
        _transports_base,
        rerun_bridge(viewer_mode=_resolve_viewer_mode(), **_fastlio_rerun_config),
    )
else:
    _with_vis_fastlio = with_vis


# need to create my own blueprint so I can add to rerun config and make the go2 stop publising it's tfs.
_unitree_go2_basic_no_tf = (
    autoconnect(
        _with_vis_fastlio,
        go2_connection(publish_tf=False),
        websocket_vis(),
    )
    .global_config(n_workers=4, robot_model="unitree_go2")
    .configurators(ClockSyncConfigurator())
)

unitree_go2_backpack = (
    autoconnect(
        _unitree_go2_basic_no_tf,
        FastLio2.blueprint(
            voxel_size=VOXEL_SIZE, map_voxel_size=VOXEL_SIZE, map_freq=-1, lidar_ip="192.168.1.157"
        ),
        TransformToRobot.blueprint(
            angle_of_mid_360_on_robot=ANGLE_OF_MID_360_ON_ROBOT,
            height_of_mid_360_on_robot=INITIAL_HEIGHT_OF_MID_360_ON_ROBOT,
            lidar_max_height=LIDAR_MAX_HEIGHT,
        ),
        OdometryToOdom.blueprint(),
        OdometryTFPublisher.blueprint(),
        VoxelGridMapper.blueprint(voxel_size=VOXEL_SIZE),
        cost_mapper(),
        replanning_a_star_planner(),
        wavefront_frontier_explorer(),
    )
    .remappings(
        [
            (GO2Connection, "lidar", "lidar_null"),
            (GO2Connection, "odom", "odom_null"),
            (TransformToRobot, "lidar_untrans", "lidar"),
            (VoxelGridMapper, "lidar", "lidar_trans"),
        ]
    )
    .global_config(n_workers=6, robot_model="unitree_go2")
)

__all__ = ["unitree_go2_fastlio", "unitree_go2_fastlio_no_go2tf"]

if __name__ == "__main__":
    fastlio_livox_replay.build().loop()
