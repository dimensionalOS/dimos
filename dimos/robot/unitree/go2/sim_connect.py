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

"""Velocity-commanded Go2 stand-in backed by a PimSim MuJoCo session."""

from __future__ import annotations

import asyncio
from collections.abc import AsyncIterator
from concurrent.futures import ThreadPoolExecutor
import math
import time
from typing import Any

import numpy as np
from pimsim import ActionBatch, ExecutionConfig, RobotSpec, SimSession, WorldSpec
from pimsim.backends.mujoco.backend import MujocoBackend
from pimsim.backends.mujoco.types import MujocoBackendConfig, MujocoRobotBinding
from pimsim.sensors.mujoco import (
    MujocoIdealSphericalLidar,
    MujocoIdealSphericalLidarConfig,
    MujocoRgbdCamera,
    MujocoRgbdCameraConfig,
)

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage

ROBOT_ID = "go2"
BASE_JOINTS = ("base_x", "base_y", "base_yaw")
MUJOCO_BASE_BODY = "base_link"
MUJOCO_CAMERA_NAME = "front_camera"
VELOCITY_GAIN = 400.0
DEPTH_TRUNC_METERS = 10.0
# Mounts copied from the real robot: connection.BASE_TO_OPTICAL and dds.extrinsics.EXT_T.
CAMERA_MOUNT = Vector3(0.3, 0.0, 0.0)
OPTICAL_ROTATION = Quaternion(-0.5, 0.5, -0.5, 0.5)
LIDAR_MOUNT = (0.28216, 0.0, -0.02467)
LIDAR_MIN_ELEVATION = math.radians(-85.0)
LIDAR_MAX_ELEVATION = math.radians(30.0)
LIDAR_MIN_RANGE = 0.1

# A planar body: two slides and a yaw hinge, so cmd_vel maps onto the three
# degrees of freedom directly and there is no articulation to control.
MODEL_XML = """
<mujoco model="go2_box">
  <visual>
    <global offwidth="1280" offheight="960"/>
  </visual>
  <asset>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.4 0.5 0.7" rgb2="0.05 0.05 0.1"
             width="256" height="256"/>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.2 0.25 0.3" rgb2="0.3 0.35 0.4"
             width="512" height="512"/>
    <material name="grid" texture="grid" texrepeat="30 30" reflectance="0.1"/>
  </asset>
  <worldbody>
    <light pos="0 0 6" dir="0 0 -1" diffuse="0.9 0.9 0.9"/>
    <geom name="floor" type="plane" size="30 30 0.1" material="grid"/>
    <body name="base_link" pos="0 0 0.2">
      <joint name="base_x" type="slide" axis="1 0 0"/>
      <joint name="base_y" type="slide" axis="0 1 0"/>
      <joint name="base_yaw" type="hinge" axis="0 0 1"/>
      <geom name="chassis" type="box" size="0.35 0.15 0.13" mass="15" rgba="0.9 0.7 0.1 1"/>
      <camera name="front_camera" pos="0.3 0 0" xyaxes="0 -1 0 0 0 1" fovy="70"/>
    </body>
    <body name="wall_front" pos="6 0 1">
      <geom type="box" size="0.2 6 1" rgba="0.55 0.55 0.6 1"/>
    </body>
    <body name="wall_left" pos="0 6 1">
      <geom type="box" size="6 0.2 1" rgba="0.55 0.55 0.6 1"/>
    </body>
    <body name="pillar" pos="2.5 1.2 0.5">
      <geom type="cylinder" size="0.25 0.5" rgba="0.7 0.25 0.2 1"/>
    </body>
    <body name="crate" pos="2.0 -1.5 0.3">
      <geom type="box" size="0.3 0.3 0.3" rgba="0.2 0.5 0.7 1"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="base_x_motor" joint="base_x" ctrlrange="-2000 2000"/>
    <motor name="base_y_motor" joint="base_y" ctrlrange="-2000 2000"/>
    <motor name="base_yaw_motor" joint="base_yaw" ctrlrange="-2000 2000"/>
  </actuator>
</mujoco>
"""


class Go2SimConfig(ModuleConfig):
    control_rate: float = 50.0
    camera_rate: float = 15.0
    image_width: int = 640
    image_height: int = 480
    # Every Nth pixel becomes a point; the full-resolution cloud is ~275k points a frame.
    pointcloud_stride: int = 4

    odom_frame_id: str = "world"
    base_frame_id: str = "base_link"
    camera_link_frame_id: str = "camera_link"
    camera_optical_frame_id: str = "camera_optical"
    lidar_frame_id: str = "lidar"

    # Unitree ships the lidar as a voxel occupancy map over webrtc rather than a raw
    # scan: 128x128x38 cells of 0.05 m recentred on the robot, in the odom frame, on a
    # 0.130 s period, ~21k points a frame. Recorded frames are in data/office_lidar.
    lidar_rate: float = 7.7
    lidar_voxel_size: float = 0.05
    lidar_box_extent: float = 3.2
    lidar_box_height: float = 1.25
    lidar_azimuth_samples: int = 720
    lidar_elevation_samples: int = 96


class Go2Sim(Module):
    """Drop-in stand-in for GO2Connection: consumes cmd_vel, publishes odom, tf and sensors."""

    dedicated_worker = True

    config: Go2SimConfig

    cmd_vel: In[Twist]
    pointcloud: Out[PointCloud2]
    odom: Out[PoseStamped]
    lidar: Out[PointCloud2]
    color_image: Out[Image]
    camera_info: Out[CameraInfo]
    tf: Out[TFMessage]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._command = np.zeros(3, dtype=np.float64)
        self._lidar_cells = np.zeros((0, 3), dtype=np.int32)

    async def main(self) -> AsyncIterator[None]:
        backend = MujocoBackend(
            MujocoBackendConfig(
                xml=MODEL_XML,
                bindings={
                    ROBOT_ID: MujocoRobotBinding(
                        root_body_name=MUJOCO_BASE_BODY,
                        joint_names=BASE_JOINTS,
                        actuator_names=tuple(f"{joint}_motor" for joint in BASE_JOINTS),
                    )
                },
            )
        )
        self._backend = backend
        self._session = SimSession(
            backend,
            ExecutionConfig(
                physics_dt=1.0 / (self.config.control_rate * 10),
                control_decimation=10,
            ),
        )
        self._session.load(
            WorldSpec(revision="go2-box-v1", robots=(RobotSpec(ROBOT_ID, "inline"),))
        )
        self._session.reset()

        self._sensor_view = backend.create_sensor_view()
        self._camera = MujocoRgbdCamera(
            self._sensor_view,
            MujocoRgbdCameraConfig(
                camera_name=MUJOCO_CAMERA_NAME,
                frame_id=self.config.camera_optical_frame_id,
                rate_hz=self.config.camera_rate,
                width=self.config.image_width,
                height=self.config.image_height,
            ),
        )
        extent = self.config.lidar_box_extent
        self._lidar = MujocoIdealSphericalLidar(
            self._sensor_view,
            MujocoIdealSphericalLidarConfig(
                body_name=MUJOCO_BASE_BODY,
                mount_position=LIDAR_MOUNT,
                mount_rpy=(0.0, 0.0, 0.0),
                frame_id=self.config.lidar_frame_id,
                rate_hz=self.config.lidar_rate,
                azimuth_samples=self.config.lidar_azimuth_samples,
                elevation_samples=self.config.lidar_elevation_samples,
                minimum_elevation=LIDAR_MIN_ELEVATION,
                maximum_elevation=LIDAR_MAX_ELEVATION,
                minimum_range=LIDAR_MIN_RANGE,
                maximum_range=math.hypot(math.hypot(extent, extent), self.config.lidar_box_height),
                maximum_height_above_origin=self.config.lidar_box_height,
                exclude_body=MUJOCO_BASE_BODY,
            ),
        )
        calibration = self._camera.calibration
        self._camera_info = CameraInfo.from_intrinsics(
            fx=calibration.fx,
            fy=calibration.fy,
            cx=calibration.cx,
            cy=calibration.cy,
            width=calibration.width,
            height=calibration.height,
            frame_id=self.config.camera_optical_frame_id,
        )
        stride = self.config.pointcloud_stride
        self._pointcloud_info = CameraInfo.from_intrinsics(
            fx=calibration.fx / stride,
            fy=calibration.fy / stride,
            cx=calibration.cx / stride,
            cy=calibration.cy / stride,
            width=calibration.width // stride,
            height=calibration.height // stride,
            frame_id=self.config.camera_optical_frame_id,
        )

        self._sensor_pool = ThreadPoolExecutor(max_workers=1)
        loops = [
            asyncio.create_task(self._control_loop()),
            asyncio.create_task(self._camera_loop()),
            asyncio.create_task(self._lidar_loop()),
        ]
        yield
        for loop in loops:
            loop.cancel()
        self._sensor_pool.shutdown(wait=True)
        self._sensor_view.close()
        self._session.close()

    async def handle_cmd_vel(self, value: Twist) -> None:
        self._command[:] = (value.linear.x, value.linear.y, value.angular.z)

    async def _control_loop(self) -> None:
        period = 1.0 / self.config.control_rate
        deadline = time.monotonic()
        while True:
            observation = self._session.step(
                ActionBatch.for_observation(
                    self._session.snapshot(),
                    {ROBOT_ID: self._world_frame_action()},
                )
            )
            self._publish_pose(
                next(entity for entity in observation.entities if entity.entity_id == ROBOT_ID)
            )
            deadline += period
            await asyncio.sleep(max(0.0, deadline - time.monotonic()))

    async def _camera_loop(self) -> None:
        period = 1.0 / self.config.camera_rate
        deadline = time.monotonic()
        while True:
            sample = await self._capture(self._camera)
            timestamp = time.time()
            color = Image.from_numpy(
                sample["rgb"],
                format=ImageFormat.RGB,
                frame_id=self.config.camera_optical_frame_id,
                ts=timestamp,
            )
            self._camera_info.ts = timestamp
            self.color_image.publish(color)
            self.camera_info.publish(self._camera_info)

            stride = self.config.pointcloud_stride
            self.pointcloud.publish(
                PointCloud2.from_rgbd(
                    Image.from_numpy(
                        np.ascontiguousarray(sample["rgb"][::stride, ::stride]),
                        format=ImageFormat.RGB,
                        frame_id=self.config.camera_optical_frame_id,
                        ts=timestamp,
                    ),
                    Image.from_numpy(
                        np.ascontiguousarray(sample["depth"][::stride, ::stride]),
                        format=ImageFormat.DEPTH,
                        frame_id=self.config.camera_optical_frame_id,
                        ts=timestamp,
                    ),
                    self._pointcloud_info,
                    depth_trunc=DEPTH_TRUNC_METERS,
                )
            )
            deadline += period
            await asyncio.sleep(max(0.0, deadline - time.monotonic()))

    async def _lidar_loop(self) -> None:
        period = 1.0 / self.config.lidar_rate
        voxel = self.config.lidar_voxel_size
        extent = self.config.lidar_box_extent
        deadline = time.monotonic()
        while True:
            sample = await self._capture(self._lidar)
            # A single sweep only grazes the floor, so like the real device this keeps a
            # rolling voxel map and drops whatever falls outside the box as the robot moves.
            scanned = np.floor(sample["points_world"] / voxel).astype(np.int32)
            cells = np.unique(np.concatenate([self._lidar_cells, scanned]), axis=0)
            points = cells * voxel + voxel / 2
            robot_x, robot_y, _ = self._session.snapshot().entities[0].pose.position
            inside = (np.abs(points[:, 0] - robot_x) <= extent) & (
                np.abs(points[:, 1] - robot_y) <= extent
            )
            self._lidar_cells = cells[inside]
            self.lidar.publish(
                PointCloud2.from_numpy(
                    points[inside].astype(np.float32),
                    frame_id=self.config.odom_frame_id,
                    timestamp=time.time(),
                )
            )
            deadline += period
            await asyncio.sleep(max(0.0, deadline - time.monotonic()))

    async def _capture(self, sensor: Any) -> Any:
        # One thread for both sensors: raycasting and rendering are slow enough to stall
        # the control loop, and MuJoCo's render context is tied to the thread that uses it.
        return await asyncio.get_running_loop().run_in_executor(
            self._sensor_pool, self._sample, sensor
        )

    def _sample(self, sensor: Any) -> Any:
        self._sensor_view.restore(self._backend.capture_kinematic_snapshot())
        return sensor.capture(self._session.snapshot())

    def _world_frame_action(self) -> dict[str, np.ndarray]:
        yaw = self._yaw
        forward, strafe, turn = self._command
        return {
            "enabled": np.ones(1),
            "position": np.zeros(3),
            "velocity": np.array(
                (
                    forward * math.cos(yaw) - strafe * math.sin(yaw),
                    forward * math.sin(yaw) + strafe * math.cos(yaw),
                    turn,
                )
            ),
            "kp": np.zeros(3),
            "kd": np.full(3, VELOCITY_GAIN),
            "effort": np.zeros(3),
        }

    @property
    def _yaw(self) -> float:
        x, y, z, w = self._session.snapshot().entities[0].pose.quaternion_xyzw
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def _publish_pose(self, state: Any) -> None:
        x, y, z = state.pose.position
        qx, qy, qz, qw = state.pose.quaternion_xyzw
        timestamp = time.time()
        pose = PoseStamped(
            x, y, z, qx, qy, qz, qw, ts=timestamp, frame_id=self.config.odom_frame_id
        )
        self.odom.publish(pose)
        base = self.config.base_frame_id
        self.tf.publish(
            TFMessage(
                Transform.from_pose(base, pose),
                Transform(
                    translation=CAMERA_MOUNT,
                    frame_id=base,
                    child_frame_id=self.config.camera_link_frame_id,
                    ts=timestamp,
                ),
                Transform(
                    rotation=OPTICAL_ROTATION,
                    frame_id=self.config.camera_link_frame_id,
                    child_frame_id=self.config.camera_optical_frame_id,
                    ts=timestamp,
                ),
                Transform(
                    translation=Vector3(*LIDAR_MOUNT),
                    frame_id=base,
                    child_frame_id=self.config.lidar_frame_id,
                    ts=timestamp,
                ),
            )
        )
