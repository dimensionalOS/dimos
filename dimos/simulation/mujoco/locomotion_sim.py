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

"""MuJoCo locomotion loop used by the Unitree connection module."""

import time
from typing import Any, Protocol

import mujoco
from mujoco import viewer
import numpy as np
from numpy.typing import NDArray

from dimos.core.global_config import GlobalConfig
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.simulation.mujoco.constants import (
    DEPTH_CAMERA_FOV,
    LIDAR_FPS,
    LIDAR_RESOLUTION,
    VIDEO_FPS,
    VIDEO_HEIGHT,
    VIDEO_WIDTH,
)
from dimos.simulation.mujoco.depth_camera import depth_image_to_point_cloud
from dimos.simulation.mujoco.model import load_model, load_scene_xml
from dimos.simulation.mujoco.person_on_track import PersonPositionController


class LocomotionSimIO(Protocol):
    """Thread-safe boundary between the physics loop and its owning module."""

    def get_command(self) -> NDArray[Any]: ...
    def publish_video(self, frame: NDArray[np.uint8]) -> None: ...
    def publish_odom(
        self,
        position: NDArray[np.float64],
        quaternion_wxyz: NDArray[np.float64],
        timestamp: float,
    ) -> None: ...
    def publish_lidar(self, message: PointCloud2) -> None: ...
    def signal_ready(self) -> None: ...
    def should_stop(self) -> bool: ...
    def stop(self) -> None: ...


def run_locomotion_sim(config: GlobalConfig, io: LocomotionSimIO) -> None:
    """Run the policy-controlled Unitree simulator in the module process."""
    import open3d as o3d  # type: ignore[import-untyped]

    robot_name = config.robot_model or "unitree_go1"
    if robot_name == "unitree_go2":
        robot_name = "unitree_go1"

    model, data = load_model(io, robot=robot_name, scene_xml=load_scene_xml(config))

    match robot_name:
        case "unitree_go1":
            z = 0.3
        case "unitree_g1":
            z = 0.8
        case _:
            z = 0.0

    start_pos = config.mujoco_start_pos_float
    data.qpos[0:3] = [start_pos[0], start_pos[1], z]
    mujoco.mj_forward(model, data)

    camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "head_camera")
    lidar_camera_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name)
        for name in ("lidar_front_camera", "lidar_left_camera", "lidar_right_camera")
    ]
    person_position_controller = PersonPositionController(model)

    camera_size = (VIDEO_WIDTH, VIDEO_HEIGHT)
    rgb_renderer = mujoco.Renderer(model, height=camera_size[1], width=camera_size[0])
    depth_renderers = [
        mujoco.Renderer(model, height=camera_size[1], width=camera_size[0])
        for _ in lidar_camera_ids
    ]
    for renderer in depth_renderers:
        renderer.enable_depth_rendering()

    scene_option = mujoco.MjvOption()
    video_interval = 1.0 / VIDEO_FPS
    lidar_interval = 1.0 / LIDAR_FPS
    last_video_time = 0.0
    last_lidar_time = 0.0

    try:
        with viewer.launch_passive(
            model, data, show_left_ui=False, show_right_ui=False
        ) as m_viewer:
            camera_position = config.mujoco_camera_position_float
            m_viewer.cam.lookat = camera_position[0:3]
            m_viewer.cam.distance = camera_position[3]
            m_viewer.cam.azimuth = camera_position[4]
            m_viewer.cam.elevation = camera_position[5]
            io.signal_ready()

            while m_viewer.is_running() and not io.should_stop():
                step_start = time.time()
                for _ in range(config.mujoco_steps_per_frame):
                    mujoco.mj_step(model, data)

                person_position_controller.tick(data)
                m_viewer.sync()

                now = time.time()
                io.publish_odom(data.qpos[0:3].copy(), data.qpos[3:7].copy(), now)

                if now - last_video_time >= video_interval:
                    rgb_renderer.update_scene(data, camera=camera_id, scene_option=scene_option)
                    io.publish_video(rgb_renderer.render().copy())
                    last_video_time = now

                if now - last_lidar_time >= lidar_interval:
                    all_points: list[NDArray[np.float64]] = []
                    for camera, renderer in zip(lidar_camera_ids, depth_renderers, strict=True):
                        renderer.update_scene(data, camera=camera, scene_option=scene_option)
                        points = depth_image_to_point_cloud(
                            renderer.render(),
                            data.cam_xpos[camera],
                            data.cam_xmat[camera].reshape(3, 3),
                            fov_degrees=DEPTH_CAMERA_FOV,
                        )
                        if points.size > 0:
                            all_points.append(points)

                    if all_points:
                        pcd = o3d.geometry.PointCloud()
                        pcd.points = o3d.utility.Vector3dVector(np.vstack(all_points))
                        io.publish_lidar(
                            PointCloud2(
                                pointcloud=pcd.voxel_down_sample(voxel_size=LIDAR_RESOLUTION),
                                ts=now,
                                frame_id="world",
                            )
                        )
                    last_lidar_time = now

                remaining = model.opt.timestep - (time.time() - step_start)
                if remaining > 0:
                    time.sleep(remaining)
    finally:
        person_position_controller.stop()
        rgb_renderer.close()
        for renderer in depth_renderers:
            renderer.close()
