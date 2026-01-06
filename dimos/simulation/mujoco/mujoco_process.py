#!/usr/bin/env python3

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

import base64
import json
import pickle
import signal
import sys
import time
from typing import Any

import mujoco  # type: ignore[import-untyped]
from mujoco import viewer
import numpy as np
from numpy.typing import NDArray
import open3d as o3d  # type: ignore[import-untyped]

from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs import Vector3
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
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
from dimos.simulation.mujoco.shared_memory import ShmReader
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class MockController:
    """Controller that reads commands from shared memory."""

    def __init__(self, shm_interface: ShmReader) -> None:
        self.shm = shm_interface
        self._command = np.zeros(3, dtype=np.float32)

    def get_command(self) -> NDArray[Any]:
        """Get the current movement command."""
        cmd_data = self.shm.read_command()
        if cmd_data is not None:
            linear, angular = cmd_data
            # MuJoCo expects [forward, lateral, rotational]
            self._command[0] = linear[0]  # forward/backward
            self._command[1] = linear[1]  # left/right
            self._command[2] = angular[2]  # rotation
        return self._command.copy()

    def stop(self) -> None:
        """Stop method to satisfy InputController protocol."""
        pass


def _run_simulation(config: GlobalConfig, shm: ShmReader) -> None:
    robot_name = config.robot_model or "unitree_go1"
    if robot_name == "unitree_go2":
        robot_name = "unitree_go1"

    controller = MockController(shm)
    model, data = load_model(controller, robot=robot_name, scene_xml=load_scene_xml(config))

    if model is None or data is None:
        raise ValueError("Failed to load MuJoCo model: model or data is None")

    match robot_name:
        case "unitree_go1":
            z = 0.3
        case "unitree_g1":
            z = 0.8
        case _:
            z = 0

    pos = config.mujoco_start_pos_float

    data.qpos[0:3] = [pos[0], pos[1], z]

    mujoco.mj_forward(model, data)

    camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "head_camera")
    lidar_camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "lidar_front_camera")
    lidar_left_camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "lidar_left_camera")
    lidar_right_camera_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_CAMERA, "lidar_right_camera"
    )

    shm.signal_ready()

    with viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False) as m_viewer:
        camera_size = (VIDEO_WIDTH, VIDEO_HEIGHT)

        # Create renderers
        rgb_renderer = mujoco.Renderer(model, height=camera_size[1], width=camera_size[0])
        depth_renderer = mujoco.Renderer(model, height=camera_size[1], width=camera_size[0])
        depth_renderer.enable_depth_rendering()

        depth_left_renderer = mujoco.Renderer(model, height=camera_size[1], width=camera_size[0])
        depth_left_renderer.enable_depth_rendering()

        depth_right_renderer = mujoco.Renderer(model, height=camera_size[1], width=camera_size[0])
        depth_right_renderer.enable_depth_rendering()

        scene_option = mujoco.MjvOption()

        # Timing control
        last_video_time = 0.0
        last_lidar_time = 0.0
        video_interval = 1.0 / VIDEO_FPS
        lidar_interval = 1.0 / LIDAR_FPS

        m_viewer.cam.lookat = config.mujoco_camera_position_float[0:3]
        m_viewer.cam.distance = config.mujoco_camera_position_float[3]
        m_viewer.cam.azimuth = config.mujoco_camera_position_float[4]
        m_viewer.cam.elevation = config.mujoco_camera_position_float[5]

        while m_viewer.is_running() and not shm.should_stop():
            step_start = time.time()

            # Step simulation
            for _ in range(config.mujoco_steps_per_frame):
                mujoco.mj_step(model, data)

            m_viewer.sync()

            # Always update odometry
            pos = data.qpos[0:3].copy()
            quat = data.qpos[3:7].copy()  # (w, x, y, z)
            shm.write_odom(pos, quat, time.time())

            current_time = time.time()

            # Video rendering
            if current_time - last_video_time >= video_interval:
                rgb_renderer.update_scene(data, camera=camera_id, scene_option=scene_option)
                pixels = rgb_renderer.render()
                shm.write_video(pixels)
                last_video_time = current_time

            # Lidar/depth rendering
            if current_time - last_lidar_time >= lidar_interval:
                # Render all depth cameras
                depth_renderer.update_scene(data, camera=lidar_camera_id, scene_option=scene_option)
                depth_front = depth_renderer.render()

                depth_left_renderer.update_scene(
                    data, camera=lidar_left_camera_id, scene_option=scene_option
                )
                depth_left = depth_left_renderer.render()

                depth_right_renderer.update_scene(
                    data, camera=lidar_right_camera_id, scene_option=scene_option
                )
                depth_right = depth_right_renderer.render()

                shm.write_depth(depth_front, depth_left, depth_right)

                # Process depth images into lidar message
                all_points = []
                cameras_data = [
                    (
                        depth_front,
                        data.cam_xpos[lidar_camera_id],
                        data.cam_xmat[lidar_camera_id].reshape(3, 3),
                    ),
                    (
                        depth_left,
                        data.cam_xpos[lidar_left_camera_id],
                        data.cam_xmat[lidar_left_camera_id].reshape(3, 3),
                    ),
                    (
                        depth_right,
                        data.cam_xpos[lidar_right_camera_id],
                        data.cam_xmat[lidar_right_camera_id].reshape(3, 3),
                    ),
                ]

                for depth_image, camera_pos, camera_mat in cameras_data:
                    points = depth_image_to_point_cloud(
                        depth_image, camera_pos, camera_mat, fov_degrees=DEPTH_CAMERA_FOV
                    )
                    if points.size > 0:
                        all_points.append(points)

                if all_points:
                    combined_points = np.vstack(all_points)
                    pcd = o3d.geometry.PointCloud()
                    pcd.points = o3d.utility.Vector3dVector(combined_points)
                    pcd = pcd.voxel_down_sample(voxel_size=LIDAR_RESOLUTION)

                    lidar_msg = LidarMessage(
                        pointcloud=pcd,
                        ts=time.time(),
                        origin=Vector3(pos[0], pos[1], pos[2]),
                        resolution=LIDAR_RESOLUTION,
                    )
                    shm.write_lidar(lidar_msg)

                last_lidar_time = current_time

            # Control simulation speed
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


if __name__ == "__main__":

    def signal_handler(_signum: int, _frame: Any) -> None:
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    global_config = pickle.loads(base64.b64decode(sys.argv[1]))
    shm_names = json.loads(sys.argv[2])

    shm = ShmReader(shm_names)
    try:
        _run_simulation(global_config, shm)
    finally:
        shm.cleanup()
