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


import atexit
import logging
import threading
import time

import mujoco
from mujoco import viewer
import numpy as np
import open3d as o3d

from dimos.msgs.geometry_msgs import Quaternion, Twist, Vector3
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.odometry import Odometry
from dimos.simulation.mujoco.depth_camera import depth_image_to_point_cloud
from dimos.simulation.mujoco.model import load_model

LIDAR_RESOLUTION = 0.05
DEPTH_CAMERA_FOV = 160
STEPS_PER_FRAME = 2
VIDEO_FPS = 20
LIDAR_FPS = 4

logger = logging.getLogger(__name__)


class MujocoThread(threading.Thread):
    def __init__(self) -> None:
        super().__init__(daemon=True)
        self.shared_pixels = None
        self.pixels_lock = threading.RLock()
        self.shared_depth_front = None
        self.depth_lock_front = threading.RLock()
        self.shared_depth_left = None
        self.depth_left_lock = threading.RLock()
        self.shared_depth_right = None
        self.depth_right_lock = threading.RLock()
        self.odom_data = None
        self.odom_lock = threading.RLock()
        self.lidar_lock = threading.RLock()
        self.model = None
        self.data = None
        self._command = np.zeros(3, dtype=np.float32)
        self._command_lock = threading.RLock()
        self._is_running = True
        self._stop_timer: threading.Timer | None = None
        self._viewer = None
        self._rgb_renderer = None
        self._depth_renderer = None
        self._depth_left_renderer = None
        self._depth_right_renderer = None
        self._cleanup_registered = False

        # Register cleanup on exit
        atexit.register(self.cleanup)

    def run(self) -> None:
        try:
            self.run_simulation()
        except Exception as e:
            logger.error(f"MuJoCo simulation thread error: {e}")
        finally:
            self._cleanup_resources()

    def run_simulation(self) -> None:
        self.model, self.data = load_model(self)

        camera_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "head_camera")
        lidar_camera_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_CAMERA, "lidar_front_camera"
        )
        lidar_left_camera_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_CAMERA, "lidar_left_camera"
        )
        lidar_right_camera_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_CAMERA, "lidar_right_camera"
        )

        with viewer.launch_passive(
            self.model, self.data, show_left_ui=False, show_right_ui=False
        ) as m_viewer:
            self._viewer = m_viewer
            camera_size = (320, 240)

            # Create separate renderers for RGB and depth
            self._rgb_renderer = mujoco.Renderer(
                self.model, height=camera_size[1], width=camera_size[0]
            )
            self._depth_renderer = mujoco.Renderer(
                self.model, height=camera_size[1], width=camera_size[0]
            )
            # Enable depth rendering only for depth renderer
            self._depth_renderer.enable_depth_rendering()

            # Create renderers for left and right depth cameras
            self._depth_left_renderer = mujoco.Renderer(
                self.model, height=camera_size[1], width=camera_size[0]
            )
            self._depth_left_renderer.enable_depth_rendering()

            self._depth_right_renderer = mujoco.Renderer(
                self.model, height=camera_size[1], width=camera_size[0]
            )
            self._depth_right_renderer.enable_depth_rendering()

            scene_option = mujoco.MjvOption()

            # Timing control variables
            last_video_time = 0
            last_lidar_time = 0
            video_interval = 1.0 / VIDEO_FPS
            lidar_interval = 1.0 / LIDAR_FPS

            while m_viewer.is_running() and self._is_running:
                step_start = time.time()

                for _ in range(STEPS_PER_FRAME):
                    mujoco.mj_step(self.model, self.data)

                m_viewer.sync()

                # Odometry happens every loop
                with self.odom_lock:
                    # base position
                    pos = self.data.qpos[0:3]
                    # base orientation
                    quat = self.data.qpos[3:7]  # (w, x, y, z)
                    self.odom_data = (pos.copy(), quat.copy())

                current_time = time.time()

                # Video rendering
                if current_time - last_video_time >= video_interval:
                    self._rgb_renderer.update_scene(
                        self.data, camera=camera_id, scene_option=scene_option
                    )
                    pixels = self._rgb_renderer.render()

                    with self.pixels_lock:
                        self.shared_pixels = pixels.copy()

                    last_video_time = current_time

                # Lidar rendering
                if current_time - last_lidar_time >= lidar_interval:
                    # Render fisheye camera for depth/lidar data
                    self._depth_renderer.update_scene(
                        self.data, camera=lidar_camera_id, scene_option=scene_option
                    )
                    # When depth rendering is enabled, render() returns depth as float array in meters
                    depth = self._depth_renderer.render()

                    with self.depth_lock_front:
                        self.shared_depth_front = depth.copy()

                    # Render left depth camera
                    self._depth_left_renderer.update_scene(
                        self.data, camera=lidar_left_camera_id, scene_option=scene_option
                    )
                    depth_left = self._depth_left_renderer.render()

                    with self.depth_left_lock:
                        self.shared_depth_left = depth_left.copy()

                    # Render right depth camera
                    self._depth_right_renderer.update_scene(
                        self.data, camera=lidar_right_camera_id, scene_option=scene_option
                    )
                    depth_right = self._depth_right_renderer.render()

                    with self.depth_right_lock:
                        self.shared_depth_right = depth_right.copy()

                    last_lidar_time = current_time

                # Control the simulation speed
                time_until_next_step = self.model.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)

    def _process_depth_camera(self, camera_name: str, depth_data, depth_lock) -> np.ndarray | None:
        """Process a single depth camera and return point cloud points."""
        with depth_lock:
            if depth_data is None:
                return None

            depth_image = depth_data.copy()
            camera_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
            if camera_id == -1:
                return None

            camera_pos = self.data.cam_xpos[camera_id]
            camera_mat = self.data.cam_xmat[camera_id].reshape(3, 3)
            points = depth_image_to_point_cloud(
                depth_image,
                camera_pos,
                camera_mat,
                fov_degrees=DEPTH_CAMERA_FOV,
            )
            return points if points.size > 0 else None

    def get_lidar_message(self) -> LidarMessage | None:
        all_points = []
        origin = None

        with self.lidar_lock:
            if self.model is not None and self.data is not None:
                pos = self.data.qpos[0:3]
                origin = Vector3(pos[0], pos[1], pos[2])

                cameras = [
                    ("lidar_front_camera", self.shared_depth_front, self.depth_lock_front),
                    ("lidar_left_camera", self.shared_depth_left, self.depth_left_lock),
                    ("lidar_right_camera", self.shared_depth_right, self.depth_right_lock),
                ]

                for camera_name, depth_data, depth_lock in cameras:
                    points = self._process_depth_camera(camera_name, depth_data, depth_lock)
                    if points is not None:
                        all_points.append(points)

        # Combine all point clouds
        if not all_points:
            return None

        combined_points = np.vstack(all_points)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(combined_points)

        # Apply voxel downsampling to remove overlapping points
        pcd = pcd.voxel_down_sample(voxel_size=LIDAR_RESOLUTION)
        lidar_to_publish = LidarMessage(
            pointcloud=pcd,
            ts=time.time(),
            origin=origin,
            resolution=LIDAR_RESOLUTION,
        )
        return lidar_to_publish

    def get_odom_message(self) -> Odometry | None:
        with self.odom_lock:
            if self.odom_data is None:
                return None
            pos, quat_wxyz = self.odom_data

        # MuJoCo uses (w, x, y, z) for quaternions.
        # ROS and Dimos use (x, y, z, w).
        orientation = Quaternion(quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0])

        odom_to_publish = Odometry(
            position=Vector3(pos[0], pos[1], pos[2]),
            orientation=orientation,
            ts=time.time(),
            frame_id="world",
        )
        return odom_to_publish

    def _stop_move(self) -> None:
        with self._command_lock:
            self._command = np.zeros(3, dtype=np.float32)
        self._stop_timer = None

    def move(self, twist: Twist, duration: float = 0.0) -> None:
        if self._stop_timer:
            self._stop_timer.cancel()

        with self._command_lock:
            self._command = np.array(
                [twist.linear.x, twist.linear.y, twist.angular.z], dtype=np.float32
            )

        if duration > 0:
            self._stop_timer = threading.Timer(duration, self._stop_move)
            self._stop_timer.daemon = True
            self._stop_timer.start()
        else:
            self._stop_timer = None

    def get_command(self) -> np.ndarray:
        with self._command_lock:
            return self._command.copy()

    def stop(self) -> None:
        """Stop the simulation thread gracefully."""
        self._is_running = False

        # Cancel any pending timers
        if self._stop_timer:
            self._stop_timer.cancel()
            self._stop_timer = None

        # Wait for thread to finish
        if self.is_alive():
            self.join(timeout=5.0)
            if self.is_alive():
                logger.warning("MuJoCo thread did not stop gracefully within timeout")

    def cleanup(self) -> None:
        """Clean up all resources. Can be called multiple times safely."""
        if self._cleanup_registered:
            return
        self._cleanup_registered = True

        logger.debug("Cleaning up MuJoCo resources")
        self.stop()
        self._cleanup_resources()

    def _cleanup_resources(self) -> None:
        """Internal method to clean up MuJoCo-specific resources."""
        try:
            # Cancel any timers
            if self._stop_timer:
                self._stop_timer.cancel()
                self._stop_timer = None

            # Clean up renderers
            if self._rgb_renderer is not None:
                try:
                    self._rgb_renderer.close()
                except Exception as e:
                    logger.debug(f"Error closing RGB renderer: {e}")
                finally:
                    self._rgb_renderer = None

            if self._depth_renderer is not None:
                try:
                    self._depth_renderer.close()
                except Exception as e:
                    logger.debug(f"Error closing depth renderer: {e}")
                finally:
                    self._depth_renderer = None

            if self._depth_left_renderer is not None:
                try:
                    self._depth_left_renderer.close()
                except Exception as e:
                    logger.debug(f"Error closing left depth renderer: {e}")
                finally:
                    self._depth_left_renderer = None

            if self._depth_right_renderer is not None:
                try:
                    self._depth_right_renderer.close()
                except Exception as e:
                    logger.debug(f"Error closing right depth renderer: {e}")
                finally:
                    self._depth_right_renderer = None

            # Clear data references
            with self.pixels_lock:
                self.shared_pixels = None

            with self.depth_lock_front:
                self.shared_depth_front = None

            with self.depth_left_lock:
                self.shared_depth_left = None

            with self.depth_right_lock:
                self.shared_depth_right = None

            with self.odom_lock:
                self.odom_data = None

            # Clear model and data
            self.model = None
            self.data = None

            # Reset MuJoCo control callback
            try:
                mujoco.set_mjcb_control(None)
            except Exception as e:
                logger.debug(f"Error resetting MuJoCo control callback: {e}")

        except Exception as e:
            logger.error(f"Error during resource cleanup: {e}")

    def __del__(self) -> None:
        """Destructor to ensure cleanup on object deletion."""
        try:
            self.cleanup()
        except Exception:
            pass
