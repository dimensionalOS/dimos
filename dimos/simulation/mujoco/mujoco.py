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
import xml.etree.ElementTree as ET
from typing import Protocol

import mujoco
import numpy as np
import onnxruntime as rt
import open3d as o3d
from etils import epath
from mujoco import viewer
from mujoco_playground._src import mjx_env


from dimos.msgs.geometry_msgs import Quaternion, Vector3
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.odometry import Odometry

RANGE_FINDER_MAX_RANGE = 4
LIDAR_RESOLUTION = 0.05
VIDEO_FREQUENCY = 30

_HERE = epath.Path(__file__).parent

logger = logging.getLogger(__name__)


def get_assets() -> dict[str, bytes]:
    assets: dict[str, bytes] = {}
    assets_path = _HERE / "../../../assets/robots/go1"
    mjx_env.update_assets(assets, assets_path, "*.xml")
    mjx_env.update_assets(assets, assets_path / "assets")
    path = mjx_env.MENAGERIE_PATH / "unitree_go1"
    mjx_env.update_assets(assets, path, "*.xml")
    mjx_env.update_assets(assets, path / "assets")
    return assets


class MujocoThread(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.shared_pixels = None
        self.pixels_lock = threading.RLock()
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
        self._renderer = None
        self._cleanup_registered = False

        # Register cleanup on exit
        atexit.register(self.cleanup)

    def run(self):
        try:
            self.model, self.data = load_model(self)

            camera_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "head_camera")
            last_render = time.time()
            render_interval = 1.0 / VIDEO_FREQUENCY

            with viewer.launch_passive(self.model, self.data) as m_viewer:
                self._viewer = m_viewer
                # Comment this out to show the rangefinders.
                m_viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = 0
                window_size = (640, 480)
                self._renderer = mujoco.Renderer(
                    self.model, height=window_size[1], width=window_size[0]
                )
                scene_option = mujoco.MjvOption()
                scene_option.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = False

                while m_viewer.is_running() and self._is_running:
                    mujoco.mj_step(self.model, self.data)

                    with self.odom_lock:
                        # base position
                        pos = self.data.qpos[0:3]
                        # base orientation
                        quat = self.data.qpos[3:7]  # (w, x, y, z)
                        self.odom_data = (pos.copy(), quat.copy())

                    now = time.time()
                    if now - last_render > render_interval:
                        last_render = now
                        self._renderer.update_scene(
                            self.data, camera=camera_id, scene_option=scene_option
                        )
                        pixels = self._renderer.render()

                        with self.pixels_lock:
                            self.shared_pixels = pixels.copy()

                    m_viewer.sync()
        except Exception as e:
            logger.error(f"MuJoCo simulation thread error: {e}")
        finally:
            self._cleanup_resources()

    def get_lidar_message(self) -> LidarMessage | None:
        num_rays = 360
        angles = np.arange(num_rays) * (2 * np.pi / num_rays)

        range_0_id = -1
        range_0_adr = -1

        points = np.array([])
        origin = None
        pcd = o3d.geometry.PointCloud()

        with self.lidar_lock:
            if self.model is not None and self.data is not None:
                pos, quat_wxyz = self.data.qpos[0:3], self.data.qpos[3:7]
                origin = Vector3(pos[0], pos[1], pos[2])

                if range_0_id == -1:
                    range_0_id = mujoco.mj_name2id(
                        self.model, mujoco.mjtObj.mjOBJ_SENSOR, "range_0"
                    )
                    if range_0_id != -1:
                        range_0_adr = self.model.sensor_adr[range_0_id]

                if range_0_adr != -1:
                    ranges = self.data.sensordata[range_0_adr : range_0_adr + num_rays]

                    rotation_matrix = o3d.geometry.get_rotation_matrix_from_quaternion(
                        [quat_wxyz[0], quat_wxyz[1], -quat_wxyz[2], quat_wxyz[3]]
                    )

                    # Filter out invalid ranges
                    valid_mask = (ranges < RANGE_FINDER_MAX_RANGE) & (ranges >= 0)
                    valid_ranges = ranges[valid_mask]
                    valid_angles = angles[valid_mask]

                    if valid_ranges.size > 0:
                        # Calculate local coordinates of all points at once
                        local_x = valid_ranges * np.sin(valid_angles)
                        local_y = -valid_ranges * np.cos(valid_angles)

                        # Shape (num_valid_points, 3)
                        local_points = np.stack((local_x, local_y, np.zeros_like(local_x)), axis=-1)

                        # Rotate all points at once
                        world_points = (rotation_matrix @ local_points.T).T

                        # Translate all points at once and assign to points
                        points = world_points + pos

        if not points.size:
            return None

        pcd.points = o3d.utility.Vector3dVector(points_to_unique_voxels(points, LIDAR_RESOLUTION))
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

    def _stop_move(self):
        with self._command_lock:
            self._command = np.zeros(3, dtype=np.float32)
        self._stop_timer = None

    def move(self, vector: Vector3, duration: float = 0.0):
        if self._stop_timer:
            self._stop_timer.cancel()

        with self._command_lock:
            self._command = np.array([vector.x, vector.y, vector.z], dtype=np.float32)

        if duration > 0:
            self._stop_timer = threading.Timer(duration, self._stop_move)
            self._stop_timer.daemon = True
            self._stop_timer.start()
        else:
            self._stop_timer = None

    def get_command(self) -> np.ndarray:
        with self._command_lock:
            return self._command.copy()

    def stop(self):
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

    def cleanup(self):
        """Clean up all resources. Can be called multiple times safely."""
        if self._cleanup_registered:
            return
        self._cleanup_registered = True

        logger.debug("Cleaning up MuJoCo resources")
        self.stop()
        self._cleanup_resources()

    def _cleanup_resources(self):
        """Internal method to clean up MuJoCo-specific resources."""
        try:
            # Cancel any timers
            if self._stop_timer:
                self._stop_timer.cancel()
                self._stop_timer = None

            # Clean up renderer
            if self._renderer is not None:
                try:
                    self._renderer.close()
                except Exception as e:
                    logger.debug(f"Error closing renderer: {e}")
                finally:
                    self._renderer = None

            # Clear data references
            with self.pixels_lock:
                self.shared_pixels = None

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

    def __del__(self):
        """Destructor to ensure cleanup on object deletion."""
        try:
            self.cleanup()
        except Exception:
            pass


class InputController(Protocol):
    """A protocol for input devices to control the robot."""

    def get_command(self) -> np.ndarray: ...
    def stop(self) -> None: ...


class OnnxController:
    """ONNX controller for the Go-1 robot."""

    def __init__(
        self,
        policy_path: str,
        default_angles: np.ndarray,
        n_substeps: int,
        action_scale: float,
        input_controller: InputController,
    ):
        self._output_names = ["continuous_actions"]
        self._policy = rt.InferenceSession(policy_path, providers=["CPUExecutionProvider"])

        self._action_scale = action_scale
        self._default_angles = default_angles
        self._last_action = np.zeros_like(default_angles, dtype=np.float32)

        self._counter = 0
        self._n_substeps = n_substeps
        self._input_controller = input_controller

    def get_obs(self, model, data) -> np.ndarray:
        linvel = data.sensor("local_linvel").data
        gyro = data.sensor("gyro").data
        imu_xmat = data.site_xmat[model.site("imu").id].reshape(3, 3)
        gravity = imu_xmat.T @ np.array([0, 0, -1])
        joint_angles = data.qpos[7:] - self._default_angles
        joint_velocities = data.qvel[6:]
        obs = np.hstack(
            [
                linvel,
                gyro,
                gravity,
                joint_angles,
                joint_velocities,
                self._last_action,
                self._input_controller.get_command(),
            ]
        )
        return obs.astype(np.float32)

    def get_control(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self._counter += 1
        if self._counter % self._n_substeps == 0:
            obs = self.get_obs(model, data)
            onnx_input = {"obs": obs.reshape(1, -1)}
            onnx_pred = self._policy.run(self._output_names, onnx_input)[0][0]
            self._last_action = onnx_pred.copy()
            data.ctrl[:] = onnx_pred * self._action_scale + self._default_angles


def get_robot_xml() -> str:
    # Generate the XML at runtime
    xml_path = (_HERE / "../../../assets/robots/go1/robot.xml").as_posix()

    tree = ET.parse(xml_path)
    root = tree.getroot()

    # Find the body element to attach the lidar sites.
    # Using XPath to find the body with childclass='go1'
    robot_body = root.find('.//body[@childclass="go1"]')
    if robot_body is None:
        raise ValueError("Could not find a body with childclass='go1' to attach lidar sites.")

    num_rays = 360
    for i in range(num_rays):
        angle = i * (2 * np.pi / num_rays)
        ET.SubElement(
            robot_body,
            "site",
            name=f"lidar_{i}",
            pos="0 0 0.12",
            euler=f"{1.5707963267948966} {angle} 0",
            size="0.01",
            rgba="1 0 0 1",
        )

    # Find the sensor element to add the rangefinders
    sensor_element = root.find("sensor")
    if sensor_element is None:
        raise ValueError("sensor element not found in XML")

    for i in range(num_rays):
        ET.SubElement(
            sensor_element,
            "rangefinder",
            name=f"range_{i}",
            site=f"lidar_{i}",
            cutoff=str(RANGE_FINDER_MAX_RANGE),
        )

    xml_content = ET.tostring(root, encoding="unicode")
    return xml_content


def load_model(input_device: InputController, model=None, data=None):
    mujoco.set_mjcb_control(None)

    xml_content = get_robot_xml()
    model = mujoco.MjModel.from_xml_string(
        xml_content,
        assets=get_assets(),
    )
    data = mujoco.MjData(model)

    mujoco.mj_resetDataKeyframe(model, data, 0)

    ctrl_dt = 0.02
    sim_dt = 0.004
    n_substeps = int(round(ctrl_dt / sim_dt))
    model.opt.timestep = sim_dt

    policy = OnnxController(
        policy_path=(_HERE / "../../../assets/policies/go1_policy.onnx").as_posix(),
        default_angles=np.array(model.keyframe("home").qpos[7:]),
        n_substeps=n_substeps,
        action_scale=0.5,
        input_controller=input_device,
    )

    mujoco.set_mjcb_control(policy.get_control)

    return model, data


def points_to_unique_voxels(points, voxel_size):
    """
    Convert 3D points to unique voxel centers (removes duplicates).

    Args:
        points: numpy array of shape (N, 3) containing 3D points
        voxel_size: size of each voxel (default 0.05m)

    Returns:
        unique_voxels: numpy array of unique voxel center coordinates
    """
    # Quantize to voxel indices
    voxel_indices = np.round(points / voxel_size).astype(np.int32)

    # Get unique voxel indices
    unique_indices = np.unique(voxel_indices, axis=0)

    # Convert back to world coordinates
    unique_voxels = unique_indices * voxel_size

    return unique_voxels
