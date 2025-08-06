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


import functools
import logging
import os
import threading
import time
import warnings
import xml.etree.ElementTree as ET
from typing import Final, Protocol, Optional

import mujoco
import numpy as np
import onnxruntime as rt
import open3d as o3d
from etils import epath
from mujoco import viewer
from mujoco_playground._src import mjx_env
from pynput import keyboard
from reactivex import Observable

from dimos.core import In, Module, Out, rpc
from dimos import core
from dimos_lcm.std_msgs import Bool
from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Vector3, Transform
from dimos.msgs.nav_msgs import OccupancyGrid, Path
from dimos.msgs.sensor_msgs import Image
from dimos.navigation.bt_navigator.navigator import BehaviorTreeNavigator
from dimos.navigation.frontier_exploration import WavefrontFrontierExplorer
from dimos.navigation.global_planner import AstarPlanner
from dimos.navigation.local_planner.holonomic_local_planner import HolonomicLocalPlanner
from dimos.perception.spatial_perception import SpatialMemory
from dimos.protocol import pubsub
from dimos.protocol.tf import TF
from dimos.robot.foxglove_bridge import FoxgloveBridge
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.map import Map
from dimos.robot.unitree_webrtc.type.odometry import Odometry
from dimos.robot.unitree_webrtc.unitree_skills import MyUnitreeSkills
from dimos.skills.skills import AbstractRobotSkill, SkillLibrary
from dimos.types.vector import Vector
from dimos.utils.logging_config import setup_logger
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule

RANGE_FINDER_MAX_RANGE = 10
LIDAR_RESOLUTION = 0.05
LIDAR_FREQUENCY = 10
ODOM_FREQUENCY = 50
VIDEO_FREQUENCY = 30

logger = setup_logger("dimos.robot.unitree_webrtc.unitree_go1_mujoco", level=logging.INFO)

# Suppress verbose loggers
logging.getLogger("aiortc.codecs.h264").setLevel(logging.ERROR)
logging.getLogger("lcm_foxglove_bridge").setLevel(logging.ERROR)
logging.getLogger("websockets.server").setLevel(logging.ERROR)
logging.getLogger("FoxgloveServer").setLevel(logging.ERROR)
logging.getLogger("asyncio").setLevel(logging.ERROR)
logging.getLogger("root").setLevel(logging.WARNING)

# Suppress warnings
warnings.filterwarnings("ignore", message="coroutine.*was never awaited")
warnings.filterwarnings("ignore", message="H264Decoder.*failed to decode")

_HERE = epath.Path(__file__).parent

# Primary knobs for top speed. Adjust these to make the robot faster or slower.
VEL_SCALE_X: Final[float] = 1.8
VEL_SCALE_Y: Final[float] = 0.8
VEL_SCALE_ROT: Final[float] = 2 * np.pi  # Max rotational speed (rad/s)

# Parameters for keyboard smoothing. Adjust these to change the "feel".
KEYBOARD_ACCEL_RATE: Final[float] = 2.5  # How quickly robot reaches top speed
# How quickly robot stops (0.0=instant, 1.0=never)
KEYBOARD_DRAG_FACTOR: Final[float] = 0.95
KEYBOARD_DRAG_FACTOR_ROT: Final[float] = 0.95
UPDATE_RATE_HZ: Final[int] = 50


def get_assets() -> dict[str, bytes]:
    assets = {}
    mjx_env.update_assets(assets, _HERE / "go1" / "xmls", "*.xml")
    mjx_env.update_assets(assets, _HERE / "go1" / "xmls" / "assets")
    path = mjx_env.MENAGERIE_PATH / "unitree_go1"
    mjx_env.update_assets(assets, path, "*.xml")
    mjx_env.update_assets(assets, path / "assets")
    return assets


class MujocoThread(threading.Thread):
    def __init__(self):
        super().__init__()
        self.shared_pixels = None
        self.pixels_lock = threading.RLock()
        self.odom_data = None
        self.odom_lock = threading.RLock()
        self.lidar_lock = threading.RLock()
        self.model = None
        self.data = None

    def run(self):
        self.model, self.data, input_controller = load_callback()

        camera_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "head_camera")
        last_render = time.time()
        render_interval = 1.0 / VIDEO_FREQUENCY

        with viewer.launch_passive(self.model, self.data) as m_viewer:
            # Comment this out to show the rangefinders.
            m_viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = 0
            window_size = (640, 480)
            renderer = mujoco.Renderer(self.model, height=window_size[1], width=window_size[0])
            scene_option = mujoco.MjvOption()
            scene_option.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = False

            while m_viewer.is_running():
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
                    renderer.update_scene(self.data, camera=camera_id, scene_option=scene_option)
                    pixels = renderer.render()

                    with self.pixels_lock:
                        self.shared_pixels = pixels.copy()

                m_viewer.sync()

        input_controller.stop()

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
            position=Vector3(pos[0], pos[1], pos[2]), orientation=orientation, ts=time.time()
        )
        return odom_to_publish


class InputController(Protocol):
    """A protocol for input devices to control the robot."""

    def get_command(self) -> np.ndarray: ...
    def stop(self) -> None: ...


class KeyboardController:
    """Reads keyboard input and applies smoothing for robot control."""

    def __init__(self, vel_scale_x: float, vel_scale_y: float, vel_scale_rot: float):
        self._vel_scale_x = vel_scale_x
        self._vel_scale_y = vel_scale_y
        self._vel_scale_rot = vel_scale_rot

        self.vx, self.vy, self.wz = 0.0, 0.0, 0.0
        self._keys_pressed = set()
        self._is_running = True

        self._listener = keyboard.Listener(on_press=self._on_press, on_release=self._on_release)
        self._listener.start()
        self._update_thread = threading.Thread(target=self._update_loop, daemon=True)
        self._update_thread.start()

    def _on_press(self, key):
        try:
            self._keys_pressed.add(key.char.lower())
        except AttributeError:
            pass

    def _on_release(self, key):
        try:
            self._keys_pressed.discard(key.char.lower())
        except AttributeError:
            pass

    def _update_loop(self):
        """Applies acceleration and drag to smooth the keyboard inputs."""
        self._last_exec = time.time() - (1 / UPDATE_RATE_HZ)
        while self._is_running:
            dt = time.time() - self._last_exec
            self._last_exec = time.time()
            # Apply acceleration based on pressed keys
            accel_x = KEYBOARD_ACCEL_RATE * dt
            if "w" in self._keys_pressed:
                self.vx += accel_x
            if "s" in self._keys_pressed:
                self.vx -= accel_x

            accel_y = KEYBOARD_ACCEL_RATE * dt
            if "a" in self._keys_pressed:
                self.vy += accel_y
            if "d" in self._keys_pressed:
                self.vy -= accel_y

            accel_rot = KEYBOARD_ACCEL_RATE * dt
            if "q" in self._keys_pressed:
                self.wz += accel_rot
            if "e" in self._keys_pressed:
                self.wz -= accel_rot

            # Apply drag/deceleration
            self.vx *= KEYBOARD_DRAG_FACTOR
            self.vy *= KEYBOARD_DRAG_FACTOR
            self.wz *= KEYBOARD_DRAG_FACTOR_ROT

            # Clamp to max velocities
            self.vx = np.clip(self.vx, -self._vel_scale_x, self._vel_scale_x)
            self.vy = np.clip(self.vy, -self._vel_scale_y, self._vel_scale_y)
            self.wz = np.clip(self.wz, -self._vel_scale_rot, self._vel_scale_rot)

            time.sleep(1 / UPDATE_RATE_HZ)

    def get_command(self) -> np.ndarray:
        return np.array([self.vx, self.vy, self.wz], dtype=np.float32)

    def stop(self) -> None:
        if self._is_running:
            self._is_running = False
            self._listener.stop()
            self._listener.join()
            self._update_thread.join()


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


def load_callback(model=None, data=None):
    mujoco.set_mjcb_control(None)

    # Generate the XML at runtime
    xml_path = (_HERE / "go1/xmls/robot.xml").as_posix()

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
            sensor_element, "rangefinder", name=f"range_{i}", site=f"lidar_{i}", cutoff="10"
        )

    xml_content = ET.tostring(root, encoding="unicode")

    model = mujoco.MjModel.from_xml_string(
        xml_content,
        assets=get_assets(),
    )
    data = mujoco.MjData(model)

    mujoco.mj_resetDataKeyframe(model, data, 0)

    input_device = KeyboardController(VEL_SCALE_X, VEL_SCALE_Y, VEL_SCALE_ROT)

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

    return model, data, input_device


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


class MujocoConnection:
    def __init__(self, *args, **kwargs):
        self.mujoco_thread = MujocoThread()

    def start(self):
        self.mujoco_thread.start()

    def standup(self):
        print("standup supressed")

    def liedown(self):
        print("liedown supressed")

    @functools.cache
    def lidar_stream(self):
        def on_subscribe(observer, scheduler):
            stop_event = threading.Event()

            def run():
                while not stop_event.is_set():
                    lidar_to_publish = self.mujoco_thread.get_lidar_message()

                    if lidar_to_publish:
                        observer.on_next(lidar_to_publish)

                    time.sleep(1 / LIDAR_FREQUENCY)

                observer.on_completed()

            thread = threading.Thread(target=run, daemon=True)
            thread.start()

            def dispose():
                stop_event.set()

            return dispose

        return Observable(on_subscribe)

    @functools.cache
    def odom_stream(self):
        print("odom stream start")

        def on_subscribe(observer, scheduler):
            stop_event = threading.Event()

            def run():
                while not stop_event.is_set():
                    odom_to_publish = self.mujoco_thread.get_odom_message()
                    if odom_to_publish:
                        observer.on_next(odom_to_publish)

                    time.sleep(1 / ODOM_FREQUENCY)
                observer.on_completed()

            thread = threading.Thread(target=run, daemon=True)
            thread.start()

            def dispose():
                stop_event.set()

            return dispose

        return Observable(on_subscribe)

    @functools.cache
    def video_stream(self):
        print("video stream start")

        def on_subscribe(observer, scheduler):
            stop_event = threading.Event()

            def run():
                while not stop_event.is_set():
                    with self.mujoco_thread.pixels_lock:
                        if self.mujoco_thread.shared_pixels is not None:
                            img = Image.from_numpy(self.mujoco_thread.shared_pixels.copy())
                            observer.on_next(img)
                    time.sleep(1 / VIDEO_FREQUENCY)
                observer.on_completed()

            thread = threading.Thread(target=run, daemon=True)
            thread.start()

            def dispose():
                stop_event.set()

            return dispose

        return Observable(on_subscribe)

    def move(self, vector: Vector):
        ...
        # print("move supressed", vector)


class ConnectionModule(Module):
    """Module that handles robot sensor data and movement commands."""

    movecmd: In[Vector3] = None
    odom: Out[PoseStamped] = None
    lidar: Out[LidarMessage] = None
    video: Out[Image] = None
    ip: str

    _odom: PoseStamped = None
    _lidar: LidarMessage = None

    def __init__(self, ip: str = None, *args, **kwargs):
        self.ip = ip
        self.tf = TF()
        self.connection = MujocoConnection()
        Module.__init__(self, *args, **kwargs)

    @rpc
    def start(self):
        """Start the connection and subscribe to sensor streams."""

        self.connection.start()

        # Connect sensor streams to outputs
        self.connection.lidar_stream().subscribe(self.lidar.publish)
        self.connection.odom_stream().subscribe(self._publish_tf)
        self.connection.video_stream().subscribe(self.video.publish)
        self.movecmd.subscribe(self.move)

    def _publish_tf(self, msg):
        self._odom = msg
        self.odom.publish(msg)
        self.tf.publish(Transform.from_pose("base_link", msg))
        camera_link = Transform(
            translation=Vector3(0.3, 0.0, 0.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id="base_link",
            child_frame_id="camera_link",
            ts=time.time(),
        )
        self.tf.publish(camera_link)

    @rpc
    def get_odom(self) -> Optional[PoseStamped]:
        """Get the robot's odometry.

        Returns:
            The robot's odometry
        """
        return self._odom

    @rpc
    def move(self, vector: Vector3, duration: float = 0.0):
        """Send movement command to robot."""
        self.connection.move(vector, duration)

    @rpc
    def standup(self):
        """Make the robot stand up."""
        return self.connection.standup()

    @rpc
    def liedown(self):
        """Make the robot lie down."""
        return self.connection.liedown()

    @rpc
    def publish_request(self, topic: str, data: dict):
        """Publish a request to the WebRTC connection.
        Args:
            topic: The RTC topic to publish to
            data: The data dictionary to publish
        Returns:
            The result of the publish request
        """
        return self.connection.publish_request(topic, data)


class UnitreeGo1:
    def __init__(
        self,
        ip: str,
        output_dir: str = None,
        websocket_port: int = 7779,
        skill_library: Optional[SkillLibrary] = None,
    ):
        """Initialize the robot system.

        Args:
            ip: Robot IP address (or None for fake connection)
            output_dir: Directory for saving outputs (default: assets/output)
            enable_perception: Whether to enable spatial memory/perception
            websocket_port: Port for web visualization
            skill_library: Skill library instance
        """
        self.ip = ip
        self.output_dir = output_dir or os.path.join(os.getcwd(), "assets", "output")
        self.websocket_port = websocket_port

        # Initialize skill library
        if skill_library is None:
            skill_library = MyUnitreeSkills()
        self.skill_library = skill_library

        self.dimos = None
        self.connection = None
        self.mapper = None
        self.global_planner = None
        self.local_planner = None
        self.navigator = None
        self.frontier_explorer = None
        self.websocket_vis = None
        self.foxglove_bridge = None
        self.spatial_memory_module = None

        self._setup_directories()

    def _setup_directories(self):
        """Setup directories for spatial memory storage."""
        os.makedirs(self.output_dir, exist_ok=True)
        logger.info(f"Robot outputs will be saved to: {self.output_dir}")

        # Initialize memory directories
        self.memory_dir = os.path.join(self.output_dir, "memory")
        os.makedirs(self.memory_dir, exist_ok=True)

        # Initialize spatial memory properties
        self.spatial_memory_dir = os.path.join(self.memory_dir, "spatial_memory")
        self.spatial_memory_collection = "spatial_memory"
        self.db_path = os.path.join(self.spatial_memory_dir, "chromadb_data")
        self.visual_memory_path = os.path.join(self.spatial_memory_dir, "visual_memory.pkl")

        # Create spatial memory directories
        os.makedirs(self.spatial_memory_dir, exist_ok=True)
        os.makedirs(self.db_path, exist_ok=True)

    def start(self):
        """Start the robot system with all modules."""
        self.dimos = core.start(4)

        self._deploy_connection()
        self._deploy_mapping()
        self._deploy_navigation()
        self._deploy_visualization()
        self._deploy_perception()

        self._start_modules()

        logger.info("UnitreeGo1 initialized and started")
        logger.info(f"WebSocket visualization available at http://localhost:{self.websocket_port}")

    def _deploy_connection(self):
        """Deploy and configure the connection module."""
        self.connection = self.dimos.deploy(ConnectionModule, self.ip)

        self.connection.lidar.transport = core.LCMTransport("/lidar", LidarMessage)
        self.connection.odom.transport = core.LCMTransport("/odom", PoseStamped)
        self.connection.video.transport = core.LCMTransport("/video", Image)
        self.connection.movecmd.transport = core.LCMTransport("/cmd_vel", Vector3)

    def _deploy_mapping(self):
        """Deploy and configure the mapping module."""
        self.mapper = self.dimos.deploy(Map, voxel_size=0.5, global_publish_interval=2.5)

        self.mapper.global_map.transport = core.LCMTransport("/global_map", LidarMessage)
        self.mapper.global_costmap.transport = core.LCMTransport("/global_costmap", OccupancyGrid)
        self.mapper.local_costmap.transport = core.LCMTransport("/local_costmap", OccupancyGrid)

        self.mapper.lidar.connect(self.connection.lidar)

    def _deploy_navigation(self):
        """Deploy and configure navigation modules."""
        self.global_planner = self.dimos.deploy(AstarPlanner)
        self.local_planner = self.dimos.deploy(HolonomicLocalPlanner)
        self.navigator = self.dimos.deploy(BehaviorTreeNavigator, local_planner=self.local_planner)
        self.frontier_explorer = self.dimos.deploy(WavefrontFrontierExplorer)

        self.navigator.goal.transport = core.LCMTransport("/navigation_goal", PoseStamped)
        self.navigator.goal_request.transport = core.LCMTransport("/goal_request", PoseStamped)
        self.navigator.goal_reached.transport = core.LCMTransport("/goal_reached", Bool)
        self.navigator.global_costmap.transport = core.LCMTransport(
            "/global_costmap", OccupancyGrid
        )
        self.global_planner.path.transport = core.LCMTransport("/global_path", Path)
        self.local_planner.cmd_vel.transport = core.LCMTransport("/cmd_vel", Vector3)
        self.frontier_explorer.goal_request.transport = core.LCMTransport(
            "/goal_request", PoseStamped
        )
        self.frontier_explorer.goal_reached.transport = core.LCMTransport("/goal_reached", Bool)

        self.global_planner.target.connect(self.navigator.goal)

        self.global_planner.global_costmap.connect(self.mapper.global_costmap)
        self.global_planner.odom.connect(self.connection.odom)

        self.local_planner.path.connect(self.global_planner.path)
        self.local_planner.local_costmap.connect(self.mapper.local_costmap)
        self.local_planner.odom.connect(self.connection.odom)

        self.connection.movecmd.connect(self.local_planner.cmd_vel)

        self.navigator.odom.connect(self.connection.odom)

        self.frontier_explorer.costmap.connect(self.mapper.global_costmap)
        self.frontier_explorer.odometry.connect(self.connection.odom)

    def _deploy_visualization(self):
        """Deploy and configure visualization modules."""
        self.websocket_vis = self.dimos.deploy(WebsocketVisModule, port=self.websocket_port)
        self.websocket_vis.click_goal.transport = core.LCMTransport("/goal_request", PoseStamped)

        self.websocket_vis.robot_pose.connect(self.connection.odom)
        self.websocket_vis.path.connect(self.global_planner.path)
        self.websocket_vis.global_costmap.connect(self.mapper.global_costmap)

        self.foxglove_bridge = FoxgloveBridge()

    def _deploy_perception(self):
        """Deploy and configure the spatial memory module."""
        self.spatial_memory_module = self.dimos.deploy(
            SpatialMemory,
            collection_name=self.spatial_memory_collection,
            db_path=self.db_path,
            visual_memory_path=self.visual_memory_path,
            output_dir=self.spatial_memory_dir,
        )

        self.spatial_memory_module.video.connect(self.connection.video)
        self.spatial_memory_module.odom.connect(self.connection.odom)

        logger.info("Spatial memory module deployed and connected")

    def _start_modules(self):
        """Start all deployed modules in the correct order."""
        self.connection.start()
        self.mapper.start()
        self.global_planner.start()
        self.local_planner.start()
        self.navigator.start()
        self.frontier_explorer.start()
        self.websocket_vis.start()
        self.foxglove_bridge.start()

        if self.spatial_memory_module:
            self.spatial_memory_module.start()

        # Initialize skills after connection is established
        if self.skill_library is not None:
            for skill in self.skill_library:
                if isinstance(skill, AbstractRobotSkill):
                    self.skill_library.create_instance(skill.__name__, robot=self)
            if isinstance(self.skill_library, MyUnitreeSkills):
                self.skill_library._robot = self
                self.skill_library.init()
                self.skill_library.initialize_skills()

    def move(self, vector: Vector3, duration: float = 0.0):
        """Send movement command to robot."""
        self.connection.move(vector, duration)

    def explore(self) -> bool:
        """Start autonomous frontier exploration.

        Returns:
            True if exploration started successfully
        """
        return self.frontier_explorer.explore()

    def navigate_to(self, pose: PoseStamped, blocking: bool = True):
        """Navigate to a target pose.

        Args:
            pose: Target pose to navigate to
            blocking: If True, block until goal is reached. If False, return immediately.

        Returns:
            If blocking=True: True if navigation was successful, False otherwise
            If blocking=False: True if goal was accepted, False otherwise
        """

        logger.info(
            f"Navigating to pose: ({pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f})"
        )
        return self.navigator.set_goal(pose, blocking=blocking)

    def stop_exploration(self) -> bool:
        """Stop autonomous exploration.

        Returns:
            True if exploration was stopped
        """
        return self.frontier_explorer.stop_exploration()

    def cancel_navigation(self) -> bool:
        """Cancel the current navigation goal.

        Returns:
            True if goal was cancelled
        """
        return self.navigator.cancel_goal()

    @property
    def spatial_memory(self) -> Optional[SpatialMemory]:
        """Get the robot's spatial memory module.

        Returns:
            SpatialMemory module instance or None if perception is disabled
        """
        return self.spatial_memory_module

    def get_skills(self):
        """Get the robot's skill library.

        Returns:
            The robot's skill library for adding/managing skills
        """
        return self.skill_library

    def get_odom(self) -> PoseStamped:
        """Get the robot's odometry.

        Returns:
            The robot's odometry
        """
        return self.connection.get_odom()


def main():
    """Main entry point."""
    ip = os.getenv("ROBOT_IP")

    pubsub.lcm.autoconf()

    robot = UnitreeGo1(ip=ip, websocket_port=7779)
    robot.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("Shutting down...")


if __name__ == "__main__":
    main()
