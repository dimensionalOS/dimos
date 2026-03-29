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

"""
NavBot class for navigation-related functionality.
Encapsulates ROS transport and topic remapping for Unitree robots.
"""

from dataclasses import field
import logging
from pathlib import Path
import platform
import threading
import time
from typing import Any

import cv2
import numpy as np

# ROS message imports: available inside the ROS2 container, but may be missing on the host
try:  # pragma: no cover - import-time environment dependent
    from geometry_msgs.msg import (  # type: ignore[attr-defined]
        PointStamped as ROSPointStamped,
        PoseStamped as ROSPoseStamped,
        TwistStamped as ROSTwistStamped,
    )
    from nav_msgs.msg import Odometry as ROSOdometry, Path as ROSPath  # type: ignore[attr-defined]
    from sensor_msgs.msg import (  # type: ignore[attr-defined]
        CompressedImage as ROSCompressedImage,
        Joy as ROSJoy,
        PointCloud2 as ROSPointCloud2,
    )
    from std_msgs.msg import (  # type: ignore[attr-defined]
        Bool as ROSBool,
        Int8 as ROSInt8,
    )
    from tf2_msgs.msg import TFMessage as ROSTFMessage  # type: ignore[attr-defined]
except ModuleNotFoundError:
    # Running outside a ROS2 environment (e.g. host CLI without ROS Python packages).
    # Define minimal placeholder types so blueprints can import without failing.
    class _Stub:  # pragma: no cover - host-only stub
        def __init__(self, *args: Any, **kwargs: Any) -> None:
            pass

    ROSPointStamped = _Stub  # type: ignore[assignment]
    ROSPoseStamped = _Stub  # type: ignore[assignment]
    ROSTwistStamped = _Stub  # type: ignore[assignment]
    ROSOdometry = _Stub  # type: ignore[assignment]
    ROSPath = _Stub  # type: ignore[assignment]
    ROSCompressedImage = _Stub  # type: ignore[assignment]
    ROSJoy = _Stub  # type: ignore[assignment]
    ROSPointCloud2 = _Stub  # type: ignore[assignment]
    ROSBool = _Stub  # type: ignore[assignment]
    ROSInt8 = _Stub  # type: ignore[assignment]
    ROSTFMessage = _Stub  # type: ignore[assignment]

from dimos_lcm.std_msgs import Bool

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.docker_module import DockerModuleConfig
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.navigation.base import NavigationInterface, NavigationState
from dimos.utils.data import get_data
from dimos.utils.generic import is_jetson
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import euler_to_quaternion

logger = setup_logger(level=logging.INFO)


class ROSNavConfig(DockerModuleConfig):
    # Module settings
    local_pointcloud_freq: float = 2.0
    global_map_freq: float = 1.0
    sensor_to_base_link_transform: Transform = field(
        default_factory=lambda: Transform(frame_id="sensor", child_frame_id="base_link")
    )

    # Docker settings
    docker_restart_policy: str = "no"  # Don't auto-restart; host process manages lifecycle
    docker_startup_timeout: float = 180
    docker_image: str = "dimos_rosnav:humble"
    docker_shm_size: str = "8g"
    docker_entrypoint: str = "/usr/local/bin/entrypoint.sh"
    docker_file: Path = Path(__file__).parent / "Dockerfile"
    docker_build_context: Path = Path(__file__).parent.parent.parent.parent
    docker_build_extra_args: list[str] = field(default_factory=lambda: ["--network", "host"])
    docker_build_args: dict[str, str] = field(
        default_factory=lambda: {
            "TARGETARCH": "arm64" if platform.machine() == "aarch64" else "amd64"
        }
    )
    docker_gpus: str | None = None if is_jetson() else "all"
    docker_extra_args: list[str] = field(
        default_factory=lambda: [
            "--cap-add=NET_ADMIN",
            *(["--runtime=nvidia"] if is_jetson() else []),
        ]
    )
    docker_env: dict[str, str] = field(
        default_factory=lambda: {
            "ROS_DISTRO": "humble",
            "ROS_DOMAIN_ID": "42",
            "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
            "FASTRTPS_DEFAULT_PROFILES_FILE": "/ros2_ws/config/fastdds.xml",
            "QT_X11_NO_MITSHM": "1",
            "NVIDIA_VISIBLE_DEVICES": "all",
            "NVIDIA_DRIVER_CAPABILITIES": "all",
            # Give DDS topic discovery enough time after Unity registers publishers.
            # Default in the entrypoint is 25s which is too short on some machines.
            "UNITY_BRIDGE_CONNECT_TIMEOUT_SEC": "60",
        }
    )
    docker_volumes: list[tuple[str, str, str]] = field(default_factory=lambda: [])
    docker_devices: list[str] = field(
        default_factory=lambda: [
            "/dev/input:/dev/input",
            *(["/dev/dri:/dev/dri"] if Path("/dev/dri").exists() else []),
        ]
    )

    # Vehicle geometry
    # Height of the robot's base_link above the ground plane (metres).
    # The CMU nav stack uses this to position the simulated sensor origin;
    # it is forwarded to the ROS launch as the ``vehicleHeight`` parameter.
    vehicle_height: float = 0.75

    # Teleop override
    # Seconds of silence after the last teleop cmd_vel before switching back
    # to the ROS nav stack.  At the end of the cooldown the module publishes
    # a goal at the robot's current position so the nav stack re-engages at
    # standstill instead of resuming the old goal.
    teleop_cooldown_sec: float = 1.0

    # Runtime mode settings
    # mode controls which ROS launch file the entrypoint selects:
    #   "simulation"    — system_simulation[_with_route_planner].launch.py + Unity if present
    #   "unity_sim"     — same as simulation but hard-exits if Unity binary is missing
    #   "external_sim"  — same launch as simulation, but no internal Unity (sensor data from LCM)
    #   "hardware"      — system_real_robot[_with_route_planner].launch.py
    #   "bagfile"       — system_bagfile[_with_route_planner].launch.py + use_sim_time
    # Setting bagfile_path automatically forces mode to "bagfile".
    mode: str = "hardware"
    use_route_planner: bool = False
    localization_method: str = "arise_slam"
    robot_config_path: str = "unitree/unitree_g1"
    robot_ip: str = ""
    bagfile_path: str | Path = ""  # host-side path to bag; remapped into container at runtime

    # use_rviz: whether to launch RViz2 inside the container.
    #   None (default) → True for simulation/unity_sim modes, False otherwise
    #   (mirrors the unconditional RViz launch in run_both.sh for simulation)
    use_rviz: bool = False
    foxglove_port: int = 8765

    # Hardware sensor / network settings (used when mode="hardware")
    # lidar_interface: host ethernet interface connected to Mid-360 lidar (e.g. "eth0")
    # lidar_computer_ip: IP to assign/use on that interface for lidar communication
    # lidar_gateway: gateway IP for the lidar subnet
    # lidar_ip: IP address of the Mid-360 lidar device itself
    # unitree_ip: Unitree robot IP for WebRTC connection
    # unitree_conn: WebRTC connection method — "LocalAP", "LocalSTA", or "Remote"
    lidar_interface: str = ""
    lidar_computer_ip: str = ""
    lidar_gateway: str = ""
    lidar_ip: str = ""
    unitree_ip: str = "192.168.12.1"
    unitree_conn: str = "LocalAP"

    # When True, download and mount sim assets (map.ply, traversable_area.ply) even
    # in hardware mode.  Used when running hardware-mode nav stack with an external
    # simulator that still needs the pre-built map data.
    # TODO: remove once the nav stack can build maps purely from incoming lidar.
    mount_sim_assets: bool = False

    def model_post_init(self, __context: object) -> None:
        import os

        effective_mode = "bagfile" if self.bagfile_path else self.mode
        self.docker_env["MODE"] = effective_mode

        # Hardware sensor env vars — read by entrypoint.sh when MODE=hardware.
        is_hardware = effective_mode == "hardware"
        if is_hardware:
            # Privileged mode is required for ip link/ip addr and sysctl inside the container.
            self.docker_privileged = True
            self.docker_env["LIDAR_INTERFACE"] = self.lidar_interface
            self.docker_env["LIDAR_COMPUTER_IP"] = self.lidar_computer_ip
            self.docker_env["LIDAR_GATEWAY"] = self.lidar_gateway
            self.docker_env["LIDAR_IP"] = self.lidar_ip
            self.docker_env["UNITREE_IP"] = self.unitree_ip
            self.docker_env["UNITREE_CONN"] = self.unitree_conn

        if self.bagfile_path:
            bag_path = Path(self.bagfile_path).expanduser()
            if bag_path.exists():
                bag_path = bag_path.resolve()
                bag_dir = bag_path.parent
                bag_name = bag_path.name
                container_bag_dir = "/ros2_ws/bagfiles"

                self.docker_volumes.append((str(bag_dir), container_bag_dir, "rw"))
                self.docker_env["BAGFILE_PATH"] = f"{container_bag_dir}/{bag_name}"
            else:
                self.docker_env["BAGFILE_PATH"] = str(self.bagfile_path)

        self.docker_env["USE_RVIZ"] = "true" if self.use_rviz else "false"
        self.docker_env["FOXGLOVE_PORT"] = str(self.foxglove_port)
        self.docker_env["USE_ROUTE_PLANNER"] = "true" if self.use_route_planner else "false"
        self.docker_env["LOCALIZATION_METHOD"] = self.localization_method
        self.docker_env["ROBOT_CONFIG_PATH"] = self.robot_config_path
        self.docker_env["ROBOT_IP"] = self.robot_ip
        self.docker_env["VEHICLE_HEIGHT"] = str(self.vehicle_height)

        # Pass host DISPLAY through for X11 forwarding (RViz, Unity)
        if display := os.environ.get("DISPLAY", ":0"):
            self.docker_env["DISPLAY"] = display

        self.docker_env["QT_X11_NO_MITSHM"] = "1"

        repo_root = Path(__file__).parent.parent.parent.parent
        self.docker_volumes += [
            # X11 socket for display forwarding (RViz, Unity)
            ("/tmp/.X11-unix", "/tmp/.X11-unix", "rw"),
            # Mount live dimos source so the module is always up-to-date
            (str(repo_root), "/workspace/dimos", "rw"),
            # Mount DDS config (fastdds.xml) from host — single file mount
            # avoids shadowing the entire /ros2_ws/config directory
            (str(Path(__file__).parent / "fastdds.xml"), "/ros2_ws/config/fastdds.xml", "ro"),
            # Note: most of the mounts below are only needed for development
            # Mount entrypoint script so changes don't require a rebuild
            (
                str(Path(__file__).parent / "entrypoint.sh"),
                "/usr/local/bin/entrypoint.sh",
                "ro",
            ),
        ]

        # Only download and mount sim assets for simulation modes (avoids slow LFS pull in hardware mode).
        # mount_sim_assets overrides this for hardware mode with external sim.
        # TODO: remove mount_sim_assets once nav stack can build maps from lidar alone.
        if effective_mode in ("simulation", "unity_sim", "external_sim") or self.mount_sim_assets:
            sim_data_dir = str(get_data("office_building_1"))
            self.docker_volumes += [
                # Mount Unity sim (office_building_1) — downloaded via get_data / LFS
                # Provides map.ply, traversable_area.ply and environment/Model.x86_64
                (
                    sim_data_dir,
                    "/ros2_ws/src/ros-navigation-autonomy-stack/src/base_autonomy/vehicle_simulator/mesh/unity/",
                    "rw",
                ),
                # real_world uses the same sim data
                (
                    sim_data_dir,
                    "/ros2_ws/src/ros-navigation-autonomy-stack/src/base_autonomy/vehicle_simulator/mesh/real_world/",
                    "rw",
                ),
                # Some CMU stack nodes (e.g., visualizationTools.cpp) rewrite install paths
                # to /ros2_ws/src/base_autonomy/... directly. Mirror the same sim asset
                # directory at that legacy path to avoid "map.ply not found" errors.
                (
                    sim_data_dir,
                    "/ros2_ws/src/base_autonomy/vehicle_simulator/mesh/unity/",
                    "rw",
                ),
                (
                    sim_data_dir,
                    "/ros2_ws/src/base_autonomy/vehicle_simulator/mesh/real_world/",
                    "rw",
                ),
            ]

        # Mount Xauthority cookie for X11 forwarding.
        # Honour $XAUTHORITY on the host (falls back to ~/.Xauthority) and
        # place it at /tmp/.Xauthority inside the container so it is
        # accessible regardless of which user the container runs as.
        xauth_host = Path(os.environ.get("XAUTHORITY", str(Path.home() / ".Xauthority")))
        if xauth_host.exists():
            self.docker_volumes.append((str(xauth_host), "/tmp/.Xauthority", "ro"))
            self.docker_env["XAUTHORITY"] = "/tmp/.Xauthority"


class ROSNav(Module, NavigationInterface):
    config: ROSNavConfig
    default_config = ROSNavConfig

    goal_request: In[PoseStamped]
    clicked_point: In[PointStamped]
    stop_explore_cmd: In[Bool]
    tele_cmd_vel: In[Twist]

    # External sensor inputs — when connected, data is republished to ROS2
    # topics inside the Docker container so the nav stack can consume them
    # (e.g. from an external simulator).
    ext_registered_scan: In[PointCloud2]
    ext_odometry: In[Odometry]

    lidar: Out[PointCloud2]
    terrain_map: Out[PointCloud2]
    global_pointcloud: Out[PointCloud2]
    rosnav_overall_map: Out[PointCloud2]
    odom: Out[PoseStamped]
    goal_active: Out[PoseStamped]
    goal_reached: Out[Bool]
    path: Out[NavPath]
    cmd_vel: Out[Twist]

    _current_position_running: bool = False
    _spin_thread: threading.Thread | None = None
    _goal_reach: bool | None = None

    # Navigation state tracking for NavigationInterface
    _navigation_state: NavigationState = NavigationState.IDLE
    _state_lock: threading.Lock
    _navigation_thread: threading.Thread | None = None
    _current_goal: PoseStamped | None = None
    _goal_reached: bool = False

    # Teleop override state
    _teleop_active: bool = False
    _teleop_lock: threading.Lock
    _teleop_timer: threading.Timer | None = None
    _last_odom: PoseStamped | None = None

    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(*args, **kwargs)
        import rclpy
        from rclpy.node import Node

        # Initialize state tracking
        self._state_lock = threading.Lock()
        self._teleop_lock = threading.Lock()
        self._navigation_state = NavigationState.IDLE
        self._goal_reached = False

        if not rclpy.ok():  # type: ignore[attr-defined]
            rclpy.init()

        self._node = Node("navigation_module")

        # ROS2 Publishers
        self.goal_pose_pub = self._node.create_publisher(ROSPoseStamped, "/goal_pose", 10)
        self.cancel_goal_pub = self._node.create_publisher(ROSBool, "/cancel_goal", 10)
        self.soft_stop_pub = self._node.create_publisher(ROSInt8, "/stop", 10)
        self.joy_pub = self._node.create_publisher(ROSJoy, "/joy", 10)

        # ROS2 Subscribers
        self.goal_reached_sub = self._node.create_subscription(
            ROSBool, "/goal_reached", self._on_ros_goal_reached, 10
        )
        from rclpy.qos import QoSProfile, ReliabilityPolicy  # type: ignore[attr-defined]

        self.cmd_vel_sub = self._node.create_subscription(
            ROSTwistStamped,
            "/cmd_vel",
            self._on_ros_cmd_vel,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.goal_waypoint_sub = self._node.create_subscription(
            ROSPointStamped, "/way_point", self._on_ros_goal_waypoint, 10
        )
        # Topic names depend on the SLAM backend:
        #   ARISE SLAM: /registered_scan, /state_estimation
        #   FAST_LIO:   /cloud_registered, /Odometry
        _use_arise = self.config.localization_method == "arise_slam"
        _scan_topic = "/registered_scan" if _use_arise else "/cloud_registered"
        _odom_topic = "/state_estimation" if _use_arise else "/Odometry"

        self.registered_scan_sub = self._node.create_subscription(
            ROSPointCloud2, _scan_topic, self._on_ros_registered_scan, 10
        )

        self.terrain_map_sub = self._node.create_subscription(
            ROSPointCloud2, "/terrain_map", self._on_ros_terrain_map, 10
        )
        self.global_pointcloud_sub = self._node.create_subscription(
            ROSPointCloud2, "/terrain_map_ext", self._on_ros_global_map, 10
        )

        self.rosnav_overall_map_sub = self._node.create_subscription(
            ROSPointCloud2, "/overall_map", self._on_ros_rosnav_overall_map, 10
        )

        self.path_sub = self._node.create_subscription(ROSPath, "/path", self._on_ros_path, 10)
        self.tf_sub = self._node.create_subscription(ROSTFMessage, "/tf", self._on_ros_tf, 10)
        self.odom_sub = self._node.create_subscription(
            ROSOdometry, _odom_topic, self._on_ros_odom, 10
        )

        # ROS2 publisher for external sensor data.
        # When ext_registered_scan input is connected, incoming DimOS PointCloud2
        # messages are converted and republished on this ROS2 topic so the nav
        # stack inside the container can consume them.
        self._ext_scan_pub = self._node.create_publisher(ROSPointCloud2, "/registered_scan", 10)
        self._ext_odom_pub = self._node.create_publisher(ROSOdometry, "/state_estimation", 10)

        logger.info(
            "NavigationModule initialized with ROS2 node "
            f"(scan={_scan_topic}, odom={_odom_topic})"
        )

    @rpc
    def start(self) -> None:
        try:
            self._running = True

            # Create and start the spin thread for ROS2 node spinning
            self._spin_thread = threading.Thread(
                target=self._spin_node, daemon=True, name="ROS2SpinThread"
            )
            self._spin_thread.start()

            self.goal_request.subscribe(self._on_goal_pose)
            self.clicked_point.subscribe(lambda pt: self._on_goal_pose(pt.to_pose_stamped()))
            self.stop_explore_cmd.subscribe(self._on_stop_cmd)
            self.tele_cmd_vel.subscribe(self._on_tele_cmd_vel)

            # External sensor inputs — republish to ROS2 topics for the nav stack
            self.ext_registered_scan.subscribe(self._on_ext_scan)
            self.ext_odometry.subscribe(self._on_ext_odom)

            logger.info("NavigationModule started with ROS2 spinning")
        except Exception as e:
            logger.error(f"ROSNav start() failed: {e}", exc_info=True)

    def _spin_node(self) -> None:
        import rclpy

        while self._running and rclpy.ok():  # type: ignore[attr-defined]
            try:
                rclpy.spin_once(self._node, timeout_sec=0.1)
            except Exception as e:
                if self._running:
                    logger.error(f"ROS2 spin error: {e}")

    def _on_ros_goal_reached(self, msg: ROSBool) -> None:
        self._goal_reach = msg.data
        self.goal_reached.publish(Bool(data=msg.data))
        if msg.data:
            with self._state_lock:
                self._goal_reached = True
                self._navigation_state = NavigationState.IDLE

    def _on_ros_goal_waypoint(self, msg: ROSPointStamped) -> None:
        dimos_pose = PoseStamped(
            ts=time.time(),
            frame_id=msg.header.frame_id,
            position=Vector3(msg.point.x, msg.point.y, msg.point.z),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        )
        self.goal_active.publish(dimos_pose)

    def _on_ros_cmd_vel(self, msg: ROSTwistStamped) -> None:
        if self._teleop_active:
            return  # Suppress nav stack cmd_vel during teleop override
        self.cmd_vel.publish(_twist_from_ros(msg.twist))

    def _on_ros_registered_scan(self, msg: ROSPointCloud2) -> None:
        self.lidar.publish(_pc2_from_ros(msg))

    def _on_ros_terrain_map(self, msg: "ROSPointCloud2") -> None:
        self.terrain_map.publish(_pc2_from_ros(msg))

    def _on_ros_global_map(self, msg: ROSPointCloud2) -> None:
        self.global_pointcloud.publish(_pc2_from_ros(msg))

    def _on_ros_rosnav_overall_map(self, msg: ROSPointCloud2) -> None:
        # FIXME: disabling for now for perf onboard G1 (and cause we don't have an overall map rn)
        # self.rosnav_overall_map.publish(_pc2_from_ros(msg))
        pass

    def _on_ros_path(self, msg: ROSPath) -> None:
        dimos_path = _path_from_ros(msg)
        # The CMU nav stack publishes the path in the "vehicle" frame which
        # corresponds to "sensor" in the DimOS TF tree (map → sensor).
        dimos_path.frame_id = "sensor"
        self.path.publish(dimos_path)

    def _on_ros_odom(self, msg: "ROSOdometry") -> None:
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        pose = PoseStamped(
            ts=ts,
            frame_id=msg.header.frame_id,
            position=Vector3(p.x, p.y, p.z),
            orientation=Quaternion(o.x, o.y, o.z, o.w),
        )
        self._last_odom = pose
        self.odom.publish(pose)

    def _on_ros_tf(self, msg: ROSTFMessage) -> None:
        # In external_sim mode, the UnityBridgeModule owns the ground-truth
        # transforms (map→sensor, map→world).  Don't republish SLAM TF here
        # or the two sources will fight and cause jitter.
        if self.config.mode == "external_sim":
            return

        ros_tf = _tfmessage_from_ros(msg)

        # In hardware/bagfile mode the SLAM initialises the sensor at the
        # map-frame origin, placing the ground plane at z = −vehicleHeight.
        # Shift the world frame down so that ground aligns with z = 0 in
        # Rerun.  In simulation the map frame already has ground at z = 0.
        is_sim = self.config.mode in ("simulation", "unity_sim")
        z_offset = 0.0 if is_sim else -self.config.vehicle_height

        map_to_world_tf = Transform(
            translation=Vector3(0.0, 0.0, z_offset),
            rotation=euler_to_quaternion(Vector3(0.0, 0.0, 0.0)),
            frame_id="map",
            child_frame_id="world",
            ts=time.time(),
        )

        self.tf.publish(
            self.config.sensor_to_base_link_transform.now(),
            map_to_world_tf,
            *ros_tf.transforms,
        )

    def _on_goal_pose(self, msg: PoseStamped) -> None:
        self.set_goal(msg)

    def _on_cancel_goal(self, msg: Bool) -> None:
        if msg.data:
            self.stop()

    def _on_stop_cmd(self, msg: Bool) -> None:
        if not msg.data:
            return
        logger.info("Stop command received, cancelling navigation")
        self.stop_navigation()
        # Set goal to current position so the nav stack re-engages at standstill
        if self._last_odom is not None:
            self._set_autonomy_mode()
            ros_pose = _pose_stamped_to_ros(self._last_odom)
            self.goal_pose_pub.publish(ros_pose)

    def _on_tele_cmd_vel(self, msg: Twist) -> None:
        with self._teleop_lock:
            if not self._teleop_active:
                self._teleop_active = True
                self.stop_navigation()
                logger.info("Teleop override: keyboard control active")

            # Cancel existing cooldown timer and start a new one
            if self._teleop_timer is not None:
                self._teleop_timer.cancel()
            self._teleop_timer = threading.Timer(
                self.config.teleop_cooldown_sec,
                self._end_teleop_override,
            )
            self._teleop_timer.daemon = True
            self._teleop_timer.start()

        # Forward teleop command to output
        self.cmd_vel.publish(msg)

    def _end_teleop_override(self) -> None:
        with self._teleop_lock:
            self._teleop_active = False
            self._teleop_timer = None

        # Set goal to current position so the nav stack resumes at standstill
        if self._last_odom is not None:
            logger.info("Teleop cooldown expired, setting goal to current position")
            self._set_autonomy_mode()
            ros_pose = _pose_stamped_to_ros(self._last_odom)
            self.goal_pose_pub.publish(ros_pose)
        else:
            logger.warning("Teleop cooldown expired but no odom available")

    # -- External sensor input callbacks --
    # Convert DimOS messages to ROS2 and republish on ROS2 topics.

    def _on_ext_scan(self, pc2: PointCloud2) -> None:
        self._ext_scan_pub.publish(_pc2_to_ros(pc2))

    def _on_ext_odom(self, odom: Odometry) -> None:
        self._ext_odom_pub.publish(_odometry_to_ros(odom))

    def _set_autonomy_mode(self) -> None:
        joy_msg = ROSJoy()  # type: ignore[no-untyped-call]
        joy_msg.axes = [
            0.0,  # axis 0
            0.0,  # axis 1
            -1.0,  # axis 2
            0.0,  # axis 3
            1.0,  # axis 4
            1.0,  # axis 5
            0.0,  # axis 6
            0.0,  # axis 7
        ]
        joy_msg.buttons = [
            0,  # button 0
            0,  # button 1
            0,  # button 2
            0,  # button 3
            0,  # button 4
            0,  # button 5
            0,  # button 6
            1,  # button 7 - controls autonomy mode
            0,  # button 8
            0,  # button 9
            0,  # button 10
        ]
        self.joy_pub.publish(joy_msg)
        logger.info("Setting autonomy mode via Joy message")

    @skill
    def goto(self, x: float, y: float) -> str:
        """
        move the robot in relative coordinates
        x is forward, y is left

        goto(1, 0) will move the robot forward by 1 meter
        """
        pose_to = PoseStamped(
            position=Vector3(x, y, 0),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id="base_link",
            ts=time.time(),
        )

        self.navigate_to(pose_to)
        return "arrived"

    @skill
    def goto_global(self, x: float, y: float) -> str:
        """
        go to coordinates x,y in the map frame
        0,0 is your starting position
        """
        target = PoseStamped(
            ts=time.time(),
            frame_id="map",
            position=Vector3(x, y, 0.0),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        )

        self.navigate_to(target)

        return f"arrived to {x:.2f}, {y:.2f}"

    @rpc
    def navigate_to(self, pose: PoseStamped, timeout: float = 60.0) -> bool:
        """
        Navigate to a target pose by publishing to ROS topics.

        Args:
            pose: Target pose to navigate to
            timeout: Maximum time to wait for goal (seconds)

        Returns:
            True if navigation was successful
        """
        logger.info(
            f"Navigating to goal: ({pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f} @ {pose.frame_id})"
        )

        self._goal_reach = None
        self._set_autonomy_mode()

        # Enable soft stop (0 = enable)
        soft_stop_msg = ROSInt8()  # type: ignore[no-untyped-call]
        soft_stop_msg.data = 0
        self.soft_stop_pub.publish(soft_stop_msg)

        ros_pose = _pose_stamped_to_ros(pose)
        self.goal_pose_pub.publish(ros_pose)

        # Wait for goal to be reached
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self._goal_reach is not None:
                soft_stop_msg.data = 2
                self.soft_stop_pub.publish(soft_stop_msg)
                return self._goal_reach
            time.sleep(0.1)

        self.stop_navigation()
        logger.warning(f"Navigation timed out after {timeout} seconds")
        return False

    @rpc
    def stop_navigation(self) -> bool:
        """
        Stop current navigation by publishing to ROS topics.

        Returns:
            True if stop command was sent successfully
        """
        logger.info("Stopping navigation")

        cancel_msg = ROSBool()  # type: ignore[no-untyped-call]
        cancel_msg.data = True
        self.cancel_goal_pub.publish(cancel_msg)

        soft_stop_msg = ROSInt8()  # type: ignore[no-untyped-call]
        soft_stop_msg.data = 2
        self.soft_stop_pub.publish(soft_stop_msg)

        # Unblock any waiting navigate_to() call
        self._goal_reach = False

        with self._state_lock:
            self._navigation_state = NavigationState.IDLE
            self._current_goal = None
            self._goal_reached = False

        return True

    @rpc
    def set_goal(self, goal: PoseStamped, timeout: float = 60.0) -> bool:
        """Set a new navigation goal (non-blocking)."""
        with self._state_lock:
            self._current_goal = goal
            self._goal_reached = False
            self._navigation_state = NavigationState.FOLLOWING_PATH

        # Cancel previous navigation and wait for thread to exit.
        # stop_navigation() sets _goal_reach = False which unblocks navigate_to().
        if self._navigation_thread and self._navigation_thread.is_alive():
            logger.warning("Previous navigation still running, cancelling")
            self.stop_navigation()
            self._navigation_thread.join(timeout=2.0)
            if self._navigation_thread.is_alive():
                logger.warning("Previous navigation thread did not exit in time, proceeding anyway")

        self._navigation_thread = threading.Thread(
            target=self._navigate_to_goal_async,
            args=(goal, timeout),
            daemon=True,
            name="ROSNavNavigationThread",
        )
        self._navigation_thread.start()

        return True

    def _navigate_to_goal_async(self, goal: PoseStamped, timeout: float = 60.0) -> None:
        """Internal method to handle navigation in a separate thread."""
        try:
            result = self.navigate_to(goal, timeout=timeout)
            with self._state_lock:
                self._goal_reached = result
                self._navigation_state = NavigationState.IDLE
        except Exception as e:
            logger.error(f"Navigation failed: {e}")
            with self._state_lock:
                self._goal_reached = False
                self._navigation_state = NavigationState.IDLE

    @rpc
    def get_state(self) -> NavigationState:
        """Get the current state of the navigator."""
        with self._state_lock:
            return self._navigation_state

    @rpc
    def is_goal_reached(self) -> bool:
        """Check if the current goal has been reached."""
        with self._state_lock:
            return self._goal_reached

    @rpc
    def cancel_goal(self) -> bool:
        """Cancel the current navigation goal."""

        with self._state_lock:
            had_goal = self._current_goal is not None

        if had_goal:
            self.stop_navigation()

        return had_goal

    @rpc
    def stop(self) -> None:
        """Stop the navigation module and clean up resources."""
        self.stop_navigation()
        try:
            self._running = False

            with self._teleop_lock:
                if self._teleop_timer is not None:
                    self._teleop_timer.cancel()
                    self._teleop_timer = None
                self._teleop_active = False

            if self._spin_thread and self._spin_thread.is_alive():
                self._spin_thread.join(timeout=1.0)

            if hasattr(self, "_node") and self._node:
                self._node.destroy_node()  # type: ignore[no-untyped-call]

        except Exception as e:
            logger.error(f"Error during shutdown: {e}")
        finally:
            super().stop()


ros_nav = ROSNav.blueprint


def _pose_stamped_to_ros(pose: PoseStamped) -> "ROSPoseStamped":
    """Convert a DimOS PoseStamped to a ROS2 geometry_msgs/PoseStamped."""
    msg = ROSPoseStamped()
    msg.header.frame_id = pose.frame_id
    ts_sec = int(pose.ts)
    msg.header.stamp.sec = ts_sec
    msg.header.stamp.nanosec = int((pose.ts - ts_sec) * 1_000_000_000)
    msg.pose.position.x = float(pose.position.x)
    msg.pose.position.y = float(pose.position.y)
    msg.pose.position.z = float(pose.position.z)
    msg.pose.orientation.x = float(pose.orientation.x)
    msg.pose.orientation.y = float(pose.orientation.y)
    msg.pose.orientation.z = float(pose.orientation.z)
    msg.pose.orientation.w = float(pose.orientation.w)
    return msg


def _image_from_ros_compressed(msg: "ROSCompressedImage") -> Image:
    """Convert a ROS2 sensor_msgs/CompressedImage to a DimOS Image."""
    ts = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
    frame_id = msg.header.frame_id
    arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if bgr is None:
        return Image(frame_id=frame_id, ts=ts)
    return Image(data=bgr, format=ImageFormat.BGR, frame_id=frame_id, ts=ts)


def _pc2_from_ros(msg: "ROSPointCloud2") -> PointCloud2:
    """Convert a ROS2 sensor_msgs/PointCloud2 to a DimOS PointCloud2."""
    ts = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
    frame_id = msg.header.frame_id

    if msg.width == 0 or msg.height == 0:
        return PointCloud2(frame_id=frame_id, ts=ts)

    # ROS PointField datatype → (numpy dtype suffix, byte size)
    _DTYPE_MAP = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}
    _SIZE_MAP = {1: 1, 2: 1, 3: 2, 4: 2, 5: 4, 6: 4, 7: 4, 8: 8}

    x_off = y_off = z_off = None
    x_dt = y_dt = z_dt = 7  # default: FLOAT32
    for f in msg.fields:
        if f.name == "x":
            x_off, x_dt = f.offset, f.datatype
        elif f.name == "y":
            y_off, y_dt = f.offset, f.datatype
        elif f.name == "z":
            z_off, z_dt = f.offset, f.datatype

    if any(o is None for o in [x_off, y_off, z_off]):
        raise ValueError("ROS PointCloud2 missing x/y/z fields")

    num_points = msg.width * msg.height
    raw = bytes(msg.data)
    step = msg.point_step
    end = ">" if msg.is_bigendian else "<"

    # Fast path: float32 x/y/z at offsets 0/4/8 (little-endian)
    if (
        x_off == 0
        and y_off == 4
        and z_off == 8
        and step >= 12
        and x_dt == 7
        and y_dt == 7
        and z_dt == 7
        and not msg.is_bigendian
    ):
        if step == 12:
            points = np.frombuffer(raw, dtype=np.float32).reshape(-1, 3)
        else:
            dt = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("_pad", f"V{step - 12}")])
            s = np.frombuffer(raw, dtype=dt, count=num_points)
            points = np.column_stack((s["x"], s["y"], s["z"]))
    # Fast path: float64 x/y/z at offsets 0/8/16 (little-endian)
    elif (
        x_off == 0
        and y_off == 8
        and z_off == 16
        and step >= 24
        and x_dt == 8
        and y_dt == 8
        and z_dt == 8
        and not msg.is_bigendian
    ):
        if step == 24:
            points = np.frombuffer(raw, dtype=np.float64).reshape(-1, 3).astype(np.float32)
        else:
            dt = np.dtype([("x", "<f8"), ("y", "<f8"), ("z", "<f8"), ("_pad", f"V{step - 24}")])
            s = np.frombuffer(raw, dtype=dt, count=num_points)
            points = np.column_stack((s["x"], s["y"], s["z"])).astype(np.float32)
    else:
        # General path: respect datatype per field
        x_np = np.dtype(end + _DTYPE_MAP.get(x_dt, "f4"))
        y_np = np.dtype(end + _DTYPE_MAP.get(y_dt, "f4"))
        z_np = np.dtype(end + _DTYPE_MAP.get(z_dt, "f4"))
        x_sz = _SIZE_MAP.get(x_dt, 4)
        y_sz = _SIZE_MAP.get(y_dt, 4)
        z_sz = _SIZE_MAP.get(z_dt, 4)
        points = np.zeros((num_points, 3), dtype=np.float32)
        for i in range(num_points):
            base = i * step
            points[i, 0] = float(
                np.frombuffer(raw[base + x_off : base + x_off + x_sz], dtype=x_np)[0]
            )
            points[i, 1] = float(
                np.frombuffer(raw[base + y_off : base + y_off + y_sz], dtype=y_np)[0]
            )
            points[i, 2] = float(
                np.frombuffer(raw[base + z_off : base + z_off + z_sz], dtype=z_np)[0]
            )

    return PointCloud2.from_numpy(points, frame_id=frame_id, timestamp=ts)


def _twist_from_ros(msg: "ROSTwistStamped") -> Twist:
    """Convert a ROS2 geometry_msgs/Twist (the .twist field of TwistStamped) to DimOS Twist."""
    return Twist(
        linear=Vector3(msg.linear.x, msg.linear.y, msg.linear.z),
        angular=Vector3(msg.angular.x, msg.angular.y, msg.angular.z),
    )


def _path_from_ros(msg: "ROSPath") -> NavPath:
    """Convert a ROS2 nav_msgs/Path to a DimOS Path."""
    ts = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
    frame_id = msg.header.frame_id
    poses = []
    for ps in msg.poses:
        pose_ts = ps.header.stamp.sec + ps.header.stamp.nanosec / 1e9
        p = ps.pose.position
        o = ps.pose.orientation
        poses.append(
            PoseStamped(
                ts=pose_ts,
                frame_id=ps.header.frame_id or frame_id,
                position=Vector3(p.x, p.y, p.z),
                orientation=Quaternion(o.x, o.y, o.z, o.w),
            )
        )
    return NavPath(ts=ts, frame_id=frame_id, poses=poses)


def _tfmessage_from_ros(msg: "ROSTFMessage") -> TFMessage:
    """Convert a ROS2 tf2_msgs/TFMessage to a DimOS TFMessage."""
    transforms = []
    for ts_msg in msg.transforms:
        ts = ts_msg.header.stamp.sec + ts_msg.header.stamp.nanosec / 1e9
        t = ts_msg.transform.translation
        r = ts_msg.transform.rotation
        transforms.append(
            Transform(
                translation=Vector3(t.x, t.y, t.z),
                rotation=Quaternion(r.x, r.y, r.z, r.w),
                frame_id=ts_msg.header.frame_id,
                child_frame_id=ts_msg.child_frame_id,
                ts=ts,
            )
        )
    return TFMessage(*transforms)


# -- DimOS → ROS2 conversion helpers (inverse of the from_ros functions above) --


def _pc2_to_ros(pc2: PointCloud2) -> "ROSPointCloud2":
    """Convert a DimOS PointCloud2 to a ROS2 sensor_msgs/PointCloud2.

    Includes a zero-filled ``intensity`` field because the CMU nav stack's
    terrain analysis nodes require it (they filter on ``intensity``).
    """
    from builtin_interfaces.msg import (
        Time as ROSTime,  # type: ignore[attr-defined,import-not-found]
    )
    from sensor_msgs.msg import PointField  # type: ignore[attr-defined]

    points, _ = pc2.as_numpy()  # (N, 3) float32
    n = points.shape[0]
    # XYZI layout: 4 floats per point (intensity = 0)
    xyzi = np.zeros((n, 4), dtype=np.float32)
    xyzi[:, :3] = points.astype(np.float32)

    ros_msg = ROSPointCloud2()  # type: ignore[no-untyped-call]
    ros_msg.header.stamp = ROSTime(sec=int(pc2.ts), nanosec=int((pc2.ts % 1) * 1e9))  # type: ignore[no-untyped-call]
    ros_msg.header.frame_id = pc2.frame_id or "sensor"
    ros_msg.height = 1
    ros_msg.width = n
    ros_msg.fields = [
        PointField(name="x", offset=0, datatype=7, count=1),  # type: ignore[no-untyped-call]
        PointField(name="y", offset=4, datatype=7, count=1),  # type: ignore[no-untyped-call]
        PointField(name="z", offset=8, datatype=7, count=1),  # type: ignore[no-untyped-call]
        PointField(name="intensity", offset=12, datatype=7, count=1),  # type: ignore[no-untyped-call]
    ]
    ros_msg.is_bigendian = False
    ros_msg.point_step = 16
    ros_msg.row_step = 16 * n
    ros_msg.data = xyzi.tobytes()
    ros_msg.is_dense = True
    return ros_msg


def _odometry_to_ros(odom: Odometry) -> "ROSOdometry":
    """Convert a DimOS Odometry to a ROS2 nav_msgs/Odometry."""
    from builtin_interfaces.msg import (
        Time as ROSTime,  # type: ignore[attr-defined,import-not-found]
    )
    from geometry_msgs.msg import (  # type: ignore[attr-defined]
        Point as ROSPoint,
        Pose as ROSPose,
        Quaternion as ROSQuat,
        Twist as ROSTwist,
        Vector3 as ROSVector3,
    )

    ros_msg = ROSOdometry()  # type: ignore[no-untyped-call]
    ros_msg.header.stamp = ROSTime(sec=int(odom.ts), nanosec=int((odom.ts % 1) * 1e9))  # type: ignore[no-untyped-call]
    ros_msg.header.frame_id = odom.frame_id or "map"
    ros_msg.child_frame_id = odom.child_frame_id or "sensor"
    ros_msg.pose.pose = ROSPose(  # type: ignore[no-untyped-call]
        position=ROSPoint(  # type: ignore[no-untyped-call]
            x=odom.pose.position.x, y=odom.pose.position.y, z=odom.pose.position.z
        ),
        orientation=ROSQuat(  # type: ignore[no-untyped-call]
            x=odom.pose.orientation.x,
            y=odom.pose.orientation.y,
            z=odom.pose.orientation.z,
            w=odom.pose.orientation.w,
        ),
    )
    ros_msg.twist.twist = ROSTwist(  # type: ignore[no-untyped-call]
        linear=ROSVector3(  # type: ignore[no-untyped-call]
            x=odom.twist.linear.x, y=odom.twist.linear.y, z=odom.twist.linear.z
        ),
        angular=ROSVector3(  # type: ignore[no-untyped-call]
            x=odom.twist.angular.x, y=odom.twist.angular.y, z=odom.twist.angular.z
        ),
    )
    return ros_msg


__all__ = ["ROSNav", "ros_nav"]

if __name__ == "__main__":
    ROSNav.blueprint().build()
