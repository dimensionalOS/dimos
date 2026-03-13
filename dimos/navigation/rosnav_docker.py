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

from dataclasses import dataclass, field
import logging
from pathlib import Path
import shutil
import struct
import threading
import time

import numpy as np

# ROS message imports: available inside the ROS2 container, but may be missing on the host
try:  # pragma: no cover - import-time environment dependent
    from geometry_msgs.msg import (  # type: ignore[attr-defined]
        PointStamped as ROSPointStamped,
        PoseStamped as ROSPoseStamped,
        TwistStamped as ROSTwistStamped,
    )
    from nav_msgs.msg import Path as ROSPath  # type: ignore[attr-defined]
    from sensor_msgs.msg import (  # type: ignore[attr-defined]
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
        def __init__(self, *args, **kwargs) -> None:
            pass

    ROSPointStamped = _Stub  # type: ignore[assignment]
    ROSPoseStamped = _Stub  # type: ignore[assignment]
    ROSTwistStamped = _Stub  # type: ignore[assignment]
    ROSPath = _Stub  # type: ignore[assignment]
    ROSJoy = _Stub  # type: ignore[assignment]
    ROSPointCloud2 = _Stub  # type: ignore[assignment]
    ROSBool = _Stub  # type: ignore[assignment]
    ROSInt8 = _Stub  # type: ignore[assignment]
    ROSTFMessage = _Stub  # type: ignore[assignment]

from dimos import spec
from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.docker_runner import DockerModuleConfig
from dimos.core.module import Module
from dimos.core.module_coordinator import ModuleCoordinator
from dimos.core.stream import In, Out
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs import (
    PoseStamped,
    Quaternion,
    Transform,
    Twist,
    Vector3,
)
from dimos.msgs.nav_msgs import Path as NavPath
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.msgs.std_msgs import Bool
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.navigation.base import NavigationInterface, NavigationState
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import euler_to_quaternion

logger = setup_logger(level=logging.INFO)


# ---------------------------------------------------------------------------
# ROS → DimOS message conversion shims
# These replace the removed from_ros_msg classmethods on the message types.
# ---------------------------------------------------------------------------


@dataclass
class ROSNavConfig(DockerModuleConfig):
    # --- Module settings ---
    local_pointcloud_freq: float = 2.0
    global_map_freq: float = 1.0
    sensor_to_base_link_transform: Transform = field(
        default_factory=lambda: Transform(frame_id="sensor", child_frame_id="base_link")
    )

    # --- Docker settings ---
    docker_image: str = "dimos_autonomy_stack:humble"
    docker_shm_size: str = "8g"
    docker_entrypoint: str = "/usr/local/bin/dimos_module_entrypoint.sh"
    docker_file: Path = Path(__file__).parent.parent.parent / "docker" / "navigation" / "Dockerfile"
    docker_build_context: Path = Path(__file__).parent.parent.parent
    # Use --runtime=nvidia (Jetson) instead of --gpus all (desktop); having both conflicts.
    docker_gpus: str | None = None
    docker_extra_args: list = field(
        default_factory=lambda: ["--cap-add=NET_ADMIN"]
        + (["--runtime=nvidia"] if shutil.which("nvcc") else [])
    )
    docker_env: dict = field(
        default_factory=lambda: {
            "ROS_DISTRO": "humble",
            "ROS_DOMAIN_ID": "42",
            "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
            "FASTRTPS_DEFAULT_PROFILES_FILE": "/ros2_ws/config/fastdds.xml",
            "QT_X11_NO_MITSHM": "1",
            "NVIDIA_VISIBLE_DEVICES": "all",
            "NVIDIA_DRIVER_CAPABILITIES": "all",
            # "ROBOT_CONFIG_PATH": "unitree/unitree_g1",
        }
    )
    docker_volumes: list = field(default_factory=lambda: [])

    # --- Runtime mode settings ---
    # mode controls which ROS launch file the entrypoint selects:
    #   "simulation"  — system_simulation[_with_route_planner].launch.py + Unity if present
    #   "unity_sim"   — same as simulation but hard-exits if Unity binary is missing
    #   "hardware"    — system_real_robot[_with_route_planner].launch.py
    #   "bagfile"     — system_bagfile[_with_route_planner].launch.py + use_sim_time
    # Setting bagfile_path automatically forces mode to "bagfile".
    mode: str = "hardware"
    bagfile_path: str | Path = ""  # host-side path to bag; remapped into container at runtime

    # use_rviz: whether to launch RViz2 inside the container.
    #   None (default) → True for simulation/unity_sim modes, False otherwise
    #   (mirrors the unconditional RViz launch in run_both.sh for simulation)
    use_rviz: bool = False

    def __post_init__(self) -> None:
        import os

        effective_mode = "bagfile" if self.bagfile_path else self.mode
        self.docker_env["MODE"] = effective_mode
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
                self.docker_env["BAGFILE_PATH"] = self.bagfile_path

        self.docker_env["USE_RVIZ"] = "true" if self.use_rviz else "false"

        # Pass host DISPLAY through for X11 forwarding (RViz, Unity)
        if display := os.environ.get("DISPLAY", ":0"):
            self.docker_env["DISPLAY"] = display

        self.docker_env["QT_X11_NO_MITSHM"] = "1"

        repo_root = Path(__file__).parent.parent.parent
        self.docker_volumes += [
            # X11 socket for display forwarding (RViz, Unity)
            ("/tmp/.X11-unix", "/tmp/.X11-unix", "rw"),
            # Mount live dimos source so the module is always up-to-date
            (str(repo_root), "/workspace/dimos", "rw"),
            # Mount DDS config (fastdds.xml) from host
            (str(repo_root / "docker" / "navigation" / "config"), "/ros2_ws/config", "rw"),
            # Note: most of the mounts below are only needed for development
            # Mount entrypoint script so changes don't require a rebuild
            (
                str(Path(__file__).parent / "dimos_module_entrypoint.sh"),
                "/usr/local/bin/dimos_module_entrypoint.sh",
                "ro",
            ),
            # Mount Unity environment (map.pwly, traversable_area.ply, etc.) into the ROS workspace
            (
                str(
                    repo_root
                    / "docker"
                    / "navigation"
                    / "ros-navigation-autonomy-stack"
                    / "src"
                    / "base_autonomy"
                    / "vehicle_simulator"
                    / "mesh"
                    / "unity"
                ),
                "/ros2_ws/src/ros-navigation-autonomy-stack/src/base_autonomy/vehicle_simulator/mesh/unity/",
                "rw",
            ),
            # the original codebase doesn't have this and the bagfile case complains about it being missing
            # so we just put the unity one in there as "real_world"
            (
                str(
                    repo_root
                    / "docker"
                    / "navigation"
                    / "ros-navigation-autonomy-stack"
                    / "src"
                    / "base_autonomy"
                    / "vehicle_simulator"
                    / "mesh"
                    / "unity"
                ),
                "/ros2_ws/src/ros-navigation-autonomy-stack/src/base_autonomy/vehicle_simulator/mesh/real_world/",
                "rw",
            ),
            # Patch ros_tcp_endpoint server.py: fixes JSON null-terminator stripping bug
            # that crashes every Unity TCP connection. The installed copy is pre-built into
            # the image; mounting the fixed source over it avoids a full rebuild.
            (
                str(
                    repo_root
                    / "docker"
                    / "navigation"
                    / "ros-navigation-autonomy-stack"
                    / "src"
                    / "utilities"
                    / "ROS-TCP-Endpoint"
                    / "ros_tcp_endpoint"
                    / "server.py"
                ),
                "/ros2_ws/install/ros_tcp_endpoint/lib/python3.10/site-packages/ros_tcp_endpoint/server.py",
                "ro",
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


class ROSNav(
    Module, NavigationInterface, spec.Nav, spec.GlobalPointcloud, spec.Pointcloud, spec.LocalPlanner
):
    config: ROSNavConfig
    default_config = ROSNavConfig

    goal_req: In[PoseStamped]

    pointcloud: Out[PointCloud2]
    global_pointcloud: Out[PointCloud2]
    goal_active: Out[PoseStamped]
    path_active: Out[NavPath]
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

    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(*args, **kwargs)
        import rclpy
        from rclpy.node import Node

        # Initialize state tracking
        self._state_lock = threading.Lock()
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
        self.registered_scan_sub = self._node.create_subscription(
            ROSPointCloud2, "/registered_scan", self._on_ros_registered_scan, 10
        )

        self.global_pointcloud_sub = self._node.create_subscription(
            ROSPointCloud2, "/terrain_map_ext", self._on_ros_global_map, 10
        )

        self.path_sub = self._node.create_subscription(ROSPath, "/path", self._on_ros_path, 10)
        self.tf_sub = self._node.create_subscription(ROSTFMessage, "/tf", self._on_ros_tf, 10)

        logger.info("NavigationModule initialized with ROS2 node")

    @rpc
    def start(self) -> None:
        self._running = True

        # Create and start the spin thread for ROS2 node spinning
        self._spin_thread = threading.Thread(
            target=self._spin_node, daemon=True, name="ROS2SpinThread"
        )
        self._spin_thread.start()

        # if self.goal_req:
        #     self.goal_req.subscribe(self._on_goal_pose)
        logger.info("NavigationModule started with ROS2 spinning")

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
        self.cmd_vel.publish(_twist_from_ros(msg.twist))

    def _on_ros_registered_scan(self, msg: ROSPointCloud2) -> None:
        self.pointcloud.publish(_pc2_from_ros(msg))

    def _on_ros_global_map(self, msg: ROSPointCloud2) -> None:
        self.global_pointcloud.publish(_pc2_from_ros(msg))

    def _on_ros_path(self, msg: ROSPath) -> None:
        dimos_path = _path_from_ros(msg)
        dimos_path.frame_id = "base_link"
        self.path_active.publish(dimos_path)

    def _on_ros_tf(self, msg: ROSTFMessage) -> None:
        ros_tf = _tfmessage_from_ros(msg)

        map_to_world_tf = Transform(
            translation=Vector3(0.0, 0.0, 0.0),
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
        self.navigate_to(msg)

    def _on_cancel_goal(self, msg: Bool) -> None:
        if msg.data:
            self.stop()

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
            orientation=Quaternion(0.0, 0.0, 0.0, 0.0),
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
            orientation=Quaternion(0.0, 0.0, 0.0, 0.0),
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

        ros_pose = pose.to_ros_msg()
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

        with self._state_lock:
            self._navigation_state = NavigationState.IDLE
            self._current_goal = None
            self._goal_reached = False

        return True

    @rpc
    def set_goal(self, goal: PoseStamped) -> bool:
        """Set a new navigation goal (non-blocking)."""
        with self._state_lock:
            self._current_goal = goal
            self._goal_reached = False
            self._navigation_state = NavigationState.FOLLOWING_PATH

        # Start navigation in a separate thread to make it non-blocking
        if self._navigation_thread and self._navigation_thread.is_alive():
            logger.warning("Previous navigation still running, cancelling")
            self.stop_navigation()
            self._navigation_thread.join(timeout=1.0)

        self._navigation_thread = threading.Thread(
            target=self._navigate_to_goal_async,
            args=(goal,),
            daemon=True,
            name="ROSNavNavigationThread",
        )
        self._navigation_thread.start()

        return True

    def _navigate_to_goal_async(self, goal: PoseStamped) -> None:
        """Internal method to handle navigation in a separate thread."""
        try:
            result = self.navigate_to(goal, timeout=60.0)
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

            if self._spin_thread and self._spin_thread.is_alive():
                self._spin_thread.join(timeout=1.0)

            if hasattr(self, "_node") and self._node:
                self._node.destroy_node()  # type: ignore[no-untyped-call]

        except Exception as e:
            logger.error(f"Error during shutdown: {e}")
        finally:
            super().stop()


ros_nav = ROSNav.blueprint


def deploy(dimos: ModuleCoordinator):  # type: ignore[no-untyped-def]
    nav = dimos.deploy(ROSNav)  # type: ignore[attr-defined]

    # Existing ports on LCM transports
    nav.pointcloud.transport = LCMTransport("/lidar", PointCloud2)
    nav.global_pointcloud.transport = LCMTransport("/map", PointCloud2)
    nav.goal_req.transport = LCMTransport("/goal_req", PoseStamped)
    nav.goal_active.transport = LCMTransport("/goal_active", PoseStamped)
    nav.path_active.transport = LCMTransport("/path_active", NavPath)
    nav.cmd_vel.transport = LCMTransport("/cmd_vel", Twist)

    nav.start()
    return nav


def _pc2_from_ros(msg: "ROSPointCloud2") -> PointCloud2:
    """Convert a ROS2 sensor_msgs/PointCloud2 to a DimOS PointCloud2."""
    ts = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
    frame_id = msg.header.frame_id

    if msg.width == 0 or msg.height == 0:
        return PointCloud2(frame_id=frame_id, ts=ts)

    x_off = y_off = z_off = None
    for f in msg.fields:
        if f.name == "x":
            x_off = f.offset
        elif f.name == "y":
            y_off = f.offset
        elif f.name == "z":
            z_off = f.offset

    if any(o is None for o in [x_off, y_off, z_off]):
        raise ValueError("ROS PointCloud2 missing x/y/z fields")

    num_points = msg.width * msg.height
    raw = bytes(msg.data)
    step = msg.point_step

    # Fast path: standard x/y/z layout at offsets 0/4/8
    if x_off == 0 and y_off == 4 and z_off == 8 and step >= 12:
        if step == 12:
            points = np.frombuffer(raw, dtype=np.float32).reshape(-1, 3)
        else:
            dt = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("_pad", f"V{step - 12}")])
            s = np.frombuffer(raw, dtype=dt, count=num_points)
            points = np.column_stack((s["x"], s["y"], s["z"]))
    else:
        points = np.zeros((num_points, 3), dtype=np.float32)
        for i in range(num_points):
            base = i * step
            points[i, 0] = struct.unpack("<f", raw[base + x_off : base + x_off + 4])[0]
            points[i, 1] = struct.unpack("<f", raw[base + y_off : base + y_off + 4])[0]
            points[i, 2] = struct.unpack("<f", raw[base + z_off : base + z_off + 4])[0]

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


__all__ = ["ROSNav", "deploy", "ros_nav"]
