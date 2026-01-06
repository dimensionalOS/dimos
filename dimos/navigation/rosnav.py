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

"""
NavBot class for navigation-related functionality.
Encapsulates ROS bridge and topic remapping for Unitree robots.
"""

from collections.abc import Generator
from dataclasses import dataclass, field
import logging
import threading
import time

from geometry_msgs.msg import (  # type: ignore[attr-defined, import-untyped]
    PointStamped as ROSPointStamped,
    PoseStamped as ROSPoseStamped,
    TwistStamped as ROSTwistStamped,
)
from nav_msgs.msg import Path as ROSPath  # type: ignore[attr-defined, import-untyped]
import rclpy  # type: ignore[import-untyped]
from rclpy.node import Node  # type: ignore[import-untyped]
from reactivex import operators as ops
from reactivex.subject import Subject
from sensor_msgs.msg import (  # type: ignore[attr-defined, import-untyped]
    Joy as ROSJoy,
    PointCloud2 as ROSPointCloud2,
)
from std_msgs.msg import (  # type: ignore[attr-defined, import-untyped]
    Bool as ROSBool,
    Int8 as ROSInt8,
)
from tf2_msgs.msg import TFMessage as ROSTFMessage  # type: ignore[attr-defined, import-untyped]

from dimos import spec
from dimos.agents import Reducer, Stream, skill  # type: ignore[attr-defined]
from dimos.core import DimosCluster, In, LCMTransport, Module, Out, pSHMTransport, rpc
from dimos.core.module import ModuleConfig
from dimos.msgs.geometry_msgs import (
    PoseStamped,
    Quaternion,
    Transform,
    Twist,
    Vector3,
)
from dimos.msgs.nav_msgs import Path
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.msgs.std_msgs import Bool
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.navigation.base import NavigationInterface, NavigationState
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import euler_to_quaternion

logger = setup_logger(level=logging.INFO)


@dataclass
class Config(ModuleConfig):
    local_pointcloud_freq: float = 2.0
    global_pointcloud_freq: float = 1.0
    sensor_to_base_link_transform: Transform = field(
        default_factory=lambda: Transform(frame_id="sensor", child_frame_id="base_link")
    )


class ROSNav(
    Module, NavigationInterface, spec.Nav, spec.Global3DMap, spec.Pointcloud, spec.LocalPlanner
):
    config: Config
    default_config = Config

    goal_req: In[PoseStamped]

    pointcloud: Out[PointCloud2]
    global_pointcloud: Out[PointCloud2]

    goal_active: Out[PoseStamped]
    path_active: Out[Path]
    cmd_vel: Out[Twist]

    # Using RxPY Subjects for reactive data flow instead of storing state
    _local_pointcloud_subject: Subject  # type: ignore[type-arg]
    _global_pointcloud_subject: Subject  # type: ignore[type-arg]

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

        # Initialize RxPY Subjects for streaming data
        self._local_pointcloud_subject = Subject()
        self._global_pointcloud_subject = Subject()

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
        self.cmd_vel_sub = self._node.create_subscription(
            ROSTwistStamped, "/cmd_vel", self._on_ros_cmd_vel, 10
        )
        self.goal_waypoint_sub = self._node.create_subscription(
            ROSPointStamped, "/way_point", self._on_ros_goal_waypoint, 10
        )
        self.registered_scan_sub = self._node.create_subscription(
            ROSPointCloud2, "/registered_scan", self._on_ros_registered_scan, 10
        )

        self.global_pointcloud_sub = self._node.create_subscription(
            ROSPointCloud2, "/terrain_map_ext", self._on_ros_global_pointcloud, 10
        )

        self.path_sub = self._node.create_subscription(ROSPath, "/path", self._on_ros_path, 10)
        self.tf_sub = self._node.create_subscription(ROSTFMessage, "/tf", self._on_ros_tf, 10)

        logger.info("NavigationModule initialized with ROS2 node")

    @rpc
    def start(self) -> None:
        self._running = True

        self._disposables.add(
            self._local_pointcloud_subject.pipe(
                ops.sample(1.0 / self.config.local_pointcloud_freq),  # Sample at desired frequency
                ops.map(lambda msg: PointCloud2.from_ros_msg(msg)),  # type: ignore[arg-type]
            ).subscribe(
                on_next=self.pointcloud.publish,
                on_error=lambda e: logger.error(f"Lidar stream error: {e}"),
            )
        )

        self._disposables.add(
            self._global_pointcloud_subject.pipe(
                ops.sample(1.0 / self.config.global_pointcloud_freq),  # Sample at desired frequency
                ops.map(lambda msg: PointCloud2.from_ros_msg(msg)),  # type: ignore[arg-type]
            ).subscribe(
                on_next=self.global_pointcloud.publish,
                on_error=lambda e: logger.error(f"Map stream error: {e}"),
            )
        )

        # Create and start the spin thread for ROS2 node spinning
        self._spin_thread = threading.Thread(
            target=self._spin_node, daemon=True, name="ROS2SpinThread"
        )
        self._spin_thread.start()

        self.goal_req.subscribe(self._on_goal_pose)
        logger.info("NavigationModule started with ROS2 spinning and RxPY streams")

    def _spin_node(self) -> None:
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
        self.cmd_vel.publish(Twist.from_ros_msg(msg.twist))

    def _on_ros_registered_scan(self, msg: ROSPointCloud2) -> None:
        self._local_pointcloud_subject.on_next(msg)

    def _on_ros_global_pointcloud(self, msg: ROSPointCloud2) -> None:
        self._global_pointcloud_subject.on_next(msg)

    def _on_ros_path(self, msg: ROSPath) -> None:
        dimos_path = Path.from_ros_msg(msg)
        dimos_path.frame_id = "base_link"
        self.path_active.publish(dimos_path)

    def _on_ros_tf(self, msg: ROSTFMessage) -> None:
        ros_tf = TFMessage.from_ros_msg(msg)

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

    @skill(stream=Stream.passive, reducer=Reducer.latest)  # type: ignore[arg-type]
    def current_position(self):  # type: ignore[no-untyped-def]
        """passively stream the current position of the robot every second"""
        if self._current_position_running:
            return "already running"
        while True:
            self._current_position_running = True
            time.sleep(1.0)
            tf = self.tf.get("map", "base_link")
            if not tf:
                continue
            yield f"current position {tf.translation.x}, {tf.translation.y}"

    @skill(stream=Stream.call_agent, reducer=Reducer.string)  # type: ignore[arg-type]
    def goto(self, x: float, y: float):  # type: ignore[no-untyped-def]
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

        yield "moving, please wait..."
        self.navigate_to(pose_to)
        yield "arrived"

    @skill(stream=Stream.call_agent, reducer=Reducer.string)  # type: ignore[arg-type]
    def goto_global(self, x: float, y: float) -> Generator[str, None, None]:
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

        pos = self.tf.get("base_link", "map").translation

        yield f"moving from {pos.x:.2f}, {pos.y:.2f} to {x:.2f}, {y:.2f}, please wait..."

        self.navigate_to(target)

        yield "arrived to {x:.2f}, {y:.2f}"

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

            self._local_pointcloud_subject.on_completed()
            self._global_pointcloud_subject.on_completed()

            if self._spin_thread and self._spin_thread.is_alive():
                self._spin_thread.join(timeout=1.0)

            if hasattr(self, "_node") and self._node:
                self._node.destroy_node()  # type: ignore[no-untyped-call]

        except Exception as e:
            logger.error(f"Error during shutdown: {e}")
        finally:
            super().stop()


ros_nav = ROSNav.blueprint


def deploy(dimos: DimosCluster):  # type: ignore[no-untyped-def]
    nav = dimos.deploy(ROSNav)  # type: ignore[attr-defined]

    nav.pointcloud.transport = pSHMTransport("/lidar")
    nav.global_pointcloud.transport = pSHMTransport("/map")
    nav.goal_req.transport = LCMTransport("/goal_req", PoseStamped)
    nav.goal_active.transport = LCMTransport("/goal_active", PoseStamped)
    nav.path_active.transport = LCMTransport("/path_active", Path)
    nav.cmd_vel.transport = LCMTransport("/cmd_vel", Twist)

    nav.start()
    return nav


__all__ = ["ROSNav", "deploy", "ros_nav"]
