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
import threading
import time

from reactivex import operators as ops
from reactivex.subject import Subject

from dimos import spec
from dimos.agents.annotation import skill
from dimos.core import DimosCluster, In, LCMTransport, Module, Out, rpc
from dimos.core._dask_exports import DimosCluster
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.core.transport import LCMTransport, ROSTransport
from dimos.msgs.geometry_msgs import (
    PoseStamped,
    Quaternion,
    Transform,
    Twist,
    TwistStamped,
    Vector3,
)
from dimos.msgs.nav_msgs import Path
from dimos.msgs.sensor_msgs import Joy, PointCloud2
from dimos.msgs.std_msgs import Bool, Int8
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.navigation.base import NavigationInterface, NavigationState
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import euler_to_quaternion

logger = setup_logger(level=logging.INFO)


@dataclass
class Config(ModuleConfig):
    local_pointcloud_freq: float = 2.0
    global_map_freq: float = 1.0
    sensor_to_base_link_transform: Transform = field(
        default_factory=lambda: Transform(frame_id="sensor", child_frame_id="base_link")
    )


class ROSNav(
    Module, NavigationInterface, spec.Nav, spec.GlobalPointcloud, spec.Pointcloud, spec.LocalPlanner
):
    config: Config
    default_config = Config

    # Existing ports (default LCM/pSHM transport)
    goal_req: In[PoseStamped]

    pointcloud: Out[PointCloud2]
    global_map: Out[PointCloud2]

    goal_active: Out[PoseStamped]
    path_active: Out[Path]
    cmd_vel: Out[Twist]

    # ROS In ports (receiving from ROS topics via ROSTransport)
    ros_goal_reached: In[Bool]
    ros_cmd_vel: In[TwistStamped]
    ros_way_point: In[PoseStamped]
    ros_registered_scan: In[PointCloud2]
    ros_global_map: In[PointCloud2]
    ros_path: In[Path]
    ros_tf: In[TFMessage]

    # ROS Out ports (publishing to ROS topics via ROSTransport)
    ros_goal_pose: Out[PoseStamped]
    ros_cancel_goal: Out[Bool]
    ros_soft_stop: Out[Int8]
    ros_joy: Out[Joy]

    # Using RxPY Subjects for reactive data flow instead of storing state
    _local_pointcloud_subject: Subject  # type: ignore[type-arg]
    _global_map_subject: Subject  # type: ignore[type-arg]

    _current_position_running: bool = False
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
        self._global_map_subject = Subject()

        # Initialize state tracking
        self._state_lock = threading.Lock()
        self._navigation_state = NavigationState.IDLE
        self._goal_reached = False

        logger.info("NavigationModule initialized")

    @rpc
    def start(self) -> None:
        self._running = True

        self._disposables.add(
            self._local_pointcloud_subject.pipe(
                ops.sample(1.0 / self.config.local_pointcloud_freq),
            ).subscribe(
                on_next=self.pointcloud.publish,
                on_error=lambda e: logger.error(f"Lidar stream error: {e}"),
            )
        )

        self._disposables.add(
            self._global_map_subject.pipe(
                ops.sample(1.0 / self.config.global_map_freq),
            ).subscribe(
                on_next=self.global_map.publish,
                on_error=lambda e: logger.error(f"Map stream error: {e}"),
            )
        )

        # Subscribe to ROS In ports
        self.ros_goal_reached.subscribe(self._on_ros_goal_reached)
        self.ros_cmd_vel.subscribe(self._on_ros_cmd_vel)
        self.ros_way_point.subscribe(self._on_ros_goal_waypoint)
        self.ros_registered_scan.subscribe(self._on_ros_registered_scan)
        self.ros_global_map.subscribe(self._on_ros_global_map)
        self.ros_path.subscribe(self._on_ros_path)
        self.ros_tf.subscribe(self._on_ros_tf)

        self.goal_req.subscribe(self._on_goal_pose)
        logger.info("NavigationModule started with ROS transport and RxPY streams")

    def _on_ros_goal_reached(self, msg: Bool) -> None:
        self._goal_reach = msg.data
        if msg.data:
            with self._state_lock:
                self._goal_reached = True
                self._navigation_state = NavigationState.IDLE

    def _on_ros_goal_waypoint(self, msg: PoseStamped) -> None:
        self.goal_active.publish(msg)

    def _on_ros_cmd_vel(self, msg: TwistStamped) -> None:
        self.cmd_vel.publish(Twist(linear=msg.linear, angular=msg.angular))

    def _on_ros_registered_scan(self, msg: PointCloud2) -> None:
        self._local_pointcloud_subject.on_next(msg)

    def _on_ros_global_map(self, msg: PointCloud2) -> None:
        self._global_map_subject.on_next(msg)

    def _on_ros_path(self, msg: Path) -> None:
        msg.frame_id = "base_link"
        self.path_active.publish(msg)

    def _on_ros_tf(self, msg: TFMessage) -> None:
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
            *msg.transforms,
        )

    def _on_goal_pose(self, msg: PoseStamped) -> None:
        self.navigate_to(msg)

    def _on_cancel_goal(self, msg: Bool) -> None:
        if msg.data:
            self.stop()

    def _set_autonomy_mode(self) -> None:
        joy_msg = Joy(
            axes=[0.0, 0.0, -1.0, 0.0, 1.0, 1.0, 0.0, 0.0],
            buttons=[0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
        )
        self.ros_joy.publish(joy_msg)
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
        self.ros_soft_stop.publish(Int8(data=0))
        self.ros_goal_pose.publish(pose)

        # Wait for goal to be reached
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self._goal_reach is not None:
                self.ros_soft_stop.publish(Int8(data=2))
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

        self.ros_cancel_goal.publish(Bool(data=True))
        self.ros_soft_stop.publish(Int8(data=2))

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
            self._global_map_subject.on_completed()

        except Exception as e:
            logger.error(f"Error during shutdown: {e}")
        finally:
            super().stop()


ros_nav = ROSNav.blueprint


def deploy(dimos: DimosCluster):  # type: ignore[no-untyped-def]
    nav = dimos.deploy(ROSNav)  # type: ignore[attr-defined]

    # Existing ports on LCM transports
    nav.pointcloud.transport = LCMTransport("/lidar", PointCloud2)
    nav.global_map.transport = LCMTransport("/map", PointCloud2)
    nav.goal_req.transport = LCMTransport("/goal_req", PoseStamped)
    nav.goal_active.transport = LCMTransport("/goal_active", PoseStamped)
    nav.path_active.transport = LCMTransport("/path_active", Path)
    nav.cmd_vel.transport = LCMTransport("/cmd_vel", Twist)

    # ROS In transports (receiving from ROS navigation stack)
    nav.ros_goal_reached.transport = ROSTransport("/goal_reached", Bool)
    nav.ros_cmd_vel.transport = ROSTransport("/cmd_vel", TwistStamped)
    nav.ros_way_point.transport = ROSTransport("/way_point", PoseStamped)
    nav.ros_registered_scan.transport = ROSTransport("/registered_scan", PointCloud2)
    nav.ros_global_map.transport = ROSTransport("/terrain_map_ext", PointCloud2)
    nav.ros_path.transport = ROSTransport("/path", Path)
    nav.ros_tf.transport = ROSTransport("/tf", TFMessage)

    # ROS Out transports (publishing to ROS navigation stack)
    nav.ros_goal_pose.transport = ROSTransport("/goal_pose", PoseStamped)
    nav.ros_cancel_goal.transport = ROSTransport("/cancel_goal", Bool)
    nav.ros_soft_stop.transport = ROSTransport("/stop", Int8)
    nav.ros_joy.transport = ROSTransport("/joy", Joy)

    nav.start()
    return nav


__all__ = ["ROSNav", "deploy", "ros_nav"]
