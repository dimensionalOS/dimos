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

"""M20 Connection Module for dimos.

Provides the primary interface between dimos and the Deep Robotics M20
quadruped. Two operating paths depending on rclpy availability:

- **Navigation Mode** (rclpy available, e.g. on GOS): Velocity via /NAV_CMD
  DDS topic, odometry from /ODOM, LiDAR from /LIDAR/POINTS. Obstacle
  avoidance enabled.
- **Regular Mode** (no rclpy, fallback): Velocity via UDP, dead-reckoning
  odometry, CycloneDDS LiDAR. Teleop only — no obstacle avoidance.

Camera always uses RTSP. UDP protocol always used for heartbeat, motion
state, gait switch, and usage mode commands.

Reference: M20 Software Development Guide, di-cetts architecture fixes
"""

import logging
import time
from threading import Thread
from typing import Any

from reactivex.disposable import Disposable

from dimos import spec
from dimos.agents.annotation import skill
from dimos.core import In, Module, Out, rpc
from dimos.core.global_config import GlobalConfig, global_config
from dimos.msgs.geometry_msgs import (
    PoseStamped,
    Quaternion,
    Transform,
    Twist,
    Vector3,
)
from dimos.msgs.sensor_msgs import CameraInfo, Image, PointCloud2

from ..protocol import (
    GaitType,
    M20Protocol,
    MotionState,
    UsageMode,
)
from .camera import M20RTSPCamera
from .odometry import M20DeadReckonOdometry
from .velocity_controller import M20SpeedLimits, M20VelocityController

try:
    from .ros_sensors import M20ROSSensors

    _ROS_AVAILABLE = True
except (ImportError, RuntimeError):
    _ROS_AVAILABLE = False

try:
    from .lidar import M20LidarDDS

    _LIDAR_AVAILABLE = True
except (ImportError, RuntimeError):
    _LIDAR_AVAILABLE = False

logger = logging.getLogger(__name__)


class M20Connection(Module, spec.Camera, spec.Pointcloud):
    """Deep Robotics M20 quadruped connection.

    When rclpy is available (GOS), operates in Navigation Mode with
    /NAV_CMD velocity, /ODOM odometry, and /LIDAR/POINTS via rclpy.
    Falls back to Regular Mode with UDP velocity, dead-reckoning, and
    CycloneDDS LiDAR when rclpy is not available.

    Streams:
        cmd_vel (In):      Twist velocity commands from the navigation stack
        color_image (Out): RGB camera frames (via RTSP)
        camera_info (Out): Camera intrinsics
        pointcloud (Out):  Processed point cloud (alias for lidar)
        lidar (Out):       Raw LiDAR point cloud
        odom (Out):        Robot odometry pose
    """

    # Input streams
    cmd_vel: In[Twist]

    # Output streams (spec.Camera)
    color_image: Out[Image]
    camera_info: Out[CameraInfo]

    # Output streams (spec.Pointcloud)
    pointcloud: Out[PointCloud2]

    # Additional output streams
    lidar: Out[PointCloud2]
    odom: Out[PoseStamped]

    # Internal state
    _protocol: M20Protocol
    _velocity_ctrl: M20VelocityController
    _global_config: GlobalConfig
    _camera_info: CameraInfo  # required by spec.Camera
    _camera_info_thread: Thread | None = None
    _camera_info_running: bool = False
    _latest_video_frame: Image | None = None

    def __init__(
        self,
        ip: str | None = None,
        port: int = 30000,
        speed_limits: M20SpeedLimits | None = None,
        enable_camera: bool = True,
        enable_lidar: bool = True,
        enable_ros: bool = True,
        camera_stream: str = "video1",
        cfg: GlobalConfig = global_config,
        *args: Any,
        **kwargs: Any,
    ) -> None:
        self._global_config = cfg

        ip = ip if ip is not None else self._global_config.robot_ip

        self._protocol = M20Protocol(host=ip, port=port)

        # Try to initialize ROS sensors for Navigation Mode
        self._ros_sensors: "M20ROSSensors | None" = None
        if enable_ros and _ROS_AVAILABLE:
            try:
                self._ros_sensors = M20ROSSensors()
            except RuntimeError as e:
                logger.warning(f"M20ROSSensors init failed — falling back to UDP: {e}")

        # Velocity controller: use /NAV_CMD if ros_sensors available, else UDP
        nav_cmd_fn = None
        if self._ros_sensors is not None and self._ros_sensors.nav_cmd_available:
            nav_cmd_fn = self._ros_sensors.publish_nav_cmd

        self._velocity_ctrl = M20VelocityController(
            protocol=self._protocol,
            speed_limits=speed_limits,
            nav_cmd_publish=nav_cmd_fn,
        )

        self._camera: M20RTSPCamera | None = None
        if enable_camera:
            self._camera = M20RTSPCamera(
                host=ip, stream_path=camera_stream
            )
            self._camera_info = self._camera.camera_info

        # CycloneDDS LiDAR fallback (used only when rclpy not available)
        self._lidar: "M20LidarDDS | None" = None
        if enable_lidar and self._ros_sensors is None:
            if _LIDAR_AVAILABLE:
                from .lidar import M20LidarDDS

                self._lidar = M20LidarDDS()
            else:
                logger.warning(
                    "LiDAR requested but neither rclpy nor CycloneDDS available"
                )

        self._odometry: M20DeadReckonOdometry | None = None

        if self._ros_sensors is not None:
            logger.info("M20Connection: rclpy available — Navigation Mode enabled")
        else:
            logger.warning(
                "M20Connection: rclpy not available — Regular Mode only, "
                "no obstacle avoidance"
            )

        # Module.__init__ must be called LAST
        Module.__init__(self, *args, **kwargs)

    @rpc
    def start(self) -> None:
        super().start()

        try:
            # Connect to M20 and start protocol services (always UDP)
            self._protocol.connect()
            self._protocol.start_heartbeat()
            self._protocol.start_listener(self._on_status_report)

            # Start velocity control loop
            self._velocity_ctrl.start()

            # Wire cmd_vel input to velocity controller
            self._disposables.add(Disposable(self.cmd_vel.subscribe(self._on_cmd_vel)))

            # Start RTSP camera
            if self._camera:
                self._camera.start()

                def _on_image(image: Image) -> None:
                    self.color_image.publish(image)
                    self._latest_video_frame = image

                self._disposables.add(
                    self._camera.image_stream().subscribe(_on_image)
                )

                # Publish camera_info at 1Hz
                self._camera_info_running = True
                self._camera_info_thread = Thread(
                    target=self._publish_camera_info, daemon=True
                )
                self._camera_info_thread.start()

            if self._ros_sensors is not None:
                self._start_ros_path()
            else:
                self._start_udp_fallback_path()

            # Stand up and select usage mode
            self._protocol.send_motion_state(MotionState.STAND)
            time.sleep(1.0)

            if self._ros_sensors is not None:
                self._protocol.send_usage_mode(UsageMode.NAVIGATION)
                # Agile Motion Mode for navigation (dev guide 2.2.2:
                # "suitable for navigation and autonomous algorithm development")
                self._protocol.send_gait_switch(GaitType.AGILE_FLAT)
            else:
                self._protocol.send_usage_mode(UsageMode.REGULAR)
                self._protocol.send_gait_switch(GaitType.STANDARD)

            mode = "Navigation" if self._ros_sensors else "Regular"
            logger.info(f"M20Connection started in {mode} Mode")
        except Exception:
            logger.exception("M20Connection.start() failed — cleaning up partial init")
            self.stop()
            raise

    def _start_ros_path(self) -> None:
        """Start rclpy-based sensor path (Navigation Mode)."""
        assert self._ros_sensors is not None
        self._ros_sensors.start()

        def _on_lidar(pc: PointCloud2) -> None:
            self.lidar.publish(pc)
            self.pointcloud.publish(pc)

        # Wire /ODOM → odom stream + TF
        self._disposables.add(
            self._ros_sensors.odom_stream().subscribe(self._publish_tf)
        )

        # Wire /ALIGNED_POINTS → lidar + pointcloud streams
        self._disposables.add(
            self._ros_sensors.lidar_stream().subscribe(_on_lidar)
        )

        logger.info("ROS sensor path active: /ODOM, /tf, /ALIGNED_POINTS, /NAV_CMD")

    def _start_udp_fallback_path(self) -> None:
        """Start UDP-only fallback path (Regular Mode)."""
        # CycloneDDS LiDAR
        if self._lidar:
            self._lidar.start()

            def _on_lidar(pc: PointCloud2) -> None:
                self.lidar.publish(pc)
                self.pointcloud.publish(pc)

            self._disposables.add(
                self._lidar.pointcloud_stream().subscribe(_on_lidar)
            )

        # Dead-reckoning odometry
        self._odometry = M20DeadReckonOdometry(
            publish_callback=self._publish_tf
        )
        self._odometry.start()

    @rpc
    def stop(self) -> None:
        # Sit down before disconnecting
        try:
            self._protocol.send_usage_mode(UsageMode.REGULAR)
            self._protocol.send_motion_state(MotionState.SIT)
            time.sleep(1.0)
        except Exception:
            logger.exception("Failed to send sit-down during stop")

        try:
            self._velocity_ctrl.stop()
        except Exception:
            logger.exception("Failed to stop velocity controller")

        try:
            if self._ros_sensors:
                self._ros_sensors.stop()
        except Exception:
            logger.exception("Failed to stop ROS sensors")

        try:
            if self._odometry:
                self._odometry.stop()
        except Exception:
            logger.exception("Failed to stop odometry")

        try:
            if self._lidar:
                self._lidar.stop()
        except Exception:
            logger.exception("Failed to stop LiDAR")

        try:
            if self._camera:
                self._camera.stop()
        except Exception:
            logger.exception("Failed to stop camera")

        self._camera_info_running = False
        if self._camera_info_thread and self._camera_info_thread.is_alive():
            self._camera_info_thread.join(timeout=1.0)

        try:
            self._protocol.close()
        except Exception:
            logger.exception("Failed to close protocol")

        super().stop()

    def _on_cmd_vel(self, twist: Twist) -> None:
        """Route Twist commands from the navigation stack to velocity controller."""
        self._velocity_ctrl.set_twist(twist)

        # Feed target velocities into dead-reckoning odometry (Regular Mode only).
        # In Navigation Mode, /ODOM provides real odometry so this is skipped.
        if self._odometry:
            with self._velocity_ctrl._lock:
                self._odometry.update_velocity(
                    self._velocity_ctrl.state.target_linear_x,
                    self._velocity_ctrl.state.target_linear_y,
                    self._velocity_ctrl.state.target_angular_yaw,
                )

    def _on_status_report(self, report: dict) -> None:
        """Handle incoming status reports from the M20."""
        # Status reports can be used for battery, fault detection, etc.
        logger.debug(f"M20 status: type={report.get('type')} items={report.get('items')}")

    def _publish_camera_info(self) -> None:
        """Publish camera intrinsics at 1Hz."""
        while self._camera_info_running and self._camera and self._camera._running:
            self.camera_info.publish(self._camera.camera_info.with_ts(time.time()))
            time.sleep(1.0)

    # --- TF publishing ---

    @classmethod
    def _odom_to_tf(cls, odom: PoseStamped) -> list[Transform]:
        """Convert odometry to TF transforms (base_link + camera frames)."""
        camera_link = Transform(
            translation=Vector3(0.3, 0.0, 0.1),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id="base_link",
            child_frame_id="camera_link",
            ts=odom.ts,
        )

        camera_optical = Transform(
            translation=Vector3(0.0, 0.0, 0.0),
            rotation=Quaternion(-0.5, 0.5, -0.5, 0.5),
            frame_id="camera_link",
            child_frame_id="camera_optical",
            ts=odom.ts,
        )

        return [
            Transform.from_pose("base_link", odom),
            camera_link,
            camera_optical,
        ]

    def _publish_tf(self, msg: PoseStamped) -> None:
        transforms = self._odom_to_tf(msg)
        self.tf.publish(*transforms)
        if self.odom.transport:
            self.odom.publish(msg)

    # --- RPC commands ---

    @rpc
    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        """Send a velocity command. If duration > 0, sends for that duration then stops."""
        self._velocity_ctrl.set_twist(twist)
        if duration > 0:
            time.sleep(duration)
            self._velocity_ctrl.set_twist(
                Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0))
            )
        return True

    @rpc
    def standup(self) -> bool:
        """Make the robot stand up."""
        self._protocol.send_motion_state(MotionState.STAND)
        return True

    @rpc
    def sitdown(self) -> bool:
        """Make the robot sit down."""
        self._protocol.send_motion_state(MotionState.SIT)
        return True

    @rpc
    def set_gait(self, gait: int) -> bool:
        """Switch gait mode. Standard=0x1001, HighObs=0x1002, Stairs=0x1003."""
        self._protocol.send_gait_switch(gait)
        return True

    @rpc
    def emergency_stop(self, engage: bool = True) -> bool:
        """Engage or release emergency stop."""
        self._velocity_ctrl.emergency_stop(engage)
        return True

    @skill
    def observe(self) -> Image | None:
        """Returns the latest video frame from the robot camera.

        Use this skill for any visual world queries.
        Returns None if no frame has been captured yet.
        """
        return self._latest_video_frame


m20_connection = M20Connection.blueprint

__all__ = ["M20Connection", "m20_connection"]
