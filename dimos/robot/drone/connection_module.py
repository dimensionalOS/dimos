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

"""DimOS module wrapper for drone connection."""

import json
import threading
import time
from typing import Any

from dimos_lcm.std_msgs import String
from reactivex.disposable import CompositeDisposable, Disposable

from dimos.agents.annotation import skill
from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.mapping.models import LatLon
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.drone.dji_video_stream import DJIDroneVideoStream
from dimos.robot.drone.gazebo_video_stream import GazeboVideoStream
from dimos.robot.drone.mavlink_connection import MavlinkConnection
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def _add_disposable(composite: CompositeDisposable, item: Disposable | Any) -> None:
    if isinstance(item, Disposable):
        composite.add(item)
    elif callable(item):
        composite.add(Disposable(item))


class Config(ModuleConfig):
    connection_string: str = "udp:0.0.0.0:14550"
    video_port: int = 5600
    video_source: str = "dji"
    outdoor: bool = False


class DroneConnectionModule(Module[Config]):
    """Module that handles drone sensor data and movement commands."""

    default_config = Config

    # Inputs
    movecmd: In[Vector3]
    movecmd_twist: In[Twist]  # Twist commands from tracking/navigation
    gps_goal: In[LatLon]
    tracking_status: In[Any]

    # Outputs
    odom: Out[PoseStamped]
    gps_location: Out[LatLon]
    status: Out[Any]  # JSON status
    telemetry: Out[Any]  # Full telemetry JSON
    video: Out[Image]
    follow_object_cmd: Out[Any]

    # Internal state
    _odom: PoseStamped | None = None
    _status: dict[str, Any] = {}
    _latest_video_frame: Image | None = None
    _latest_telemetry: dict[str, Any] | None = None
    _latest_status: dict[str, Any] | None = None
    _latest_status_lock: threading.RLock

    def __init__(self, **kwargs: Any) -> None:
        """Initialize drone connection module.

        Args:
            connection_string: MAVLink connection string
            video_port: UDP port for video stream
            outdoor: Use GPS only mode (no velocity integration)
        """
        super().__init__(**kwargs)
        self.connection: MavlinkConnection | None = None
        self.video_stream: DJIDroneVideoStream | GazeboVideoStream | None = None
        self._latest_video_frame = None
        self._latest_telemetry = None
        self._latest_status = None
        self._latest_status_lock = threading.RLock()
        self._running = False
        self._telemetry_thread: threading.Thread | None = None
        self._last_twist_log_time: float = 0.0

    @rpc
    def start(self) -> None:
        """Start the connection and subscribe to sensor streams."""
        # Check for replay mode
        if self.config.connection_string == "replay":
            from dimos.robot.drone.dji_video_stream import FakeDJIVideoStream
            from dimos.robot.drone.mavlink_connection import FakeMavlinkConnection

            self.connection = FakeMavlinkConnection("replay")
            self.video_stream = FakeDJIVideoStream(port=self.config.video_port)
        else:
            self.connection = MavlinkConnection(
                self.config.connection_string, outdoor=self.config.outdoor
            )
            self.connection.connect()

            if self.config.video_source == "gazebo":
                self.video_stream = GazeboVideoStream(port=self.config.video_port)
            else:
                self.video_stream = DJIDroneVideoStream(port=self.config.video_port)

        if not self.connection.connected:
            logger.error("Failed to connect to drone")
            return

        # Start video stream (already created above)
        if self.video_stream.start():
            logger.info("Video stream started")
            # Subscribe to video, store latest frame and publish it
            _add_disposable(
                self._disposables,
                self.video_stream.get_stream().subscribe(self._store_and_publish_frame),
            )
            # # TEMPORARY - DELETE AFTER RECORDING
            # from dimos.utils.testing import TimedSensorStorage
            # self._video_storage = TimedSensorStorage("drone/video")
            # self._video_subscription = self._video_storage.save_stream(self.video_stream.get_stream()).subscribe()
            # logger.info("Recording video to data/drone/video/")
        else:
            logger.warning("Video stream failed to start")

        # Subscribe to drone streams
        _add_disposable(
            self._disposables, self.connection.odom_stream().subscribe(self._publish_tf)
        )
        _add_disposable(
            self._disposables, self.connection.status_stream().subscribe(self._publish_status)
        )
        _add_disposable(
            self._disposables, self.connection.telemetry_stream().subscribe(self._publish_telemetry)
        )

        # Subscribe to movement commands
        _add_disposable(self._disposables, self.movecmd.subscribe(self.move))

        # Subscribe to Twist movement commands
        if self.movecmd_twist.transport:
            _add_disposable(self._disposables, self.movecmd_twist.subscribe(self._on_move_twist))

        if self.gps_goal.transport:
            _add_disposable(self._disposables, self.gps_goal.subscribe(self._on_gps_goal))

        if self.tracking_status.transport:
            _add_disposable(
                self._disposables, self.tracking_status.subscribe(self._on_tracking_status)
            )

        # Start telemetry update thread
        import threading

        self._running = True
        self._telemetry_thread = threading.Thread(target=self._telemetry_loop, daemon=True)
        self._telemetry_thread.start()

        logger.info("Drone connection module started")
        return

    def _store_and_publish_frame(self, frame: Image) -> None:
        """Store the latest video frame and publish it."""
        self._latest_video_frame = frame
        self.video.publish(frame)

    def _publish_tf(self, msg: PoseStamped) -> None:
        """Publish odometry and TF transforms."""
        self._odom = msg

        # Publish odometry
        self.odom.publish(msg)

        # Publish base_link transform
        base_link = Transform(
            translation=msg.position,
            rotation=msg.orientation,
            frame_id="world",
            child_frame_id="base_link",
            ts=msg.ts if hasattr(msg, "ts") else time.time(),
        )
        self.tf.publish(base_link)

        # Publish camera_link transform (camera mounted on front of drone, no gimbal factored in yet)
        camera_link = Transform(
            translation=Vector3(0.1, 0.0, -0.05),  # 10cm forward, 5cm down
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),  # No rotation relative to base
            frame_id="base_link",
            child_frame_id="camera_link",
            ts=time.time(),
        )
        self.tf.publish(camera_link)

    def _publish_status(self, status: dict[str, Any]) -> None:
        """Publish drone status as JSON string."""
        self._status = status

        status_str = String(json.dumps(status))
        self.status.publish(status_str)

    def _publish_telemetry(self, telemetry: dict[str, Any]) -> None:
        """Publish full telemetry as JSON string."""
        telemetry_str = String(json.dumps(telemetry))
        self.telemetry.publish(telemetry_str)
        self._latest_telemetry = telemetry

        if "GLOBAL_POSITION_INT" in telemetry:
            tel = telemetry["GLOBAL_POSITION_INT"]
            self.gps_location.publish(LatLon(lat=tel["lat"], lon=tel["lon"]))

    def _telemetry_loop(self) -> None:
        """Continuously update telemetry at 30Hz."""
        frame_count = 0
        while self._running:
            try:
                # Update telemetry from drone
                if self.connection is not None:
                    self.connection.update_telemetry(timeout=0.01)

                # Publish default odometry if we don't have real data yet
                if frame_count % 10 == 0:  # Every ~3Hz
                    if self._odom is None:
                        # Publish default pose
                        default_pose = PoseStamped(
                            position=Vector3(0, 0, 0),
                            orientation=Quaternion(0, 0, 0, 1),
                            frame_id="world",
                            ts=time.time(),
                        )
                        self._publish_tf(default_pose)
                        logger.debug("Publishing default odometry")

                frame_count += 1
                time.sleep(0.033)  # ~30Hz
            except Exception as e:
                logger.debug(f"Telemetry update error: {e}")
                time.sleep(0.1)

    @rpc
    def get_odom(self) -> PoseStamped | None:
        """Get current odometry.

        Returns:
            Current pose or None
        """
        return self._odom

    @rpc
    def get_status(self) -> dict[str, Any]:
        """Get current drone status.

        Returns:
            Status dictionary
        """
        return self._status.copy()

    @skill
    def move(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, duration: float = 0.0) -> None:
        """Send velocity command (MAV_FRAME_BODY_NED via SET_POSITION_TARGET_LOCAL_NED).

        Use for continuous or manual velocity. Tracking uses move_twist, not this.

        Args:
            x: Body right velocity in m/s.
            y: Forward velocity in m/s.
            z: Down velocity in m/s (positive = descend; negative = climb).
            duration: How long to move (0 = continuous).
        """
        if self.connection:
            self.connection.move(Vector3(x, y, z), duration)

    @skill
    def move_by_distance(
        self,
        forward_m: float = 0.0,
        right_m: float = 0.0,
        down_m: float = 0.0,
        speed: float = 0.3,
    ) -> str:
        """Move by body-frame displacement in meters.

        Prefers a local NED position setpoint when telemetry allows; otherwise timed forward
        velocity. For absolute NED coordinates use go_to_position. For follow/track use
        follow_object (velocity via move_twist), not this.

        Args:
            forward_m: Forward/backward distance (m); positive = forward.
            right_m: Right/left distance (m); positive = right.
            down_m: Down distance (m); positive = down (NED).
            speed: Fallback forward speed (m/s) when local position is unavailable.

        Returns:
            Status message for the agent.
        """
        if not self.connection:
            return "Failed: no connection."
        ok = self.connection.move_by_distance_body_m(forward_m, right_m, down_m, speed)
        if ok:
            return (
                f"move_by_distance applied: forward={forward_m}m right={right_m}m down={down_m}m "
                "(local NED setpoint if available, else forward velocity fallback)."
            )
        return (
            "Failed move_by_distance (e.g. lateral/vertical without LOCAL_POSITION_NED, or FC rejected)."
        )

    @skill
    def go_to_position(
        self, x: float, y: float, z: float,
        vx_ff: float = 0.0, vy_ff: float = 0.0, vz_ff: float = 0.0,
    ) -> bool:
        """Send position target in local NED with optional velocity feedforward.

        Args:
            x: North position in meters (local NED).
            y: East position in meters (local NED).
            z: Down position in meters (negative = up).
            vx_ff: Velocity feedforward North (m/s). Optional.
            vy_ff: Velocity feedforward East (m/s). Optional.
            vz_ff: Velocity feedforward Down (m/s). Optional.

        Returns:
            True if position target sent successfully
        """
        if self.connection:
            return self.connection.set_position_target(x, y, z, vx_ff, vy_ff, vz_ff)
        return False

    @skill
    def rotate_to(self, heading_deg: float, timeout: float = 60.0) -> bool:
        """Rotate drone to a specific heading (yaw control).

        Args:
            heading_deg: Target heading in degrees (0–360, 0=North, 90=East).
            timeout: Max time to rotate in seconds.

        Returns:
            True if rotation completed successfully
        """
        if self.connection:
            return self.connection.rotate_to(heading_deg, timeout)
        return False

    @skill
    def takeoff(self, altitude: float = 3.0) -> bool:
        """Takeoff to specified altitude.

        Args:
            altitude: Target altitude in meters

        Returns:
            True if takeoff initiated
        """
        if self.connection:
            return self.connection.takeoff(altitude)
        return False

    @skill
    def land(self) -> bool:
        """Land the drone.

        Returns:
            True if land command sent
        """
        if self.connection:
            return self.connection.land()
        return False

    @skill
    def arm(self) -> bool:
        """Arm the drone.

        Returns:
            True if armed successfully
        """
        if self.connection:
            return self.connection.arm()
        return False

    @skill
    def disarm(self) -> bool:
        """Disarm the drone.

        Returns:
            True if disarmed successfully
        """
        if self.connection:
            return self.connection.disarm()
        return False

    @skill
    def set_mode(self, mode: str) -> bool:
        """Set flight mode.

        Args:
            mode: Flight mode name

        Returns:
            True if mode set successfully
        """
        if self.connection:
            return self.connection.set_mode(mode)
        return False

    def move_twist(self, twist: Twist, duration: float = 0.0, lock_altitude: bool = True) -> bool:
        """Move using ROS-style Twist commands.

        Args:
            twist: Twist message with linear velocities
            duration: How long to move (0 = single command)
            lock_altitude: If True, ignore Z velocity

        Returns:
            True if command sent successfully
        """
        if self.connection:
            return self.connection.move_twist(twist, duration, lock_altitude)
        return False

    @skill
    def is_flying_to_target(self) -> bool:
        """Check if drone is currently flying to a GPS target.

        Returns:
            True if flying to target, False otherwise
        """
        if self.connection and hasattr(self.connection, "is_flying_to_target"):
            return self.connection.is_flying_to_target
        return False

    @skill
    def fly_to(self, lat: float, lon: float, alt: float) -> str:
        """Fly drone to GPS coordinates (blocking operation).

        Args:
            lat: Latitude in degrees
            lon: Longitude in degrees
            alt: Altitude in meters (relative to home)

        Returns:
            String message indicating success or failure reason
        """
        if self.connection:
            return self.connection.fly_to(lat, lon, alt)
        return "Failed: No connection to drone"

    @skill
    def follow_object(
        self, object_description: str, duration: float = 0.0
    ) -> str:
        """Follow an object with visual servoing (velocity via move_twist, not position setpoints).

        Example:

            follow_object(object_description="red car", duration=0)  # 0 = track until stop_follow

        Args:
            object_description (str): A short and clear description of the object.
            duration (float, optional): Max tracking time in seconds; 0 = until stop_follow. Defaults to 0.
        """
        msg = {"object_description": object_description, "duration": duration}
        self.follow_object_cmd.publish(String(json.dumps(msg)))

        # Poll for initial tracking result (found / not found / timeout)
        start_time = time.time()
        timeout = 30.0 if duration <= 0 else min(duration, 30.0)

        while time.time() - start_time < timeout:
            time.sleep(0.1)
            with self._latest_status_lock:
                if not self._latest_status:
                    continue
                match self._latest_status.get("status"):
                    case "not_found" | "failed":
                        return f"The '{object_description}' object has not been found. Stopped tracking."
                    case "tracking":
                        return (
                            f"Now following '{object_description}'."
                            + (" Run stop_follow to stop." if duration <= 0 else f" For up to {duration}s.")
                        )
                    case "lost":
                        return f"The '{object_description}' object was lost. Stopped tracking."
                    case "stopped":
                        return f"Tracking '{object_description}' has stopped."

        return f"Started tracking command for '{object_description}'. No status update yet."

    def _on_move_twist(self, msg: Twist) -> None:
        """Handle Twist movement commands from tracking/navigation.

        Args:
            msg: Twist message with linear and angular velocities
        """
        if self.connection:
            now = time.time()
            if now - self._last_twist_log_time >= 0.5:
                self._last_twist_log_time = now
                logger.info(
                    "follow→ardupilot: linear x=%.3f y=%.3f z=%.3f yaw_rate=%.3f (m/s, rad/s)",
                    msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z,
                )
            self.connection.move_twist(msg, duration=0, lock_altitude=False)

    def _on_gps_goal(self, cmd: LatLon) -> None:
        if self._latest_telemetry is None or self.connection is None:
            return
        current_alt = self._latest_telemetry.get("GLOBAL_POSITION_INT", {}).get("relative_alt", 0)
        self.connection.fly_to(cmd.lat, cmd.lon, current_alt)

    def _on_tracking_status(self, status: String) -> None:
        with self._latest_status_lock:
            self._latest_status = json.loads(status.data)

    @rpc
    def stop(self) -> None:
        """Stop the module."""
        # Stop the telemetry loop
        self._running = False

        # Wait for telemetry thread to finish
        if self._telemetry_thread and self._telemetry_thread.is_alive():
            self._telemetry_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)

        # Stop video stream
        if self.video_stream:
            self.video_stream.stop()

        # Disconnect from drone
        if self.connection:
            self.connection.disconnect()

        logger.info("Drone connection module stopped")

        # Call parent stop to clean up Module infrastructure (event loop, LCM, disposables, etc.)
        super().stop()

    @skill
    def observe(self) -> Image | None:
        """Returns the latest video frame from the drone camera. Use this skill for any visual world queries.

        This skill provides the current camera view for perception tasks.
        Returns None if no frame has been captured yet.
        """
        return self._latest_video_frame
