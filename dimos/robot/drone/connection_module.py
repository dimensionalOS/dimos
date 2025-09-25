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

"""DimOS module wrapper for drone connection."""

import time
import json
from typing import Any, Optional

from dimos.core import In, Module, Out, rpc
from dimos.mapping.types import LatLon
from dimos.msgs.geometry_msgs import PoseStamped, Transform, Vector3, Quaternion
from dimos.msgs.sensor_msgs import Image
from dimos_lcm.std_msgs import String
from dimos.robot.drone.mavlink_connection import MavlinkConnection
from dimos.protocol.skill.skill import skill
from dimos.protocol.skill.type import Output
from dimos.utils.logging_config import setup_logger

logger = setup_logger(__name__)

from dimos.robot.drone.dji_video_stream import DJIDroneVideoStream


class DroneConnectionModule(Module):
    """Module that handles drone sensor data and movement commands."""

    # Inputs
    movecmd: In[Vector3] = None
    gps_goal: In[LatLon] = None

    # Outputs
    odom: Out[PoseStamped] = None
    gps_location: Out[LatLon] = None
    status: Out[String] = None  # JSON status
    telemetry: Out[String] = None  # Full telemetry JSON
    video: Out[Image] = None

    # Parameters
    connection_string: str

    # Internal state
    _odom: Optional[PoseStamped] = None
    _status: dict = {}
    _latest_video_frame: Optional[Image] = None
    _latest_telemetry: Optional[dict[str, Any]] = None

    def __init__(
        self,
        connection_string: str = "udp:0.0.0.0:14550",
        video_port: int = 5600,
        outdoor: bool = False,
        *args,
        **kwargs,
    ):
        """Initialize drone connection module.

        Args:
            connection_string: MAVLink connection string
            video_port: UDP port for video stream
            outdoor: Use GPS only mode (no velocity integration)
        """
        self.connection_string = connection_string
        self.video_port = video_port
        self.outdoor = outdoor
        self.connection = None
        self.video_stream = None
        self._latest_video_frame = None
        self._latest_telemetry = None
        Module.__init__(self, *args, **kwargs)

    @rpc
    def start(self):
        """Start the connection and subscribe to sensor streams."""
        # Check for replay mode
        if self.connection_string == "replay":
            from dimos.robot.drone.mavlink_connection import FakeMavlinkConnection
            from dimos.robot.drone.dji_video_stream import FakeDJIVideoStream

            self.connection = FakeMavlinkConnection("replay")
            self.video_stream = FakeDJIVideoStream(port=self.video_port)
        else:
            self.connection = MavlinkConnection(self.connection_string, outdoor=self.outdoor)
            self.connection.connect()

            self.video_stream = DJIDroneVideoStream(port=self.video_port)

        if not self.connection.connected:
            logger.error("Failed to connect to drone")
            return False

        # Start video stream (already created above)
        if self.video_stream.start():
            logger.info("Video stream started")
            # Subscribe to video, store latest frame and publish it
            self._video_subscription = self.video_stream.get_stream().subscribe(
                self._store_and_publish_frame
            )
            # # TEMPORARY - DELETE AFTER RECORDING
            # from dimos.utils.testing import TimedSensorStorage
            # self._video_storage = TimedSensorStorage("drone/video")
            # self._video_subscription = self._video_storage.save_stream(self.video_stream.get_stream()).subscribe()
            # logger.info("Recording video to data/drone/video/")
        else:
            logger.warning("Video stream failed to start")

        # Subscribe to drone streams
        self.connection.odom_stream().subscribe(self._publish_tf)
        self.connection.status_stream().subscribe(self._publish_status)
        self.connection.telemetry_stream().subscribe(self._publish_telemetry)

        # Subscribe to movement commands
        self.movecmd.subscribe(self.move)

        self.gps_goal.subscribe(self._on_gps_goal)

        # Start telemetry update thread
        import threading

        self._running = True
        self._telemetry_thread = threading.Thread(target=self._telemetry_loop, daemon=True)
        self._telemetry_thread.start()

        logger.info("Drone connection module started")
        return True

    def _store_and_publish_frame(self, frame: Image):
        """Store the latest video frame and publish it."""
        self._latest_video_frame = frame
        self.video.publish(frame)

    def _publish_tf(self, msg: PoseStamped):
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

    def _publish_status(self, status: dict):
        """Publish drone status as JSON string."""
        self._status = status

        status_str = String(json.dumps(status))
        self.status.publish(status_str)

    def _publish_telemetry(self, telemetry: dict):
        """Publish full telemetry as JSON string."""
        telemetry_str = String(json.dumps(telemetry))
        self.telemetry.publish(telemetry_str)
        self._latest_telemetry = telemetry

        if "GLOBAL_POSITION_INT" in telemetry:
            tel = telemetry["GLOBAL_POSITION_INT"]
            self.gps_location.publish(LatLon(lat=tel["lat"], lon=tel["lon"]))

    def _telemetry_loop(self):
        """Continuously update telemetry at 30Hz."""
        frame_count = 0
        while self._running:
            try:
                # Update telemetry from drone
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
    def get_odom(self) -> Optional[PoseStamped]:
        """Get current odometry.

        Returns:
            Current pose or None
        """
        return self._odom

    @rpc
    def get_status(self) -> dict:
        """Get current drone status.

        Returns:
            Status dictionary
        """
        return self._status.copy()

    @skill()
    def move(self, vector: Vector3, duration: float = 0.0):
        """Send movement command to drone.

        Args:
            vector: Velocity vector [x, y, z] in m/s
            duration: How long to move (0 = continuous)
        """
        if self.connection:
            self.connection.move(vector, duration)

    @skill()
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

    @skill()
    def land(self) -> bool:
        """Land the drone.

        Returns:
            True if land command sent
        """
        if self.connection:
            return self.connection.land()
        return False

    @skill()
    def arm(self) -> bool:
        """Arm the drone.

        Returns:
            True if armed successfully
        """
        if self.connection:
            return self.connection.arm()
        return False

    @skill()
    def disarm(self) -> bool:
        """Disarm the drone.

        Returns:
            True if disarmed successfully
        """
        if self.connection:
            return self.connection.disarm()
        return False

    @skill()
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

    def move_twist(self, twist, duration: float = 0.0, lock_altitude: bool = True) -> bool:
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

    @skill()
    def is_flying_to_target(self) -> bool:
        """Check if drone is currently flying to a GPS target.

        Returns:
            True if flying to target, False otherwise
        """
        if self.connection and hasattr(self.connection, "is_flying_to_target"):
            return self.connection.is_flying_to_target
        return False

    @skill()
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

    def _on_gps_goal(self, cmd: LatLon) -> None:
        current_alt = self._latest_telemetry.get("GLOBAL_POSITION_INT", {}).get("relative_alt", 0)
        self.connection.fly_to(cmd.lat, cmd.lon, current_alt)

    @rpc
    def stop(self):
        """Stop the module."""
        self._running = False
        if self.video_stream:
            self.video_stream.stop()
        if self.connection:
            self.connection.disconnect()
        logger.info("Drone connection module stopped")

    @skill(output=Output.image)
    def observe(self) -> Optional[Image]:
        """Returns the latest video frame from the drone camera. Use this skill for any visual world queries.

        This skill provides the current camera view for perception tasks.
        Returns None if no frame has been captured yet.
        """
        return self._latest_video_frame
