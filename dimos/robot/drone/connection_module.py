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

from collections.abc import Generator
import json
import threading
import time
from typing import Any

from dimos_lcm.std_msgs import String  # type: ignore[import-untyped]
from reactivex.disposable import CompositeDisposable, Disposable

from dimos.core import In, Module, Out, rpc
from dimos.mapping.types import LatLon
from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Transform, Twist, Vector3
from dimos.msgs.sensor_msgs import Image
from dimos.protocol.skill.skill import skill
from dimos.protocol.skill.type import Output
from dimos.robot.drone.dji_video_stream import DJIDroneVideoStream
from dimos.robot.drone.mavlink_connection import MavlinkConnection
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def _add_disposable(composite: CompositeDisposable, item: Disposable | Any) -> None:
    if isinstance(item, Disposable):
        composite.add(item)
    elif callable(item):
        composite.add(Disposable(item))


class DroneConnectionModule(Module):
    """Module that handles drone sensor data and movement commands."""

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

    # Parameters
    connection_string: str

    # Internal state
    _odom: PoseStamped | None = None
    _status: dict[str, Any] = {}
    _latest_video_frame: Image | None = None
    _latest_telemetry: dict[str, Any] | None = None
    _latest_status: dict[str, Any] | None = None
    _latest_status_lock: threading.RLock

    def __init__(
        self,
        connection_string: str = "udp:0.0.0.0:14550",
        video_port: int = 5600,
        outdoor: bool = False,
        *args: Any,
        **kwargs: Any,
    ) -> None:
        """Initialize drone connection module.

        Args:
            connection_string: MAVLink connection string
            video_port: UDP port for video stream
            outdoor: Use GPS only mode (no velocity integration)
        """
        self.connection_string = connection_string
        self.video_port = video_port
        self.outdoor = outdoor
        self.connection: MavlinkConnection | None = None
        self.video_stream: DJIDroneVideoStream | None = None
        self._latest_video_frame = None
        self._latest_telemetry = None
        self._latest_status = None
        self._latest_status_lock = threading.RLock()
        self._running = False
        self._telemetry_thread: threading.Thread | None = None
        Module.__init__(self, *args, **kwargs)

    @rpc
    def start(self) -> bool:
        """Start the connection and subscribe to sensor streams."""
        # Check for replay mode
        if self.connection_string == "replay":
            from dimos.robot.drone.dji_video_stream import FakeDJIVideoStream
            from dimos.robot.drone.mavlink_connection import FakeMavlinkConnection

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
        return True

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

    @skill()
    def move(self, vector: Vector3, duration: float = 0.0) -> None:
        """Send movement command to drone.

        Args:
            vector: Velocity vector [x, y, z] in m/s
            duration: How long to move (0 = continuous)
        """
        if self.connection:
            # Convert dict/list to Vector3
            if isinstance(vector, dict):
                vector = Vector3(vector.get("x", 0), vector.get("y", 0), vector.get("z", 0))
            elif isinstance(vector, (list, tuple)):
                vector = Vector3(
                    vector[0] if len(vector) > 0 else 0,
                    vector[1] if len(vector) > 1 else 0,
                    vector[2] if len(vector) > 2 else 0,
                )
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

    @skill()
    def follow_object(
        self, object_description: str, duration: float = 120.0
    ) -> Generator[str, None, None]:
        """Follow an object with visual servoing.

        Example:

            follow_object(object_description="red car", duration=120)

        Args:
            object_description (str): A short and clear description of the object.
            duration (float, optional): How long to track for. Defaults to 120.0.
        """
        msg = {"object_description": object_description, "duration": duration}
        self.follow_object_cmd.publish(String(json.dumps(msg)))

        yield "Started trying to track. First, trying to find the object."

        start_time = time.time()

        started_tracking = False

        while time.time() - start_time < duration:
            time.sleep(0.01)
            with self._latest_status_lock:
                if not self._latest_status:
                    continue
                match self._latest_status.get("status"):
                    case "not_found" | "failed":
                        yield f"The '{object_description}' object has not been found. Stopped tracking."
                        break
                    case "tracking":
                        # Only return tracking once.
                        if not started_tracking:
                            started_tracking = True
                            yield f"The '{object_description}' object is now being followed."
                    case "lost":
                        yield f"The '{object_description}' object has been lost. Stopped tracking."
                        break
                    case "stopped":
                        yield f"Tracking '{object_description}' has stopped."
                        break
        else:
            yield f"Stopped tracking '{object_description}'"

    def _on_move_twist(self, msg: Twist) -> None:
        """Handle Twist movement commands from tracking/navigation.

        Args:
            msg: Twist message with linear and angular velocities
        """
        if self.connection:
            # Use move_twist to properly handle Twist messages
            self.connection.move_twist(msg, duration=0, lock_altitude=True)

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
            self._telemetry_thread.join(timeout=2.0)

        # Stop video stream
        if self.video_stream:
            self.video_stream.stop()

        # Disconnect from drone
        if self.connection:
            self.connection.disconnect()

        logger.info("Drone connection module stopped")

        # Call parent stop to clean up Module infrastructure (event loop, LCM, disposables, etc.)
        super().stop()

    @skill(output=Output.image)
    def observe(self) -> Image | None:
        """Returns the latest video frame from the drone camera. Use this skill for any visual world queries.

        This skill provides the current camera view for perception tasks.
        Returns None if no frame has been captured yet.
        """
        return self._latest_video_frame
