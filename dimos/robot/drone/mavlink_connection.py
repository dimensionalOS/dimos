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

"""MAVLink-based drone connection for DimOS."""

import functools
import logging
import time
from typing import Optional, Dict, Any

from pymavlink import mavutil
from reactivex import Subject
from reactivex import operators as ops

from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Vector3
from dimos.robot.connection_interface import ConnectionInterface
from dimos.utils.logging_config import setup_logger
from dimos.core import In, Module, Out, rpc

logger = setup_logger(__name__, level=logging.INFO)


class MavlinkConnection:
    """MAVLink connection for drone control."""

    def __init__(self, connection_string: str = "udp:0.0.0.0:14550", outdoor: bool = False):
        """Initialize drone connection.

        Args:
            connection_string: MAVLink connection string
            outdoor: Use GPS only mode (no velocity integration)
        """
        self.connection_string = connection_string
        self.outdoor = outdoor
        self.mavlink = None
        self.connected = False
        self.telemetry = {}

        self._odom_subject = Subject()
        self._status_subject = Subject()
        self._telemetry_subject = Subject()
        self._raw_mavlink_subject = Subject()

        # # TEMPORARY - DELETE AFTER RECORDING
        # from dimos.utils.testing import TimedSensorStorage
        # self._mavlink_storage = TimedSensorStorage("drone/mavlink")
        # self._mavlink_subscription = self._mavlink_storage.save_stream(self._raw_mavlink_subject).subscribe()
        # logger.info("Recording MAVLink to data/drone/mavlink/")

    def connect(self):
        """Connect to drone via MAVLink."""
        try:
            logger.info(f"Connecting to {self.connection_string}")
            self.mavlink = mavutil.mavlink_connection(self.connection_string)
            self.mavlink.wait_heartbeat(timeout=30)
            self.connected = True
            logger.info(f"Connected to system {self.mavlink.target_system}")

            self.update_telemetry()
            return True
        except Exception as e:
            logger.error(f"Connection failed: {e}")
            return False

    def update_telemetry(self, timeout: float = 0.1):
        """Update telemetry data from available messages."""
        if not self.connected:
            return

        end_time = time.time() + timeout
        while time.time() < end_time:
            msg = self.mavlink.recv_match(blocking=False)
            if not msg:
                time.sleep(0.001)
                continue
            msg_type = msg.get_type()
            msg_dict = msg.to_dict()
            if msg_type == "HEARTBEAT":
                armed = bool(
                    msg_dict.get("base_mode", 0) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )
                # print("HEARTBEAT:", msg_dict, "ARMED:", armed)
            # print("MESSAGE", msg_dict)
            # print("MESSAGE TYPE", msg_type)

            # TEMPORARY - DELETE AFTER RECORDING
            # self._raw_mavlink_subject.on_next(msg_dict)

            self.telemetry[msg_type] = msg_dict

            # Apply unit conversions for known fields
            if msg_type == "GLOBAL_POSITION_INT":
                msg_dict["lat"] = msg_dict.get("lat", 0) / 1e7
                msg_dict["lon"] = msg_dict.get("lon", 0) / 1e7
                msg_dict["alt"] = msg_dict.get("alt", 0) / 1000.0
                msg_dict["relative_alt"] = msg_dict.get("relative_alt", 0) / 1000.0
                msg_dict["vx"] = msg_dict.get("vx", 0) / 100.0  # cm/s to m/s
                msg_dict["vy"] = msg_dict.get("vy", 0) / 100.0
                msg_dict["vz"] = msg_dict.get("vz", 0) / 100.0
                msg_dict["hdg"] = msg_dict.get("hdg", 0) / 100.0  # centidegrees to degrees
                self._publish_odom()

            elif msg_type == "GPS_RAW_INT":
                msg_dict["lat"] = msg_dict.get("lat", 0) / 1e7
                msg_dict["lon"] = msg_dict.get("lon", 0) / 1e7
                msg_dict["alt"] = msg_dict.get("alt", 0) / 1000.0
                msg_dict["vel"] = msg_dict.get("vel", 0) / 100.0
                msg_dict["cog"] = msg_dict.get("cog", 0) / 100.0

            elif msg_type == "SYS_STATUS":
                msg_dict["voltage_battery"] = msg_dict.get("voltage_battery", 0) / 1000.0
                msg_dict["current_battery"] = msg_dict.get("current_battery", 0) / 100.0
                self._publish_status()

            elif msg_type == "POWER_STATUS":
                msg_dict["Vcc"] = msg_dict.get("Vcc", 0) / 1000.0
                msg_dict["Vservo"] = msg_dict.get("Vservo", 0) / 1000.0

            elif msg_type == "HEARTBEAT":
                # Extract armed status
                base_mode = msg_dict.get("base_mode", 0)
                msg_dict["armed"] = bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self._publish_status()

            elif msg_type == "ATTITUDE":
                self._publish_odom()

            self.telemetry[msg_type] = msg_dict

            self._publish_telemetry()

    def _publish_odom(self):
        """Publish odometry data - GPS for outdoor mode, velocity integration for indoor mode."""
        attitude = self.telemetry.get("ATTITUDE", {})
        roll = attitude.get("roll", 0)
        pitch = attitude.get("pitch", 0)
        yaw = attitude.get("yaw", 0)

        # Use heading from GLOBAL_POSITION_INT if no ATTITUDE data
        if "roll" not in attitude and "GLOBAL_POSITION_INT" in self.telemetry:
            import math

            heading = self.telemetry["GLOBAL_POSITION_INT"].get("hdg", 0)
            yaw = math.radians(heading)

        if "roll" not in attitude and "GLOBAL_POSITION_INT" not in self.telemetry:
            logger.debug("No attitude or position data available")
            return

        # MAVLink --> ROS conversion
        # MAVLink: positive pitch = nose up, positive yaw = clockwise
        # ROS: positive pitch = nose down, positive yaw = counter-clockwise
        quaternion = Quaternion.from_euler(Vector3(roll, -pitch, -yaw))

        if not hasattr(self, "_position"):
            self._position = {"x": 0.0, "y": 0.0, "z": 0.0}
            self._last_update = time.time()
            if self.outdoor:
                self._gps_origin = None

        current_time = time.time()
        dt = current_time - self._last_update

        # Get position data from GLOBAL_POSITION_INT
        pos_data = self.telemetry.get("GLOBAL_POSITION_INT", {})

        # Outdoor mode: Use GPS coordinates
        if self.outdoor and pos_data:
            lat = pos_data.get("lat", 0)  # Already in degrees from update_telemetry
            lon = pos_data.get("lon", 0)  # Already in degrees from update_telemetry

            if lat != 0 and lon != 0:  # Valid GPS fix
                if self._gps_origin is None:
                    self._gps_origin = {"lat": lat, "lon": lon}
                    logger.debug(f"GPS origin set: lat={lat:.7f}, lon={lon:.7f}")

                # Convert GPS to local X/Y coordinates
                import math

                R = 6371000  # Earth radius in meters
                dlat = math.radians(lat - self._gps_origin["lat"])
                dlon = math.radians(lon - self._gps_origin["lon"])

                # X = North, Y = West (ROS convention)
                self._position["x"] = dlat * R
                self._position["y"] = -dlon * R * math.cos(math.radians(self._gps_origin["lat"]))

        # Indoor mode: Use velocity integration (ORIGINAL CODE - UNCHANGED)
        elif pos_data and dt > 0:
            vx = pos_data.get("vx", 0)  # North velocity in m/s (already converted)
            vy = pos_data.get("vy", 0)  # East velocity in m/s (already converted)

            # +vx is North, +vy is East in NED mavlink frame
            # ROS/Foxglove: X=forward(North), Y=left(West), Z=up
            self._position["x"] += vx * dt  # North → X (forward)
            self._position["y"] += -vy * dt  # East → -Y (right in ROS, Y points left/West)

        # Altitude handling (same for both modes)
        if "ALTITUDE" in self.telemetry:
            self._position["z"] = self.telemetry["ALTITUDE"].get("altitude_relative", 0)
        elif pos_data:
            self._position["z"] = pos_data.get(
                "relative_alt", 0
            )  # Already in m from update_telemetry

        self._last_update = current_time

        # Debug logging
        mode = "GPS" if self.outdoor else "VELOCITY"
        logger.debug(
            f"[{mode}] Position: x={self._position['x']:.2f}m, y={self._position['y']:.2f}m, z={self._position['z']:.2f}m"
        )

        pose = PoseStamped(
            position=Vector3(self._position["x"], self._position["y"], self._position["z"]),
            orientation=quaternion,
            frame_id="world",
            ts=current_time,
        )

        self._odom_subject.on_next(pose)

    def _publish_status(self):
        """Publish drone status with key telemetry."""
        heartbeat = self.telemetry.get("HEARTBEAT", {})
        sys_status = self.telemetry.get("SYS_STATUS", {})
        gps_raw = self.telemetry.get("GPS_RAW_INT", {})
        global_pos = self.telemetry.get("GLOBAL_POSITION_INT", {})
        altitude = self.telemetry.get("ALTITUDE", {})

        status = {
            "armed": heartbeat.get("armed", False),
            "mode": heartbeat.get("custom_mode", -1),
            "battery_voltage": sys_status.get("voltage_battery", 0),
            "battery_current": sys_status.get("current_battery", 0),
            "battery_remaining": sys_status.get("battery_remaining", 0),
            "satellites": gps_raw.get("satellites_visible", 0),
            "altitude": altitude.get("altitude_relative", global_pos.get("relative_alt", 0)),
            "heading": global_pos.get("hdg", 0),
            "vx": global_pos.get("vx", 0),
            "vy": global_pos.get("vy", 0),
            "vz": global_pos.get("vz", 0),
            "lat": global_pos.get("lat", 0),
            "lon": global_pos.get("lon", 0),
            "ts": time.time(),
        }
        self._status_subject.on_next(status)

    def _publish_telemetry(self):
        """Publish full telemetry data."""
        telemetry_with_ts = self.telemetry.copy()
        telemetry_with_ts["timestamp"] = time.time()
        self._telemetry_subject.on_next(telemetry_with_ts)

    def move(self, velocity: Vector3, duration: float = 0.0) -> bool:
        """Send movement command to drone.

        Args:
            velocity: Velocity vector [x, y, z] in m/s
            duration: How long to move (0 = continuous)

        Returns:
            True if command sent successfully
        """
        if not self.connected:
            return False

        # MAVLink body frame velocities
        forward = velocity.y  # Forward/backward
        right = velocity.x  # Left/right
        down = -velocity.z  # Up/down (negative for up)

        logger.debug(f"Moving: forward={forward}, right={right}, down={down}")

        if duration > 0:
            # Send velocity for duration
            end_time = time.time() + duration
            while time.time() < end_time:
                self.mavlink.mav.set_position_target_local_ned_send(
                    0,  # time_boot_ms
                    self.mavlink.target_system,
                    self.mavlink.target_component,
                    mavutil.mavlink.MAV_FRAME_BODY_NED,
                    0b0000111111000111,  # type_mask (only velocities)
                    0,
                    0,
                    0,  # positions
                    forward,
                    right,
                    down,  # velocities
                    0,
                    0,
                    0,  # accelerations
                    0,
                    0,  # yaw, yaw_rate
                )
                time.sleep(0.1)
            self.stop()
        else:
            # Single velocity command
            self.mavlink.mav.set_position_target_local_ned_send(
                0,
                self.mavlink.target_system,
                self.mavlink.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                0b0000111111000111,
                0,
                0,
                0,
                forward,
                right,
                down,
                0,
                0,
                0,
                0,
                0,
            )

        return True

    def stop(self) -> bool:
        """Stop all movement."""
        if not self.connected:
            return False

        self.mavlink.mav.set_position_target_local_ned_send(
            0,
            self.mavlink.target_system,
            self.mavlink.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        return True

    def arm(self) -> bool:
        """Arm the drone."""
        if not self.connected:
            return False

        logger.info("Arming motors...")
        self.update_telemetry()

        self.mavlink.mav.command_long_send(
            self.mavlink.target_system,
            self.mavlink.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0,
            0,
            0,
            0,
            0,
            0,
        )

        # Wait for ACK
        ack = self.mavlink.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                logger.info("Arm command accepted")

                # Verify armed status
                for i in range(10):
                    msg = self.mavlink.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
                    if msg:
                        armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                        if armed:
                            logger.info("Motors ARMED successfully!")
                            return True
                    time.sleep(0.5)
            else:
                logger.error(f"Arm failed with result: {ack.result}")

        return False

    def disarm(self) -> bool:
        """Disarm the drone."""
        if not self.connected:
            return False

        logger.info("Disarming motors...")

        self.mavlink.mav.command_long_send(
            self.mavlink.target_system,
            self.mavlink.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )

        time.sleep(1)
        return True

    def takeoff(self, altitude: float = 3.0) -> bool:
        """Takeoff to specified altitude."""
        if not self.connected:
            return False

        logger.info(f"Taking off to {altitude}m...")

        # Set GUIDED mode
        if not self.set_mode("GUIDED"):
            logger.error("Failed to set GUIDED mode for takeoff")
            return False

        # Send takeoff command
        self.mavlink.mav.command_long_send(
            self.mavlink.target_system,
            self.mavlink.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            altitude,
        )

        logger.info(f"Takeoff command sent for {altitude}m altitude")
        return True

    def land(self) -> bool:
        """Land the drone at current position."""
        if not self.connected:
            return False

        logger.info("Landing...")

        # Send initial land command
        self.mavlink.mav.command_long_send(
            self.mavlink.target_system,
            self.mavlink.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )

        # Wait for disarm with confirmations
        disarm_count = 0
        for _ in range(120):  # 60 seconds max (120 * 0.5s)
            # Keep sending land command
            self.mavlink.mav.command_long_send(
                self.mavlink.target_system,
                self.mavlink.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
            )

            # Check armed status
            msg = self.mavlink.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
            if msg:
                msg_dict = msg.to_dict()
                armed = bool(
                    msg_dict.get("base_mode", 0) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )
                logger.debug(f"HEARTBEAT: {msg_dict} ARMED: {armed}")

                disarm_count = 0 if armed else disarm_count + 1

                if disarm_count >= 5:  # 2.5 seconds of continuous disarm
                    logger.info("Drone landed and disarmed")
                    return True

            time.sleep(0.5)

        logger.warning("Land timeout")
        return self.set_mode("LAND")

    def fly_to(self, lat: float, lon: float, alt: float) -> bool:
        """Fly to GPS coordinates - sends commands continuously until reaching target.

        Args:
            lat: Latitude in degrees
            lon: Longitude in degrees
            alt: Altitude in meters (relative to home)

        Returns:
            True if command sent successfully
        """
        if not self.connected:
            return False

        # Ensure GUIDED mode for GPS navigation
        if not self.set_mode("GUIDED"):
            logger.error("Failed to set GUIDED mode for GPS navigation")
            return False

        logger.info(f"Flying to GPS: lat={lat:.7f}, lon={lon:.7f}, alt={alt:.1f}m")

        # Send velocity commands towards GPS target at 1Hz
        acceptance_radius = 10.0  # meters
        max_duration = 120  # seconds max flight time
        start_time = time.time()
        max_speed = 5.0  # m/s max speed

        import math

        loop_count = 0
        while time.time() - start_time < max_duration:
            loop_start = time.time()

            # Update telemetry to get fresh GPS position
            self.update_telemetry(timeout=0.01)  # Short timeout to not block

            # Check current position from telemetry
            if "GLOBAL_POSITION_INT" in self.telemetry:
                t1 = time.time()

                # Telemetry already has converted values (see update_telemetry lines 104-107)
                current_lat = self.telemetry["GLOBAL_POSITION_INT"].get(
                    "lat", 0
                )  # Already in degrees
                current_lon = self.telemetry["GLOBAL_POSITION_INT"].get(
                    "lon", 0
                )  # Already in degrees
                current_alt = self.telemetry["GLOBAL_POSITION_INT"].get(
                    "relative_alt", 0
                )  # Already in meters

                t2 = time.time()

                logger.info(
                    f"DEBUG: Current GPS: lat={current_lat:.10f}, lon={current_lon:.10f}, alt={current_alt:.2f}m"
                )
                logger.info(f"DEBUG: Target GPS:  lat={lat:.10f}, lon={lon:.10f}, alt={alt:.2f}m")

                # Calculate vector to target with high precision
                dlat = lat - current_lat
                dlon = lon - current_lon
                dalt = alt - current_alt

                logger.info(f"DEBUG: Delta: dlat={dlat:.10f}, dlon={dlon:.10f}, dalt={dalt:.2f}m")

                t3 = time.time()

                # Convert lat/lon difference to meters with high precision
                # Using more accurate calculation
                lat_rad = current_lat * math.pi / 180.0
                meters_per_degree_lat = (
                    111132.92 - 559.82 * math.cos(2 * lat_rad) + 1.175 * math.cos(4 * lat_rad)
                )
                meters_per_degree_lon = 111412.84 * math.cos(lat_rad) - 93.5 * math.cos(3 * lat_rad)

                x_dist = dlat * meters_per_degree_lat  # North distance in meters
                y_dist = dlon * meters_per_degree_lon  # East distance in meters

                logger.info(
                    f"DEBUG: Distance in meters: North={x_dist:.2f}m, East={y_dist:.2f}m, Up={dalt:.2f}m"
                )

                # Calculate total distance
                distance = math.sqrt(x_dist**2 + y_dist**2 + dalt**2)
                logger.info(f"DEBUG: Total distance to target: {distance:.2f}m")

                t4 = time.time()

                if distance < acceptance_radius:
                    logger.info(f"Reached GPS target (within {distance:.1f}m)")
                    self.stop()
                    break

                # Calculate velocity commands (NED frame)
                if distance > 0.1:  # Only send commands if we're far enough
                    # Normalize and scale by max speed
                    speed = min(
                        max_speed, max(0.5, distance / 10.0)
                    )  # Min 0.5 m/s, scale down as we approach
                    vx = (x_dist / distance) * speed  # North velocity
                    vy = (y_dist / distance) * speed  # East velocity
                    # This system uses opposite of NED for Z axis:
                    # Positive vz = UP, Negative vz = DOWN
                    # If dalt is negative (target below), we want negative vz (go down)
                    # If dalt is positive (target above), we want positive vz (go up)
                    vz = (dalt / distance) * speed

                    logger.info(
                        f"DEBUG: Sending velocity: vx={vx:.3f} vy={vy:.3f} vz={vz:.3f} m/s (speed={speed:.2f})"
                    )

                    # Log if we're not moving as expected
                    if loop_count > 20 and loop_count % 10 == 0:
                        logger.warning(
                            f"STUCK? Been sending commands for {loop_count} iterations but distance still {distance:.1f}m"
                        )

                    t5 = time.time()

                    # Send velocity command in NED frame
                    self.mavlink.mav.set_position_target_local_ned_send(
                        0,  # time_boot_ms
                        self.mavlink.target_system,
                        self.mavlink.target_component,
                        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # Local NED frame
                        0b0000111111000111,  # type_mask (only velocities enabled)
                        0,
                        0,
                        0,  # positions (not used)
                        vx,
                        vy,
                        vz,  # velocities in m/s (NED)
                        0,
                        0,
                        0,  # accelerations (not used)
                        0,
                        0,  # yaw, yaw_rate (not used)
                    )

                    t6 = time.time()

                    # Log timing every 10 loops
                    loop_count += 1
                    if loop_count % 10 == 0:
                        logger.info(
                            f"TIMING: telemetry_read={t2 - t1:.4f}s, delta_calc={t3 - t2:.4f}s, "
                            f"distance_calc={t4 - t3:.4f}s, velocity_calc={t5 - t4:.4f}s, "
                            f"mavlink_send={t6 - t5:.4f}s, total_loop={t6 - loop_start:.4f}s"
                        )
                else:
                    logger.info("DEBUG: Too close to send velocity commands")

            else:
                logger.warning("DEBUG: No GLOBAL_POSITION_INT in telemetry!")

            time.sleep(0.1)  # Send at 10Hz for smooth control

        logger.info("Stopped sending GPS velocity commands")
        return True

    def set_mode(self, mode: str) -> bool:
        """Set flight mode."""
        if not self.connected:
            return False

        mode_mapping = {
            "STABILIZE": 0,
            "GUIDED": 4,
            "LOITER": 5,
            "RTL": 6,
            "LAND": 9,
            "POSHOLD": 16,
        }

        if mode not in mode_mapping:
            logger.error(f"Unknown mode: {mode}")
            return False

        mode_id = mode_mapping[mode]
        logger.info(f"Setting mode to {mode}")

        self.update_telemetry()

        self.mavlink.mav.command_long_send(
            self.mavlink.target_system,
            self.mavlink.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
            0,
            0,
            0,
            0,
            0,
        )

        ack = self.mavlink.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
        if ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            logger.info(f"Mode changed to {mode}")
            self.telemetry["mode"] = mode_id
            return True

        return False

    @functools.cache
    def odom_stream(self):
        """Get odometry stream."""
        return self._odom_subject

    @functools.cache
    def status_stream(self):
        """Get status stream."""
        return self._status_subject

    @functools.cache
    def telemetry_stream(self):
        """Get full telemetry stream."""
        return self._telemetry_subject

    def get_telemetry(self) -> Dict[str, Any]:
        """Get current telemetry."""
        # Update telemetry multiple times to ensure we get data
        for _ in range(5):
            self.update_telemetry(timeout=0.2)
        return self.telemetry.copy()

    def disconnect(self):
        """Disconnect from drone."""
        if self.mavlink:
            self.mavlink.close()
        self.connected = False
        logger.info("Disconnected")

    def get_video_stream(self, fps: int = 30):
        """Get video stream (to be implemented with GStreamer)."""
        # Will be implemented in camera module
        return None


class FakeMavlinkConnection(MavlinkConnection):
    """Replay MAVLink for testing."""

    def __init__(self, connection_string: str):
        # Call parent init (which no longer calls connect())
        super().__init__(connection_string)

        # Create fake mavlink object
        class FakeMavlink:
            def __init__(self):
                from dimos.utils.testing import TimedSensorReplay
                from dimos.utils.data import get_data

                get_data("drone")

                self.replay = TimedSensorReplay("drone/mavlink")
                self.messages = []
                # The stream() method returns an Observable that emits messages with timing
                self.replay.stream().subscribe(self.messages.append)

                # Properties that get accessed
                self.target_system = 1
                self.target_component = 1
                self.mav = self  # self.mavlink.mav is used in many places

            def recv_match(self, blocking=False, type=None, timeout=None):
                """Return next replay message as fake message object."""
                if not self.messages:
                    return None

                msg_dict = self.messages.pop(0)

                # Create message object with ALL attributes that might be accessed
                class FakeMsg:
                    def __init__(self, d):
                        self._dict = d
                        # Set any direct attributes that get accessed
                        self.base_mode = d.get("base_mode", 0)
                        self.command = d.get("command", 0)
                        self.result = d.get("result", 0)

                    def get_type(self):
                        return self._dict.get("mavpackettype", "")

                    def to_dict(self):
                        return self._dict

                # Filter by type if requested
                if type and msg_dict.get("type") != type:
                    return None

                return FakeMsg(msg_dict)

            def wait_heartbeat(self, timeout=30):
                """Fake heartbeat received."""
                pass

            def close(self):
                """Fake close."""
                pass

            # Command methods that get called but don't need to do anything in replay
            def command_long_send(self, *args, **kwargs):
                pass

            def set_position_target_local_ned_send(self, *args, **kwargs):
                pass

            def set_position_target_global_int_send(self, *args, **kwargs):
                pass

        # Set up fake mavlink
        self.mavlink = FakeMavlink()
        self.connected = True

        # Initialize position tracking (parent __init__ doesn't do this since connect wasn't called)
        self._position = {"x": 0.0, "y": 0.0, "z": 0.0}
        self._last_update = time.time()
