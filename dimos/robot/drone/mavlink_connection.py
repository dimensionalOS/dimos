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
from typing import Any

from pymavlink import mavutil  # type: ignore[import-untyped,import-not-found]
from reactivex import Subject

from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Twist, Vector3
from dimos.utils.logging_config import setup_logger

logger = setup_logger(level=logging.INFO)


class MavlinkConnection:
    """MAVLink connection for drone control."""

    def __init__(
        self,
        connection_string: str = "udp:0.0.0.0:14550",
        outdoor: bool = False,
        max_velocity: float = 5.0,
    ) -> None:
        """Initialize drone connection.

        Args:
            connection_string: MAVLink connection string
            outdoor: Use GPS only mode (no velocity integration)
            max_velocity: Maximum velocity in m/s
        """
        self.connection_string = connection_string
        self.outdoor = outdoor
        self.max_velocity = max_velocity
        self.mavlink: Any = None  # MAVLink connection object
        self.connected = False
        self.telemetry: dict[str, Any] = {}

        self._odom_subject: Subject[PoseStamped] = Subject()
        self._status_subject: Subject[dict[str, Any]] = Subject()
        self._telemetry_subject: Subject[dict[str, Any]] = Subject()
        self._raw_mavlink_subject: Subject[dict[str, Any]] = Subject()

        # Velocity tracking for smoothing
        self.prev_vx = 0.0
        self.prev_vy = 0.0
        self.prev_vz = 0.0

        # Flag to prevent concurrent fly_to commands
        self.flying_to_target = False

    def connect(self) -> bool:
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

    def update_telemetry(self, timeout: float = 0.1) -> None:
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
                bool(msg_dict.get("base_mode", 0) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                # print("HEARTBEAT:", msg_dict, "ARMED:", armed)
            # print("MESSAGE", msg_dict)
            # print("MESSAGE TYPE", msg_type)
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

    def _publish_odom(self) -> None:
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

    def _publish_status(self) -> None:
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

    def _publish_telemetry(self) -> None:
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
        down = velocity.z  # Up/down (negative for DOWN, positive for UP)

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

    def move_twist(self, twist: Twist, duration: float = 0.0, lock_altitude: bool = True) -> bool:
        """Move using ROS-style Twist commands.

        Args:
            twist: Twist message with linear velocities (angular.z ignored for now)
            duration: How long to move (0 = single command)
            lock_altitude: If True, ignore Z velocity and maintain current altitude

        Returns:
            True if command sent successfully
        """
        if not self.connected:
            return False

        # Extract velocities
        forward = twist.linear.x  # m/s forward (body frame)
        right = twist.linear.y  # m/s right (body frame)
        down = 0.0 if lock_altitude else -twist.linear.z  # Lock altitude by default

        if duration > 0:
            # Send velocity for duration
            end_time = time.time() + duration
            while time.time() < end_time:
                self.mavlink.mav.set_position_target_local_ned_send(
                    0,  # time_boot_ms
                    self.mavlink.target_system,
                    self.mavlink.target_component,
                    mavutil.mavlink.MAV_FRAME_BODY_NED,  # Body frame for strafing
                    0b0000111111000111,  # type_mask - velocities only, no rotation
                    0,
                    0,
                    0,  # positions (ignored)
                    forward,
                    right,
                    down,  # velocities in m/s
                    0,
                    0,
                    0,  # accelerations (ignored)
                    0,
                    0,  # yaw, yaw_rate (ignored)
                )
                time.sleep(0.05)  # 20Hz
            # Send stop command
            self.stop()
        else:
            # Send single command for continuous movement
            self.mavlink.mav.set_position_target_local_ned_send(
                0,  # time_boot_ms
                self.mavlink.target_system,
                self.mavlink.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_NED,  # Body frame for strafing
                0b0000111111000111,  # type_mask - velocities only, no rotation
                0,
                0,
                0,  # positions (ignored)
                forward,
                right,
                down,  # velocities in m/s
                0,
                0,
                0,  # accelerations (ignored)
                0,
                0,  # yaw, yaw_rate (ignored)
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

    def rotate_to(self, target_heading_deg: float, timeout: float = 60.0) -> bool:
        """Rotate drone to face a specific heading.

        Args:
            target_heading_deg: Target heading in degrees (0-360, 0=North, 90=East)
            timeout: Maximum time to spend rotating in seconds

        Returns:
            True if rotation completed successfully
        """
        if not self.connected:
            return False

        logger.info(f"Rotating to heading {target_heading_deg:.1f}°")

        import math
        import time

        start_time = time.time()
        loop_count = 0

        while time.time() - start_time < timeout:
            loop_count += 1

            # Don't call update_telemetry - let background thread handle it
            # Just read the current telemetry which should be continuously updated

            if "GLOBAL_POSITION_INT" not in self.telemetry:
                logger.warning("No GLOBAL_POSITION_INT in telemetry dict")
                time.sleep(0.1)
                continue

            # Debug: Log what's in telemetry
            gps_telem = self.telemetry["GLOBAL_POSITION_INT"]

            # Get current heading - check if already converted or still in centidegrees
            raw_hdg = gps_telem.get("hdg", 0)

            # Debug logging to figure out the issue
            if loop_count % 5 == 0:  # Log every 5th iteration
                logger.info(f"DEBUG TELEMETRY: raw hdg={raw_hdg}, type={type(raw_hdg)}")
                logger.info(f"DEBUG TELEMETRY keys: {list(gps_telem.keys())[:5]}")  # First 5 keys

                # Check if hdg is already converted (should be < 360 if in degrees, > 360 if in centidegrees)
                if raw_hdg > 360:
                    logger.info(f"HDG appears to be in centidegrees: {raw_hdg}")
                    current_heading_deg = raw_hdg / 100.0
                else:
                    logger.info(f"HDG appears to be in degrees already: {raw_hdg}")
                    current_heading_deg = raw_hdg
            else:
                # Normal conversion
                if raw_hdg > 360:
                    current_heading_deg = raw_hdg / 100.0
                else:
                    current_heading_deg = raw_hdg

            # Normalize to 0-360
            if current_heading_deg > 360:
                current_heading_deg = current_heading_deg % 360

            # Calculate heading error (shortest angular distance)
            heading_error = target_heading_deg - current_heading_deg
            if heading_error > 180:
                heading_error -= 360
            elif heading_error < -180:
                heading_error += 360

            logger.info(
                f"ROTATION: current={current_heading_deg:.1f}° → target={target_heading_deg:.1f}° (error={heading_error:.1f}°)"
            )

            # Check if we're close enough
            if abs(heading_error) < 10:  # Complete within 10 degrees
                logger.info(
                    f"ROTATION COMPLETE: current={current_heading_deg:.1f}° ≈ target={target_heading_deg:.1f}° (within {abs(heading_error):.1f}°)"
                )
                # Don't stop - let fly_to immediately transition to forward movement
                return True

            # Calculate yaw rate with minimum speed to avoid slow approach
            yaw_rate = heading_error * 0.3  # Higher gain for faster rotation
            # Ensure minimum rotation speed of 15 deg/s to avoid crawling near target
            if abs(yaw_rate) < 15.0:
                yaw_rate = 15.0 if heading_error > 0 else -15.0
            yaw_rate = max(-60.0, min(60.0, yaw_rate))  # Cap at 60 deg/s max
            yaw_rate_rad = math.radians(yaw_rate)

            logger.info(
                f"ROTATING: yaw_rate={yaw_rate:.1f} deg/s to go from {current_heading_deg:.1f}° → {target_heading_deg:.1f}°"
            )

            # Send rotation command
            self.mavlink.mav.set_position_target_local_ned_send(
                0,  # time_boot_ms
                self.mavlink.target_system,
                self.mavlink.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_NED,  # Body frame for rotation
                0b0000011111111111,  # type_mask - ignore everything except yaw_rate
                0,
                0,
                0,  # positions (ignored)
                0,
                0,
                0,  # velocities (ignored)
                0,
                0,
                0,  # accelerations (ignored)
                0,  # yaw (ignored)
                yaw_rate_rad,  # yaw_rate in rad/s
            )

            time.sleep(0.1)  # 10Hz control loop

        logger.warning("Rotation timeout")
        self.stop()
        return False

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
                for _i in range(10):
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

    def fly_to(self, lat: float, lon: float, alt: float) -> str:
        """Fly to GPS coordinates - sends commands continuously until reaching target.

        Args:
            lat: Latitude in degrees
            lon: Longitude in degrees
            alt: Altitude in meters (relative to home)

        Returns:
            String message indicating success or failure reason
        """
        if not self.connected:
            return "Failed: Not connected to drone"

        # Check if already flying to a target
        if self.flying_to_target:
            logger.warning(
                "Already flying to target, ignoring new fly_to command. Wait until completed to send new fly_to command."
            )
            return (
                "Already flying to target - wait for completion before sending new fly_to command"
            )

        self.flying_to_target = True

        # Ensure GUIDED mode for GPS navigation
        if not self.set_mode("GUIDED"):
            logger.error("Failed to set GUIDED mode for GPS navigation")
            self.flying_to_target = False
            return "Failed: Could not set GUIDED mode for GPS navigation"

        logger.info(f"Flying to GPS: lat={lat:.7f}, lon={lon:.7f}, alt={alt:.1f}m")

        # Reset velocity tracking for smooth start
        self.prev_vx = 0.0
        self.prev_vy = 0.0
        self.prev_vz = 0.0

        # Send velocity commands towards GPS target at 10Hz
        acceptance_radius = 30.0  # meters
        max_duration = 120  # seconds max flight time
        start_time = time.time()
        max_speed = self.max_velocity  # m/s max speed

        import math

        loop_count = 0

        try:
            while time.time() - start_time < max_duration:
                loop_start = time.time()

                # Don't update telemetry here - let background thread handle it
                # self.update_telemetry(timeout=0.01)  # Removed to prevent message conflicts

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
                    logger.info(
                        f"DEBUG: Target GPS:  lat={lat:.10f}, lon={lon:.10f}, alt={alt:.2f}m"
                    )

                    # Calculate vector to target with high precision
                    dlat = lat - current_lat
                    dlon = lon - current_lon
                    dalt = alt - current_alt

                    logger.info(
                        f"DEBUG: Delta: dlat={dlat:.10f}, dlon={dlon:.10f}, dalt={dalt:.2f}m"
                    )

                    t3 = time.time()

                    # Convert lat/lon difference to meters with high precision
                    # Using more accurate calculation
                    lat_rad = current_lat * math.pi / 180.0
                    meters_per_degree_lat = (
                        111132.92 - 559.82 * math.cos(2 * lat_rad) + 1.175 * math.cos(4 * lat_rad)
                    )
                    meters_per_degree_lon = 111412.84 * math.cos(lat_rad) - 93.5 * math.cos(
                        3 * lat_rad
                    )

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
                        # Return to manual control
                        self.set_mode("STABILIZE")
                        logger.info("Returned to STABILIZE mode for manual control")
                        self.flying_to_target = False
                        return f"Success: Reached target location (lat={lat:.7f}, lon={lon:.7f}, alt={alt:.1f}m)"

                    # Only send velocity commands if we're far enough
                    if distance > 0.1:
                        # On first loop, rotate to face the target
                        if loop_count == 0:
                            # Calculate bearing to target
                            bearing_rad = math.atan2(
                                y_dist, x_dist
                            )  # East, North -> angle from North
                            target_heading_deg = math.degrees(bearing_rad)
                            if target_heading_deg < 0:
                                target_heading_deg += 360

                            logger.info(
                                f"Rotating to face target at heading {target_heading_deg:.1f}°"
                            )
                            self.rotate_to(target_heading_deg, timeout=45.0)
                            logger.info("Rotation complete, starting movement")

                        # Now just move towards target (no rotation)
                        t5 = time.time()

                        # Calculate movement speed - maintain max speed until 20m from target
                        if distance > 20:
                            speed = max_speed  # Full speed when far from target
                        else:
                            # Ramp down speed from 20m to target
                            speed = max(
                                0.5, distance / 4.0
                            )  # At 20m: 5m/s, at 10m: 2.5m/s, at 2m: 0.5m/s

                        # Calculate target velocities
                        target_vx = (x_dist / distance) * speed  # North velocity
                        target_vy = (y_dist / distance) * speed  # East velocity
                        target_vz = (dalt / distance) * speed  # Up velocity (positive = up)

                        # Direct velocity assignment (no acceleration limiting)
                        vx = target_vx
                        vy = target_vy
                        vz = target_vz

                        # Store for next iteration
                        self.prev_vx = vx
                        self.prev_vy = vy
                        self.prev_vz = vz

                        logger.info(
                            f"MOVING: vx={vx:.3f} vy={vy:.3f} vz={vz:.3f} m/s, distance={distance:.1f}m"
                        )

                        # Send velocity command in LOCAL_NED frame
                        self.mavlink.mav.set_position_target_local_ned_send(
                            0,  # time_boot_ms
                            self.mavlink.target_system,
                            self.mavlink.target_component,
                            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # Local NED for movement
                            0b0000111111000111,  # type_mask - use velocities only
                            0,
                            0,
                            0,  # positions (not used)
                            vx,
                            vy,
                            vz,  # velocities in m/s
                            0,
                            0,
                            0,  # accelerations (not used)
                            0,  # yaw (not used)
                            0,  # yaw_rate (not used)
                        )

                        # Log if stuck
                        if loop_count > 20 and loop_count % 10 == 0:
                            logger.warning(
                                f"STUCK? Been sending commands for {loop_count} iterations but distance still {distance:.1f}m"
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

                time.sleep(0.1)  # Send at 10Hz

        except Exception as e:
            logger.error(f"Error during fly_to: {e}")
            self.flying_to_target = False  # Clear flag immediately
            raise  # Re-raise the exception so caller sees the error
        finally:
            # Always clear the flag when exiting
            if self.flying_to_target:
                logger.info("Stopped sending GPS velocity commands (timeout)")
                self.flying_to_target = False
                self.set_mode("BRAKE")
                time.sleep(0.5)
                # Return to manual control
                self.set_mode("STABILIZE")
                logger.info("Returned to STABILIZE mode for manual control")

        return "Failed: Timeout - did not reach target within 120 seconds"

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
            "BRAKE": 17,
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
    def odom_stream(self) -> Subject[PoseStamped]:
        """Get odometry stream."""
        return self._odom_subject

    @functools.cache
    def status_stream(self) -> Subject[dict[str, Any]]:
        """Get status stream."""
        return self._status_subject

    @functools.cache
    def telemetry_stream(self) -> Subject[dict[str, Any]]:
        """Get full telemetry stream."""
        return self._telemetry_subject

    def get_telemetry(self) -> dict[str, Any]:
        """Get current telemetry."""
        # Update telemetry multiple times to ensure we get data
        for _ in range(5):
            self.update_telemetry(timeout=0.2)
        return self.telemetry.copy()

    def disconnect(self) -> None:
        """Disconnect from drone."""
        if self.mavlink:
            self.mavlink.close()
        self.connected = False
        logger.info("Disconnected")

    @property
    def is_flying_to_target(self) -> bool:
        """Check if drone is currently flying to a GPS target."""
        return self.flying_to_target

    def get_video_stream(self, fps: int = 30) -> None:
        """Get video stream (to be implemented with GStreamer)."""
        # Will be implemented in camera module
        return None


class FakeMavlinkConnection(MavlinkConnection):
    """Replay MAVLink for testing."""

    def __init__(self, connection_string: str) -> None:
        # Call parent init (which no longer calls connect())
        super().__init__(connection_string)

        # Create fake mavlink object
        class FakeMavlink:
            def __init__(self) -> None:
                from dimos.utils.data import get_data
                from dimos.utils.testing import TimedSensorReplay

                get_data("drone")

                self.replay: Any = TimedSensorReplay("drone/mavlink")
                self.messages: list[dict[str, Any]] = []
                # The stream() method returns an Observable that emits messages with timing
                self.replay.stream().subscribe(self.messages.append)

                # Properties that get accessed
                self.target_system = 1
                self.target_component = 1
                self.mav = self  # self.mavlink.mav is used in many places

            def recv_match(
                self, blocking: bool = False, type: Any = None, timeout: Any = None
            ) -> Any:
                """Return next replay message as fake message object."""
                if not self.messages:
                    return None

                msg_dict = self.messages.pop(0)

                # Create message object with ALL attributes that might be accessed
                class FakeMsg:
                    def __init__(self, d: dict[str, Any]) -> None:
                        self._dict = d
                        # Set any direct attributes that get accessed
                        self.base_mode = d.get("base_mode", 0)
                        self.command = d.get("command", 0)
                        self.result = d.get("result", 0)

                    def get_type(self) -> Any:
                        return self._dict.get("mavpackettype", "")

                    def to_dict(self) -> dict[str, Any]:
                        return self._dict

                # Filter by type if requested
                if type and msg_dict.get("type") != type:
                    return None

                return FakeMsg(msg_dict)

            def wait_heartbeat(self, timeout: int = 30) -> None:
                """Fake heartbeat received."""
                pass

            def close(self) -> None:
                """Fake close."""
                pass

            # Command methods that get called but don't need to do anything in replay
            def command_long_send(self, *args: Any, **kwargs: Any) -> None:
                pass

            def set_position_target_local_ned_send(self, *args: Any, **kwargs: Any) -> None:
                pass

            def set_position_target_global_int_send(self, *args: Any, **kwargs: Any) -> None:
                pass

        # Set up fake mavlink
        self.mavlink = FakeMavlink()
        self.connected = True

        # Initialize position tracking (parent __init__ doesn't do this since connect wasn't called)
        self._position = {"x": 0.0, "y": 0.0, "z": 0.0}
        self._last_update = time.time()

    def takeoff(self, altitude: float = 3.0) -> bool:
        """Fake takeoff - return immediately without blocking."""
        logger.info(f"[FAKE] Taking off to {altitude}m...")
        return True

    def land(self) -> bool:
        """Fake land - return immediately without blocking."""
        logger.info("[FAKE] Landing...")
        return True
