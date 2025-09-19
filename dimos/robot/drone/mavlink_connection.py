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

logger = setup_logger(__name__, level=logging.DEBUG)


class MavlinkConnection:
    """MAVLink connection for drone control."""

    def __init__(self, connection_string: str = "udp:0.0.0.0:14550"):
        """Initialize drone connection.

        Args:
            connection_string: MAVLink connection string
        """
        self.connection_string = connection_string
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
            print("MESSAGE", msg_dict)
            print("MESSAGE TYPE", msg_type)

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
        """Publish odometry data with velocity integration for indoor flight."""
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
            logger.debug(f"No attitude or position data available")
            return

        # MAVLink --> ROS conversion
        # MAVLink: positive pitch = nose up, positive yaw = clockwise
        # ROS: positive pitch = nose down, positive yaw = counter-clockwise
        quaternion = Quaternion.from_euler(Vector3(roll, -pitch, -yaw))

        if not hasattr(self, "_position"):
            self._position = {"x": 0.0, "y": 0.0, "z": 0.0}
            self._last_update = time.time()

        # Use velocity integration when GPS is invalid / indoor flight
        current_time = time.time()
        dt = current_time - self._last_update

        # Get position data from GLOBAL_POSITION_INT
        pos_data = self.telemetry.get("GLOBAL_POSITION_INT", {})

        # Integrate velocities to update position (NED frame)
        if pos_data and dt > 0:
            vx = pos_data.get("vx", 0)  # North velocity in m/s
            vy = pos_data.get("vy", 0)  # East velocity in m/s

            # +vx is North, +vy is East in NED mavlink frame
            # ROS/Foxglove: X=forward(North), Y=left(West), Z=up
            self._position["x"] += vx * dt  # North → X (forward)
            self._position["y"] += -vy * dt  # East → -Y (right in ROS, Y points left/West)

        if "ALTITUDE" in self.telemetry:
            self._position["z"] = self.telemetry["ALTITUDE"].get("altitude_relative", 0)
        elif pos_data:
            self._position["z"] = pos_data.get("relative_alt", 0)

        self._last_update = current_time

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
                            self.telemetry["armed"] = True
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
        self.telemetry["armed"] = False
        return True

    def takeoff(self, altitude: float = 3.0) -> bool:
        """Takeoff to specified altitude."""
        if not self.connected:
            return False

        logger.info(f"Taking off to {altitude}m...")

        # Set GUIDED mode
        if not self.set_mode("GUIDED"):
            return False

        # Ensure armed
        self.update_telemetry()
        if not self.telemetry.get("armed", False):
            if not self.arm():
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

        ack = self.mavlink.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
        if ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            logger.info("Takeoff command accepted")
            return True

        logger.error("Takeoff failed")
        return False

    def land(self) -> bool:
        """Land the drone."""
        if not self.connected:
            return False

        logger.info("Landing...")

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

        ack = self.mavlink.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
        if ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            logger.info("Land command accepted")
            return True

        # Fallback to LAND mode
        logger.info("Trying LAND mode as fallback")
        return self.set_mode("LAND")

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

        # Set up fake mavlink
        self.mavlink = FakeMavlink()
        self.connected = True

        # Initialize position tracking (parent __init__ doesn't do this since connect wasn't called)
        self._position = {"x": 0.0, "y": 0.0, "z": 0.0}
        self._last_update = time.time()
