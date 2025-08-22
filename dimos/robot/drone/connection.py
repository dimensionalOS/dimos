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

logger = setup_logger(__name__, level=logging.INFO)


class DroneConnection(ConnectionInterface):
    """MAVLink connection for drone control."""
    
    def __init__(self, connection_string: str = 'udp:0.0.0.0:14550'):
        """Initialize drone connection.
        
        Args:
            connection_string: MAVLink connection string
        """
        self.connection_string = connection_string
        self.master = None
        self.connected = False
        self.telemetry = {}
        
        # Subjects for streaming data
        self._odom_subject = Subject()
        self._status_subject = Subject()
        
        self.connect()
    
    def connect(self):
        """Connect to drone via MAVLink."""
        try:
            logger.info(f"Connecting to {self.connection_string}")
            self.master = mavutil.mavlink_connection(self.connection_string)
            self.master.wait_heartbeat(timeout=30)
            self.connected = True
            logger.info(f"Connected to system {self.master.target_system}")
            
            # Update initial telemetry
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
            msg = self.master.recv_match(
                type=['ATTITUDE', 'GLOBAL_POSITION_INT', 'VFR_HUD', 
                      'HEARTBEAT', 'SYS_STATUS', 'GPS_RAW_INT'],
                blocking=False
            )
            if not msg:
                time.sleep(0.001)  # Small sleep then continue
                continue
                
            msg_type = msg.get_type()
            
            if msg_type == 'ATTITUDE':
                self.telemetry['roll'] = msg.roll
                self.telemetry['pitch'] = msg.pitch
                self.telemetry['yaw'] = msg.yaw
                self._publish_odom()
                
            elif msg_type == 'GLOBAL_POSITION_INT':
                self.telemetry['lat'] = msg.lat / 1e7
                self.telemetry['lon'] = msg.lon / 1e7
                self.telemetry['alt'] = msg.alt / 1000.0
                self.telemetry['relative_alt'] = msg.relative_alt / 1000.0
                self._publish_odom()
                
            elif msg_type == 'VFR_HUD':
                self.telemetry['groundspeed'] = msg.groundspeed
                self.telemetry['airspeed'] = msg.airspeed
                self.telemetry['heading'] = msg.heading
                self.telemetry['climb'] = msg.climb
                
            elif msg_type == 'HEARTBEAT':
                self.telemetry['armed'] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.telemetry['mode'] = msg.custom_mode
                self._publish_status()
                
            elif msg_type == 'SYS_STATUS':
                self.telemetry['battery_voltage'] = msg.voltage_battery / 1000.0
                self.telemetry['battery_current'] = msg.current_battery / 100.0
                self._publish_status()
    
    def _publish_odom(self):
        """Publish odometry data."""
        if not all(k in self.telemetry for k in ['roll', 'pitch', 'yaw']):
            logger.debug(f"Missing telemetry for odom: {self.telemetry.keys()}")
            return
            
        # Convert Euler angles to quaternion
        quaternion = Quaternion.from_euler(
            Vector3(
                self.telemetry.get('roll', 0),
                self.telemetry.get('pitch', 0),
                self.telemetry.get('yaw', 0)
            )
        )
        
        # Create pose with proper local position
        # For now, integrate velocity to get position (proper solution needs GPS->local conversion)
        if not hasattr(self, '_position'):
            self._position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
        # Use altitude directly for Z
        self._position['z'] = self.telemetry.get('relative_alt', 0)
        
        pose = PoseStamped(
            position=Vector3(
                self._position['x'],
                self._position['y'],
                self._position['z']
            ),
            orientation=quaternion,
            frame_id="world",
            ts=time.time()
        )
        
        self._odom_subject.on_next(pose)
    
    def _publish_status(self):
        """Publish drone status."""
        status = {
            'armed': self.telemetry.get('armed', False),
            'mode': self.telemetry.get('mode', -1),
            'battery_voltage': self.telemetry.get('battery_voltage', 0),
            'battery_current': self.telemetry.get('battery_current', 0),
            'ts': time.time()
        }
        self._status_subject.on_next(status)
    
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
        right = velocity.x    # Left/right
        down = -velocity.z    # Up/down (negative for up)
        
        logger.debug(f"Moving: forward={forward}, right={right}, down={down}")
        
        if duration > 0:
            # Send velocity for duration
            end_time = time.time() + duration
            while time.time() < end_time:
                self.master.mav.set_position_target_local_ned_send(
                    0,  # time_boot_ms
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_FRAME_BODY_NED,
                    0b0000111111000111,  # type_mask (only velocities)
                    0, 0, 0,  # positions
                    forward, right, down,  # velocities
                    0, 0, 0,  # accelerations
                    0, 0  # yaw, yaw_rate
                )
                time.sleep(0.1)
            self.stop()
        else:
            # Single velocity command
            self.master.mav.set_position_target_local_ned_send(
                0, self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                0b0000111111000111,
                0, 0, 0, forward, right, down, 0, 0, 0, 0, 0
            )
        
        return True
    
    def stop(self) -> bool:
        """Stop all movement."""
        if not self.connected:
            return False
            
        self.master.mav.set_position_target_local_ned_send(
            0, self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        )
        return True
    
    def arm(self) -> bool:
        """Arm the drone."""
        if not self.connected:
            return False
            
        logger.info("Arming motors...")
        self.update_telemetry()
        
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        
        # Wait for ACK
        ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                logger.info("Arm command accepted")
                
                # Verify armed status
                for i in range(10):
                    msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
                    if msg:
                        armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                        if armed:
                            logger.info("Motors ARMED successfully!")
                            self.telemetry['armed'] = True
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
        
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        
        time.sleep(1)
        self.telemetry['armed'] = False
        return True
    
    def takeoff(self, altitude: float = 3.0) -> bool:
        """Takeoff to specified altitude."""
        if not self.connected:
            return False
            
        logger.info(f"Taking off to {altitude}m...")
        
        # Set GUIDED mode
        if not self.set_mode('GUIDED'):
            return False
        
        # Ensure armed
        self.update_telemetry()
        if not self.telemetry.get('armed', False):
            if not self.arm():
                return False
        
        # Send takeoff command
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, altitude
        )
        
        ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
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
        
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        
        ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            logger.info("Land command accepted")
            return True
        
        # Fallback to LAND mode
        logger.info("Trying LAND mode as fallback")
        return self.set_mode('LAND')
    
    def set_mode(self, mode: str) -> bool:
        """Set flight mode."""
        if not self.connected:
            return False
            
        mode_mapping = {
            'STABILIZE': 0,
            'GUIDED': 4,
            'LOITER': 5,
            'RTL': 6,
            'LAND': 9,
            'POSHOLD': 16
        }
        
        if mode not in mode_mapping:
            logger.error(f"Unknown mode: {mode}")
            return False
        
        mode_id = mode_mapping[mode]
        logger.info(f"Setting mode to {mode}")
        
        self.update_telemetry()
        
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
            0, 0, 0, 0, 0
        )
        
        ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            logger.info(f"Mode changed to {mode}")
            self.telemetry['mode'] = mode_id
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
    
    def get_telemetry(self) -> Dict[str, Any]:
        """Get current telemetry."""
        # Update telemetry multiple times to ensure we get data
        for _ in range(5):
            self.update_telemetry(timeout=0.2)
        return self.telemetry.copy()
    
    def disconnect(self):
        """Disconnect from drone."""
        if self.master:
            self.master.close()
        self.connected = False
        logger.info("Disconnected")
    
    def get_video_stream(self, fps: int = 30):
        """Get video stream (to be implemented with GStreamer)."""
        # Will be implemented in camera module
        return None