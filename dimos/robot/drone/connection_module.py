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
from typing import Optional

from dimos.core import In, Module, Out, rpc
from dimos.msgs.geometry_msgs import PoseStamped, Transform, Vector3, Quaternion
from dimos.msgs.sensor_msgs import Image
from dimos_lcm.std_msgs import String
from dimos.robot.drone.mavlink_connection import MavlinkConnection
from dimos.utils.logging_config import setup_logger

logger = setup_logger(__name__)

from dimos.robot.drone.dji_video_stream import DJIDroneVideoStream


class DroneConnectionModule(Module):
    """Module that handles drone sensor data and movement commands."""
    
    # Inputs
    movecmd: In[Vector3] = None
    
    # Outputs
    odom: Out[PoseStamped] = None
    status: Out[String] = None  # JSON status
    telemetry: Out[String] = None  # Full telemetry JSON
    video: Out[Image] = None
    
    # Parameters
    connection_string: str
    
    # Internal state
    _odom: Optional[PoseStamped] = None
    _status: dict = {}
    
    def __init__(self, connection_string: str = 'udp:0.0.0.0:14550', video_port: int = 5600, *args, **kwargs):
        """Initialize drone connection module.
        
        Args:
            connection_string: MAVLink connection string
            video_port: UDP port for video stream
        """
        self.connection_string = connection_string
        self.video_port = video_port
        self.connection = None
        self.video_stream = None
        Module.__init__(self, *args, **kwargs)
    
    @rpc
    def start(self):
        """Start the connection and subscribe to sensor streams."""
        # Check for replay mode
        if self.connection_string == 'replay':
            from dimos.robot.drone.mavlink_connection import FakeMavlinkConnection
            from dimos.robot.drone.dji_video_stream import FakeDJIVideoStream
            
            self.connection = FakeMavlinkConnection('replay')
            self.video_stream = FakeDJIVideoStream(port=self.video_port)
        else:
            self.connection = MavlinkConnection(self.connection_string)
            self.connection.connect()
            
            self.video_stream = DJIDroneVideoStream(port=self.video_port)
        
        if not self.connection.connected:
            logger.error("Failed to connect to drone")
            return False
        
        # Start video stream (already created above)
        if self.video_stream.start():
            logger.info("Video stream started")
            # Subscribe to video and publish it - pass method directly like Unitree does
            self._video_subscription = self.video_stream.get_stream().subscribe(self.video.publish)
            
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
        
        # Start telemetry update thread
        import threading
        self._running = True
        self._telemetry_thread = threading.Thread(target=self._telemetry_loop, daemon=True)
        self._telemetry_thread.start()
        
        logger.info("Drone connection module started")
        return True
    
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
            ts=msg.ts if hasattr(msg, 'ts') else time.time()
        )
        self.tf.publish(base_link)
        
        # Publish camera_link transform (camera mounted on front of drone, no gimbal factored in yet)
        camera_link = Transform(
            translation=Vector3(0.1, 0.0, -0.05),  # 10cm forward, 5cm down
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),  # No rotation relative to base
            frame_id="base_link",
            child_frame_id="camera_link",
            ts=time.time()
        )
        self.tf.publish(camera_link)
    
    def _publish_status(self, status: dict):
        """Publish drone status as JSON string."""
        self._status = status
        import json
        status_str = String(json.dumps(status))
        self.status.publish(status_str)
    
    def _publish_telemetry(self, telemetry: dict):
        """Publish full telemetry as JSON string."""
        import json
        telemetry_str = String(json.dumps(telemetry))
        self.telemetry.publish(telemetry_str)
    
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
                            ts=time.time()
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
    
    @rpc
    def move(self, vector: Vector3, duration: float = 0.0):
        """Send movement command to drone.
        
        Args:
            vector: Velocity vector [x, y, z] in m/s
            duration: How long to move (0 = continuous)
        """
        if self.connection:
            self.connection.move(vector, duration)
    
    @rpc
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
    
    @rpc
    def land(self) -> bool:
        """Land the drone.
        
        Returns:
            True if land command sent
        """
        if self.connection:
            return self.connection.land()
        return False
    
    @rpc
    def arm(self) -> bool:
        """Arm the drone.
        
        Returns:
            True if armed successfully
        """
        if self.connection:
            return self.connection.arm()
        return False
    
    @rpc
    def disarm(self) -> bool:
        """Disarm the drone.
        
        Returns:
            True if disarmed successfully
        """
        if self.connection:
            return self.connection.disarm()
        return False
    
    @rpc
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
    
    @rpc
    def stop(self):
        """Stop the module."""
        self._running = False
        if self.video_stream:
            self.video_stream.stop()
        if self.connection:
            self.connection.disconnect()
        logger.info("Drone connection module stopped")