#!/usr/bin/env python3
# Copyright 2025 Dimensional Inc.

"""Production tests for drone module."""

import unittest
import time
from unittest.mock import MagicMock, patch

from dimos.robot.drone.connection import DroneConnection
from dimos.robot.drone.connection_module import DroneConnectionModule
from dimos.msgs.geometry_msgs import Vector3, PoseStamped


class TestDroneConnection(unittest.TestCase):
    """Test DroneConnection class."""
    
    def test_connection_init(self):
        """Test connection initialization."""
        with patch('pymavlink.mavutil.mavlink_connection') as mock_conn:
            mock_master = MagicMock()
            mock_master.wait_heartbeat.return_value = True
            mock_master.target_system = 1
            mock_conn.return_value = mock_master
            
            conn = DroneConnection('udp:0.0.0.0:14550')
            
            self.assertTrue(conn.connected)
            mock_conn.assert_called_once_with('udp:0.0.0.0:14550')
    
    def test_telemetry_update(self):
        """Test telemetry update."""
        with patch('pymavlink.mavutil.mavlink_connection') as mock_conn:
            mock_master = MagicMock()
            mock_master.wait_heartbeat.return_value = True
            mock_master.target_system = 1
            
            # Mock attitude message
            mock_msg = MagicMock()
            mock_msg.get_type.return_value = 'ATTITUDE'
            mock_msg.roll = 0.1
            mock_msg.pitch = 0.2
            mock_msg.yaw = 0.3
            
            mock_master.recv_match.return_value = mock_msg
            mock_conn.return_value = mock_master
            
            conn = DroneConnection('udp:0.0.0.0:14550')
            conn.update_telemetry(timeout=0.1)
            
            self.assertEqual(conn.telemetry['roll'], 0.1)
            self.assertEqual(conn.telemetry['pitch'], 0.2)
            self.assertEqual(conn.telemetry['yaw'], 0.3)
    
    def test_move_command(self):
        """Test movement command."""
        with patch('pymavlink.mavutil.mavlink_connection') as mock_conn:
            mock_master = MagicMock()
            mock_master.wait_heartbeat.return_value = True
            mock_master.target_system = 1
            mock_master.target_component = 1
            mock_conn.return_value = mock_master
            
            conn = DroneConnection('udp:0.0.0.0:14550')
            
            # Test move command
            velocity = Vector3(1.0, 2.0, 3.0)
            result = conn.move(velocity, duration=0)
            
            self.assertTrue(result)
            mock_master.mav.set_position_target_local_ned_send.assert_called()


class TestDroneConnectionModule(unittest.TestCase):
    """Test DroneConnectionModule."""
    
    def test_module_init(self):
        """Test module initialization."""
        module = DroneConnectionModule(connection_string='udp:0.0.0.0:14550')
        
        self.assertEqual(module.connection_string, 'udp:0.0.0.0:14550')
        self.assertIsNotNone(module.odom)
        self.assertIsNotNone(module.status)
        self.assertIsNotNone(module.movecmd)
    
    def test_get_odom(self):
        """Test get_odom RPC method."""
        module = DroneConnectionModule()
        
        # Initially None
        self.assertIsNone(module.get_odom())
        
        # Set odom
        test_pose = PoseStamped(
            position=Vector3(1, 2, 3),
            frame_id="world"
        )
        module._odom = test_pose
        
        result = module.get_odom()
        self.assertEqual(result.position.x, 1)
        self.assertEqual(result.position.y, 2)
        self.assertEqual(result.position.z, 3)


class TestVideoStream(unittest.TestCase):
    """Test video streaming."""
    
    def test_video_stream_init(self):
        """Test video stream initialization."""
        from dimos.robot.drone.video_stream import DroneVideoStream
        
        stream = DroneVideoStream(port=5600)
        self.assertEqual(stream.port, 5600)
        self.assertFalse(stream._running)
    
    @patch('subprocess.Popen')
    def test_video_stream_start(self, mock_popen):
        """Test video stream start."""
        from dimos.robot.drone.video_stream import DroneVideoStream
        
        mock_process = MagicMock()
        mock_popen.return_value = mock_process
        
        stream = DroneVideoStream(port=5600)
        result = stream.start()
        
        self.assertTrue(result)
        self.assertTrue(stream._running)
        mock_popen.assert_called_once()
        
        # Check gst-launch command was used
        call_args = mock_popen.call_args[0][0]
        self.assertEqual(call_args[0], 'gst-launch-1.0')
        self.assertIn('port=5600', call_args)


if __name__ == '__main__':
    unittest.main()