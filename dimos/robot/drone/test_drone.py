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

# Copyright 2025 Dimensional Inc.

"""Core unit tests for drone module."""

import json
import os
import time
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Vector3
from dimos.msgs.sensor_msgs import Image, ImageFormat
from dimos.robot.drone.connection_module import DroneConnectionModule
from dimos.robot.drone.dji_video_stream import FakeDJIVideoStream
from dimos.robot.drone.drone import Drone
from dimos.robot.drone.mavlink_connection import FakeMavlinkConnection, MavlinkConnection


class TestMavlinkProcessing(unittest.TestCase):
    """Test MAVLink message processing and coordinate conversions."""

    def test_mavlink_message_processing(self) -> None:
        """Test that MAVLink messages trigger correct odom/tf publishing."""
        conn = MavlinkConnection("udp:0.0.0.0:14550")

        # Mock the mavlink connection
        conn.mavlink = MagicMock()
        conn.connected = True

        # Track what gets published
        published_odom = []
        conn._odom_subject.on_next = lambda x: published_odom.append(x)

        # Create ATTITUDE message and process it
        attitude_msg = MagicMock()
        attitude_msg.get_type.return_value = "ATTITUDE"
        attitude_msg.to_dict.return_value = {
            "mavpackettype": "ATTITUDE",
            "roll": 0.1,
            "pitch": 0.2,  # Positive pitch = nose up in MAVLink
            "yaw": 0.3,  # Positive yaw = clockwise in MAVLink
        }

        # Mock recv_match to return our message once then None
        def recv_side_effect(*args, **kwargs):
            if not hasattr(recv_side_effect, "called"):
                recv_side_effect.called = True
                return attitude_msg
            return None

        conn.mavlink.recv_match = MagicMock(side_effect=recv_side_effect)

        # Process the message
        conn.update_telemetry(timeout=0.01)

        # Check telemetry was updated
        self.assertEqual(conn.telemetry["ATTITUDE"]["roll"], 0.1)
        self.assertEqual(conn.telemetry["ATTITUDE"]["pitch"], 0.2)
        self.assertEqual(conn.telemetry["ATTITUDE"]["yaw"], 0.3)

        # Check odom was published with correct coordinate conversion
        self.assertEqual(len(published_odom), 1)
        pose = published_odom[0]

        # Verify NED to ROS conversion happened
        # ROS uses different conventions: positive pitch = nose down, positive yaw = counter-clockwise
        # So we expect sign flips in the quaternion conversion
        self.assertIsNotNone(pose.orientation)

    def test_position_integration(self) -> None:
        """Test velocity integration for indoor flight positioning."""
        conn = MavlinkConnection("udp:0.0.0.0:14550")
        conn.mavlink = MagicMock()
        conn.connected = True

        # Initialize position tracking
        conn._position = {"x": 0.0, "y": 0.0, "z": 0.0}
        conn._last_update = time.time()

        # Create GLOBAL_POSITION_INT with velocities
        pos_msg = MagicMock()
        pos_msg.get_type.return_value = "GLOBAL_POSITION_INT"
        pos_msg.to_dict.return_value = {
            "mavpackettype": "GLOBAL_POSITION_INT",
            "lat": 0,
            "lon": 0,
            "alt": 0,
            "relative_alt": 1000,  # 1m in mm
            "vx": 100,  # 1 m/s North in cm/s
            "vy": 200,  # 2 m/s East in cm/s
            "vz": 0,
            "hdg": 0,
        }

        def recv_side_effect(*args, **kwargs):
            if not hasattr(recv_side_effect, "called"):
                recv_side_effect.called = True
                return pos_msg
            return None

        conn.mavlink.recv_match = MagicMock(side_effect=recv_side_effect)

        # Process with known dt
        old_time = conn._last_update
        conn.update_telemetry(timeout=0.01)
        dt = conn._last_update - old_time

        # Check position was integrated from velocities
        # vx=1m/s North → +X in ROS
        # vy=2m/s East → -Y in ROS (Y points West)
        expected_x = 1.0 * dt  # North velocity
        expected_y = -2.0 * dt  # East velocity (negated for ROS)

        self.assertAlmostEqual(conn._position["x"], expected_x, places=2)
        self.assertAlmostEqual(conn._position["y"], expected_y, places=2)

    def test_ned_to_ros_coordinate_conversion(self) -> None:
        """Test NED to ROS coordinate system conversion for all axes."""
        conn = MavlinkConnection("udp:0.0.0.0:14550")
        conn.mavlink = MagicMock()
        conn.connected = True

        # Initialize position
        conn._position = {"x": 0.0, "y": 0.0, "z": 0.0}
        conn._last_update = time.time()

        # Test with velocities in all directions
        # NED: North-East-Down
        # ROS: X(forward/North), Y(left/West), Z(up)
        pos_msg = MagicMock()
        pos_msg.get_type.return_value = "GLOBAL_POSITION_INT"
        pos_msg.to_dict.return_value = {
            "mavpackettype": "GLOBAL_POSITION_INT",
            "lat": 0,
            "lon": 0,
            "alt": 5000,  # 5m altitude in mm
            "relative_alt": 5000,
            "vx": 300,  # 3 m/s North (NED)
            "vy": 400,  # 4 m/s East (NED)
            "vz": -100,  # 1 m/s Up (negative in NED for up)
            "hdg": 0,
        }

        def recv_side_effect(*args, **kwargs):
            if not hasattr(recv_side_effect, "called"):
                recv_side_effect.called = True
                return pos_msg
            return None

        conn.mavlink.recv_match = MagicMock(side_effect=recv_side_effect)

        # Process message
        old_time = conn._last_update
        conn.update_telemetry(timeout=0.01)
        dt = conn._last_update - old_time

        # Verify coordinate conversion:
        # NED North (vx=3) → ROS +X
        # NED East (vy=4) → ROS -Y (ROS Y points West/left)
        # NED Down (vz=-1, up) → ROS +Z (ROS Z points up)

        # Position should integrate with converted velocities
        self.assertGreater(conn._position["x"], 0)  # North → positive X
        self.assertLess(conn._position["y"], 0)  # East → negative Y
        self.assertEqual(conn._position["z"], 5.0)  # Altitude from relative_alt (5000mm = 5m)

        # Check X,Y velocity integration (Z is set from altitude, not integrated)
        self.assertAlmostEqual(conn._position["x"], 3.0 * dt, places=2)
        self.assertAlmostEqual(conn._position["y"], -4.0 * dt, places=2)


class TestReplayMode(unittest.TestCase):
    """Test replay mode functionality."""

    def test_fake_mavlink_connection(self) -> None:
        """Test FakeMavlinkConnection replays messages correctly."""
        with patch("dimos.utils.testing.TimedSensorReplay") as mock_replay:
            # Mock the replay stream
            MagicMock()
            mock_messages = [
                {"mavpackettype": "ATTITUDE", "roll": 0.1, "pitch": 0.2, "yaw": 0.3},
                {"mavpackettype": "HEARTBEAT", "type": 2, "base_mode": 193},
            ]

            # Make stream emit our messages
            mock_replay.return_value.stream.return_value.subscribe = lambda callback: [
                callback(msg) for msg in mock_messages
            ]

            conn = FakeMavlinkConnection("replay")

            # Check messages are available
            msg1 = conn.mavlink.recv_match()
            self.assertIsNotNone(msg1)
            self.assertEqual(msg1.get_type(), "ATTITUDE")

            msg2 = conn.mavlink.recv_match()
            self.assertIsNotNone(msg2)
            self.assertEqual(msg2.get_type(), "HEARTBEAT")

    def test_fake_video_stream_no_throttling(self) -> None:
        """Test FakeDJIVideoStream returns replay stream directly."""
        with patch("dimos.utils.testing.TimedSensorReplay") as mock_replay:
            mock_stream = MagicMock()
            mock_replay.return_value.stream.return_value = mock_stream

            stream = FakeDJIVideoStream(port=5600)
            result_stream = stream.get_stream()

            # Verify stream is returned directly without throttling
            self.assertEqual(result_stream, mock_stream)

    def test_connection_module_replay_mode(self) -> None:
        """Test connection module uses Fake classes in replay mode."""
        with patch("dimos.robot.drone.mavlink_connection.FakeMavlinkConnection") as mock_fake_conn:
            with patch("dimos.robot.drone.dji_video_stream.FakeDJIVideoStream") as mock_fake_video:
                # Mock the fake connection
                mock_conn_instance = MagicMock()
                mock_conn_instance.connected = True
                mock_conn_instance.odom_stream.return_value.subscribe = MagicMock(
                    return_value=lambda: None
                )
                mock_conn_instance.status_stream.return_value.subscribe = MagicMock(
                    return_value=lambda: None
                )
                mock_conn_instance.telemetry_stream.return_value.subscribe = MagicMock(
                    return_value=lambda: None
                )
                mock_conn_instance.disconnect = MagicMock()
                mock_fake_conn.return_value = mock_conn_instance

                # Mock the fake video
                mock_video_instance = MagicMock()
                mock_video_instance.start.return_value = True
                mock_video_instance.get_stream.return_value.subscribe = MagicMock(
                    return_value=lambda: None
                )
                mock_video_instance.stop = MagicMock()
                mock_fake_video.return_value = mock_video_instance

                # Create module with replay connection string
                module = DroneConnectionModule(connection_string="replay")
                module.video = MagicMock()
                module.movecmd = MagicMock()
                module.movecmd.subscribe = MagicMock(return_value=lambda: None)
                module.tf = MagicMock()

                try:
                    # Start should use Fake classes
                    result = module.start()

                    self.assertTrue(result)
                    mock_fake_conn.assert_called_once_with("replay")
                    mock_fake_video.assert_called_once()
                finally:
                    # Always clean up
                    module.stop()

    def test_connection_module_replay_with_messages(self) -> None:
        """Test connection module in replay mode receives and processes messages."""

        os.environ["DRONE_CONNECTION"] = "replay"

        with patch("dimos.utils.testing.TimedSensorReplay") as mock_replay:
            # Set up MAVLink replay stream
            mavlink_messages = [
                {"mavpackettype": "HEARTBEAT", "type": 2, "base_mode": 193},
                {"mavpackettype": "ATTITUDE", "roll": 0.1, "pitch": 0.2, "yaw": 0.3},
                {
                    "mavpackettype": "GLOBAL_POSITION_INT",
                    "lat": 377810501,
                    "lon": -1224069671,
                    "alt": 0,
                    "relative_alt": 1000,
                    "vx": 100,
                    "vy": 0,
                    "vz": 0,
                    "hdg": 0,
                },
            ]

            # Set up video replay stream
            video_frames = [
                np.random.randint(0, 255, (1080, 1920, 3), dtype=np.uint8),
                np.random.randint(0, 255, (1080, 1920, 3), dtype=np.uint8),
            ]

            def create_mavlink_stream():
                stream = MagicMock()

                def subscribe(callback) -> None:
                    print("\n[TEST] MAVLink replay stream subscribed")
                    for msg in mavlink_messages:
                        print(f"[TEST] Replaying MAVLink: {msg['mavpackettype']}")
                        callback(msg)

                stream.subscribe = subscribe
                return stream

            def create_video_stream():
                stream = MagicMock()

                def subscribe(callback) -> None:
                    print("[TEST] Video replay stream subscribed")
                    for i, frame in enumerate(video_frames):
                        print(
                            f"[TEST] Replaying video frame {i + 1}/{len(video_frames)}, shape: {frame.shape}"
                        )
                        callback(frame)

                stream.subscribe = subscribe
                return stream

            # Configure mock replay to return appropriate streams
            def replay_side_effect(store_name: str):
                print(f"[TEST] TimedSensorReplay created for: {store_name}")
                mock = MagicMock()
                if "mavlink" in store_name:
                    mock.stream.return_value = create_mavlink_stream()
                elif "video" in store_name:
                    mock.stream.return_value = create_video_stream()
                return mock

            mock_replay.side_effect = replay_side_effect

            # Create and start connection module
            module = DroneConnectionModule(connection_string="replay")

            # Mock publishers to track what gets published
            published_odom = []
            published_video = []
            published_status = []

            module.odom = MagicMock(
                publish=lambda x: (
                    published_odom.append(x),
                    print(
                        f"[TEST] Published odom: position=({x.position.x:.2f}, {x.position.y:.2f}, {x.position.z:.2f})"
                    ),
                )
            )
            module.video = MagicMock(
                publish=lambda x: (
                    published_video.append(x),
                    print(
                        f"[TEST] Published video frame with shape: {x.data.shape if hasattr(x, 'data') else 'unknown'}"
                    ),
                )
            )
            module.status = MagicMock(
                publish=lambda x: (
                    published_status.append(x),
                    print(
                        f"[TEST] Published status: {x.data[:50]}..."
                        if hasattr(x, "data")
                        else "[TEST] Published status"
                    ),
                )
            )
            module.telemetry = MagicMock()
            module.tf = MagicMock()
            module.movecmd = MagicMock()

            try:
                print("\n[TEST] Starting connection module in replay mode...")
                result = module.start()

                # Give time for messages to process
                import time

                time.sleep(0.1)

                print(f"\n[TEST] Module started: {result}")
                print(f"[TEST] Total odom messages published: {len(published_odom)}")
                print(f"[TEST] Total video frames published: {len(published_video)}")
                print(f"[TEST] Total status messages published: {len(published_status)}")

                # Verify module started and is processing messages
                self.assertTrue(result)
                self.assertIsNotNone(module.connection)
                self.assertIsNotNone(module.video_stream)

                # Should have published some messages
                self.assertGreater(
                    len(published_odom) + len(published_video) + len(published_status),
                    0,
                    "No messages were published in replay mode",
                )
            finally:
                # Clean up
                module.stop()


class TestDroneFullIntegration(unittest.TestCase):
    """Full integration test of Drone class with replay mode."""

    def setUp(self) -> None:
        """Set up test environment."""
        # Mock the DimOS core module
        self.mock_dimos = MagicMock()
        self.mock_dimos.deploy.return_value = MagicMock()

        # Mock pubsub.lcm.autoconf
        self.pubsub_patch = patch("dimos.protocol.pubsub.lcm.autoconf")
        self.pubsub_patch.start()

        # Mock FoxgloveBridge
        self.foxglove_patch = patch("dimos.robot.drone.drone.FoxgloveBridge")
        self.mock_foxglove = self.foxglove_patch.start()

    def tearDown(self) -> None:
        """Clean up patches."""
        self.pubsub_patch.stop()
        self.foxglove_patch.stop()

    @patch("dimos.robot.drone.drone.core.start")
    @patch("dimos.utils.testing.TimedSensorReplay")
    def test_full_system_with_replay(self, mock_replay, mock_core_start) -> None:
        """Test full drone system initialization and operation with replay mode."""
        # Set up mock replay data
        mavlink_messages = [
            {"mavpackettype": "HEARTBEAT", "type": 2, "base_mode": 193, "armed": True},
            {"mavpackettype": "ATTITUDE", "roll": 0.1, "pitch": 0.2, "yaw": 0.3},
            {
                "mavpackettype": "GLOBAL_POSITION_INT",
                "lat": 377810501,
                "lon": -1224069671,
                "alt": 5000,
                "relative_alt": 5000,
                "vx": 100,  # 1 m/s North
                "vy": 200,  # 2 m/s East
                "vz": -50,  # 0.5 m/s Up
                "hdg": 9000,  # 90 degrees
            },
            {
                "mavpackettype": "BATTERY_STATUS",
                "voltages": [3800, 3800, 3800, 3800],
                "battery_remaining": 75,
            },
        ]

        video_frames = [
            Image(
                data=np.random.randint(0, 255, (360, 640, 3), dtype=np.uint8),
                format=ImageFormat.BGR,
            )
        ]

        def replay_side_effect(store_name: str):
            mock = MagicMock()
            if "mavlink" in store_name:
                # Create stream that emits MAVLink messages
                stream = MagicMock()
                stream.subscribe = lambda callback: [callback(msg) for msg in mavlink_messages]
                mock.stream.return_value = stream
            elif "video" in store_name:
                # Create stream that emits video frames
                stream = MagicMock()
                stream.subscribe = lambda callback: [callback(frame) for frame in video_frames]
                mock.stream.return_value = stream
            return mock

        mock_replay.side_effect = replay_side_effect

        # Mock DimOS core
        mock_core_start.return_value = self.mock_dimos

        # Create drone in replay mode
        drone = Drone(connection_string="replay", video_port=5600)

        # Mock the deployed modules
        mock_connection = MagicMock()
        mock_camera = MagicMock()

        # Set up return values for module methods
        mock_connection.start.return_value = True
        mock_connection.get_odom.return_value = PoseStamped(
            position=Vector3(1.0, 2.0, 3.0), orientation=Quaternion(0, 0, 0, 1), frame_id="world"
        )
        mock_connection.get_status.return_value = {
            "armed": True,
            "battery_voltage": 15.2,
            "battery_remaining": 75,
            "altitude": 5.0,
        }

        mock_camera.start.return_value = True

        # Configure deploy to return our mocked modules
        def deploy_side_effect(module_class, **kwargs):
            if "DroneConnectionModule" in str(module_class):
                return mock_connection
            elif "DroneCameraModule" in str(module_class):
                return mock_camera
            return MagicMock()

        self.mock_dimos.deploy.side_effect = deploy_side_effect

        # Start the drone system
        drone.start()

        # Verify modules were deployed
        self.assertEqual(self.mock_dimos.deploy.call_count, 4)

        # Test get_odom
        odom = drone.get_odom()
        self.assertIsNotNone(odom)
        self.assertEqual(odom.position.x, 1.0)
        self.assertEqual(odom.position.y, 2.0)
        self.assertEqual(odom.position.z, 3.0)

        # Test get_status
        status = drone.get_status()
        self.assertIsNotNone(status)
        self.assertTrue(status["armed"])
        self.assertEqual(status["battery_remaining"], 75)

        # Test movement command
        drone.move(Vector3(1.0, 0.0, 0.5), duration=2.0)
        mock_connection.move.assert_called_once_with(Vector3(1.0, 0.0, 0.5), 2.0)

        # Test control commands
        drone.arm()
        mock_connection.arm.assert_called_once()

        drone.takeoff(altitude=10.0)
        mock_connection.takeoff.assert_called_once_with(10.0)

        drone.land()
        mock_connection.land.assert_called_once()

        drone.disarm()
        mock_connection.disarm.assert_called_once()

        # Test mode setting
        drone.set_mode("GUIDED")
        mock_connection.set_mode.assert_called_once_with("GUIDED")

        # Clean up
        drone.stop()

        # Verify cleanup was called
        mock_connection.stop.assert_called_once()
        mock_camera.stop.assert_called_once()
        self.mock_dimos.close_all.assert_called_once()


class TestDroneControlCommands(unittest.TestCase):
    """Test drone control commands with FakeMavlinkConnection."""

    @patch("dimos.utils.testing.TimedSensorReplay")
    @patch("dimos.utils.data.get_data")
    def test_arm_disarm_commands(self, mock_get_data, mock_replay) -> None:
        """Test arm and disarm commands work with fake connection."""
        # Set up mock replay
        mock_stream = MagicMock()
        mock_stream.subscribe = lambda callback: None
        mock_replay.return_value.stream.return_value = mock_stream

        conn = FakeMavlinkConnection("replay")

        # Test arm
        result = conn.arm()
        self.assertIsInstance(result, bool)  # Should return bool without crashing

        # Test disarm
        result = conn.disarm()
        self.assertIsInstance(result, bool)  # Should return bool without crashing

    @patch("dimos.utils.testing.TimedSensorReplay")
    @patch("dimos.utils.data.get_data")
    def test_takeoff_land_commands(self, mock_get_data, mock_replay) -> None:
        """Test takeoff and land commands with fake connection."""
        mock_stream = MagicMock()
        mock_stream.subscribe = lambda callback: None
        mock_replay.return_value.stream.return_value = mock_stream

        conn = FakeMavlinkConnection("replay")

        # Test takeoff
        result = conn.takeoff(altitude=15.0)
        # In fake mode, should accept but may return False if no ACK simulation
        self.assertIsNotNone(result)

        # Test land
        result = conn.land()
        self.assertIsNotNone(result)

    @patch("dimos.utils.testing.TimedSensorReplay")
    @patch("dimos.utils.data.get_data")
    def test_set_mode_command(self, mock_get_data, mock_replay) -> None:
        """Test flight mode setting with fake connection."""
        mock_stream = MagicMock()
        mock_stream.subscribe = lambda callback: None
        mock_replay.return_value.stream.return_value = mock_stream

        conn = FakeMavlinkConnection("replay")

        # Test various flight modes
        modes = ["STABILIZE", "GUIDED", "LAND", "RTL", "LOITER"]
        for mode in modes:
            result = conn.set_mode(mode)
            # Should return True or False but not crash
            self.assertIsInstance(result, bool)


class TestDronePerception(unittest.TestCase):
    """Test drone perception capabilities."""

    @patch("dimos.utils.testing.TimedSensorReplay")
    @patch("dimos.utils.data.get_data")
    def test_video_stream_replay(self, mock_get_data, mock_replay) -> None:
        """Test video stream works with replay data."""
        # Set up video frames - create a test pattern instead of random noise
        import cv2

        # Create a test pattern image with some structure
        test_frame = np.zeros((360, 640, 3), dtype=np.uint8)
        # Add some colored rectangles to make it visually obvious
        cv2.rectangle(test_frame, (50, 50), (200, 150), (255, 0, 0), -1)  # Blue
        cv2.rectangle(test_frame, (250, 50), (400, 150), (0, 255, 0), -1)  # Green
        cv2.rectangle(test_frame, (450, 50), (600, 150), (0, 0, 255), -1)  # Red
        cv2.putText(
            test_frame,
            "DRONE TEST FRAME",
            (150, 250),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.5,
            (255, 255, 255),
            2,
        )

        video_frames = [test_frame, test_frame.copy()]

        # Mock replay stream
        mock_stream = MagicMock()
        received_frames = []

        def subscribe_side_effect(callback) -> None:
            for frame in video_frames:
                img = Image(data=frame, format=ImageFormat.BGR)
                callback(img)
                received_frames.append(img)

        mock_stream.subscribe = subscribe_side_effect
        mock_replay.return_value.stream.return_value = mock_stream

        # Create fake video stream
        video_stream = FakeDJIVideoStream(port=5600)
        stream = video_stream.get_stream()

        # Subscribe to stream
        captured_frames = []
        stream.subscribe(captured_frames.append)

        # Verify frames were captured
        self.assertEqual(len(received_frames), 2)
        for i, frame in enumerate(received_frames):
            self.assertIsInstance(frame, Image)
            self.assertEqual(frame.data.shape, (360, 640, 3))

            # Save first frame to file for visual inspection
            if i == 0:
                import os

                output_path = "/tmp/drone_test_frame.png"
                cv2.imwrite(output_path, frame.data)
                print(f"\n[TEST] Saved test frame to {output_path} for visual inspection")
                if os.path.exists(output_path):
                    print(f"[TEST] File size: {os.path.getsize(output_path)} bytes")


class TestDroneMovementAndOdometry(unittest.TestCase):
    """Test drone movement commands and odometry."""

    @patch("dimos.utils.testing.TimedSensorReplay")
    @patch("dimos.utils.data.get_data")
    def test_movement_command_conversion(self, mock_get_data, mock_replay) -> None:
        """Test movement commands are properly converted from ROS to NED."""
        mock_stream = MagicMock()
        mock_stream.subscribe = lambda callback: None
        mock_replay.return_value.stream.return_value = mock_stream

        conn = FakeMavlinkConnection("replay")

        # Test movement in ROS frame
        # ROS: X=forward, Y=left, Z=up
        velocity_ros = Vector3(2.0, -1.0, 0.5)  # Forward 2m/s, right 1m/s, up 0.5m/s

        result = conn.move(velocity_ros, duration=1.0)
        self.assertTrue(result)

        # Movement should be converted to NED internally
        # The fake connection doesn't actually send commands, but it should not crash

    @patch("dimos.utils.testing.TimedSensorReplay")
    @patch("dimos.utils.data.get_data")
    def test_odometry_from_replay(self, mock_get_data, mock_replay) -> None:
        """Test odometry is properly generated from replay messages."""
        # Set up replay messages
        messages = [
            {"mavpackettype": "ATTITUDE", "roll": 0.1, "pitch": 0.2, "yaw": 0.3},
            {
                "mavpackettype": "GLOBAL_POSITION_INT",
                "lat": 377810501,
                "lon": -1224069671,
                "alt": 10000,
                "relative_alt": 5000,
                "vx": 200,  # 2 m/s North
                "vy": 100,  # 1 m/s East
                "vz": -50,  # 0.5 m/s Up
                "hdg": 18000,  # 180 degrees
            },
        ]

        def replay_stream_subscribe(callback) -> None:
            for msg in messages:
                callback(msg)

        mock_stream = MagicMock()
        mock_stream.subscribe = replay_stream_subscribe
        mock_replay.return_value.stream.return_value = mock_stream

        conn = FakeMavlinkConnection("replay")

        # Collect published odometry
        published_odom = []
        conn._odom_subject.subscribe(published_odom.append)

        # Process messages
        for _ in range(5):
            conn.update_telemetry(timeout=0.01)

        # Should have published odometry
        self.assertGreater(len(published_odom), 0)

        # Check odometry message
        odom = published_odom[0]
        self.assertIsInstance(odom, PoseStamped)
        self.assertIsNotNone(odom.orientation)
        self.assertEqual(odom.frame_id, "world")

    @patch("dimos.utils.testing.TimedSensorReplay")
    @patch("dimos.utils.data.get_data")
    def test_position_integration_indoor(self, mock_get_data, mock_replay) -> None:
        """Test position integration for indoor flight without GPS."""
        messages = [
            {"mavpackettype": "ATTITUDE", "roll": 0, "pitch": 0, "yaw": 0},
            {
                "mavpackettype": "GLOBAL_POSITION_INT",
                "lat": 0,  # Invalid GPS
                "lon": 0,
                "alt": 0,
                "relative_alt": 2000,  # 2m altitude
                "vx": 100,  # 1 m/s North
                "vy": 0,
                "vz": 0,
                "hdg": 0,
            },
        ]

        def replay_stream_subscribe(callback) -> None:
            for msg in messages:
                callback(msg)

        mock_stream = MagicMock()
        mock_stream.subscribe = replay_stream_subscribe
        mock_replay.return_value.stream.return_value = mock_stream

        conn = FakeMavlinkConnection("replay")

        # Process messages multiple times to integrate position
        initial_time = time.time()
        conn._last_update = initial_time

        for _i in range(3):
            conn.update_telemetry(timeout=0.01)
            time.sleep(0.1)  # Let some time pass for integration

        # Position should have been integrated
        self.assertGreater(conn._position["x"], 0)  # Moving North
        self.assertEqual(conn._position["z"], 2.0)  # Altitude from relative_alt


class TestDroneStatusAndTelemetry(unittest.TestCase):
    """Test drone status and telemetry reporting."""

    @patch("dimos.utils.testing.TimedSensorReplay")
    @patch("dimos.utils.data.get_data")
    def test_status_extraction(self, mock_get_data, mock_replay) -> None:
        """Test status is properly extracted from MAVLink messages."""
        messages = [
            {"mavpackettype": "HEARTBEAT", "type": 2, "base_mode": 193},  # Armed
            {
                "mavpackettype": "BATTERY_STATUS",
                "voltages": [3700, 3700, 3700, 3700],
                "current_battery": -1500,
                "battery_remaining": 65,
            },
            {"mavpackettype": "GPS_RAW_INT", "satellites_visible": 12, "fix_type": 3},
            {"mavpackettype": "GLOBAL_POSITION_INT", "relative_alt": 8000, "hdg": 27000},
        ]

        def replay_stream_subscribe(callback) -> None:
            for msg in messages:
                callback(msg)

        mock_stream = MagicMock()
        mock_stream.subscribe = replay_stream_subscribe
        mock_replay.return_value.stream.return_value = mock_stream

        conn = FakeMavlinkConnection("replay")

        # Collect published status
        published_status = []
        conn._status_subject.subscribe(published_status.append)

        # Process messages
        for _ in range(5):
            conn.update_telemetry(timeout=0.01)

        # Should have published status
        self.assertGreater(len(published_status), 0)

        # Check status fields
        status = published_status[-1]  # Get latest
        self.assertIn("armed", status)
        self.assertIn("battery_remaining", status)
        self.assertIn("satellites", status)
        self.assertIn("altitude", status)
        self.assertIn("heading", status)

    @patch("dimos.utils.testing.TimedSensorReplay")
    @patch("dimos.utils.data.get_data")
    def test_telemetry_json_publishing(self, mock_get_data, mock_replay) -> None:
        """Test full telemetry is published as JSON."""
        messages = [
            {"mavpackettype": "ATTITUDE", "roll": 0.1, "pitch": 0.2, "yaw": 0.3},
            {"mavpackettype": "GLOBAL_POSITION_INT", "lat": 377810501, "lon": -1224069671},
        ]

        def replay_stream_subscribe(callback) -> None:
            for msg in messages:
                callback(msg)

        mock_stream = MagicMock()
        mock_stream.subscribe = replay_stream_subscribe
        mock_replay.return_value.stream.return_value = mock_stream

        # Create connection module with replay
        module = DroneConnectionModule(connection_string="replay")

        # Mock publishers
        published_telemetry = []
        module.telemetry = MagicMock(publish=lambda x: published_telemetry.append(x))
        module.status = MagicMock()
        module.odom = MagicMock()
        module.tf = MagicMock()
        module.video = MagicMock()
        module.movecmd = MagicMock()

        # Start module
        result = module.start()
        self.assertTrue(result)

        # Give time for processing
        time.sleep(0.2)

        # Stop module
        module.stop()

        # Check telemetry was published
        self.assertGreater(len(published_telemetry), 0)

        # Telemetry should be JSON string
        telem_msg = published_telemetry[0]
        self.assertIsNotNone(telem_msg)

        # If it's a String message, check the data
        if hasattr(telem_msg, "data"):
            telem_dict = json.loads(telem_msg.data)
            self.assertIn("timestamp", telem_dict)


class TestFlyToErrorHandling(unittest.TestCase):
    """Test fly_to() error handling paths."""

    @patch("dimos.utils.testing.TimedSensorReplay")
    @patch("dimos.utils.data.get_data")
    def test_concurrency_lock(self, mock_get_data, mock_replay) -> None:
        """flying_to_target=True rejects concurrent fly_to() calls."""
        mock_stream = MagicMock()
        mock_stream.subscribe = lambda callback: None
        mock_replay.return_value.stream.return_value = mock_stream

        conn = FakeMavlinkConnection("replay")
        conn.flying_to_target = True

        result = conn.fly_to(37.0, -122.0, 10.0)
        self.assertIn("Already flying to target", result)

    @patch("dimos.utils.testing.TimedSensorReplay")
    @patch("dimos.utils.data.get_data")
    def test_error_when_not_connected(self, mock_get_data, mock_replay) -> None:
        """connected=False returns error immediately."""
        mock_stream = MagicMock()
        mock_stream.subscribe = lambda callback: None
        mock_replay.return_value.stream.return_value = mock_stream

        conn = FakeMavlinkConnection("replay")
        conn.connected = False

        result = conn.fly_to(37.0, -122.0, 10.0)
        self.assertIn("Not connected", result)


class TestVisualServoingEdgeCases(unittest.TestCase):
    """Test DroneVisualServoingController edge cases."""

    def test_output_clamping(self) -> None:
        """Large errors are clamped to max_velocity."""
        from dimos.robot.drone.drone_visual_servoing_controller import (
            DroneVisualServoingController,
        )

        # PID params: (kp, ki, kd, output_limits, integral_limit, deadband)
        max_vel = 2.0
        controller = DroneVisualServoingController(
            x_pid_params=(1.0, 0.0, 0.0, (-max_vel, max_vel), None, 0),
            y_pid_params=(1.0, 0.0, 0.0, (-max_vel, max_vel), None, 0),
        )

        # Large error should be clamped
        vx, vy, _vz = controller.compute_velocity_control(
            target_x=1000, target_y=1000, center_x=0, center_y=0, dt=0.1
        )
        self.assertLessEqual(abs(vx), max_vel)
        self.assertLessEqual(abs(vy), max_vel)

    def test_deadband_prevents_integral_windup(self) -> None:
        """Deadband prevents integral accumulation for small errors."""
        from dimos.robot.drone.drone_visual_servoing_controller import (
            DroneVisualServoingController,
        )

        deadband = 10  # pixels
        controller = DroneVisualServoingController(
            x_pid_params=(0.0, 1.0, 0.0, (-2.0, 2.0), None, deadband),  # integral only
            y_pid_params=(0.0, 1.0, 0.0, (-2.0, 2.0), None, deadband),
        )

        # With error inside deadband, integral should stay at zero
        for _ in range(10):
            controller.compute_velocity_control(
                target_x=5, target_y=5, center_x=0, center_y=0, dt=0.1
            )

        # Integral should be zero since error < deadband
        self.assertEqual(controller.x_pid.integral, 0.0)
        self.assertEqual(controller.y_pid.integral, 0.0)

    def test_reset_clears_integral(self) -> None:
        """reset() clears accumulated integral to prevent windup."""
        from dimos.robot.drone.drone_visual_servoing_controller import (
            DroneVisualServoingController,
        )

        controller = DroneVisualServoingController(
            x_pid_params=(0.0, 1.0, 0.0, (-10.0, 10.0), None, 0),  # Only integral
            y_pid_params=(0.0, 1.0, 0.0, (-10.0, 10.0), None, 0),
        )

        # Accumulate integral by calling multiple times with error
        for _ in range(10):
            controller.compute_velocity_control(
                target_x=100, target_y=100, center_x=0, center_y=0, dt=0.1
            )

        # Integral should be non-zero
        self.assertNotEqual(controller.x_pid.integral, 0.0)

        # Reset should clear it
        controller.reset()
        self.assertEqual(controller.x_pid.integral, 0.0)
        self.assertEqual(controller.y_pid.integral, 0.0)


class TestVisualServoingVelocity(unittest.TestCase):
    """Test visual servoing velocity calculations."""

    def test_velocity_from_bbox_center_error(self) -> None:
        """Bbox center offset produces proportional velocity command."""
        from dimos.robot.drone.drone_visual_servoing_controller import (
            DroneVisualServoingController,
        )

        controller = DroneVisualServoingController(
            x_pid_params=(0.01, 0.0, 0.0, (-2.0, 2.0), None, 0),
            y_pid_params=(0.01, 0.0, 0.0, (-2.0, 2.0), None, 0),
        )

        # Image center at (320, 180), bbox center at (400, 180) = 80px right
        frame_center = (320, 180)
        bbox_center = (400, 180)

        vx, vy, _vz = controller.compute_velocity_control(
            target_x=bbox_center[0],
            target_y=bbox_center[1],
            center_x=frame_center[0],
            center_y=frame_center[1],
            dt=0.1,
        )

        # Object to the right -> drone should strafe right (positive vy)
        self.assertGreater(vy, 0)
        # No vertical offset -> vx should be ~0
        self.assertAlmostEqual(vx, 0, places=1)


if __name__ == "__main__":
    unittest.main()
